//! Passive SDL rendering of the simulation state.

use crate::geometry::{CAR_LEN, CX, CY, FIXED_HZ, H, LANE_W, ROAD_HALF, Route, W};
use crate::simulation::Sim;
use crate::sprites::{route_color, SpriteSet};
use crate::ui_font::draw_text;
use crate::vehicle::{Vehicle, VehicleVisual};
use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::Canvas;
use sdl2::video::Window;

pub const PANEL_W: u32 = 300;
pub const CANVAS_W: u32 = W + PANEL_W;

const BACKGROUND: Color = Color::RGB(35, 43, 38);
const ASPHALT: Color = Color::RGB(29, 45, 64);
const ROAD_EDGE: Color = Color::RGB(105, 123, 139);
const LANE_MARK: Color = Color::RGB(145, 157, 170);
const CENTER_MARK: Color = Color::RGB(239, 201, 73);
const STOP_LINE: Color = Color::RGB(235, 235, 240);
const PANEL_BG: Color = Color::RGB(20, 21, 31);
const CARD_BG: Color = Color::RGB(27, 29, 41);
const CARD_EDGE: Color = Color::RGB(63, 66, 77);
const TEXT: Color = Color::RGB(233, 233, 237);
const MUTED: Color = Color::RGB(147, 151, 171);
const ACCENT: Color = Color::RGB(145, 132, 217);
const GOOD: Color = Color::RGB(57, 217, 138);
const BAD: Color = Color::RGB(255, 85, 96);
const TURN_SIGNAL: Color = Color::RGB(255, 166, 48);
const TURN_SIGNAL_HALF_PERIOD_TICKS: u64 = FIXED_HZ as u64 / 3;
const CONTROL_KEY_W: u32 = 72;
const CONTROL_DESCRIPTION_X: i32 = 84;
const TREE_SIZE: u32 = 42;

const TREE_POSITIONS: [(i32, i32); 16] = [
    (290, 80),
    (610, 80),
    (290, 200),
    (610, 200),
    (290, 700),
    (610, 700),
    (290, 820),
    (610, 820),
    (80, 290),
    (200, 290),
    (700, 290),
    (820, 290),
    (80, 610),
    (200, 610),
    (700, 610),
    (820, 610),
];

pub fn draw(
    canvas: &mut Canvas<Window>,
    sim: &Sim,
    sprites: &SpriteSet<'_>,
    auto_spawn: bool,
    paused: bool,
) -> Result<(), String> {
    canvas.set_draw_color(BACKGROUND);
    canvas.clear();

    draw_trees(canvas, sprites)?;
    draw_roads(canvas)?;

    for vehicle in &sim.vehicles {
        draw_vehicle(canvas, sprites, vehicle)?;

        let path = &sim.paths[vehicle.origin][vehicle.route.index()];
        if turn_signal_visible(path, vehicle.route, vehicle.progress, sim.tick()) {
            draw_turn_signal(canvas, vehicle)?;
        }
    }

    draw_panel(canvas, sim, auto_spawn, paused)?;
    canvas.present();
    Ok(())
}

pub fn update_title(canvas: &mut Canvas<Window>, sim: &Sim, auto_spawn: bool, paused: bool) {
    let title = format!(
        "Smart Road | passed {} | on road {} | auto {}{}",
        sim.stats.passed,
        sim.vehicles.len(),
        if auto_spawn { "ON" } else { "OFF" },
        if paused { " | PAUSED" } else { "" },
    );
    let _ = canvas.window_mut().set_title(&title);
}

fn vehicle_render_size(visual: VehicleVisual) -> (u32, u32) {
    match visual {
        VehicleVisual::Sedan => (16, 32),
        VehicleVisual::Sport => (16, 32),
        VehicleVisual::RoboTaxi => (17, 30),
        VehicleVisual::Bus => (18, 46),
        VehicleVisual::Police => (16, 32),
        VehicleVisual::Ambulance => (19, 40),
        VehicleVisual::Fire => (19, 44),
    }
}

fn draw_vehicle(
    canvas: &mut Canvas<Window>,
    sprites: &SpriteSet<'_>,
    vehicle: &Vehicle,
) -> Result<(), String> {
    let source = sprites.vehicle_source(vehicle.visual);
    let (visual_width, visual_length) = vehicle_render_size(vehicle.visual);
    let destination = Rect::from_center(
        (
            vehicle.position.0.round() as i32,
            vehicle.position.1.round() as i32,
        ),
        visual_width,
        visual_length,
    );

    // Atlas vehicles face north. Path headings use mathematical radians where
    // zero points east, so +90 degrees aligns the sprite nose with the route.
    canvas.copy_ex(
        sprites.atlas(),
        Some(source),
        Some(destination),
        vehicle.angle.to_degrees() + 90.0,
        None,
        false,
        false,
    )
}

fn draw_trees(canvas: &mut Canvas<Window>, sprites: &SpriteSet<'_>) -> Result<(), String> {
    let source = sprites.tree_source();
    for &(x, y) in &TREE_POSITIONS {
        canvas.copy(
            sprites.atlas(),
            Some(source),
            Some(Rect::from_center((x, y), TREE_SIZE, TREE_SIZE)),
        )?;
    }
    Ok(())
}

fn draw_roads(canvas: &mut Canvas<Window>) -> Result<(), String> {
    let x0 = (CX - ROAD_HALF).round() as i32;
    let y0 = (CY - ROAD_HALF).round() as i32;
    let x1 = (CX + ROAD_HALF).round() as i32;
    let y1 = (CY + ROAD_HALF).round() as i32;
    let road_width = (ROAD_HALF * 2.0).round() as u32;

    // One 240 px carriageway = six 40 px lanes: three lanes in each direction.
    // The visual markings are generated from the same LANE_W geometry as the
    // physical paths, so there is no second incompatible lane layout underneath.
    canvas.set_draw_color(ASPHALT);
    canvas.fill_rect(Rect::new(x0, 0, road_width, H))?;
    canvas.fill_rect(Rect::new(0, y0, W, road_width))?;

    draw_road_edges(canvas, x0, y0, x1, y1)?;

    // Two dashed separators on either side of the directional divider produce
    // exactly three lanes per travel direction.
    for offset in [-2.0, -1.0, 1.0, 2.0] {
        draw_vertical_lane(canvas, CX + offset * LANE_W, LANE_MARK)?;
        draw_horizontal_lane(canvas, CY + offset * LANE_W, LANE_MARK)?;
    }

    draw_center_divider(canvas, x0, y0, x1, y1)?;
    draw_stop_lines(canvas)?;
    Ok(())
}

fn draw_road_edges(
    canvas: &mut Canvas<Window>,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
) -> Result<(), String> {
    canvas.set_draw_color(ROAD_EDGE);

    for x in [x0, x1] {
        canvas.draw_line((x, 0), (x, y0))?;
        canvas.draw_line((x, y1), (x, H as i32))?;
    }
    for y in [y0, y1] {
        canvas.draw_line((0, y), (x0, y))?;
        canvas.draw_line((x1, y), (W as i32, y))?;
    }
    Ok(())
}

fn draw_center_divider(
    canvas: &mut Canvas<Window>,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
) -> Result<(), String> {
    canvas.set_draw_color(CENTER_MARK);
    let cx = CX.round() as i32;
    let cy = CY.round() as i32;

    for x in [cx - 2, cx + 2] {
        canvas.draw_line((x, 0), (x, y0))?;
        canvas.draw_line((x, y1), (x, H as i32))?;
    }
    for y in [cy - 2, cy + 2] {
        canvas.draw_line((0, y), (x0, y))?;
        canvas.draw_line((x1, y), (W as i32, y))?;
    }
    Ok(())
}

fn draw_vertical_lane(canvas: &mut Canvas<Window>, x: f64, color: Color) -> Result<(), String> {
    draw_dashed_line(
        canvas,
        (x.round() as i32, 0),
        (x.round() as i32, (CY - ROAD_HALF).round() as i32),
        true,
        color,
    )?;
    draw_dashed_line(
        canvas,
        (x.round() as i32, (CY + ROAD_HALF).round() as i32),
        (x.round() as i32, H as i32),
        true,
        color,
    )
}

fn draw_horizontal_lane(canvas: &mut Canvas<Window>, y: f64, color: Color) -> Result<(), String> {
    draw_dashed_line(
        canvas,
        (0, y.round() as i32),
        ((CX - ROAD_HALF).round() as i32, y.round() as i32),
        false,
        color,
    )?;
    draw_dashed_line(
        canvas,
        ((CX + ROAD_HALF).round() as i32, y.round() as i32),
        (W as i32, y.round() as i32),
        false,
        color,
    )
}

fn draw_dashed_line(
    canvas: &mut Canvas<Window>,
    start: (i32, i32),
    end: (i32, i32),
    vertical: bool,
    color: Color,
) -> Result<(), String> {
    canvas.set_draw_color(color);
    let dash = 16;
    let gap = 14;
    let mut cursor = if vertical { start.1 } else { start.0 };
    let limit = if vertical { end.1 } else { end.0 };
    while cursor < limit {
        let dash_end = (cursor + dash).min(limit);
        if vertical {
            canvas.draw_line((start.0, cursor), (start.0, dash_end))?;
        } else {
            canvas.draw_line((cursor, start.1), (dash_end, start.1))?;
        }
        cursor += dash + gap;
    }
    Ok(())
}

fn draw_stop_lines(canvas: &mut Canvas<Window>) -> Result<(), String> {
    canvas.set_draw_color(STOP_LINE);
    let half = ROAD_HALF.round() as u32;
    let x0 = (CX - ROAD_HALF).round() as i32;
    let x1 = CX.round() as i32;
    let y0 = (CY - ROAD_HALF).round() as i32;
    let y1 = CY.round() as i32;

    // Only the three incoming lanes receive a stop boundary. Right-turn traffic
    // does not use it operationally, but the line still marks the approach.
    canvas.fill_rect(Rect::new(x0, y0 - 5, half, 4))?;
    canvas.fill_rect(Rect::new(x1, (CY + ROAD_HALF).round() as i32 + 1, half, 4))?;
    canvas.fill_rect(Rect::new((CX + ROAD_HALF).round() as i32 + 1, y0, 4, half))?;
    canvas.fill_rect(Rect::new((CX - ROAD_HALF).round() as i32 - 5, y1, 4, half))?;
    Ok(())
}

fn turn_signal_visible(
    path: &crate::geometry::Path,
    route: Route,
    progress: f64,
    tick: u64,
) -> bool {
    if matches!(route, Route::Straight) || path.cumulative.len() < 3 {
        return false;
    }
    let curve_end = path.cumulative[path.cumulative.len() - 2];
    let signal_end = (curve_end + CAR_LEN / 2.0).min(path.len);
    progress >= path.control_entry
        && progress < signal_end
        && (tick / TURN_SIGNAL_HALF_PERIOD_TICKS) % 2 == 0
}

fn draw_turn_signal(canvas: &mut Canvas<Window>, vehicle: &Vehicle) -> Result<(), String> {
    let forward = (vehicle.angle.cos(), vehicle.angle.sin());
    let side = (-forward.1, forward.0);
    let (visual_width, visual_length) = vehicle_render_size(vehicle.visual);
    let longitudinal_offset = visual_length as f64 / 2.0 - 2.0;
    let lateral_offset = match vehicle.route {
        Route::Left => -(visual_width as f64 / 2.0 - 1.0),
        Route::Right => visual_width as f64 / 2.0 - 1.0,
        Route::Straight => return Ok(()),
    };

    canvas.set_draw_color(TURN_SIGNAL);
    for longitudinal in [-longitudinal_offset, longitudinal_offset] {
        let x = vehicle.position.0 + forward.0 * longitudinal + side.0 * lateral_offset;
        let y = vehicle.position.1 + forward.1 * longitudinal + side.1 * lateral_offset;
        canvas.fill_rect(Rect::from_center(
            (x.round() as i32, y.round() as i32),
            2,
            2,
        ))?;
    }
    Ok(())
}

fn draw_panel(
    canvas: &mut Canvas<Window>,
    sim: &Sim,
    auto_spawn: bool,
    paused: bool,
) -> Result<(), String> {
    let panel_x = W as i32;
    canvas.set_draw_color(PANEL_BG);
    canvas.fill_rect(Rect::new(panel_x, 0, PANEL_W, H))?;
    canvas.set_draw_color(CARD_EDGE);
    canvas.draw_line((panel_x, 0), (panel_x, H as i32))?;

    draw_text(canvas, "SMART ROAD", panel_x + 22, 22, 3, TEXT)?;
    draw_text(canvas, "AUTONOMOUS INTERSECTION", panel_x + 22, 52, 1, MUTED)?;

    let status_y = 82;
    card(canvas, panel_x + 18, status_y, PANEL_W as i32 - 36, 104)?;
    draw_text(canvas, "SMART CONTROL", panel_x + 32, status_y + 16, 2, ACCENT)?;
    draw_text(
        canvas,
        if paused { "SIMULATION: PAUSED" } else { "SIMULATION: RUNNING" },
        panel_x + 32,
        status_y + 46,
        1,
        if paused { TURN_SIGNAL } else { GOOD },
    )?;
    draw_text(
        canvas,
        if auto_spawn { "AUTO SPAWN: ON" } else { "AUTO SPAWN: OFF" },
        panel_x + 32,
        status_y + 64,
        1,
        if auto_spawn { GOOD } else { MUTED },
    )?;
    let reserved = sim.vehicles.iter().filter(|vehicle| vehicle.reserved).count();
    let waiting = sim
        .vehicles
        .iter()
        .filter(|vehicle| {
            vehicle.route != Route::Right
                && vehicle.detected_tick.is_some()
                && !vehicle.reserved
        })
        .count();
    draw_text(
        canvas,
        &format!("RESERVED: {reserved}  WAITING: {waiting}"),
        panel_x + 32,
        status_y + 82,
        1,
        MUTED,
    )?;

    let routes_y = 202;
    card(canvas, panel_x + 18, routes_y, PANEL_W as i32 - 36, 84)?;
    draw_text(canvas, "ROUTES", panel_x + 32, routes_y + 14, 2, ACCENT)?;
    route_legend(canvas, panel_x + 32, routes_y + 48, Route::Right, "RIGHT")?;
    route_legend(canvas, panel_x + 112, routes_y + 48, Route::Straight, "STRAIGHT")?;
    route_legend(canvas, panel_x + 214, routes_y + 48, Route::Left, "LEFT")?;

    let stats_y = 302;
    card(canvas, panel_x + 18, stats_y, PANEL_W as i32 - 36, 126)?;
    draw_text(canvas, "LIVE STATS", panel_x + 32, stats_y + 14, 2, ACCENT)?;
    draw_text(canvas, &format!("PASSED: {}", sim.stats.passed), panel_x + 32, stats_y + 48, 1, TEXT)?;
    draw_text(canvas, &format!("ON ROAD: {}", sim.vehicles.len()), panel_x + 156, stats_y + 48, 1, TEXT)?;
    draw_text(canvas, &format!("SPAWNED: {}", sim.stats.spawned), panel_x + 32, stats_y + 68, 1, TEXT)?;
    draw_text(canvas, &format!("REJECTED: {}", sim.stats.rejected_spawns), panel_x + 156, stats_y + 68, 1, MUTED)?;
    draw_text(canvas, &format!("CLOSE CALLS: {}", sim.stats.close_calls), panel_x + 32, stats_y + 88, 1, TEXT)?;
    draw_text(
        canvas,
        &format!("COLLISIONS: {}", sim.stats.collisions),
        panel_x + 156,
        stats_y + 88,
        1,
        if sim.stats.collisions == 0 { GOOD } else { BAD },
    )?;

    let controls_y = 444;
    card(canvas, panel_x + 18, controls_y, PANEL_W as i32 - 36, 420)?;
    draw_text(canvas, "CONTROLS", panel_x + 32, controls_y + 16, 2, ACCENT)?;
    control_line(canvas, panel_x + 32, controls_y + 54, "UP", "FROM SOUTH")?;
    control_line(canvas, panel_x + 32, controls_y + 88, "DOWN", "FROM NORTH")?;
    control_line(canvas, panel_x + 32, controls_y + 122, "RIGHT", "FROM WEST")?;
    control_line(canvas, panel_x + 32, controls_y + 156, "LEFT", "FROM EAST")?;
    control_line(canvas, panel_x + 32, controls_y + 204, "R", "AUTO SPAWN ON/OFF")?;
    control_line(canvas, panel_x + 32, controls_y + 238, "SPACE", "PAUSE/RESUME")?;
    control_line(canvas, panel_x + 32, controls_y + 272, "BACKSPACE", "RESET")?;
    control_line(canvas, panel_x + 32, controls_y + 306, "ESC", "EXIT + STATISTICS")?;

    draw_text(canvas, "NO TRAFFIC LIGHTS", panel_x + 32, controls_y + 354, 1, MUTED)?;
    draw_text(canvas, "SPEED + RESERVATIONS", panel_x + 32, controls_y + 374, 1, MUTED)?;
    Ok(())
}

fn card(canvas: &mut Canvas<Window>, x: i32, y: i32, width: i32, height: i32) -> Result<(), String> {
    let rect = Rect::new(x, y, width as u32, height as u32);
    canvas.set_draw_color(CARD_BG);
    canvas.fill_rect(rect)?;
    canvas.set_draw_color(CARD_EDGE);
    canvas.draw_rect(rect)?;
    Ok(())
}

fn route_legend(
    canvas: &mut Canvas<Window>,
    x: i32,
    y: i32,
    route: Route,
    label: &str,
) -> Result<(), String> {
    canvas.set_draw_color(route_color(route));
    canvas.fill_rect(Rect::new(x, y, 12, 12))?;
    draw_text(canvas, label, x + 18, y + 2, 1, TEXT)
}

fn control_line(
    canvas: &mut Canvas<Window>,
    x: i32,
    y: i32,
    key: &str,
    description: &str,
) -> Result<(), String> {
    canvas.set_draw_color(Color::RGB(36, 38, 51));
    canvas.fill_rect(Rect::new(x, y, CONTROL_KEY_W, 24))?;
    canvas.set_draw_color(CARD_EDGE);
    canvas.draw_rect(Rect::new(x, y, CONTROL_KEY_W, 24))?;
    draw_text(canvas, key, x + 9, y + 8, 1, TEXT)?;
    draw_text(canvas, description, x + CONTROL_DESCRIPTION_X, y + 8, 1, MUTED)
}
