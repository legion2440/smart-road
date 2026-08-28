//! Passive SDL rendering of the simulation state.

use crate::geometry::{CAR_LEN, CX, CY, FIXED_HZ, H, LANE_W, ROAD_HALF, Route, W};
use crate::simulation::Sim;
use crate::sprites::{route_color, SpriteSet, CAR_TEXTURE_H, CAR_TEXTURE_W};
use crate::ui_font::draw_text;
use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::Canvas;
use sdl2::video::Window;

pub const PANEL_W: u32 = 300;
pub const CANVAS_W: u32 = W + PANEL_W;

const BACKGROUND: Color = Color::RGB(35, 43, 38);
const ASPHALT: Color = Color::RGB(42, 44, 50);
const ROAD_EDGE: Color = Color::RGB(205, 208, 214);
const LANE_MARK: Color = Color::RGB(130, 134, 143);
const CENTER_MARK: Color = Color::RGB(225, 190, 74);
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

pub fn draw(
    canvas: &mut Canvas<Window>,
    sim: &Sim,
    sprites: &SpriteSet<'_>,
    auto_spawn: bool,
    paused: bool,
) -> Result<(), String> {
    canvas.set_draw_color(BACKGROUND);
    canvas.clear();

    draw_roads(canvas)?;

    for vehicle in &sim.vehicles {
        let destination = Rect::from_center(
            (
                vehicle.position.0.round() as i32,
                vehicle.position.1.round() as i32,
            ),
            CAR_TEXTURE_W,
            CAR_TEXTURE_H,
        );
        canvas.copy_ex(
            sprites.car(vehicle.route),
            None,
            Some(destination),
            vehicle.angle.to_degrees(),
            None,
            false,
            false,
        )?;

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

fn draw_roads(canvas: &mut Canvas<Window>) -> Result<(), String> {
    canvas.set_draw_color(ASPHALT);
    canvas.fill_rect(Rect::new(
        (CX - ROAD_HALF).round() as i32,
        0,
        (ROAD_HALF * 2.0).round() as u32,
        H,
    ))?;
    canvas.fill_rect(Rect::new(
        0,
        (CY - ROAD_HALF).round() as i32,
        W,
        (ROAD_HALF * 2.0).round() as u32,
    ))?;

    canvas.set_draw_color(ROAD_EDGE);
    for x in [CX - ROAD_HALF, CX + ROAD_HALF] {
        canvas.draw_line((x.round() as i32, 0), (x.round() as i32, H as i32))?;
    }
    for y in [CY - ROAD_HALF, CY + ROAD_HALF] {
        canvas.draw_line((0, y.round() as i32), (W as i32, y.round() as i32))?;
    }

    for offset in [-2.0, -1.0, 0.0, 1.0, 2.0] {
        let x = CX + offset * LANE_W;
        let y = CY + offset * LANE_W;
        let color = if offset == 0.0 { CENTER_MARK } else { LANE_MARK };
        draw_vertical_lane(canvas, x, color)?;
        draw_horizontal_lane(canvas, y, color)?;
    }

    draw_stop_lines(canvas)?;
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
    let dash = 14;
    let gap = 12;
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
        && (tick / TURN_SIGNAL_HALF_PERIOD_TICKS).is_multiple_of(2)
}

fn draw_turn_signal(
    canvas: &mut Canvas<Window>,
    vehicle: &crate::vehicle::Vehicle,
) -> Result<(), String> {
    let forward = (vehicle.angle.cos(), vehicle.angle.sin());
    let side = (-forward.1, forward.0);
    let longitudinal_offset = CAR_LEN / 2.0 - 1.5;
    let lateral_offset = match vehicle.route {
        Route::Left => -(crate::geometry::CAR_W / 2.0 - 2.5),
        Route::Right => crate::geometry::CAR_W / 2.0 - 2.5,
        Route::Straight => return Ok(()),
    };

    canvas.set_draw_color(TURN_SIGNAL);
    for longitudinal in [-longitudinal_offset, longitudinal_offset] {
        let x = vehicle.position.0 + forward.0 * longitudinal + side.0 * lateral_offset;
        let y = vehicle.position.1 + forward.1 * longitudinal + side.1 * lateral_offset;
        canvas.fill_rect(Rect::from_center(
            (x.round() as i32, y.round() as i32),
            3,
            3,
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
        .filter(|vehicle| vehicle.detected_tick.is_some() && !vehicle.reserved)
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
