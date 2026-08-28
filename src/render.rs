//! Passive SDL rendering of the simulation state.

use crate::geometry::{CX, CY, H, LANE_W, ROAD_HALF, Route, W};
use crate::simulation::Sim;
use crate::sprites::{route_color, SpriteSet, CAR_TEXTURE_H, CAR_TEXTURE_W};
use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::Canvas;
use sdl2::video::Window;

const BACKGROUND: Color = Color::RGB(35, 43, 38);
const ASPHALT: Color = Color::RGB(42, 44, 50);
const ROAD_EDGE: Color = Color::RGB(205, 208, 214);
const LANE_MARK: Color = Color::RGB(130, 134, 143);
const CENTER_MARK: Color = Color::RGB(225, 190, 74);
const STOP_LINE: Color = Color::RGB(235, 235, 240);
const RESERVED: Color = Color::RGB(94, 226, 142);
const AUTO_ON: Color = Color::RGB(94, 226, 142);
const AUTO_OFF: Color = Color::RGB(190, 74, 74);

pub fn draw(
    canvas: &mut Canvas<Window>,
    sim: &Sim,
    sprites: &SpriteSet<'_>,
    auto_spawn: bool,
) -> Result<(), String> {
    canvas.set_draw_color(BACKGROUND);
    canvas.clear();

    draw_roads(canvas)?;
    draw_route_hints(canvas, sim)?;

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

        if vehicle.reserved {
            canvas.set_draw_color(RESERVED);
            let outline = Rect::from_center(
                (
                    vehicle.position.0.round() as i32,
                    vehicle.position.1.round() as i32,
                ),
                CAR_TEXTURE_W + 4,
                CAR_TEXTURE_H + 4,
            );
            canvas.draw_rect(outline)?;
        }
    }

    canvas.set_draw_color(if auto_spawn { AUTO_ON } else { AUTO_OFF });
    canvas.fill_rect(Rect::new(14, 14, 18, 18))?;

    canvas.present();
    Ok(())
}

pub fn update_title(canvas: &mut Canvas<Window>, sim: &Sim, auto_spawn: bool, paused: bool) {
    let title = format!(
        "Smart Road | passed {} | on road {} | R auto {}{}",
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

fn draw_route_hints(canvas: &mut Canvas<Window>, sim: &Sim) -> Result<(), String> {
    for origin in 0..4 {
        for route in Route::ALL {
            let path = &sim.paths[origin][route.index()];
            let marker_progress = (path.stop_progress - 62.0).max(0.0);
            let (x, y, _) = path.at(marker_progress);
            canvas.set_draw_color(route_color(route));
            canvas.fill_rect(Rect::from_center(
                (x.round() as i32, y.round() as i32),
                8,
                8,
            ))?;
        }
    }
    Ok(())
}
