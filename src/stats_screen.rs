//! Final in-window statistics view shown after the simulation ends.

use crate::geometry::H;
use crate::render::CANVAS_W;
use crate::simulation::Sim;
use crate::ui_font::draw_text;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::Canvas;
use sdl2::video::Window;
use std::time::Duration;

const BACKGROUND: Color = Color::RGB(18, 20, 29);
const PANEL: Color = Color::RGB(27, 29, 41);
const PANEL_EDGE: Color = Color::RGB(70, 73, 88);
const TEXT: Color = Color::RGB(236, 236, 240);
const MUTED: Color = Color::RGB(151, 155, 174);
const ACCENT: Color = Color::RGB(145, 132, 217);

pub fn show(
    canvas: &mut Canvas<Window>,
    events: &mut sdl2::EventPump,
    sim: &Sim,
) -> Result<(), String> {
    'statistics: loop {
        // Draw first so an already queued key event cannot close the view before
        // the final statistics have been presented at least once.
        draw(canvas, sim)?;

        for event in events.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape | Keycode::Return | Keycode::Q),
                    repeat: false,
                    ..
                } => break 'statistics,
                _ => {}
            }
        }

        std::thread::sleep(Duration::from_millis(16));
    }

    Ok(())
}

fn draw(canvas: &mut Canvas<Window>, sim: &Sim) -> Result<(), String> {
    canvas.set_draw_color(BACKGROUND);
    canvas.clear();

    let panel = Rect::new(110, 70, CANVAS_W - 220, H - 140);
    canvas.set_draw_color(PANEL);
    canvas.fill_rect(panel)?;
    canvas.set_draw_color(PANEL_EDGE);
    canvas.draw_rect(panel)?;

    draw_text(canvas, "SMART ROAD", 155, 110, 4, TEXT)?;
    draw_text(canvas, "FINAL STATISTICS", 158, 158, 2, ACCENT)?;

    let summary = sim.statistics_summary();
    let mut y = 220;
    for line in summary.lines() {
        if line.trim().is_empty() {
            y += 14;
            continue;
        }
        let is_section = line == "Additional statistics";
        draw_text(
            canvas,
            line,
            160,
            y,
            if is_section { 2 } else { 1 },
            if is_section { ACCENT } else { TEXT },
        )?;
        y += if is_section { 34 } else { 27 };
    }

    draw_text(
        canvas,
        "ESC / ENTER / Q  CLOSE",
        160,
        H as i32 - 90,
        1,
        MUTED,
    )?;
    canvas.present();
    Ok(())
}
