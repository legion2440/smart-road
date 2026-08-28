//! Smart Road: autonomous intersection simulation without traffic lights.

mod collision;
mod controller;
mod geometry;
mod render;
mod simulation;
mod sprites;
mod stats;
mod vehicle;

use geometry::{FIXED_HZ, H, W};
use render::{draw, update_title};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::messagebox::{show_simple_message_box, MessageBoxFlag};
use sdl2::render::Canvas;
use sdl2::video::Window;
use simulation::Sim;
use sprites::SpriteSet;
use std::time::{Duration, Instant};

const FALLBACK_RENDER_HZ: u64 = 60;

fn main() -> Result<(), String> {
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let (mut canvas, vsync_active) = create_canvas(&video)?;
    canvas
        .set_logical_size(W, H)
        .map_err(|error| format!("unable to set logical size: {error}"))?;

    let texture_creator = canvas.texture_creator();
    let sprites = SpriteSet::build(&mut canvas, &texture_creator)?;
    let mut events = sdl.event_pump()?;
    let mut sim = Sim::new();
    let mut auto_spawn = false;
    let mut paused = false;

    let update_interval = Duration::from_nanos(1_000_000_000 / FIXED_HZ as u64);
    let fallback_render_interval = Duration::from_nanos(1_000_000_000 / FALLBACK_RENDER_HZ);
    let mut previous_time = Instant::now();
    let mut accumulator = Duration::ZERO;

    'running: loop {
        let frame_started = Instant::now();
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. } => break 'running,
                Event::KeyDown {
                    keycode: Some(key),
                    repeat: false,
                    ..
                } => match key {
                    Keycode::Escape => break 'running,
                    // Subject mapping: arrows indicate the travel direction.
                    // Up = from south, Down = from north,
                    // Right = from west, Left = from east.
                    Keycode::Up => {
                        sim.spawn_from(2);
                    }
                    Keycode::Down => {
                        sim.spawn_from(0);
                    }
                    Keycode::Right => {
                        sim.spawn_from(3);
                    }
                    Keycode::Left => {
                        sim.spawn_from(1);
                    }
                    Keycode::R => auto_spawn = !auto_spawn,
                    Keycode::Space => paused = !paused,
                    Keycode::Backspace => {
                        sim = Sim::new();
                        auto_spawn = false;
                        paused = false;
                    }
                    _ => {}
                },
                _ => {}
            }
        }

        let now = Instant::now();
        accumulator += now
            .saturating_duration_since(previous_time)
            .min(Duration::from_millis(250));
        previous_time = now;

        while accumulator >= update_interval {
            if !paused {
                sim.step();
                if auto_spawn && sim.auto_spawn_due() {
                    sim.spawn_random();
                }
            }
            accumulator -= update_interval;
        }

        update_title(&mut canvas, &sim, auto_spawn, paused);
        draw(&mut canvas, &sim, &sprites, auto_spawn)?;

        if !vsync_active {
            let delay = fallback_render_interval.saturating_sub(frame_started.elapsed());
            if !delay.is_zero() {
                std::thread::sleep(delay);
            }
        }
    }

    let summary = sim.statistics_summary();
    show_simple_message_box(
        MessageBoxFlag::INFORMATION,
        "Smart Road statistics",
        &summary,
        Some(canvas.window()),
    )
    .map_err(|error| format!("unable to show statistics: {error}"))?;

    Ok(())
}

fn create_window(video: &sdl2::VideoSubsystem) -> Result<Window, String> {
    video
        .window("Smart Road", W, H)
        .position_centered()
        .resizable()
        .build()
        .map_err(|error| format!("unable to create SDL window: {error}"))
}

fn create_canvas(video: &sdl2::VideoSubsystem) -> Result<(Canvas<Window>, bool), String> {
    let window = create_window(video)?;
    match window.into_canvas().present_vsync().build() {
        Ok(canvas) => Ok((canvas, true)),
        Err(vsync_error) => {
            let fallback = create_window(video)?
                .into_canvas()
                .build()
                .map_err(|fallback_error| {
                    format!(
                        "unable to create renderer: VSync failed ({vsync_error}); fallback failed ({fallback_error})"
                    )
                })?;
            Ok((fallback, false))
        }
    }
}
