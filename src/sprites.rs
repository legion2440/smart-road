//! Car sprite loading.
//!
//! The original `road_intersection` car sheet is stored as base64 text so the
//! repository remains self-contained even when binary GitHub transport is not
//! available to the project authoring workflow.

use base64::{engine::general_purpose::STANDARD, Engine as _};
use crate::geometry::{CAR_LEN, CAR_W, Route};
use sdl2::hint::Hint;
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::rwops::RWops;
use sdl2::surface::Surface;
use sdl2::video::{Window, WindowContext};

pub const CAR_FRAME_W: u32 = 30;
pub const CAR_FRAME_H: u32 = 17;
pub const CAR_ROUTES: u32 = 3;
pub const CAR_PAD: u32 = 2;
pub const CAR_TEXTURE_W: u32 = CAR_LEN as u32 + 2 * CAR_PAD;
pub const CAR_TEXTURE_H: u32 = CAR_W as u32 + 2 * CAR_PAD;

const CHROMA_KEY: Color = Color::RGB(255, 0, 255);
const CARS_B64: &str = include_str!("../assets/cars.bmp.b64");

pub struct SpriteSet<'a> {
    cars: Vec<Texture<'a>>,
}

impl<'a> SpriteSet<'a> {
    pub fn build(
        canvas: &mut Canvas<Window>,
        creator: &'a TextureCreator<WindowContext>,
    ) -> Result<Self, String> {
        Ok(Self {
            cars: build_cars(canvas, creator)?,
        })
    }

    pub fn car(&self, route: Route) -> &Texture<'a> {
        &self.cars[route.sprite_index()]
    }
}

pub fn route_color(route: Route) -> Color {
    match route {
        Route::Right => Color::RGB(68, 199, 179),
        Route::Straight => Color::RGB(153, 113, 228),
        Route::Left => Color::RGB(240, 163, 58),
    }
}

fn build_cars<'a>(
    canvas: &mut Canvas<Window>,
    creator: &'a TextureCreator<WindowContext>,
) -> Result<Vec<Texture<'a>>, String> {
    if !sdl2::hint::set_with_priority("SDL_RENDER_SCALE_QUALITY", "0", &Hint::Override) {
        return Err("failed to enable nearest-neighbour texture sampling".to_string());
    }

    let bytes = STANDARD
        .decode(CARS_B64.lines().collect::<String>())
        .map_err(|error| format!("unable to decode embedded cars.bmp: {error}"))?;
    let mut rwops = RWops::from_bytes(&bytes)
        .map_err(|error| format!("unable to open embedded cars.bmp: {error}"))?;
    let surface = Surface::load_bmp_rw(&mut rwops)
        .map_err(|error| format!("unable to load embedded cars.bmp: {error}"))?;
    if surface.width() != CAR_FRAME_W || surface.height() != CAR_FRAME_H * CAR_ROUTES {
        return Err(format!(
            "invalid cars.bmp dimensions: expected {}x{}, got {}x{}",
            CAR_FRAME_W,
            CAR_FRAME_H * CAR_ROUTES,
            surface.width(),
            surface.height(),
        ));
    }

    let mut surface = surface
        .convert_format(PixelFormatEnum::RGBA32)
        .map_err(|error| format!("unable to convert cars.bmp: {error}"))?;
    remove_chroma_fringe(&mut surface);
    surface
        .set_color_key(true, CHROMA_KEY)
        .map_err(|error| format!("unable to set cars.bmp color key: {error}"))?;
    let source = creator
        .create_texture_from_surface(&surface)
        .map_err(|error| format!("unable to create cars.bmp texture: {error}"))?;

    let mut cars = Vec::with_capacity(CAR_ROUTES as usize);
    for sprite_index in 0..CAR_ROUTES {
        let mut texture = creator
            .create_texture_target(None, CAR_TEXTURE_W, CAR_TEXTURE_H)
            .map_err(|error| format!("unable to create padded car texture: {error}"))?;
        texture.set_blend_mode(BlendMode::Blend);

        let source_rect = Rect::new(
            0,
            (sprite_index * CAR_FRAME_H) as i32,
            CAR_FRAME_W,
            CAR_FRAME_H,
        );
        let destination_rect = Rect::new(
            CAR_PAD as i32,
            CAR_PAD as i32,
            CAR_FRAME_W,
            CAR_FRAME_H,
        );
        let mut copy_error = None;
        canvas
            .with_texture_canvas(&mut texture, |target| {
                target.set_blend_mode(BlendMode::None);
                target.set_draw_color(Color::RGBA(0, 0, 0, 0));
                target.clear();
                target.set_blend_mode(BlendMode::Blend);
                if let Err(error) = target.copy(&source, Some(source_rect), Some(destination_rect)) {
                    copy_error = Some(error);
                }
            })
            .map_err(|error| format!("unable to build car texture: {error}"))?;
        if let Some(error) = copy_error {
            return Err(format!("unable to copy car sprite: {error}"));
        }
        cars.push(texture);
    }

    Ok(cars)
}

fn remove_chroma_fringe(surface: &mut Surface<'_>) {
    let width = surface.width() as usize;
    let height = surface.height() as usize;
    let pitch = surface.pitch() as usize;
    surface.with_lock_mut(|pixels| {
        for y in 0..height {
            for x in 0..width {
                let offset = y * pitch + x * 4;
                let red = pixels[offset];
                let green = pixels[offset + 1];
                let blue = pixels[offset + 2];
                if red > 180 && green < 64 && blue > 140 {
                    pixels[offset..offset + 4].copy_from_slice(&[255, 0, 255, 255]);
                }
            }
        }
    });
}
