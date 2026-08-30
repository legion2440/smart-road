//! Smart-road sprite atlas loading and source-rectangle mapping.

use crate::geometry::Route;
use crate::vehicle::VehicleVisual;
use png::{BitDepth, ColorType, Decoder, Transformations};
use sdl2::hint::Hint;
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::video::{Window, WindowContext};
use std::io::Cursor;

const ATLAS_PNG: &[u8] = include_bytes!("../assets/smart-road-atlas.png");
const ATLAS_W: u32 = 512;
const ATLAS_H: u32 = 896;

const SEDAN: (i32, i32, u32, u32) = (0, 0, 88, 176);
const SPORT: (i32, i32, u32, u32) = (96, 0, 92, 184);
const ROBOTAXI: (i32, i32, u32, u32) = (196, 0, 84, 144);
const POLICE: (i32, i32, u32, u32) = (288, 0, 88, 176);
const AMBULANCE: (i32, i32, u32, u32) = (384, 0, 96, 208);
const BUS: (i32, i32, u32, u32) = (0, 216, 96, 280);
const FIRE: (i32, i32, u32, u32) = (104, 216, 104, 272);
const TREE: (i32, i32, u32, u32) = (216, 216, 112, 112);
const ROAD_H: (i32, i32, u32, u32) = (0, 504, 192, 192);
const ROAD_V: (i32, i32, u32, u32) = (200, 504, 192, 192);
const INTERSECTION: (i32, i32, u32, u32) = (0, 704, 192, 192);

pub struct SpriteSet<'a> {
    atlas: Texture<'a>,
}

impl<'a> SpriteSet<'a> {
    pub fn build(
        _canvas: &mut Canvas<Window>,
        creator: &'a TextureCreator<WindowContext>,
    ) -> Result<Self, String> {
        if !sdl2::hint::set_with_priority("SDL_RENDER_SCALE_QUALITY", "1", &Hint::Override) {
            return Err("failed to enable linear texture sampling".to_string());
        }

        let atlas = load_png_texture(creator, ATLAS_PNG, "smart-road-atlas.png")?;
        let query = atlas.query();
        if query.width != ATLAS_W || query.height != ATLAS_H {
            return Err(format!(
                "invalid smart-road atlas dimensions: expected {}x{}, got {}x{}",
                ATLAS_W, ATLAS_H, query.width, query.height
            ));
        }

        Ok(Self { atlas })
    }

    pub fn atlas(&self) -> &Texture<'a> {
        &self.atlas
    }

    pub fn vehicle_source(&self, visual: VehicleVisual) -> Rect {
        rect(match visual {
            VehicleVisual::Sedan => SEDAN,
            VehicleVisual::Sport => SPORT,
            VehicleVisual::RoboTaxi => ROBOTAXI,
            VehicleVisual::Bus => BUS,
            VehicleVisual::Police => POLICE,
            VehicleVisual::Ambulance => AMBULANCE,
            VehicleVisual::Fire => FIRE,
        })
    }

    pub fn road_horizontal_source(&self) -> Rect {
        rect(ROAD_H)
    }

    pub fn road_vertical_source(&self) -> Rect {
        rect(ROAD_V)
    }

    pub fn intersection_source(&self) -> Rect {
        rect(INTERSECTION)
    }

    pub fn tree_source(&self) -> Rect {
        rect(TREE)
    }
}

pub fn route_color(route: Route) -> Color {
    match route {
        Route::Right => Color::RGB(68, 199, 179),
        Route::Straight => Color::RGB(153, 113, 228),
        Route::Left => Color::RGB(240, 163, 58),
    }
}

fn rect((x, y, width, height): (i32, i32, u32, u32)) -> Rect {
    Rect::new(x, y, width, height)
}

fn load_png_texture<'a>(
    creator: &'a TextureCreator<WindowContext>,
    bytes: &[u8],
    name: &str,
) -> Result<Texture<'a>, String> {
    let mut decoder = Decoder::new(Cursor::new(bytes));
    decoder.set_transformations(Transformations::EXPAND | Transformations::STRIP_16);
    let mut reader = decoder
        .read_info()
        .map_err(|error| format!("unable to read embedded {name}: {error}"))?;
    let mut buffer = vec![0; reader.output_buffer_size()];
    let info = reader
        .next_frame(&mut buffer)
        .map_err(|error| format!("unable to decode embedded {name}: {error}"))?;

    if info.bit_depth != BitDepth::Eight {
        return Err(format!(
            "unsupported {name} bit depth after PNG expansion: {:?}",
            info.bit_depth
        ));
    }

    let source = &buffer[..info.buffer_size()];
    let rgba = to_rgba(source, info.color_type, name)?;
    let mut texture = creator
        .create_texture_streaming(PixelFormatEnum::RGBA32, info.width, info.height)
        .map_err(|error| format!("unable to create {name} texture: {error}"))?;
    texture.set_blend_mode(BlendMode::Blend);
    texture
        .update(None, &rgba, info.width as usize * 4)
        .map_err(|error| format!("unable to upload {name} texture: {error}"))?;
    Ok(texture)
}

fn to_rgba(source: &[u8], color_type: ColorType, name: &str) -> Result<Vec<u8>, String> {
    let mut rgba = Vec::with_capacity(source.len().saturating_mul(4));
    match color_type {
        ColorType::Rgba => return Ok(source.to_vec()),
        ColorType::Rgb => {
            for pixel in source.chunks_exact(3) {
                rgba.extend_from_slice(&[pixel[0], pixel[1], pixel[2], 255]);
            }
        }
        ColorType::Grayscale => {
            for &value in source {
                rgba.extend_from_slice(&[value, value, value, 255]);
            }
        }
        ColorType::GrayscaleAlpha => {
            for pixel in source.chunks_exact(2) {
                rgba.extend_from_slice(&[pixel[0], pixel[0], pixel[0], pixel[1]]);
            }
        }
        ColorType::Indexed => {
            return Err(format!(
                "embedded {name} remained indexed after PNG expansion"
            ));
        }
    }
    Ok(rgba)
}
