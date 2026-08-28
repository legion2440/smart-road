//! Small code-generated car sprites. Keeping them generated makes the project
//! self-contained while still rendering and rotating actual textures.

use crate::geometry::Route;
use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::video::{Window, WindowContext};

pub const CAR_TEXTURE_W: u32 = 34;
pub const CAR_TEXTURE_H: u32 = 21;

pub struct SpriteSet<'a> {
    cars: Vec<Texture<'a>>,
}

impl<'a> SpriteSet<'a> {
    pub fn build(
        canvas: &mut Canvas<Window>,
        creator: &'a TextureCreator<WindowContext>,
    ) -> Result<Self, String> {
        let mut cars = Vec::with_capacity(3);
        for route in Route::ALL {
            cars.push(build_car_texture(canvas, creator, route)?);
        }
        Ok(Self { cars })
    }

    pub fn car(&self, route: Route) -> &Texture<'a> {
        &self.cars[route.index()]
    }
}

pub fn route_color(route: Route) -> Color {
    match route {
        Route::Right => Color::RGB(68, 199, 179),
        Route::Straight => Color::RGB(153, 113, 228),
        Route::Left => Color::RGB(240, 163, 58),
    }
}

fn build_car_texture<'a>(
    canvas: &mut Canvas<Window>,
    creator: &'a TextureCreator<WindowContext>,
    route: Route,
) -> Result<Texture<'a>, String> {
    let mut texture = creator
        .create_texture_target(None, CAR_TEXTURE_W, CAR_TEXTURE_H)
        .map_err(|error| format!("unable to create car texture: {error}"))?;
    texture.set_blend_mode(BlendMode::Blend);

    canvas
        .with_texture_canvas(&mut texture, |target| {
            target.set_blend_mode(BlendMode::None);
            target.set_draw_color(Color::RGBA(0, 0, 0, 0));
            target.clear();
            target.set_blend_mode(BlendMode::Blend);

            let body = route_color(route);
            target.set_draw_color(body);
            let _ = target.fill_rect(Rect::new(2, 4, 30, 13));
            let _ = target.fill_rect(Rect::new(7, 2, 17, 17));

            target.set_draw_color(Color::RGB(30, 34, 46));
            let _ = target.fill_rect(Rect::new(9, 4, 6, 4));
            let _ = target.fill_rect(Rect::new(17, 4, 5, 4));
            let _ = target.fill_rect(Rect::new(9, 13, 6, 4));
            let _ = target.fill_rect(Rect::new(17, 13, 5, 4));

            target.set_draw_color(Color::RGB(12, 13, 18));
            for x in [7, 24] {
                let _ = target.fill_rect(Rect::new(x, 1, 5, 2));
                let _ = target.fill_rect(Rect::new(x, 18, 5, 2));
            }

            target.set_draw_color(Color::RGB(255, 242, 194));
            let _ = target.fill_rect(Rect::new(30, 6, 2, 3));
            let _ = target.fill_rect(Rect::new(30, 12, 2, 3));
            target.set_draw_color(Color::RGB(200, 60, 60));
            let _ = target.fill_rect(Rect::new(2, 6, 2, 3));
            let _ = target.fill_rect(Rect::new(2, 12, 2, 3));
        })
        .map_err(|error| format!("unable to draw car texture: {error}"))?;

    Ok(texture)
}
