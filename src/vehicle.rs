//! Vehicle state for the smart-intersection simulation.

use crate::geometry::{movement_id, Path, Route, MAX_ACCELERATION, MAX_BRAKING, SPEED_CRUISE};

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub id: u64,
    pub origin: usize,
    pub route: Route,
    pub progress: f64,
    pub position: (f64, f64),
    pub angle: f64,
    pub time: f64,
    pub distance: f64,
    pub velocity: f64,
    pub max_acceleration: f64,
    pub max_braking: f64,
    pub detected_tick: Option<u64>,
    pub wait_ticks: u64,
    pub reserved: bool,
}

impl Vehicle {
    pub fn new(id: u64, origin: usize, route: Route, path: &Path) -> Self {
        let (x, y, angle) = path.at(0.0);
        let acceleration_factor = dynamics_factor(id, 0xA076_1D64_78BD_642F);
        let braking_factor = dynamics_factor(id, 0xE703_7ED1_A0B4_28DB);

        Self {
            id,
            origin,
            route,
            progress: 0.0,
            position: (x, y),
            angle,
            time: 0.0,
            distance: 0.0,
            velocity: SPEED_CRUISE,
            max_acceleration: MAX_ACCELERATION * acceleration_factor,
            max_braking: MAX_BRAKING * braking_factor,
            detected_tick: None,
            wait_ticks: 0,
            reserved: false,
        }
    }

    pub fn movement_id(&self) -> usize {
        movement_id(self.origin, self.route)
    }

    pub fn update_pose(&mut self, path: &Path) {
        let (x, y, angle) = path.at(self.progress);
        self.position = (x, y);
        self.angle = angle;
    }
}

/// Produces a stable pseudo-random factor in the 0.80..=1.25 range.
/// It deliberately depends only on vehicle ID so stress scenarios remain reproducible.
fn dynamics_factor(id: u64, salt: u64) -> f64 {
    let mut value = id.wrapping_add(salt);
    value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    value ^= value >> 31;

    let unit = (value >> 11) as f64 / ((1_u64 << 53) as f64);
    0.80 + unit * 0.45
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::build_paths;

    #[test]
    fn vehicles_have_individual_acceleration_and_braking_profiles() {
        let paths = build_paths();
        let path = &paths[0][Route::Straight.index()];
        let first = Vehicle::new(1, 0, Route::Straight, path);
        let second = Vehicle::new(2, 0, Route::Straight, path);
        let third = Vehicle::new(3, 0, Route::Straight, path);

        assert_ne!(first.max_acceleration, second.max_acceleration);
        assert_ne!(second.max_acceleration, third.max_acceleration);
        assert_ne!(first.max_braking, second.max_braking);
        assert_ne!(second.max_braking, third.max_braking);

        for vehicle in [first, second, third] {
            assert!((MAX_ACCELERATION * 0.80..=MAX_ACCELERATION * 1.25)
                .contains(&vehicle.max_acceleration));
            assert!((MAX_BRAKING * 0.80..=MAX_BRAKING * 1.25)
                .contains(&vehicle.max_braking));
        }
    }
}
