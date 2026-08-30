//! Vehicle state for the smart-intersection simulation.

use crate::geometry::{movement_id, Path, Route, MAX_ACCELERATION, MAX_BRAKING, SPEED_CRUISE};

pub const MIN_DYNAMICS_FACTOR: f64 = 0.80;
pub const MAX_DYNAMICS_FACTOR: f64 = 1.25;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VehicleVisual {
    Sedan,
    Sport,
    RoboTaxi,
    Bus,
    Police,
    Ambulance,
    Fire,
}

impl VehicleVisual {
    pub fn from_id(id: u64) -> Self {
        match id.saturating_sub(1) % 7 {
            0 => Self::Sedan,
            1 => Self::Sport,
            2 => Self::RoboTaxi,
            3 => Self::Bus,
            4 => Self::Police,
            5 => Self::Ambulance,
            _ => Self::Fire,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub id: u64,
    pub origin: usize,
    pub route: Route,
    pub visual: VehicleVisual,
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
            visual: VehicleVisual::from_id(id),
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

/// Produces a stable pseudo-random factor in the configured dynamics range.
/// It deliberately depends only on vehicle ID so stress scenarios remain reproducible.
fn dynamics_factor(id: u64, salt: u64) -> f64 {
    let mut value = id.wrapping_add(salt);
    value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    value ^= value >> 31;

    let unit = (value >> 11) as f64 / ((1_u64 << 53) as f64);
    MIN_DYNAMICS_FACTOR + unit * (MAX_DYNAMICS_FACTOR - MIN_DYNAMICS_FACTOR)
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
            assert!((MAX_ACCELERATION * MIN_DYNAMICS_FACTOR
                ..=MAX_ACCELERATION * MAX_DYNAMICS_FACTOR)
                .contains(&vehicle.max_acceleration));
            assert!((MAX_BRAKING * MIN_DYNAMICS_FACTOR
                ..=MAX_BRAKING * MAX_DYNAMICS_FACTOR)
                .contains(&vehicle.max_braking));
        }
    }

    #[test]
    fn first_seven_vehicles_cover_all_visual_variants() {
        assert_eq!(VehicleVisual::from_id(1), VehicleVisual::Sedan);
        assert_eq!(VehicleVisual::from_id(2), VehicleVisual::Sport);
        assert_eq!(VehicleVisual::from_id(3), VehicleVisual::RoboTaxi);
        assert_eq!(VehicleVisual::from_id(4), VehicleVisual::Bus);
        assert_eq!(VehicleVisual::from_id(5), VehicleVisual::Police);
        assert_eq!(VehicleVisual::from_id(6), VehicleVisual::Ambulance);
        assert_eq!(VehicleVisual::from_id(7), VehicleVisual::Fire);
    }
}
