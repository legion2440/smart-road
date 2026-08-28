//! Vehicle state for the smart-intersection simulation.

use crate::geometry::{
    movement_id, Path, Route, MAX_ACCELERATION, MAX_BRAKING, SPEED_CRUISE,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VehiclePhase {
    Approaching,
    Controlled,
    Crossing,
    Leaving,
}

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
    pub target_velocity: f64,
    pub max_acceleration: f64,
    pub max_braking: f64,
    pub detected_tick: Option<u64>,
    pub wait_ticks: u64,
    pub reserved: bool,
    pub phase: VehiclePhase,
}

impl Vehicle {
    pub fn new(id: u64, origin: usize, route: Route, path: &Path) -> Self {
        let (x, y, angle) = path.at(0.0);
        let (acceleration_factor, braking_factor) = match id % 3 {
            0 => (0.80, 0.82),
            1 => (1.00, 1.00),
            _ => (1.22, 1.25),
        };

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
            target_velocity: SPEED_CRUISE,
            max_acceleration: MAX_ACCELERATION * acceleration_factor,
            max_braking: MAX_BRAKING * braking_factor,
            detected_tick: None,
            wait_ticks: 0,
            reserved: false,
            phase: VehiclePhase::Approaching,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::build_paths;

    #[test]
    fn vehicles_have_different_acceleration_and_braking_profiles() {
        let paths = build_paths();
        let path = &paths[0][Route::Straight.index()];
        let first = Vehicle::new(1, 0, Route::Straight, path);
        let second = Vehicle::new(2, 0, Route::Straight, path);
        let third = Vehicle::new(3, 0, Route::Straight, path);

        assert_ne!(first.max_acceleration, second.max_acceleration);
        assert_ne!(second.max_acceleration, third.max_acceleration);
        assert_ne!(first.max_braking, second.max_braking);
        assert_ne!(second.max_braking, third.max_braking);
    }
}
