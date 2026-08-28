//! Vehicle state for the smart-intersection simulation.

use crate::geometry::{movement_id, Path, Route, SPEED_CRUISE};

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
    pub velocity: f64,
    pub target_velocity: f64,
    pub distance: f64,
    pub detected_tick: Option<u64>,
    pub wait_ticks: u64,
    pub reserved: bool,
    pub phase: VehiclePhase,
}

impl Vehicle {
    pub fn new(id: u64, origin: usize, route: Route, path: &Path) -> Self {
        let (x, y, angle) = path.at(0.0);
        Self {
            id,
            origin,
            route,
            progress: 0.0,
            position: (x, y),
            angle,
            velocity: SPEED_CRUISE,
            target_velocity: SPEED_CRUISE,
            distance: 0.0,
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
