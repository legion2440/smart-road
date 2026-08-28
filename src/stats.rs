//! Simulation statistics required by the subject plus operational metrics.

use crate::geometry::{Path, SAFETY_GAP};
use crate::vehicle::Vehicle;
use std::collections::HashSet;

const MOVING_EPSILON: f64 = 0.5;

#[derive(Debug)]
pub struct Statistics {
    pub spawned: u32,
    pub passed: u32,
    pub rejected_spawns: u32,
    pub close_calls: u32,
    pub collisions: u32,
    pub emergency_clamps: u32,
    pub peak_vehicles: usize,
    pub peak_lane_queue: usize,
    pub peak_approach_vehicles: usize,
    max_velocity: f64,
    min_velocity: f64,
    min_moving_velocity: f64,
    max_time: Option<f64>,
    min_time: Option<f64>,
    total_time: f64,
    total_distance: f64,
    close_pairs: HashSet<(u64, u64)>,
    collision_pairs: HashSet<(u64, u64)>,
}

impl Statistics {
    pub fn new() -> Self {
        Self {
            spawned: 0,
            passed: 0,
            rejected_spawns: 0,
            close_calls: 0,
            collisions: 0,
            emergency_clamps: 0,
            peak_vehicles: 0,
            peak_lane_queue: 0,
            peak_approach_vehicles: 0,
            max_velocity: 0.0,
            min_velocity: f64::INFINITY,
            min_moving_velocity: f64::INFINITY,
            max_time: None,
            min_time: None,
            total_time: 0.0,
            total_distance: 0.0,
            close_pairs: HashSet::new(),
            collision_pairs: HashSet::new(),
        }
    }

    pub fn observe_velocity(&mut self, velocity: f64) {
        self.max_velocity = self.max_velocity.max(velocity);
        self.min_velocity = self.min_velocity.min(velocity);
        if velocity > MOVING_EPSILON {
            self.min_moving_velocity = self.min_moving_velocity.min(velocity);
        }
    }

    pub fn record_completion(&mut self, controlled_time: f64, distance: f64) {
        self.passed = self.passed.saturating_add(1);
        self.max_time = Some(self.max_time.map_or(controlled_time, |value| value.max(controlled_time)));
        self.min_time = Some(self.min_time.map_or(controlled_time, |value| value.min(controlled_time)));
        self.total_time += controlled_time;
        self.total_distance += distance;
    }

    pub fn update_peaks(
        &mut self,
        on_road: usize,
        peak_lane_queue: usize,
        peak_approach_vehicles: usize,
    ) {
        self.peak_vehicles = self.peak_vehicles.max(on_road);
        self.peak_lane_queue = self.peak_lane_queue.max(peak_lane_queue);
        self.peak_approach_vehicles = self
            .peak_approach_vehicles
            .max(peak_approach_vehicles);
    }

    pub fn observe_proximity(&mut self, vehicles: &[Vehicle], paths: &[[Path; 3]; 4]) {
        let mut close_now = HashSet::new();
        let mut collisions_now = HashSet::new();

        for first in 0..vehicles.len() {
            for second in (first + 1)..vehicles.len() {
                let a = &vehicles[first];
                let b = &vehicles[second];
                let pair = if a.id < b.id { (a.id, b.id) } else { (b.id, a.id) };
                let a_path = &paths[a.origin][a.route.index()];
                let b_path = &paths[b.origin][b.route.index()];
                let a_body = a_path.vehicle_bounds(a.progress);
                let b_body = b_path.vehicle_bounds(b.progress);
                let colliding = a_body.intersects(b_body);

                if colliding {
                    collisions_now.insert(pair);
                    if !self.collision_pairs.contains(&pair) {
                        self.collisions = self.collisions.saturating_add(1);
                    }
                    continue;
                }

                let close_gap = (SAFETY_GAP - 0.5).max(0.1);
                if a_body.expanded(close_gap).intersects(b_body.expanded(close_gap)) {
                    close_now.insert(pair);
                    if !self.close_pairs.contains(&pair) {
                        self.close_calls = self.close_calls.saturating_add(1);
                    }
                }
            }
        }

        self.close_pairs = close_now;
        self.collision_pairs = collisions_now;
    }

    pub fn summary(&self) -> String {
        let min_velocity = finite_or_zero(self.min_velocity);
        let min_moving_velocity = finite_or_zero(self.min_moving_velocity);
        let max_time = self.max_time.unwrap_or(0.0);
        let min_time = self.min_time.unwrap_or(0.0);
        let average_time = if self.passed == 0 {
            0.0
        } else {
            self.total_time / self.passed as f64
        };
        let average_distance = if self.passed == 0 {
            0.0
        } else {
            self.total_distance / self.passed as f64
        };

        format!(
            "Max number of vehicles that passed the intersection: {}\n\
             Max velocity: {:.1} px/s\n\
             Min velocity: {:.1} px/s\n\
             Max time that took a vehicle to pass the intersection: {:.2} s\n\
             Min time that took a vehicle to pass the intersection: {:.2} s\n\
             Close calls: {}\n\n\
             Additional statistics\n\
             Min moving velocity: {:.1} px/s\n\
             Spawned: {}\n\
             Rejected spawns: {}\n\
             Peak vehicles on road: {}\n\
             Peak queue in one lane: {}\n\
             Peak vehicles on one approach: {}\n\
             Collisions detected: {}\n\
             Emergency safety clamps: {}\n\
             Average controlled time: {:.2} s\n\
             Average distance travelled: {:.1} px",
            self.passed,
            self.max_velocity,
            min_velocity,
            max_time,
            min_time,
            self.close_calls,
            min_moving_velocity,
            self.spawned,
            self.rejected_spawns,
            self.peak_vehicles,
            self.peak_lane_queue,
            self.peak_approach_vehicles,
            self.collisions,
            self.emergency_clamps,
            average_time,
            average_distance,
        )
    }
}

impl Default for Statistics {
    fn default() -> Self {
        Self::new()
    }
}

fn finite_or_zero(value: f64) -> f64 {
    if value.is_finite() { value } else { 0.0 }
}
