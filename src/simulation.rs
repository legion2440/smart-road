//! Deterministic fixed-timestep simulation and vehicle physics.

use crate::controller::IntersectionManager;
use crate::geometry::{
    build_paths, movement_id, Path, Route, AUTO_SPAWN_TICKS, FIXED_DT, FOLLOW_DISTANCE,
    MAX_ACCELERATION, MAX_BRAKING, MOVEMENT_COUNT, SPEED_CRUISE,
};
use crate::stats::Statistics;
use crate::vehicle::{Vehicle, VehiclePhase};
use rand::{seq::SliceRandom, thread_rng};

const EPSILON: f64 = 1.0e-6;

pub struct Sim {
    pub paths: [[Path; 3]; 4],
    pub vehicles: Vec<Vehicle>,
    pub stats: Statistics,
    manager: IntersectionManager,
    tick: u64,
    next_vehicle_id: u64,
}

impl Sim {
    pub fn new() -> Self {
        let paths = build_paths();
        let manager = IntersectionManager::new(&paths);
        Self {
            paths,
            vehicles: Vec::new(),
            stats: Statistics::new(),
            manager,
            tick: 0,
            next_vehicle_id: 1,
        }
    }

    pub fn tick(&self) -> u64 {
        self.tick
    }

    pub fn auto_spawn_due(&self) -> bool {
        self.tick > 0 && self.tick % AUTO_SPAWN_TICKS == 0
    }

    pub fn spawn_from(&mut self, origin: usize) -> bool {
        if origin >= 4 {
            return false;
        }
        let mut routes = Route::ALL;
        routes.shuffle(&mut thread_rng());
        for route in routes {
            if self.spawn_exact(origin, route) {
                return true;
            }
        }
        self.stats.rejected_spawns = self.stats.rejected_spawns.saturating_add(1);
        false
    }

    pub fn spawn_random(&mut self) -> bool {
        let mut movements = Vec::with_capacity(MOVEMENT_COUNT);
        for origin in 0..4 {
            for route in Route::ALL {
                movements.push((origin, route));
            }
        }
        movements.shuffle(&mut thread_rng());
        for (origin, route) in movements {
            if self.spawn_exact(origin, route) {
                return true;
            }
        }
        self.stats.rejected_spawns = self.stats.rejected_spawns.saturating_add(1);
        false
    }

    fn spawn_exact(&mut self, origin: usize, route: Route) -> bool {
        if !self.can_spawn(origin, route) {
            return false;
        }
        let path = &self.paths[origin][route.index()];
        let vehicle = Vehicle::new(self.next_vehicle_id, origin, route, path);
        self.next_vehicle_id = self.next_vehicle_id.saturating_add(1);
        self.vehicles.push(vehicle);
        self.stats.spawned = self.stats.spawned.saturating_add(1);
        self.stats
            .update_peaks(self.vehicles.len(), self.peak_lane_queue());
        true
    }

    fn can_spawn(&self, origin: usize, route: Route) -> bool {
        let path = &self.paths[origin][route.index()];
        let spawn_bounds = path.safety_bounds(0.0);
        let movement = movement_id(origin, route);

        self.vehicles.iter().all(|vehicle| {
            if vehicle.movement_id() == movement && vehicle.progress < FOLLOW_DISTANCE {
                return false;
            }
            let other_path = &self.paths[vehicle.origin][vehicle.route.index()];
            !spawn_bounds.intersects(other_path.safety_bounds(vehicle.progress))
        })
    }

    pub fn step(&mut self) {
        self.tick = self.tick.saturating_add(1);
        self.detect_vehicles();
        self.manager.update(&self.paths, &mut self.vehicles);

        let mut proposed = Vec::with_capacity(self.vehicles.len());
        let mut target_velocities = Vec::with_capacity(self.vehicles.len());

        for vehicle in &self.vehicles {
            let path = &self.paths[vehicle.origin][vehicle.route.index()];
            let target = self.manager.target_speed(path, vehicle).min(SPEED_CRUISE);
            let next_velocity = approach_velocity(vehicle.velocity, target);
            let mut progress = (vehicle.progress + next_velocity * FIXED_DT).min(path.len);

            if vehicle.detected_tick.is_some()
                && !vehicle.reserved
                && vehicle.progress <= path.stop_progress + EPSILON
            {
                progress = progress.min(path.stop_progress);
            }

            proposed.push(progress.max(vehicle.progress));
            target_velocities.push(target);
        }

        self.apply_lane_following(&mut proposed);

        for index in 0..self.vehicles.len() {
            let vehicle = &mut self.vehicles[index];
            let path = &self.paths[vehicle.origin][vehicle.route.index()];
            let previous = vehicle.progress;
            vehicle.progress = proposed[index];
            let travelled = (vehicle.progress - previous).max(0.0);

            // The actual velocity is distance / time for this fixed physics step.
            vehicle.velocity = travelled / FIXED_DT;
            vehicle.target_velocity = target_velocities[index];
            vehicle.distance += travelled;
            if vehicle.detected_tick.is_some() {
                vehicle.time += FIXED_DT;
            }
            vehicle.update_pose(path);
            vehicle.phase = if vehicle.progress + EPSILON >= path.conflict_exit {
                VehiclePhase::Leaving
            } else if vehicle.progress + EPSILON >= path.conflict_entry {
                VehiclePhase::Crossing
            } else if vehicle.detected_tick.is_some() {
                VehiclePhase::Controlled
            } else {
                VehiclePhase::Approaching
            };
        }

        for vehicle in &self.vehicles {
            self.stats.observe_velocity(vehicle.velocity);
        }
        self.stats.observe_proximity(&self.vehicles, &self.paths);
        self.stats
            .update_peaks(self.vehicles.len(), self.peak_lane_queue());
        self.remove_completed();
    }

    fn detect_vehicles(&mut self) {
        for vehicle in &mut self.vehicles {
            if vehicle.detected_tick.is_some() {
                continue;
            }
            let path = &self.paths[vehicle.origin][vehicle.route.index()];
            if vehicle.progress + EPSILON >= path.control_entry {
                vehicle.detected_tick = Some(self.tick);
                vehicle.wait_ticks = 0;
            }
        }
    }

    fn apply_lane_following(&self, proposed: &mut [f64]) {
        for movement in 0..MOVEMENT_COUNT {
            let mut order: Vec<usize> = self
                .vehicles
                .iter()
                .enumerate()
                .filter_map(|(index, vehicle)| {
                    (vehicle.movement_id() == movement).then_some(index)
                })
                .collect();
            order.sort_by(|&first, &second| {
                self.vehicles[second]
                    .progress
                    .total_cmp(&self.vehicles[first].progress)
            });

            for position in 1..order.len() {
                let leader = order[position - 1];
                let follower = order[position];
                let safe_limit = proposed[leader] - FOLLOW_DISTANCE;
                proposed[follower] = proposed[follower]
                    .min(safe_limit)
                    .max(self.vehicles[follower].progress);
            }
        }
    }

    fn peak_lane_queue(&self) -> usize {
        let mut queues = [0usize; MOVEMENT_COUNT];
        for vehicle in &self.vehicles {
            if vehicle.detected_tick.is_some() && !vehicle.reserved {
                queues[vehicle.movement_id()] += 1;
            }
        }
        queues.into_iter().max().unwrap_or(0)
    }

    fn remove_completed(&mut self) {
        let mut index = 0;
        while index < self.vehicles.len() {
            let vehicle = &self.vehicles[index];
            let path = &self.paths[vehicle.origin][vehicle.route.index()];
            if vehicle.progress + EPSILON < path.len {
                index += 1;
                continue;
            }

            let vehicle = self.vehicles.remove(index);
            self.stats.record_completion(vehicle.time, vehicle.distance);
        }
    }

    pub fn statistics_summary(&self) -> String {
        self.stats.summary()
    }
}

impl Default for Sim {
    fn default() -> Self {
        Self::new()
    }
}

fn approach_velocity(current: f64, target: f64) -> f64 {
    if target > current {
        (current + MAX_ACCELERATION * FIXED_DT).min(target)
    } else {
        (current - MAX_BRAKING * FIXED_DT).max(target).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn three_target_velocity_levels_exist() {
        use crate::geometry::{SPEED_SLOW, SPEED_STOP};
        assert!(SPEED_STOP < SPEED_SLOW);
        assert!(SPEED_SLOW < SPEED_CRUISE);
    }

    #[test]
    fn spawn_spacing_rejects_overlapping_vehicle() {
        let mut sim = Sim::new();
        assert!(sim.spawn_exact(0, Route::Straight));
        assert!(!sim.spawn_exact(0, Route::Straight));
    }
}
