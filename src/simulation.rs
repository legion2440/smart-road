//! Deterministic fixed-timestep simulation and vehicle physics.

use crate::controller::IntersectionManager;
use crate::geometry::{
    build_paths, movement_id, Path, Route, AUTO_SPAWN_TICKS, FIXED_DT, FOLLOW_DISTANCE,
    MAX_BRAKING, MOVEMENT_COUNT, SPEED_CRUISE, SPEED_SLOW,
};
use crate::stats::Statistics;
use crate::vehicle::{Vehicle, MIN_DYNAMICS_FACTOR};
use rand::{seq::SliceRandom, thread_rng, Rng};

const EPSILON: f64 = 1.0e-6;
const STOP_MARGIN: f64 = 8.0;
const CLAMP_TOLERANCE: f64 = 1.0e-3;
// A newly spawned car starts at cruise speed. Keep enough room for the weakest
// braking profile to stop behind a stationary leader without using a hard clamp.
const SPAWN_CLEARANCE: f64 = FOLLOW_DISTANCE
    + STOP_MARGIN
    + SPEED_CRUISE * SPEED_CRUISE / (2.0 * MAX_BRAKING * MIN_DYNAMICS_FACTOR);

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
        let mut rng = thread_rng();
        self.spawn_random_with_rng(&mut rng)
    }

    fn spawn_random_with_rng<R: Rng + ?Sized>(&mut self, rng: &mut R) -> bool {
        let mut movements = Vec::with_capacity(MOVEMENT_COUNT);
        for origin in 0..4 {
            for route in Route::ALL {
                movements.push((origin, route));
            }
        }
        movements.shuffle(rng);
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

        let (peak_queue, peak_approach) = self.lane_load_peaks();
        self.stats
            .update_peaks(self.vehicles.len(), peak_queue, peak_approach);
        true
    }

    fn can_spawn(&self, origin: usize, route: Route) -> bool {
        let path = &self.paths[origin][route.index()];
        let spawn_bounds = path.safety_bounds(0.0);
        let movement = movement_id(origin, route);

        self.vehicles.iter().all(|vehicle| {
            if vehicle.movement_id() == movement && vehicle.progress < SPAWN_CLEARANCE {
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
        let mut emergency_clamps = 0_u32;

        for index in 0..self.vehicles.len() {
            let vehicle = &self.vehicles[index];
            let path = &self.paths[vehicle.origin][vehicle.route.index()];
            let mut target = self.manager.target_speed(path, vehicle);

            if vehicle.detected_tick.is_some()
                && !vehicle.reserved
                && vehicle.progress < path.stop_progress
            {
                let distance_to_stop = path.stop_progress - vehicle.progress;
                target = target.min(braking_speed_limit(distance_to_stop, vehicle.max_braking));
            }

            if let Some(leader_progress) = self.leader_progress(index) {
                let clearance = leader_progress - vehicle.progress - FOLLOW_DISTANCE;
                target = target.min(braking_speed_limit(clearance, vehicle.max_braking));
            }

            let next_velocity = approach_velocity(
                vehicle.velocity,
                target,
                vehicle.max_acceleration,
                vehicle.max_braking,
            );
            let mut progress = (vehicle.progress + next_velocity * FIXED_DT).min(path.len);

            if vehicle.detected_tick.is_some()
                && !vehicle.reserved
                && vehicle.progress <= path.stop_progress + EPSILON
                && progress > path.stop_progress
            {
                if progress > path.stop_progress + CLAMP_TOLERANCE {
                    emergency_clamps = emergency_clamps.saturating_add(1);
                }
                progress = path.stop_progress;
            }

            proposed.push(progress.max(vehicle.progress));
        }

        emergency_clamps = emergency_clamps
            .saturating_add(self.apply_lane_following_guard(&mut proposed));
        self.stats.emergency_clamps = self
            .stats
            .emergency_clamps
            .saturating_add(emergency_clamps);

        for index in 0..self.vehicles.len() {
            let vehicle = &mut self.vehicles[index];
            let path = &self.paths[vehicle.origin][vehicle.route.index()];
            let previous = vehicle.progress;
            vehicle.progress = proposed[index];
            let travelled = (vehicle.progress - previous).max(0.0);

            vehicle.velocity = travelled / FIXED_DT;
            vehicle.distance += travelled;
            if vehicle.detected_tick.is_some() {
                vehicle.time += FIXED_DT;
            }
            vehicle.update_pose(path);
        }

        for vehicle in &self.vehicles {
            self.stats.observe_velocity(vehicle.velocity);
        }
        self.stats.observe_proximity(&self.vehicles, &self.paths);
        let (peak_queue, peak_approach) = self.lane_load_peaks();
        self.stats
            .update_peaks(self.vehicles.len(), peak_queue, peak_approach);
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

    fn leader_progress(&self, follower_index: usize) -> Option<f64> {
        let follower = &self.vehicles[follower_index];
        let movement = follower.movement_id();
        let mut nearest: Option<f64> = None;

        for (index, vehicle) in self.vehicles.iter().enumerate() {
            if index == follower_index
                || vehicle.movement_id() != movement
                || vehicle.progress <= follower.progress + EPSILON
            {
                continue;
            }
            nearest = Some(nearest.map_or(vehicle.progress, |current| current.min(vehicle.progress)));
        }
        nearest
    }

    fn apply_lane_following_guard(&self, proposed: &mut [f64]) -> u32 {
        let mut clamp_count = 0_u32;

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
                if proposed[follower] > safe_limit + CLAMP_TOLERANCE {
                    clamp_count = clamp_count.saturating_add(1);
                }
                proposed[follower] = proposed[follower]
                    .min(safe_limit)
                    .max(self.vehicles[follower].progress);
            }
        }

        clamp_count
    }

    fn lane_load_peaks(&self) -> (usize, usize) {
        let mut queued = [0usize; MOVEMENT_COUNT];
        let mut approaching = [0usize; MOVEMENT_COUNT];

        for vehicle in &self.vehicles {
            let path = &self.paths[vehicle.origin][vehicle.route.index()];
            if vehicle.progress + EPSILON >= path.conflict_entry {
                continue;
            }

            let movement = vehicle.movement_id();
            approaching[movement] += 1;
            if vehicle.velocity <= SPEED_SLOW + 0.5 {
                queued[movement] += 1;
            }
        }

        (
            queued.into_iter().max().unwrap_or(0),
            approaching.into_iter().max().unwrap_or(0),
        )
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

fn braking_speed_limit(distance: f64, max_braking: f64) -> f64 {
    (2.0 * max_braking * (distance - STOP_MARGIN).max(0.0)).sqrt()
}

fn approach_velocity(current: f64, target: f64, max_acceleration: f64, max_braking: f64) -> f64 {
    if target > current {
        (current + max_acceleration * FIXED_DT).min(target)
    } else {
        (current - max_braking * FIXED_DT).max(target).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{FIXED_HZ, SPEED_STOP};
    use rand::{rngs::StdRng, SeedableRng};

    #[test]
    fn three_target_velocity_levels_exist() {
        assert!(SPEED_STOP < SPEED_SLOW);
        assert!(SPEED_SLOW < SPEED_CRUISE);
    }

    #[test]
    fn spawn_spacing_rejects_overlapping_vehicle() {
        let mut sim = Sim::new();
        assert!(sim.spawn_exact(0, Route::Straight));
        assert!(!sim.spawn_exact(0, Route::Straight));
    }

    #[test]
    fn spawn_requires_cruise_speed_braking_clearance() {
        let mut sim = Sim::new();
        assert!(sim.spawn_exact(0, Route::Straight));

        sim.vehicles[0].progress = FOLLOW_DISTANCE + 1.0;
        assert!(!sim.spawn_exact(0, Route::Straight));

        sim.vehicles[0].progress = SPAWN_CLEARANCE + 1.0;
        assert!(sim.spawn_exact(0, Route::Straight));
    }

    #[test]
    fn lone_vehicle_does_not_brake_on_an_empty_intersection() {
        let mut sim = Sim::new();
        assert!(sim.spawn_exact(0, Route::Straight));
        let mut minimum = SPEED_CRUISE;

        while !sim.vehicles.is_empty() {
            sim.step();
            for vehicle in &sim.vehicles {
                minimum = minimum.min(vehicle.velocity);
            }
        }

        assert!(minimum > SPEED_SLOW);
        assert_eq!(sim.stats.emergency_clamps, 0);
    }

    #[test]
    fn seeded_three_minute_soak_stays_safe_and_below_congestion_limit() {
        for seed in [1_u64, 42, 20_260_828, 7] {
            let mut sim = Sim::new();
            let mut rng = StdRng::seed_from_u64(seed);
            let total_ticks = FIXED_HZ as u64 * 180;

            for _ in 0..total_ticks {
                sim.step();
                if sim.auto_spawn_due() {
                    let _ = sim.spawn_random_with_rng(&mut rng);
                }
            }

            assert_eq!(sim.stats.collisions, 0, "seed {seed}");
            assert_eq!(sim.stats.close_calls, 0, "seed {seed}");
            assert_eq!(sim.stats.emergency_clamps, 0, "seed {seed}");
            assert!(
                sim.stats.peak_lane_queue < 8,
                "seed {seed}: peak lane queue reached {}",
                sim.stats.peak_lane_queue
            );
            assert!(
                sim.stats.peak_approach_vehicles < 8,
                "seed {seed}: peak approach load reached {}",
                sim.stats.peak_approach_vehicles
            );
        }
    }
}
