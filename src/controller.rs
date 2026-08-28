//! Central reservation manager for autonomous vehicles.
//!
//! The manager is intentionally not a traffic light. It grants independent
//! movement reservations to several non-conflicting lanes at the same time and
//! lets blocked vehicles approach at a reduced speed until their route is safe.

use crate::geometry::{build_conflict_matrix, ConflictMatrix, Path, MOVEMENT_COUNT, SLOW_ZONE};
use crate::vehicle::Vehicle;

const RESERVATION_LOOKAHEAD: f64 = 160.0;

pub struct IntersectionManager {
    conflicts: ConflictMatrix,
}

impl IntersectionManager {
    pub fn new(paths: &[[Path; 3]; 4]) -> Self {
        Self {
            conflicts: build_conflict_matrix(paths),
        }
    }

    pub fn update(&self, paths: &[[Path; 3]; 4], vehicles: &mut [Vehicle]) {
        for vehicle in vehicles.iter_mut() {
            let path = &paths[vehicle.origin][vehicle.route.index()];
            if vehicle.reserved && vehicle.progress >= path.conflict_exit {
                vehicle.reserved = false;
            }
        }

        let mut active_movements = Vec::new();
        for vehicle in vehicles.iter() {
            if vehicle.reserved {
                active_movements.push(vehicle.movement_id());
            }
        }

        let mut queue_pressure = [0usize; MOVEMENT_COUNT];
        let mut front_candidate: [Option<usize>; MOVEMENT_COUNT] = [None; MOVEMENT_COUNT];

        for (index, vehicle) in vehicles.iter().enumerate() {
            if vehicle.detected_tick.is_none() || vehicle.reserved {
                continue;
            }
            let movement = vehicle.movement_id();
            queue_pressure[movement] += 1;
            let path = &paths[vehicle.origin][vehicle.route.index()];
            let request_progress =
                (path.stop_progress - RESERVATION_LOOKAHEAD).max(path.control_entry);
            if vehicle.progress < request_progress {
                continue;
            }

            match front_candidate[movement] {
                Some(current) if vehicles[current].progress >= vehicle.progress => {}
                _ => front_candidate[movement] = Some(index),
            }
        }

        let mut candidates: Vec<usize> = front_candidate.into_iter().flatten().collect();
        candidates.sort_by(|&first, &second| {
            let first_vehicle = &vehicles[first];
            let second_vehicle = &vehicles[second];
            let first_score = first_vehicle
                .wait_ticks
                .saturating_add((queue_pressure[first_vehicle.movement_id()] as u64) * 60);
            let second_score = second_vehicle
                .wait_ticks
                .saturating_add((queue_pressure[second_vehicle.movement_id()] as u64) * 60);

            second_score
                .cmp(&first_score)
                .then_with(|| second_vehicle.progress.total_cmp(&first_vehicle.progress))
                .then_with(|| first_vehicle.id.cmp(&second_vehicle.id))
        });

        for index in candidates {
            let movement = vehicles[index].movement_id();
            let route_is_available = active_movements.iter().all(|&active| {
                // Vehicles following the exact same immutable path are protected
                // by the longitudinal following-distance layer. Treating them as
                // crossing conflicts would serialize an otherwise safe convoy.
                active == movement || !self.conflicts[movement][active]
            });
            if route_is_available {
                vehicles[index].reserved = true;
                vehicles[index].wait_ticks = 0;
                active_movements.push(movement);
            }
        }

        for vehicle in vehicles.iter_mut() {
            if vehicle.detected_tick.is_some() && !vehicle.reserved {
                vehicle.wait_ticks = vehicle.wait_ticks.saturating_add(1);
            }
        }
    }

    pub fn target_speed(&self, path: &Path, vehicle: &Vehicle) -> f64 {
        use crate::geometry::{SPEED_CRUISE, SPEED_SLOW, SPEED_STOP};

        if vehicle.detected_tick.is_none()
            || vehicle.reserved
            || vehicle.progress >= path.conflict_entry
        {
            return SPEED_CRUISE;
        }

        let distance_to_stop = (path.stop_progress - vehicle.progress).max(0.0);
        if distance_to_stop > SLOW_ZONE {
            SPEED_CRUISE
        } else if distance_to_stop > 2.0 {
            SPEED_SLOW
        } else {
            SPEED_STOP
        }
    }
}
