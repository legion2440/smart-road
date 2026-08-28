//! Intersection geometry and immutable route paths.

use crate::collision::OrientedBox;

pub const W: u32 = 900;
pub const H: u32 = 900;
pub const CX: f64 = W as f64 / 2.0;
pub const CY: f64 = H as f64 / 2.0;
pub const LANE_W: f64 = 40.0;
pub const ROAD_HALF: f64 = 3.0 * LANE_W;
pub const ROAD_WIDTH: f64 = ROAD_HALF * 2.0;
pub const START: f64 = -50.0;
pub const END: f64 = H as f64 + 50.0;

pub const CAR_LEN: f64 = 30.0;
pub const CAR_W: f64 = 17.0;
pub const SAFETY_GAP: f64 = 14.0;
// Extra curvature allowance keeps rotated OBBs separated on the 40 px left-turn radius.
pub const FOLLOW_DISTANCE: f64 = CAR_LEN + SAFETY_GAP + 22.0;

pub const FIXED_HZ: u32 = 60;
pub const FIXED_DT: f64 = 1.0 / FIXED_HZ as f64;
pub const DETECTION_DISTANCE: f64 = 180.0;
pub const SLOW_ZONE: f64 = 140.0;
pub const AUTO_SPAWN_TICKS: u64 = (FIXED_HZ as u64 * 2) / 3;

pub const SPEED_STOP: f64 = 0.0;
pub const SPEED_SLOW: f64 = 50.0;
pub const SPEED_CRUISE: f64 = 120.0;
pub const MAX_ACCELERATION: f64 = 100.0;
pub const MAX_BRAKING: f64 = 180.0;

pub const MOVEMENT_COUNT: usize = 12;
pub type ConflictMatrix = [[bool; MOVEMENT_COUNT]; MOVEMENT_COUNT];

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
#[repr(usize)]
pub enum Route {
    Right = 0,
    Straight = 1,
    Left = 2,
}

impl Route {
    pub const ALL: [Route; 3] = [Route::Right, Route::Straight, Route::Left];

    pub fn index(self) -> usize {
        self as usize
    }

    /// The sprite sheet stores straight, left and right in this order.
    pub fn sprite_index(self) -> usize {
        match self {
            Route::Straight => 0,
            Route::Left => 1,
            Route::Right => 2,
        }
    }
}

pub fn movement_id(origin: usize, route: Route) -> usize {
    origin * 3 + route.index()
}

#[derive(Clone, Debug)]
pub struct Path {
    pub points: Vec<(f64, f64)>,
    pub cumulative: Vec<f64>,
    pub len: f64,
    pub control_entry: f64,
    pub stop_progress: f64,
    pub conflict_entry: f64,
    pub conflict_exit: f64,
}

impl Path {
    fn new(points: Vec<(f64, f64)>) -> Self {
        let mut cumulative = vec![0.0];
        let mut len = 0.0;
        for index in 1..points.len() {
            len += (points[index].0 - points[index - 1].0)
                .hypot(points[index].1 - points[index - 1].1);
            cumulative.push(len);
        }

        let mut path = Self {
            points,
            cumulative,
            len,
            control_entry: 0.0,
            stop_progress: 0.0,
            conflict_entry: 0.0,
            conflict_exit: len,
        };
        (path.conflict_entry, path.conflict_exit) = path.measure_conflict_span();
        path.stop_progress = (path.conflict_entry - SAFETY_GAP - 2.0).max(0.0);
        path.control_entry = (path.stop_progress - DETECTION_DISTANCE).max(0.0);
        path
    }

    pub fn at(&self, progress: f64) -> (f64, f64, f64) {
        let progress = progress.clamp(0.0, self.len);
        if progress >= self.len {
            let count = self.points.len();
            let a = self.points[count - 2];
            let b = self.points[count - 1];
            return (b.0, b.1, (b.1 - a.1).atan2(b.0 - a.0));
        }

        let mut index = 0;
        while index + 1 < self.cumulative.len() && progress > self.cumulative[index + 1] {
            index += 1;
        }
        let a = self.points[index];
        let b = self.points[index + 1];
        let segment_len = (self.cumulative[index + 1] - self.cumulative[index]).max(1.0e-9);
        let factor = (progress - self.cumulative[index]) / segment_len;
        (
            a.0 + (b.0 - a.0) * factor,
            a.1 + (b.1 - a.1) * factor,
            (b.1 - a.1).atan2(b.0 - a.0),
        )
    }

    pub fn vehicle_bounds(&self, progress: f64) -> OrientedBox {
        let (x, y, angle) = self.at(progress);
        OrientedBox::new((x, y), angle, CAR_LEN, CAR_W)
    }

    pub fn safety_bounds(&self, progress: f64) -> OrientedBox {
        self.vehicle_bounds(progress).expanded(SAFETY_GAP)
    }

    fn measure_conflict_span(&self) -> (f64, f64) {
        let conflict = OrientedBox::axis_aligned(
            CX - ROAD_HALF,
            CY - ROAD_HALF,
            CX + ROAD_HALF,
            CY + ROAD_HALF,
        );
        let inside = |progress: f64| self.vehicle_bounds(progress).intersects(conflict);
        let sample_step = 0.5;
        let sample_count = (self.len / sample_step).ceil() as usize;
        let mut entry = None;
        let mut exit = None;
        let mut previous_progress = 0.0;
        let mut previous_inside = inside(0.0);

        for sample in 1..=sample_count {
            let progress = (sample as f64 * sample_step).min(self.len);
            let current_inside = inside(progress);
            if !previous_inside && current_inside && entry.is_none() {
                entry = Some((previous_progress, progress));
            }
            if previous_inside && !current_inside {
                exit = Some((previous_progress, progress));
            }
            previous_progress = progress;
            previous_inside = current_inside;
        }

        let (entry_low, entry_high) = entry.expect("every route must enter the intersection");
        let (exit_low, exit_high) = exit.expect("every route must leave the intersection");
        (
            refine_transition(entry_low, entry_high, &inside, true),
            refine_transition(exit_low, exit_high, &inside, false),
        )
    }
}

fn refine_transition(
    mut low: f64,
    mut high: f64,
    inside: &impl Fn(f64) -> bool,
    target: bool,
) -> f64 {
    for _ in 0..24 {
        let middle = (low + high) * 0.5;
        if inside(middle) == target {
            high = middle;
        } else {
            low = middle;
        }
    }
    high
}

fn arc(
    center: (f64, f64),
    radius: f64,
    start_deg: f64,
    end_deg: f64,
    steps: usize,
) -> Vec<(f64, f64)> {
    (1..=steps)
        .map(|index| {
            let angle = (start_deg + (end_deg - start_deg) * index as f64 / steps as f64)
                .to_radians();
            (center.0 + radius * angle.cos(), center.1 + radius * angle.sin())
        })
        .collect()
}

fn rotate_once(point: (f64, f64)) -> (f64, f64) {
    (CX - (point.1 - CY), CY + (point.0 - CX))
}

fn rotate_path(points: &[(f64, f64)], turns: usize) -> Vec<(f64, f64)> {
    let mut result = points.to_vec();
    for _ in 0..turns {
        result = result.into_iter().map(rotate_once).collect();
    }
    result
}

/// Builds the twelve immutable movement paths. Origins rotate clockwise as
/// north, east, south and west. Each origin owns three dedicated entry lanes:
/// right, straight and left.
pub fn build_paths() -> [[Path; 3]; 4] {
    let right_x = CX - 2.5 * LANE_W;
    let straight_x = CX - 1.5 * LANE_W;
    let left_x = CX - 0.5 * LANE_W;

    let right_radius = 0.5 * LANE_W;
    let right_center = (CX - 3.0 * LANE_W, CY - 3.0 * LANE_W);
    let mut right = vec![(right_x, START), (right_x, CY - 3.0 * LANE_W)];
    right.extend(arc(right_center, right_radius, 0.0, 90.0, 20));
    right.push((START, CY - 2.5 * LANE_W));

    let straight = vec![(straight_x, START), (straight_x, END)];

    let left_radius = LANE_W;
    let left_center = (CX + 0.5 * LANE_W, CY - 0.5 * LANE_W);
    let mut left = vec![(left_x, START), (left_x, CY - 0.5 * LANE_W)];
    left.extend(arc(left_center, left_radius, 180.0, 90.0, 28));
    left.push((END, CY + 0.5 * LANE_W));

    let base = [right, straight, left];
    std::array::from_fn(|origin| {
        std::array::from_fn(|route| Path::new(rotate_path(&base[route], origin)))
    })
}

pub fn build_conflict_matrix(paths: &[[Path; 3]; 4]) -> ConflictMatrix {
    std::array::from_fn(|first| {
        std::array::from_fn(|second| {
            if first == second {
                return false;
            }
            let first_path = &paths[first / 3][first % 3];
            let second_path = &paths[second / 3][second % 3];
            paths_conflict(first_path, second_path)
        })
    })
}

fn paths_conflict(first: &Path, second: &Path) -> bool {
    let step = 2.0;
    let first_start = (first.conflict_entry - CAR_LEN).max(0.0);
    let first_end = (first.conflict_exit + CAR_LEN).min(first.len);
    let second_start = (second.conflict_entry - CAR_LEN).max(0.0);
    let second_end = (second.conflict_exit + CAR_LEN).min(second.len);

    let mut first_progress = first_start;
    loop {
        let first_bounds = first
            .vehicle_bounds(first_progress)
            .expanded(SAFETY_GAP + 2.0);
        let mut second_progress = second_start;
        loop {
            if first_bounds.intersects(
                second
                    .vehicle_bounds(second_progress)
                    .expanded(SAFETY_GAP + 2.0),
            ) {
                return true;
            }
            if second_progress >= second_end {
                break;
            }
            second_progress = (second_progress + step).min(second_end);
        }
        if first_progress >= first_end {
            break;
        }
        first_progress = (first_progress + step).min(first_end);
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_twelve_paths_enter_and_leave_the_intersection() {
        let paths = build_paths();
        for path in paths.iter().flatten() {
            assert!(path.control_entry < path.stop_progress);
            assert!(path.stop_progress < path.conflict_entry);
            assert!(path.conflict_entry < path.conflict_exit);
            assert!(path.conflict_exit < path.len);
        }
    }

    #[test]
    fn dedicated_lanes_start_at_three_distinct_positions() {
        let paths = build_paths();
        for origin_paths in &paths {
            let starts: Vec<_> = origin_paths
                .iter()
                .map(|path| {
                    let (x, y, _) = path.at(0.0);
                    (x, y)
                })
                .collect();
            for first in 0..3 {
                for second in (first + 1)..3 {
                    assert!(
                        (starts[first].0 - starts[second].0).hypot(starts[first].1 - starts[second].1)
                            > LANE_W / 2.0
                    );
                }
            }
        }
    }

    #[test]
    fn conflict_matrix_is_symmetric_and_does_not_self_conflict() {
        let paths = build_paths();
        let conflicts = build_conflict_matrix(&paths);
        for first in 0..MOVEMENT_COUNT {
            assert!(!conflicts[first][first]);
            for second in 0..MOVEMENT_COUNT {
                assert_eq!(conflicts[first][second], conflicts[second][first]);
            }
        }
    }
}
