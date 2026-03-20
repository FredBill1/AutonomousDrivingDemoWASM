// Derived from the rsplan Python library:
// https://github.com/builtrobotics/rsplan/tree/47b3ab572a4b7ffc314dfc2aa68b8349f061fe13
// This Rust translation was completed entirely by GPT-5.4 and is provided
// without any guarantee of correctness.
// Copyright (c) 2023 Built Robotics
// SPDX-License-Identifier: MIT

use std::f64::consts::{FRAC_PI_2, PI};

use wasm_bindgen::prelude::*;

use crate::geometry::{distance, rotate};

type Pose = (f64, f64, f64);
type CurveFormula = fn(f64, f64, f64, f64, f64, f64) -> Option<(f64, f64, f64)>;

const NEAR_ZERO_TOL: f64 = 1e-12;
const PI_DIVS: [f64; 4] = [FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2, -FRAC_PI_2];

const PATHS: [&[(SegmentKind, i8)]; 48] = [
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
    ],
    &[
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
        (SegmentKind::Straight, -1),
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
    ],
    &[
        (SegmentKind::Left, -1),
        (SegmentKind::Right, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Left, 1),
        (SegmentKind::Right, -1),
    ],
    &[
        (SegmentKind::Right, -1),
        (SegmentKind::Left, 1),
        (SegmentKind::Straight, 1),
        (SegmentKind::Right, 1),
        (SegmentKind::Left, -1),
    ],
];

const PATH_TYPE_INDICES: [(usize, usize, usize, usize); 12] = [
    (0, 1, 2, 3),
    (4, 5, 6, 7),
    (8, 9, 10, 11),
    (12, 13, 14, 15),
    (16, 17, 18, 19),
    (20, 21, 22, 23),
    (24, 25, 26, 27),
    (28, 29, 30, 31),
    (32, 33, 34, 35),
    (36, 37, 38, 39),
    (40, 41, 42, 43),
    (44, 45, 46, 47),
];

#[wasm_bindgen]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SegmentKind {
    Left,
    Right,
    Straight,
}

#[derive(Clone, Copy, Debug)]
struct Waypoint {
    x: f64,
    y: f64,
    yaw: f64,
    curvature: f64,
    driving_direction: i8,
    is_runway: bool,
}

impl Waypoint {
    fn is_close(&self, other: &Self) -> bool {
        py_round_to(self.x, 5) == py_round_to(other.x, 5)
            && py_round_to(self.y, 5) == py_round_to(other.y, 5)
            && py_round_to(self.yaw, 5) == py_round_to(other.yaw, 5)
    }
}

#[derive(Default)]
struct PathParameter {
    t: f64,
    u: f64,
    v: f64,
    path_ix: usize,
}

#[derive(Default)]
struct PathParameters {
    values: Vec<PathParameter>,
}

impl PathParameters {
    fn push_from(&mut self, other: PathParameters) {
        self.values.extend(other.values);
    }

    fn iter(&self) -> impl Iterator<Item = (usize, f64, f64, f64, usize)> + '_ {
        self.values
            .iter()
            .enumerate()
            .map(|(ix, params)| (ix, params.t, params.u, params.v, params.path_ix))
    }
}

#[wasm_bindgen]
#[derive(Clone, Debug)]
pub struct ReedsSheppSegment {
    kind: SegmentKind,
    direction: i8,
    length: f64,
    turn_radius: f64,
}

#[wasm_bindgen]
impl ReedsSheppSegment {
    #[wasm_bindgen(constructor)]
    pub fn new(kind: SegmentKind, direction: i32, length: f64, turn_radius: f64) -> Self {
        let direction = if direction < 0 { -1 } else { 1 };
        Self {
            kind,
            direction,
            length,
            turn_radius,
        }
    }

    #[wasm_bindgen(getter)]
    pub fn kind(&self) -> SegmentKind {
        self.kind
    }

    #[wasm_bindgen(getter)]
    pub fn direction(&self) -> i32 {
        self.direction as i32
    }

    #[wasm_bindgen(getter)]
    pub fn length(&self) -> f64 {
        self.length
    }

    #[wasm_bindgen(getter)]
    pub fn turn_radius(&self) -> f64 {
        self.turn_radius
    }

    #[wasm_bindgen(getter)]
    pub fn is_straight(&self) -> bool {
        self.kind == SegmentKind::Straight
    }

    pub fn curvature(&self) -> f64 {
        match self.kind {
            SegmentKind::Left => 1.0 / self.turn_radius,
            SegmentKind::Right => -1.0 / self.turn_radius,
            SegmentKind::Straight => 0.0,
        }
    }
}

#[wasm_bindgen]
#[derive(Clone, Debug)]
pub struct ReedsSheppPath {
    start_pose: [f64; 3],
    end_pose: [f64; 3],
    segments: Vec<ReedsSheppSegment>,
    turn_radius: f64,
    step_size: f64,
}

#[wasm_bindgen]
impl ReedsSheppPath {
    #[wasm_bindgen(constructor)]
    pub fn new(
        start_x: f64,
        start_y: f64,
        start_yaw: f64,
        end_x: f64,
        end_y: f64,
        end_yaw: f64,
        turn_radius: f64,
        step_size: f64,
    ) -> Self {
        Self {
            start_pose: [start_x, start_y, start_yaw],
            end_pose: [end_x, end_y, end_yaw],
            segments: Vec::new(),
            turn_radius,
            step_size,
        }
    }

    pub fn push_segment(&mut self, segment: &ReedsSheppSegment) {
        self.segments.push(segment.clone());
    }

    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    pub fn turn_radius(&self) -> f64 {
        self.turn_radius
    }

    pub fn step_size(&self) -> f64 {
        self.step_size
    }

    pub fn total_length(&self) -> f64 {
        self.segments
            .iter()
            .map(|segment| segment.length.abs())
            .sum()
    }

    pub fn runway_length(&self) -> f64 {
        if self.has_runway() {
            self.segments
                .last()
                .map(|segment| segment.length.abs())
                .unwrap_or(0.0)
        } else {
            0.0
        }
    }

    pub fn has_runway(&self) -> bool {
        self.segments
            .last()
            .is_some_and(ReedsSheppSegment::is_straight)
    }

    pub fn number_of_cusp_points(&self) -> usize {
        let waypoints = self.waypoints();
        waypoints
            .windows(2)
            .filter(|pair| pair[0].driving_direction != pair[1].driving_direction)
            .count()
    }

    pub fn waypoint_count(&self) -> usize {
        self.waypoints().len()
    }

    pub fn flat_waypoints(&self) -> Vec<f64> {
        self.waypoints()
            .into_iter()
            .flat_map(|point| {
                [
                    point.x,
                    point.y,
                    point.yaw,
                    point.curvature,
                    point.driving_direction as f64,
                    if point.is_runway { 1.0 } else { 0.0 },
                ]
            })
            .collect()
    }

    pub fn flat_coordinates(&self) -> Vec<f64> {
        self.waypoints()
            .into_iter()
            .flat_map(|point| [point.x, point.y, point.yaw])
            .collect()
    }

    pub fn start_x(&self) -> f64 {
        self.start_pose[0]
    }

    pub fn start_y(&self) -> f64 {
        self.start_pose[1]
    }

    pub fn start_yaw(&self) -> f64 {
        self.start_pose[2]
    }

    pub fn end_x(&self) -> f64 {
        self.end_pose[0]
    }

    pub fn end_y(&self) -> f64 {
        self.end_pose[1]
    }

    pub fn end_yaw(&self) -> f64 {
        self.end_pose[2]
    }
}

impl ReedsSheppPath {
    pub(crate) fn segments(&self) -> &[ReedsSheppSegment] {
        &self.segments
    }

    fn from_segments(
        start_pose: Pose,
        end_pose: Pose,
        segments: Vec<ReedsSheppSegment>,
        turn_radius: f64,
        step_size: f64,
    ) -> Self {
        Self {
            start_pose: [start_pose.0, start_pose.1, start_pose.2],
            end_pose: [end_pose.0, end_pose.1, end_pose.2],
            segments,
            turn_radius,
            step_size,
        }
    }

    fn end_pose_tuple(&self) -> Pose {
        (self.end_pose[0], self.end_pose[1], self.end_pose[2])
    }

    fn waypoints(&self) -> Vec<Waypoint> {
        let mut x0 = self.start_pose[0];
        let mut y0 = self.start_pose[1];
        let mut yaw0 = self.start_pose[2];
        let mut path_points = Vec::<Waypoint>::new();

        for (index, segment) in self.segments.iter().enumerate() {
            let is_runway = self.has_runway() && index == self.segments.len() - 1;
            let mut segment_points = segment.calc_waypoints(
                (x0, y0, yaw0),
                self.step_size,
                is_runway,
                self.end_pose_tuple(),
            );

            if let (Some(last), Some(first)) = (path_points.last(), segment_points.first())
                && last.is_close(first)
            {
                segment_points.remove(0);
            }

            if let Some(last) = segment_points.last() {
                x0 = last.x;
                y0 = last.y;
                yaw0 = last.yaw;
            }

            path_points.extend(segment_points);
        }

        path_points
    }
}

impl ReedsSheppSegment {
    fn calc_waypoints(
        &self,
        start_pose: Pose,
        step_size: f64,
        is_runway: bool,
        end_pose: Pose,
    ) -> Vec<Waypoint> {
        let segment_points = self.interpolated(step_size);
        let (xs, ys, mut yaws) = self.segment_coords(start_pose, &segment_points);

        if is_runway {
            yaws.iter_mut().for_each(|yaw| *yaw = end_pose.2);
        }

        xs.into_iter()
            .zip(ys)
            .zip(yaws)
            .map(|((x, y), yaw)| Waypoint {
                x,
                y,
                yaw,
                curvature: self.curvature(),
                driving_direction: self.direction,
                is_runway,
            })
            .collect()
    }

    fn segment_coords(
        &self,
        start_pose: Pose,
        segment_points: &[f64],
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut xs = Vec::with_capacity(segment_points.len());
        let mut ys = Vec::with_capacity(segment_points.len());
        let mut yaws = Vec::with_capacity(segment_points.len());

        for &point in segment_points {
            let direction = self.direction as f64;
            let (local_x, local_y, local_yaw) = match self.kind {
                SegmentKind::Left => (
                    direction * self.turn_radius * point.sin(),
                    self.turn_radius * (1.0 - point.cos()),
                    direction * point,
                ),
                SegmentKind::Right => (
                    direction * self.turn_radius * point.sin(),
                    -self.turn_radius * (1.0 - point.cos()),
                    -direction * point,
                ),
                SegmentKind::Straight => (direction * point, 0.0, 0.0),
            };

            let (rotated_x, rotated_y) = if start_pose.2 != 0.0 {
                rotate(local_x, local_y, start_pose.2)
            } else {
                (local_x, local_y)
            };

            xs.push(rotated_x + start_pose.0);
            ys.push(rotated_y + start_pose.1);
            yaws.push(local_yaw + start_pose.2);
        }

        (xs, ys, yaws)
    }

    fn interpolated(&self, step_size: f64) -> Vec<f64> {
        let magnitude = if self.is_straight() {
            self.length.abs()
        } else {
            self.length.abs() / self.turn_radius
        };

        let step = if self.is_straight() {
            step_size
        } else {
            step_size / self.turn_radius
        };

        let num_steps = ((magnitude / step) + 2.0).floor().max(2.0) as usize;
        linspace(0.0, magnitude, num_steps)
    }
}

#[wasm_bindgen]
pub fn rs_change_base(
    start_x: f64,
    start_y: f64,
    start_yaw: f64,
    end_x: f64,
    end_y: f64,
    end_yaw: f64,
) -> Vec<f64> {
    let dx = end_x - start_x;
    let dy = end_y - start_y;
    let (xb, yb) = rotate(dx, dy, -start_yaw);
    vec![xb, yb, end_yaw - start_yaw]
}

#[wasm_bindgen]
pub fn rs_steering_angles(phi: f64, turn_radius: f64) -> Vec<f64> {
    vec![
        turn_radius * (phi.cos() - 1.0),
        turn_radius * (phi.cos() + 1.0),
    ]
}

#[wasm_bindgen]
pub fn rs_polar(x: f64, y: f64) -> Vec<f64> {
    vec![x.hypot(y), y.atan2(x)]
}

#[wasm_bindgen]
pub fn rs_wrap_to_pi(angle: f64) -> f64 {
    wrap_to_pi(angle)
}

#[wasm_bindgen]
pub fn rs_euclidean_distance(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    distance(ax, ay, bx, by)
}

#[wasm_bindgen]
pub fn rs_solve_path(
    start_x: f64,
    start_y: f64,
    start_yaw: f64,
    end_x: f64,
    end_y: f64,
    end_yaw: f64,
    turn_radius: f64,
    runway_length: f64,
    step_size: f64,
    length_tolerance: f64,
) -> Result<ReedsSheppPath, JsValue> {
    let start_pose = (start_x, start_y, start_yaw);
    let end_pose = (end_x, end_y, end_yaw);

    solve_best_path(
        start_pose,
        end_pose,
        turn_radius,
        runway_length,
        step_size,
        length_tolerance,
    )
    .ok_or_else(|| JsValue::from_str("No valid Reeds-Shepp path found"))
}

pub(crate) fn solve_best_path(
    start_pose: Pose,
    end_pose: Pose,
    turn_radius: f64,
    runway_length: f64,
    step_size: f64,
    length_tolerance: f64,
) -> Option<ReedsSheppPath> {
    let paths = solve_all_paths(start_pose, end_pose, turn_radius, runway_length, step_size);
    get_optimal_path(paths, length_tolerance)
}

pub(crate) fn solve_all_paths(
    start_pose: Pose,
    end_pose: Pose,
    turn_radius: f64,
    runway_length: f64,
    step_size: f64,
) -> Vec<ReedsSheppPath> {
    if runway_length != 0.0 {
        let runway_direction = if runway_length < 0.0 { -1 } else { 1 };
        let runway_start_pose =
            calc_runway_start_pose(end_pose, runway_direction, runway_length.abs());
        solve_paths(start_pose, runway_start_pose, turn_radius, step_size)
            .into_iter()
            .map(|path| {
                let mut segments = path.segments.clone();
                segments.push(calc_runway_segment(
                    runway_start_pose,
                    end_pose,
                    runway_direction,
                    turn_radius,
                ));
                ReedsSheppPath::from_segments(
                    start_pose,
                    end_pose,
                    segments,
                    turn_radius,
                    step_size,
                )
            })
            .collect()
    } else {
        solve_paths(start_pose, end_pose, turn_radius, step_size)
    }
}

fn solve_paths(
    start_pose: Pose,
    end_pose: Pose,
    turn_radius: f64,
    step_size: f64,
) -> Vec<ReedsSheppPath> {
    let changed = rs_change_base(
        start_pose.0,
        start_pose.1,
        start_pose.2,
        end_pose.0,
        end_pose.1,
        end_pose.2,
    );
    let x = changed[0];
    let y = changed[1];
    let phi = changed[2];

    let mut paths = Vec::new();
    paths.extend(csc(start_pose, end_pose, step_size, x, y, phi, turn_radius));
    paths.extend(ccc(start_pose, end_pose, step_size, x, y, phi, turn_radius));
    paths.extend(cccc(
        start_pose,
        end_pose,
        step_size,
        x,
        y,
        phi,
        turn_radius,
    ));
    paths.extend(ccsc(
        start_pose,
        end_pose,
        step_size,
        x,
        y,
        phi,
        turn_radius,
    ));
    paths.extend(cscc(
        start_pose,
        end_pose,
        step_size,
        x,
        y,
        phi,
        turn_radius,
    ));
    paths.extend(ccscc(
        start_pose,
        end_pose,
        step_size,
        x,
        y,
        phi,
        turn_radius,
    ));
    paths
}

fn get_optimal_path(
    mut paths: Vec<ReedsSheppPath>,
    length_tolerance: f64,
) -> Option<ReedsSheppPath> {
    paths.sort_by(|left, right| left.total_length().total_cmp(&right.total_length()));

    let roughly_equivalent = (paths[1].total_length() - paths[0].total_length()) < length_tolerance;
    let fewer_segments = paths[1].segment_count() < paths[0].segment_count();

    if roughly_equivalent && fewer_segments {
        Some(paths.swap_remove(1))
    } else {
        Some(paths.swap_remove(0))
    }
}

fn calc_runway_start_pose(end_pose: Pose, driving_direction: i8, runway_length: f64) -> Pose {
    let x = end_pose.0 - driving_direction as f64 * runway_length * end_pose.2.cos();
    let y = end_pose.1 - driving_direction as f64 * runway_length * end_pose.2.sin();
    (x, y, end_pose.2)
}

fn calc_runway_segment(
    start_pose: Pose,
    end_pose: Pose,
    direction: i8,
    turn_radius: f64,
) -> ReedsSheppSegment {
    let path_length = py_round_to(
        distance(start_pose.0, start_pose.1, end_pose.0, end_pose.1),
        3,
    );
    ReedsSheppSegment {
        kind: SegmentKind::Straight,
        direction,
        length: path_length,
        turn_radius,
    }
}

fn csc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let csca_params = gen_path_parameters(
        csca,
        PATH_TYPE_INDICES[0],
        x,
        y,
        phi,
        steering[0],
        turn_radius,
    );
    let cscb_params = gen_path_parameters(
        cscb,
        PATH_TYPE_INDICES[1],
        x,
        y,
        phi,
        steering[1],
        turn_radius,
    );

    let mut all = PathParameters::default();
    all.push_from(csca_params);
    all.push_from(cscb_params);

    all.iter()
        .map(|(_, t, u, v, path_ix)| {
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, u, v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

fn ccc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let mut all = PathParameters::default();
    all.push_from(gen_path_parameters(
        c_c_c,
        PATH_TYPE_INDICES[2],
        x,
        y,
        phi,
        steering[0],
        turn_radius,
    ));
    all.push_from(gen_path_parameters(
        c_cc,
        PATH_TYPE_INDICES[3],
        x,
        y,
        phi,
        steering[0],
        turn_radius,
    ));
    all.push_from(gen_path_parameters(
        cc_c,
        PATH_TYPE_INDICES[4],
        x,
        y,
        phi,
        steering[0],
        turn_radius,
    ));

    all.iter()
        .map(|(_, t, u, v, path_ix)| {
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, u, v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

fn cccc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let mut all = PathParameters::default();
    all.push_from(gen_path_parameters(
        ccu_cuc,
        PATH_TYPE_INDICES[5],
        x,
        y,
        phi,
        steering[1],
        turn_radius,
    ));
    all.push_from(gen_path_parameters(
        c_cucu_c,
        PATH_TYPE_INDICES[6],
        x,
        y,
        phi,
        steering[1],
        turn_radius,
    ));

    all.iter()
        .map(|(_, t, u, v, path_ix)| {
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, u, u, v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

fn ccsc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let mut all = PathParameters::default();
    all.push_from(gen_path_parameters(
        c_c2sca,
        PATH_TYPE_INDICES[7],
        x,
        y,
        phi,
        steering[0],
        turn_radius,
    ));
    all.push_from(gen_path_parameters(
        c_c2scb,
        PATH_TYPE_INDICES[8],
        x,
        y,
        phi,
        steering[1],
        turn_radius,
    ));

    all.iter()
        .map(|(ix, t, u, v, path_ix)| {
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, PI_DIVS[ix], u, v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

fn cscc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let mut all = PathParameters::default();
    all.push_from(gen_path_parameters(
        csc2_ca,
        PATH_TYPE_INDICES[9],
        x,
        y,
        phi,
        steering[0],
        turn_radius,
    ));
    all.push_from(gen_path_parameters(
        csc2_cb,
        PATH_TYPE_INDICES[10],
        x,
        y,
        phi,
        steering[1],
        turn_radius,
    ));

    all.iter()
        .map(|(ix, t, u, v, path_ix)| {
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, u, PI_DIVS[ix], v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

fn ccscc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let params = gen_path_parameters(
        c_c2sc2_c,
        PATH_TYPE_INDICES[11],
        x,
        y,
        phi,
        steering[1],
        turn_radius,
    );

    params
        .iter()
        .map(|(ix, t, u, v, path_ix)| {
            let pi_div = PI_DIVS[ix];
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, pi_div, u, pi_div, v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

fn create_path(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    segment_params: &[f64],
    path_segment_types: &[(SegmentKind, i8)],
    turn_radius: f64,
) -> ReedsSheppPath {
    let segments = segment_params
        .iter()
        .zip(path_segment_types.iter())
        .map(|(segment_param, (kind, direction))| {
            create_segment(*segment_param, *kind, *direction, turn_radius)
        })
        .collect();

    ReedsSheppPath::from_segments(start_pose, end_pose, segments, turn_radius, step_size)
}

fn create_segment(
    segment_param: f64,
    kind: SegmentKind,
    direction: i8,
    turn_radius: f64,
) -> ReedsSheppSegment {
    let length = if kind == SegmentKind::Straight {
        segment_param
    } else {
        segment_param * turn_radius
    };

    ReedsSheppSegment {
        kind,
        direction,
        length,
        turn_radius,
    }
}

fn gen_path_parameters(
    curve_func: CurveFormula,
    path_type_indices: (usize, usize, usize, usize),
    x: f64,
    y: f64,
    phi: f64,
    cos_phi_param: f64,
    turn_radius: f64,
) -> PathParameters {
    let pos_sin_phi = turn_radius * phi.sin();
    let neg_sin_phi = -turn_radius * phi.sin();

    let standard = curve_func(x, y, phi, pos_sin_phi, cos_phi_param, turn_radius);
    let reflection = curve_func(x, -y, -phi, neg_sin_phi, cos_phi_param, turn_radius);
    let time_flip = curve_func(-x, y, -phi, neg_sin_phi, cos_phi_param, turn_radius)
        .map(|(t, u, v)| (-t, -u, -v));
    let reflection_time_flip = curve_func(-x, -y, phi, pos_sin_phi, cos_phi_param, turn_radius)
        .map(|(t, u, v)| (-t, -u, -v));

    let candidates = [
        (standard, path_type_indices.0),
        (reflection, path_type_indices.1),
        (time_flip, path_type_indices.2),
        (reflection_time_flip, path_type_indices.3),
    ];

    let mut params = PathParameters::default();
    for (candidate, path_ix) in candidates {
        if let Some((t, u, v)) = candidate {
            params.values.push(PathParameter { t, u, v, path_ix });
        }
    }
    params
}

fn csca(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    _turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let a = x - rsin;
    let b = y + rcos;
    let polar = rs_polar(a, b);
    let u = polar[0];
    let t = polar[1];
    let v = wrap_to_pi(phi - t);
    if t >= 0.0 && u >= 0.0 && v >= 0.0 {
        Some((t, u, v))
    } else {
        None
    }
}

fn cscb(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let a = x + rsin;
    let b = y - rcos;
    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];

    if r >= turn_radius_x2 {
        let u = (r * r - turn_radius_x2 * turn_radius_x2).sqrt();
        let alpha = turn_radius_x2.atan2(u);
        let t = wrap_to_pi(theta + alpha);
        let v = wrap_to_pi(t - phi);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some((t, u, v));
        }
    }

    None
}

fn c_c_c(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x - rsin;
    let b = y + rcos;

    if a.abs() < NEAR_ZERO_TOL && b.abs() < NEAR_ZERO_TOL {
        return None;
    }

    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];
    if r < turn_radius_x4 {
        let alpha = (r / turn_radius_x4).acos();
        let t = wrap_to_pi(FRAC_PI_2 + alpha + theta);
        let u = wrap_to_pi(PI - 2.0 * alpha);
        let v = wrap_to_pi(phi - t - u);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some((t, u, v));
        }
    }

    None
}

fn c_cc(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x - rsin;
    let b = y + rcos;

    if a.abs() < NEAR_ZERO_TOL && b.abs() < NEAR_ZERO_TOL {
        return None;
    }

    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];
    if r <= turn_radius_x4 {
        let alpha = (r / turn_radius_x4).acos();
        let t = wrap_to_pi(FRAC_PI_2 + alpha + theta);
        let u = wrap_to_pi(PI - 2.0 * alpha);
        let v = wrap_to_pi(t + u - phi);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some((t, u, v));
        }
    }

    None
}

fn cc_c(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x - rsin;
    let b = y + rcos;

    if a.abs() < NEAR_ZERO_TOL && b.abs() < NEAR_ZERO_TOL {
        return None;
    }

    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];
    if r <= turn_radius_x4 {
        let u =
            ((8.0 * turn_radius * turn_radius - r * r) / (8.0 * turn_radius * turn_radius)).acos();
        let mut sin_u = u.sin();
        if sin_u.abs() < 0.001 {
            sin_u = 0.0;
        }
        if sin_u.abs() < 0.001 && r.abs() < 0.001 {
            return None;
        }
        let alpha = ((turn_radius_x2 * sin_u) / r).asin();
        let t = wrap_to_pi(FRAC_PI_2 - alpha + theta);
        let v = wrap_to_pi(t - u - phi);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some((t, u, v));
        }
    }

    None
}

fn ccu_cuc(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x + rsin;
    let b = y - rcos;

    if a.abs() < NEAR_ZERO_TOL && b.abs() < NEAR_ZERO_TOL {
        return None;
    }

    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];
    if r <= turn_radius_x4 {
        let (t, u, v) = if r > turn_radius_x2 {
            let alpha = ((r / 2.0 - turn_radius) / turn_radius_x2).acos();
            let t = wrap_to_pi(FRAC_PI_2 + theta - alpha);
            let u = wrap_to_pi(PI - alpha);
            let v = wrap_to_pi(phi - t + 2.0 * u);
            (t, u, v)
        } else {
            let alpha = ((r / 2.0 + turn_radius) / turn_radius_x2).acos();
            let t = wrap_to_pi(FRAC_PI_2 + theta + alpha);
            let u = wrap_to_pi(alpha);
            let v = wrap_to_pi(phi - t + 2.0 * u);
            (t, u, v)
        };

        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some((t, u, v));
        }
    }

    None
}

fn c_cucu_c(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let a = x + rsin;
    let b = y - rcos;

    if a.abs() < NEAR_ZERO_TOL && b.abs() < NEAR_ZERO_TOL {
        return None;
    }

    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];
    if r > 6.0 * turn_radius {
        return None;
    }

    let va = (5.0 * turn_radius * turn_radius - r * r / 4.0) / (turn_radius_x2 * turn_radius_x2);
    if !(0.0..=1.0).contains(&va) {
        return None;
    }

    let u = va.acos();
    let alpha = ((turn_radius_x2 * u.sin()) / r).asin();
    let t = wrap_to_pi(FRAC_PI_2 + theta + alpha);
    let v = wrap_to_pi(t - phi);
    if t >= 0.0 && u >= 0.0 && v >= 0.0 {
        Some((t, u, v))
    } else {
        None
    }
}

fn c_c2sca(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let a = x - rsin;
    let b = y + rcos;
    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];

    if r >= turn_radius_x2 {
        let u = (r * r - turn_radius_x2 * turn_radius_x2).sqrt() - turn_radius_x2;
        if u >= 0.0 {
            let alpha = turn_radius_x2.atan2(u + turn_radius_x2);
            let t = wrap_to_pi(FRAC_PI_2 + theta + alpha);
            let v = wrap_to_pi(t + FRAC_PI_2 - phi);
            if t >= 0.0 && u >= 0.0 && v >= 0.0 {
                return Some((t, u, v));
            }
        }
    }

    None
}

fn c_c2scb(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let a = x + rsin;
    let b = y - rcos;
    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];

    if r >= turn_radius_x2 {
        let t = wrap_to_pi(FRAC_PI_2 + theta);
        let u = r - turn_radius_x2;
        let v = wrap_to_pi(phi - t - FRAC_PI_2);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some((t, u, v));
        }
    }

    None
}

fn csc2_ca(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let a = x - rsin;
    let b = y + rcos;
    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];

    if r >= turn_radius_x2 {
        let u = (r * r - turn_radius_x2 * turn_radius_x2).sqrt() - turn_radius_x2;
        if u >= 0.0 {
            let alpha = (u + turn_radius_x2).atan2(turn_radius_x2);
            let t = wrap_to_pi(FRAC_PI_2 + theta - alpha);
            let v = wrap_to_pi(t - FRAC_PI_2 - phi);
            if t >= 0.0 && u >= 0.0 && v >= 0.0 {
                return Some((t, u, v));
            }
        }
    }

    None
}

fn csc2_cb(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let a = x + rsin;
    let b = y - rcos;
    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];

    if r >= turn_radius_x2 {
        let t = wrap_to_pi(theta);
        let u = r - turn_radius_x2;
        let v = wrap_to_pi(-t - FRAC_PI_2 + phi);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some((t, u, v));
        }
    }

    None
}

fn c_c2sc2_c(
    x: f64,
    y: f64,
    phi: f64,
    rsin: f64,
    rcos: f64,
    turn_radius: f64,
) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x + rsin;
    let b = y - rcos;
    let polar = rs_polar(a, b);
    let r = polar[0];
    let theta = polar[1];

    if r >= turn_radius_x4 {
        let u = (r * r - turn_radius_x2 * turn_radius_x2).sqrt() - turn_radius_x4;
        if u >= 0.0 {
            let alpha = turn_radius_x2.atan2(u + turn_radius_x4);
            let t = wrap_to_pi(FRAC_PI_2 + theta + alpha);
            let v = wrap_to_pi(t - phi);
            if t >= 0.0 && u >= 0.0 && v >= 0.0 {
                return Some((t, u, v));
            }
        }
    }

    None
}

fn wrap_to_pi(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

fn linspace(start: f64, end: f64, count: usize) -> Vec<f64> {
    if count == 0 {
        return Vec::new();
    }
    if count == 1 {
        return vec![end];
    }

    let mut values = Vec::with_capacity(count);
    let step = (end - start) / (count - 1) as f64;
    for index in 0..count {
        values.push(start + step * index as f64);
    }
    if let Some(last) = values.last_mut() {
        *last = end;
    }
    values
}

fn py_round_to(value: f64, digits: i32) -> f64 {
    let factor = 10_f64.powi(digits);
    round_ties_even(value * factor) / factor
}

fn round_ties_even(value: f64) -> f64 {
    if !value.is_finite() {
        return value;
    }

    let floor = value.floor();
    let diff = value - floor;
    if diff < 0.5 {
        floor
    } else if diff > 0.5 {
        floor + 1.0
    } else if (floor / 2.0).fract() == 0.0 {
        floor
    } else {
        floor + 1.0
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::{
        ReedsSheppPath, ReedsSheppSegment, SegmentKind, rs_change_base, rs_euclidean_distance,
        rs_polar, rs_solve_path, rs_steering_angles,
    };

    #[test]
    fn helper_functions_match_python_reference_behavior() {
        let changed = rs_change_base(1.0, 2.0, std::f64::consts::FRAC_PI_2, 3.0, 2.0, 0.25);
        assert_relative_eq!(changed[0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(changed[1], -2.0, epsilon = 1e-12);
        assert_relative_eq!(
            changed[2],
            0.25 - std::f64::consts::FRAC_PI_2,
            epsilon = 1e-12
        );

        let steering = rs_steering_angles(0.4, 3.0);
        assert_relative_eq!(steering[0], 3.0 * (0.4_f64.cos() - 1.0), epsilon = 1e-12);
        assert_relative_eq!(steering[1], 3.0 * (0.4_f64.cos() + 1.0), epsilon = 1e-12);

        let polar = rs_polar(3.0, 4.0);
        assert_relative_eq!(polar[0], 5.0, epsilon = 1e-12);
        assert_relative_eq!(polar[1], 4.0_f64.atan2(3.0), epsilon = 1e-12);
    }

    #[test]
    fn straight_path_waypoints_follow_python_discretization() {
        let mut path = ReedsSheppPath::new(0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.5);
        path.push_segment(&ReedsSheppSegment::new(SegmentKind::Straight, 1, 2.0, 1.0));

        assert_relative_eq!(path.total_length(), 2.0, epsilon = 1e-12);
        assert_eq!(path.segment_count(), 1);
        assert_eq!(path.waypoint_count(), 6);

        let coordinates = path.flat_coordinates();
        assert_relative_eq!(coordinates[0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(coordinates[1], 0.0, epsilon = 1e-12);
        assert_relative_eq!(coordinates[coordinates.len() - 3], 2.0, epsilon = 1e-12);
        assert_relative_eq!(coordinates[coordinates.len() - 2], 0.0, epsilon = 1e-12);
        assert_relative_eq!(coordinates[coordinates.len() - 1], 0.0, epsilon = 1e-12);
    }

    #[test]
    fn curved_and_runway_segments_produce_expected_metrics() {
        let mut path = ReedsSheppPath::new(
            0.0,
            0.0,
            0.0,
            2.0,
            3.5,
            std::f64::consts::FRAC_PI_2,
            2.0,
            0.5,
        );
        path.push_segment(&ReedsSheppSegment::new(
            SegmentKind::Left,
            1,
            std::f64::consts::PI,
            2.0,
        ));
        path.push_segment(&ReedsSheppSegment::new(SegmentKind::Straight, -1, 1.5, 2.0));

        assert_relative_eq!(
            path.total_length(),
            std::f64::consts::PI + 1.5,
            epsilon = 1e-12
        );
        assert_relative_eq!(path.runway_length(), 1.5, epsilon = 1e-12);
        assert_eq!(path.number_of_cusp_points(), 1);

        let coordinates = path.flat_coordinates();
        let last_x = coordinates[coordinates.len() - 3];
        let last_y = coordinates[coordinates.len() - 2];
        let last_yaw = coordinates[coordinates.len() - 1];

        assert_relative_eq!(last_x, 2.0, epsilon = 1e-12);
        assert_relative_eq!(last_y, 0.5, epsilon = 1e-12);
        assert_relative_eq!(last_yaw, std::f64::consts::FRAC_PI_2, epsilon = 1e-12);
    }

    #[test]
    fn solved_straight_path_matches_euclidean_distance() {
        let path = rs_solve_path(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 2.0, 0.0, 0.05, 2.0)
            .expect("path should exist");

        assert_relative_eq!(
            path.total_length(),
            rs_euclidean_distance(0.0, 0.0, 10.0, 0.0),
            epsilon = 1e-9
        );

        let coordinates = path.flat_coordinates();
        assert_relative_eq!(coordinates[coordinates.len() - 3], 10.0, epsilon = 1e-9);
        assert_relative_eq!(coordinates[coordinates.len() - 2], 0.0, epsilon = 1e-9);
        assert_relative_eq!(coordinates[coordinates.len() - 1], 0.0, epsilon = 1e-9);
    }

    #[test]
    fn solved_path_with_runway_reports_runway_length() {
        let path = rs_solve_path(
            0.0,
            0.0,
            0.0,
            -4.0,
            -5.0,
            std::f64::consts::PI / 10.0,
            2.0,
            2.0,
            0.05,
            2.0,
        )
        .expect("path should exist");

        assert!(path.total_length() >= rs_euclidean_distance(0.0, 0.0, -4.0, -5.0));
        assert_relative_eq!(path.runway_length(), 2.0, epsilon = 1e-9);
        assert!(path.segment_count() >= 2);
    }
}
