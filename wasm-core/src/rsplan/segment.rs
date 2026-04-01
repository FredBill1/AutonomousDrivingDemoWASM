use wasm_bindgen::prelude::*;

use super::math::{linspace, py_round_to};
use super::types::{SegmentKind, Waypoint};

#[wasm_bindgen]
#[derive(Clone, Debug)]
pub struct ReedsSheppSegment {
    pub(crate) kind: SegmentKind,
    pub(crate) direction: i8,
    pub(crate) length: f64,
    pub(crate) turn_radius: f64,
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

impl ReedsSheppSegment {
    pub(super) fn calc_waypoints(
        &self,
        start_pose: (f64, f64, f64),
        step_size: f64,
        is_runway: bool,
        end_pose: (f64, f64, f64),
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

    fn segment_coords(&self, start_pose: (f64, f64, f64), segment_points: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
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
                crate::geometry::rotate(local_x, local_y, start_pose.2)
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

pub(crate) fn round_segment_length(start_pose: (f64, f64, f64), end_pose: (f64, f64, f64)) -> f64 {
    py_round_to(
        crate::geometry::distance(start_pose.0, start_pose.1, end_pose.0, end_pose.1),
        3,
    )
}
