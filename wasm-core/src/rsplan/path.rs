use wasm_bindgen::prelude::*;

use super::segment::ReedsSheppSegment;
use super::types::{Pose, Waypoint};

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

    pub(super) fn from_segments(
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

    pub(super) fn end_pose_tuple(&self) -> Pose {
        (self.end_pose[0], self.end_pose[1], self.end_pose[2])
    }

    pub(super) fn waypoints(&self) -> Vec<Waypoint> {
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
