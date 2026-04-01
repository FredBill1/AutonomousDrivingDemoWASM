use wasm_bindgen::prelude::*;

use super::math::{rs_change_base, rs_steering_angles};
use super::path::ReedsSheppPath;
use super::path_generators::{ccc, cccc, ccsc, ccscc, csc, cscc};
use super::segment::{round_segment_length, ReedsSheppSegment};
use super::types::{Pose, SegmentKind};

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
                let mut segments = path.segments().to_vec();
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
    paths.extend(cccc(start_pose, end_pose, step_size, x, y, phi, turn_radius));
    paths.extend(ccsc(start_pose, end_pose, step_size, x, y, phi, turn_radius));
    paths.extend(cscc(start_pose, end_pose, step_size, x, y, phi, turn_radius));
    paths.extend(ccscc(start_pose, end_pose, step_size, x, y, phi, turn_radius));
    paths
}

fn get_optimal_path(
    mut paths: Vec<ReedsSheppPath>,
    length_tolerance: f64,
) -> Option<ReedsSheppPath> {
    paths.sort_by(|left, right| left.total_length().total_cmp(&right.total_length()));

    let roughly_equivalent =
        (paths[1].total_length() - paths[0].total_length()) < length_tolerance;
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
    let path_length = round_segment_length(start_pose, end_pose);
    ReedsSheppSegment {
        kind: SegmentKind::Straight,
        direction,
        length: path_length,
        turn_radius,
    }
}
