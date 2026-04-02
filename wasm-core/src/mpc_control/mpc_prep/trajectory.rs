use super::types::{PreparedTrajectory, clamp, distance, midpoint, wrap_angle};
use crate::car::CarConfig;
use crate::mpc_control::MpcConfig;
use wasm_bindgen::prelude::*;

pub(crate) fn decode_trajectory(flat: &[f64]) -> Result<Vec<[f64; 4]>, JsValue> {
    if !flat.len().is_multiple_of(4) {
        return Err(JsValue::from_str("Trajectory must be flat [x, y, yaw, direction] data"));
    }

    let mut points = Vec::with_capacity(flat.len() / 4);
    for chunk in flat.chunks_exact(4) {
        points.push([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    Ok(points)
}

pub(crate) fn process_reference_trajectory(
    mut points: Vec<[f64; 4]>,
    mpc_config: &MpcConfig,
    car_config: &CarConfig,
) -> Result<PreparedTrajectory, &'static str> {
    if points.is_empty() {
        return Err("Reference trajectory is empty");
    }
    if points.iter().any(|point| point[3] == 0.0) {
        return Err("the direction on each point of the trajectory should not be zero");
    }

    points = remove_duplicate_xy(points);
    smooth_yaws(&mut points);

    let mut expanded = vec![points[0]];
    for point in points.iter().skip(1).copied() {
        let last = *expanded.last().unwrap_or(&point);
        if last[3] != point[3] {
            if expanded.len() < 2 {
                return Err("Invalid direction change at first segment");
            }
            let previous = expanded[expanded.len() - 2];
            let midpoint_before = midpoint(last, previous);
            let mut stop_point = last;
            stop_point[3] = 0.0;
            let mut midpoint_after = midpoint(point, last);
            midpoint_after[3] = point[3];
            let replace_index = expanded.len() - 1;
            expanded[replace_index] = midpoint_before;
            expanded.push(stop_point);
            expanded.push(midpoint_after);
        }
        expanded.push(point);
    }

    if let Some(last) = expanded.last_mut() {
        last[3] = 0.0;
    }

    for point in &mut expanded {
        point[3] = clamp(
            point[3] * car_config.target_speed(),
            car_config.min_speed(),
            car_config.max_speed(),
        );
    }

    let mut reordered = expanded
        .into_iter()
        .map(|point| [point[0], point[1], point[3], point[2]])
        .collect::<Vec<_>>();

    let us = cumulative_distances(&reordered);
    limit_velocity_for_stops(&mut reordered, &us, mpc_config, car_config);
    limit_velocity_by_curvature(&mut reordered, &us, car_config);
    let direction_change_us = reordered
        .iter()
        .zip(us.iter())
        .filter_map(|(point, &u)| if point[2] == 0.0 { Some(u) } else { None })
        .collect::<Vec<_>>();

    let direction_change_len = direction_change_us.len();

    Ok(PreparedTrajectory {
        points: reordered,
        us,
        direction_change_us: direction_change_us
            .into_iter()
            .take(direction_change_len.saturating_sub(1))
            .collect(),
    })
}

fn remove_duplicate_xy(points: Vec<[f64; 4]>) -> Vec<[f64; 4]> {
    let mut filtered = Vec::with_capacity(points.len());
    for point in points {
        let should_push = filtered
            .last()
            .is_none_or(|last: &[f64; 4]| last[0] != point[0] || last[1] != point[1]);
        if should_push {
            filtered.push(point);
        }
    }
    filtered
}

pub(crate) fn smooth_yaws(points: &mut [[f64; 4]]) {
    if points.is_empty() {
        return;
    }
    let mut accumulated = 0.0;
    let mut previous = 0.0;
    for point in points.iter_mut() {
        let diff = wrap_angle(point[2] - previous);
        accumulated += diff;
        previous = point[2];
        point[2] = accumulated;
    }
}

fn cumulative_distances(points: &[[f64; 4]]) -> Vec<f64> {
    let mut us = Vec::with_capacity(points.len());
    us.push(0.0);
    for index in 1..points.len() {
        let dist = distance(
            points[index - 1][0],
            points[index - 1][1],
            points[index][0],
            points[index][1],
        );
        us.push(us[index - 1] + dist);
    }
    us
}

fn limit_velocity_for_stops(points: &mut [[f64; 4]], us: &[f64], mpc_config: &MpcConfig, car_config: &CarConfig) {
    let mut last_zero = None;
    for index in (0..points.len()).rev() {
        if points[index][2] == 0.0 {
            last_zero = Some(us[index]);
        } else if let Some(stop_u) = last_zero {
            let dist = stop_u - us[index];
            let limit = (2.0 * mpc_config.desired_max_accel_ratio * car_config.max_accel() * dist.max(0.0)).sqrt();
            points[index][2] = clamp(points[index][2], -limit, limit);
        }
    }
}

fn limit_velocity_by_curvature(points: &mut [[f64; 4]], us: &[f64], car_config: &CarConfig) {
    if points.len() < 3 {
        return;
    }

    for index in 0..points.len() {
        let curvature = estimate_curvature(points, us, index);
        if curvature.is_finite() && curvature > 0.0 {
            let max_v = (car_config.max_centripetal_accel() / curvature).sqrt();
            points[index][2] = clamp(points[index][2], -max_v, max_v);
        } else if curvature.is_infinite() {
            points[index][2] = 0.0;
        }
    }
}

fn quadratic_xy_derivatives(
    us: &[f64],
    points: &[[f64; 4]],
    start: usize,
    mid: usize,
    end: usize,
    coord_index: usize,
    target_u: f64,
) -> (f64, f64) {
    let first = quadratic_first_derivative(
        us[start],
        us[mid],
        us[end],
        points[start][coord_index],
        points[mid][coord_index],
        points[end][coord_index],
        target_u,
    );
    let second = quadratic_second_derivative(
        us[start],
        us[mid],
        us[end],
        points[start][coord_index],
        points[mid][coord_index],
        points[end][coord_index],
    );
    (first, second)
}

fn estimate_curvature(points: &[[f64; 4]], us: &[f64], index: usize) -> f64 {
    let (start, mid, end) = if index == 0 {
        (0, 1, 2)
    } else if index + 1 >= points.len() {
        (points.len() - 3, points.len() - 2, points.len() - 1)
    } else {
        (index - 1, index, index + 1)
    };

    let target_u = us[index];
    let (dx, ddx) = quadratic_xy_derivatives(us, points, start, mid, end, 0, target_u);
    let (dy, ddy) = quadratic_xy_derivatives(us, points, start, mid, end, 1, target_u);

    let denom = (dx * dx + dy * dy).powf(1.5);
    if denom == 0.0 {
        f64::INFINITY
    } else {
        (dx * ddy - dy * ddx).abs() / denom
    }
}

fn quadratic_first_derivative(u0: f64, u1: f64, u2: f64, p0: f64, p1: f64, p2: f64, u: f64) -> f64 {
    let d0 = (u0 - u1) * (u0 - u2);
    let d1 = (u1 - u0) * (u1 - u2);
    let d2 = (u2 - u0) * (u2 - u1);
    p0 * (2.0 * u - u1 - u2) / d0 + p1 * (2.0 * u - u0 - u2) / d1 + p2 * (2.0 * u - u0 - u1) / d2
}

fn quadratic_second_derivative(u0: f64, u1: f64, u2: f64, p0: f64, p1: f64, p2: f64) -> f64 {
    let d0 = (u0 - u1) * (u0 - u2);
    let d1 = (u1 - u0) * (u1 - u2);
    let d2 = (u2 - u0) * (u2 - u1);
    2.0 * (p0 / d0 + p1 / d1 + p2 / d2)
}

pub(crate) fn linspace(start: f64, end: f64, count: usize) -> Vec<f64> {
    if count <= 1 {
        return vec![start];
    }
    let step = (end - start) / (count - 1) as f64;
    (0..count).map(|index| start + step * index as f64).collect()
}
