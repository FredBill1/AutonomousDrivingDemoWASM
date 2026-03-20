use wasm_bindgen::prelude::*;

const TARGET_SPEED: f64 = 40.0 / 3.6;
const MAX_SPEED: f64 = 55.0 / 3.6;
const MIN_SPEED: f64 = -30.0 / 3.6;
const MAX_ACCEL: f64 = 15.0;
const MAX_CENTRIPETAL_ACCEL: f64 = 16.0;
const DESIRED_MAX_ACCEL_RATIO: f64 = 0.7;
const HORIZON_LENGTH: usize = 5;
const MIN_HORIZON_DISTANCE: f64 = 0.3;
const MOTION_RESOLUTION: f64 = 0.5;
const DIRECTION_CHANGE_DIST: f64 = 0.1;

#[wasm_bindgen]
pub struct MpcReferenceResult {
    model_reference_states: Vec<f64>,
    reference_states: Vec<f64>,
    brake_trajectory: Vec<f64>,
}

#[wasm_bindgen]
impl MpcReferenceResult {
    #[wasm_bindgen(getter)]
    pub fn model_reference_states(&self) -> Vec<f64> {
        self.model_reference_states.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn reference_states(&self) -> Vec<f64> {
        self.reference_states.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn brake_trajectory(&self) -> Vec<f64> {
        self.brake_trajectory.clone()
    }
}

#[wasm_bindgen]
pub struct MpcReferenceTracker {
    prepared: PreparedTrajectory,
    cur_u: f64,
    u_limit: f64,
    brake: bool,
    braked: bool,
    brake_trajectory: Vec<[f64; 4]>,
}

#[wasm_bindgen]
impl MpcReferenceTracker {
    #[wasm_bindgen(constructor)]
    pub fn new(flat_trajectory: Vec<f64>) -> Result<MpcReferenceTracker, JsValue> {
        let trajectory = decode_trajectory(&flat_trajectory)?;
        let prepared = process_reference_trajectory(trajectory).map_err(JsValue::from_str)?;
        let u_limit = *prepared.us.last().unwrap_or(&0.0);
        Ok(MpcReferenceTracker {
            prepared,
            cur_u: 0.0,
            u_limit,
            brake: false,
            braked: false,
            brake_trajectory: Vec::new(),
        })
    }

    #[wasm_bindgen(getter)]
    pub fn current_progress(&self) -> f64 {
        self.cur_u
    }

    #[wasm_bindgen(getter)]
    pub fn progress_limit(&self) -> f64 {
        self.u_limit
    }

    pub fn brake(&mut self) {
        self.brake = true;
    }

    pub fn update(
        &mut self,
        state_x: f64,
        state_y: f64,
        _state_yaw: f64,
        state_velocity: f64,
        dt: f64,
    ) -> MpcReferenceResult {
        let model_reference_states = self.find_xref(state_x, state_y, state_velocity, dt);
        let reference_states = reorder_public_states(&model_reference_states);
        let brake_trajectory = reorder_public_states(&self.brake_trajectory);
        MpcReferenceResult {
            model_reference_states: model_reference_states
                .iter()
                .flat_map(|state| *state)
                .collect(),
            reference_states,
            brake_trajectory,
        }
    }
}

#[wasm_bindgen]
pub fn mpc_prepare_reference(
    flat_trajectory: Vec<f64>,
    state_x: f64,
    state_y: f64,
    state_yaw: f64,
    state_velocity: f64,
    dt: f64,
    brake: bool,
) -> Result<MpcReferenceResult, JsValue> {
    let mut tracker = MpcReferenceTracker::new(flat_trajectory)?;
    if brake {
        tracker.brake();
    }
    Ok(tracker.update(state_x, state_y, state_yaw, state_velocity, dt))
}

#[derive(Clone)]
struct PreparedTrajectory {
    points: Vec<[f64; 4]>,
    us: Vec<f64>,
    direction_change_us: Vec<f64>,
}

fn decode_trajectory(flat: &[f64]) -> Result<Vec<[f64; 4]>, JsValue> {
    if !flat.len().is_multiple_of(4) {
        return Err(JsValue::from_str(
            "Trajectory must be flat [x, y, yaw, direction] data",
        ));
    }

    let mut points = Vec::with_capacity(flat.len() / 4);
    for chunk in flat.chunks_exact(4) {
        points.push([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    Ok(points)
}

fn process_reference_trajectory(
    mut points: Vec<[f64; 4]>,
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
        point[3] = clamp(point[3] * TARGET_SPEED, MIN_SPEED, MAX_SPEED);
    }

    let mut reordered = expanded
        .into_iter()
        .map(|point| [point[0], point[1], point[3], point[2]])
        .collect::<Vec<_>>();

    let us = cumulative_distances(&reordered);
    limit_velocity_for_stops(&mut reordered, &us);
    limit_velocity_by_curvature(&mut reordered, &us);
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

fn smooth_yaws(points: &mut [[f64; 4]]) {
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

fn limit_velocity_for_stops(points: &mut [[f64; 4]], us: &[f64]) {
    let mut last_zero = None;
    for index in (0..points.len()).rev() {
        if points[index][2] == 0.0 {
            last_zero = Some(us[index]);
        } else if let Some(stop_u) = last_zero {
            let dist = stop_u - us[index];
            let limit = (2.0 * DESIRED_MAX_ACCEL_RATIO * MAX_ACCEL * dist.max(0.0)).sqrt();
            points[index][2] = clamp(points[index][2], -limit, limit);
        }
    }
}

fn limit_velocity_by_curvature(points: &mut [[f64; 4]], us: &[f64]) {
    if points.len() < 3 {
        return;
    }

    for index in 0..points.len() {
        let curvature = estimate_curvature(points, us, index);
        if curvature.is_finite() && curvature > 0.0 {
            let max_v = (MAX_CENTRIPETAL_ACCEL / curvature).sqrt();
            points[index][2] = clamp(points[index][2], -max_v, max_v);
        } else if curvature.is_infinite() {
            points[index][2] = 0.0;
        }
    }
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
    let dx = quadratic_first_derivative(
        us[start],
        us[mid],
        us[end],
        points[start][0],
        points[mid][0],
        points[end][0],
        target_u,
    );
    let dy = quadratic_first_derivative(
        us[start],
        us[mid],
        us[end],
        points[start][1],
        points[mid][1],
        points[end][1],
        target_u,
    );
    let ddx = quadratic_second_derivative(
        us[start],
        us[mid],
        us[end],
        points[start][0],
        points[mid][0],
        points[end][0],
    );
    let ddy = quadratic_second_derivative(
        us[start],
        us[mid],
        us[end],
        points[start][1],
        points[mid][1],
        points[end][1],
    );

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

impl MpcReferenceTracker {
    fn find_nearest_point(&mut self, state_x: f64, state_y: f64) {
        let mut min_dist = f64::INFINITY;
        let mut min_u = self.cur_u;
        let search_limit = self.u_limit.min(self.cur_u + 20.0);
        let mut u = self.cur_u;
        while u <= search_limit + 0.05 {
            let point = sample_state(&self.prepared, u);
            let dist = distance(point[0], point[1], state_x, state_y);
            if dist < min_dist {
                min_dist = dist;
                min_u = u;
            } else {
                break;
            }
            u += 0.1;
        }
        self.cur_u = min_u;
    }

    fn next_direction_change(&self) -> f64 {
        self.prepared
            .direction_change_us
            .iter()
            .copied()
            .find(|&value| value > self.cur_u)
            .unwrap_or(f64::INFINITY)
    }

    fn update_brake_trajectory(&mut self, state_velocity: f64, changing_point: f64) {
        let brake_length = state_velocity.powi(2) / (2.0 * MAX_ACCEL * DESIRED_MAX_ACCEL_RATIO);
        let brake_limit = self
            .u_limit
            .min(self.cur_u + brake_length)
            .min(changing_point);
        let mut brake_us = Vec::new();
        let mut u = self.cur_u;
        while u <= brake_limit + MOTION_RESOLUTION / 2.0 {
            brake_us.push(u.min(brake_limit));
            u += MOTION_RESOLUTION;
        }
        self.brake_trajectory = brake_us
            .into_iter()
            .map(|brake_u| sample_state(&self.prepared, brake_u))
            .collect();
        let velocity_sign = self
            .brake_trajectory
            .iter()
            .map(|state| state[2])
            .sum::<f64>()
            .signum();
        for state in &mut self.brake_trajectory {
            state[2] = velocity_sign;
        }

        if self.brake {
            self.braked = true;
            self.u_limit = brake_limit;
        }
    }

    fn find_xref(
        &mut self,
        state_x: f64,
        state_y: f64,
        state_velocity: f64,
        dt: f64,
    ) -> Vec<[f64; 4]> {
        loop {
            self.find_nearest_point(state_x, state_y);

            let signed_velocity =
                sample_state(&self.prepared, self.cur_u)[2].signum() * state_velocity;
            let length =
                MIN_HORIZON_DISTANCE.max(signed_velocity.max(0.0) * dt * HORIZON_LENGTH as f64);
            let mut ref_us = linspace(self.cur_u, self.cur_u + length, HORIZON_LENGTH + 1);
            for value in &mut ref_us {
                *value = value.min(self.u_limit);
            }

            let changing_point = self.next_direction_change();
            if ref_us.last().copied().unwrap_or(self.cur_u) >= changing_point {
                if self.cur_u + DIRECTION_CHANGE_DIST >= changing_point {
                    self.cur_u = changing_point;
                    continue;
                }

                let cutoff = ref_us.partition_point(|&value| value <= changing_point);
                ref_us.truncate(cutoff);
                if ref_us.len() < HORIZON_LENGTH + 1 {
                    ref_us.push(changing_point);
                    while ref_us.len() < HORIZON_LENGTH + 1 {
                        ref_us.push(changing_point);
                    }
                }
            }

            let mut xref = ref_us
                .iter()
                .copied()
                .map(|u| sample_state(&self.prepared, u))
                .collect::<Vec<_>>();

            if !self.braked {
                self.update_brake_trajectory(state_velocity, changing_point);
            }

            if self.brake {
                for state in &mut xref {
                    state[2] = 0.0;
                }
                if let Some(last) = xref.last_mut() {
                    last[2] = state_velocity * -0.5;
                }
            } else if ref_us.last().copied() == Some(self.u_limit) {
                for (index, &u) in ref_us.iter().enumerate() {
                    if u == self.u_limit {
                        xref[index][2] = 0.0;
                    }
                }
                if let Some(last) = xref.last_mut() {
                    last[2] = state_velocity * -0.5;
                }
            }

            return xref;
        }
    }
}

fn sample_state(trajectory: &PreparedTrajectory, u: f64) -> [f64; 4] {
    if u <= 0.0 {
        return trajectory.points[0];
    }
    let last_u = *trajectory.us.last().unwrap_or(&0.0);
    if u >= last_u {
        return *trajectory.points.last().unwrap_or(&trajectory.points[0]);
    }

    let index = trajectory.us.partition_point(|&value| value < u);
    let upper = index.min(trajectory.points.len() - 1);
    let lower = upper.saturating_sub(1);
    let u0 = trajectory.us[lower];
    let u1 = trajectory.us[upper];
    let t = if (u1 - u0).abs() < 1e-9 {
        0.0
    } else {
        (u - u0) / (u1 - u0)
    };

    let p0 = trajectory.points[lower];
    let p1 = trajectory.points[upper];
    [
        lerp(p0[0], p1[0], t),
        lerp(p0[1], p1[1], t),
        lerp(p0[2], p1[2], t),
        lerp(p0[3], p1[3], t),
    ]
}

fn reorder_public_states(states: &[[f64; 4]]) -> Vec<f64> {
    states
        .iter()
        .flat_map(|state| [state[0], state[1], state[3], state[2]])
        .collect()
}

fn midpoint(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        (a[0] + b[0]) / 2.0,
        (a[1] + b[1]) / 2.0,
        (a[2] + b[2]) / 2.0,
        (a[3] + b[3]) / 2.0,
    ]
}

fn distance(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    (ax - bx).hypot(ay - by)
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn wrap_angle(mut angle: f64) -> f64 {
    while angle >= std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}

fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

fn linspace(start: f64, end: f64, count: usize) -> Vec<f64> {
    if count <= 1 {
        return vec![start];
    }
    let step = (end - start) / (count - 1) as f64;
    (0..count)
        .map(|index| start + step * index as f64)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::{
        MpcReferenceTracker, mpc_prepare_reference, process_reference_trajectory, smooth_yaws,
    };

    #[test]
    fn prepares_reference_and_brake_preview() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 1.0, 15.0, 0.0, 0.0, 1.0,
        ];

        let result =
            mpc_prepare_reference(trajectory, 1.0, 0.0, 0.0, 6.0, 0.07, false).expect("mpc prep");
        assert_eq!(result.model_reference_states().len(), 24);
        assert_eq!(result.reference_states().len(), 24);
        assert!(!result.brake_trajectory().is_empty());
    }

    #[test]
    fn handles_direction_change_stop_points() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 4.0, 0.0, 0.0, 1.0, 4.0, -2.0, -1.57, -1.0, 4.0, -5.0, -1.57, -1.0,
        ];

        let tracker = MpcReferenceTracker::new(trajectory).expect("tracker");
        assert!(!tracker.prepared.direction_change_us.is_empty());
    }

    #[test]
    fn smooth_yaws_preserves_first_heading_like_python() {
        let mut points = vec![
            [0.0, 0.0, 1.2, 1.0],
            [1.0, 0.0, 1.25, 1.0],
            [2.0, 0.0, 1.3, 1.0],
        ];

        smooth_yaws(&mut points);

        assert!((points[0][2] - 1.2).abs() < 1e-9);
        assert!((points[1][2] - 1.25).abs() < 1e-9);
        assert!((points[2][2] - 1.3).abs() < 1e-9);
    }

    #[test]
    fn tracker_searches_forward_from_current_progress() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 1.0, 15.0, 0.0, 0.0, 1.0,
        ];

        let mut tracker = MpcReferenceTracker::new(trajectory).expect("tracker");
        let _ = tracker.update(8.0, 0.0, 0.0, 4.0, 0.07);
        let progressed_u = tracker.current_progress();
        let _ = tracker.update(1.0, 0.0, 0.0, 4.0, 0.07);

        assert!(tracker.current_progress() >= progressed_u - 1e-9);
        assert!(tracker.current_progress() > 5.0);
    }

    #[test]
    fn tracker_freezes_brake_limit_after_brake() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 1.0, 15.0, 0.0, 0.0, 1.0, 20.0,
            0.0, 0.0, 1.0,
        ];

        let mut tracker = MpcReferenceTracker::new(trajectory).expect("tracker");
        let _ = tracker.update(2.0, 0.0, 0.0, 6.0, 0.07);
        tracker.brake();
        let first = tracker.update(2.5, 0.0, 0.0, 6.0, 0.07);
        let frozen_limit = tracker.progress_limit();
        let second = tracker.update(3.0, 0.0, 0.0, 5.0, 0.07);

        assert!(frozen_limit < 20.0);
        assert!((tracker.progress_limit() - frozen_limit).abs() < 1e-9);
        assert_eq!(first.brake_trajectory(), second.brake_trajectory());
    }

    #[test]
    fn rejects_zero_direction_input_like_python_assertion() {
        let error = process_reference_trajectory(vec![[0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0]])
            .err()
            .expect("zero direction must fail");

        assert_eq!(
            error,
            "the direction on each point of the trajectory should not be zero"
        );
    }

    #[test]
    fn removes_only_exact_adjacent_duplicate_xy_points() {
        let prepared = process_reference_trajectory(vec![
            [0.0, 0.0, 0.1, 1.0],
            [0.0, 0.0, 0.2, 1.0],
            [1e-12, 0.0, 0.3, 1.0],
            [1.0, 0.0, 0.4, 1.0],
        ])
        .expect("prepared trajectory");

        assert_eq!(prepared.points.len(), 3);
        assert_eq!(prepared.points[0][0], 0.0);
        assert_eq!(prepared.points[1][0], 1e-12);
    }

    #[test]
    fn exposes_public_reference_and_brake_states_in_python_order() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 1.0, 15.0, 0.0, 0.0, 1.0,
        ];

        let result =
            mpc_prepare_reference(trajectory, 1.0, 0.0, 0.0, 6.0, 0.07, false).expect("mpc prep");
        let model = result.model_reference_states();
        let public = result.reference_states();
        let brake = result.brake_trajectory();

        assert_eq!(model.len(), public.len());
        assert!(!brake.is_empty());
        assert_eq!(public[0], model[0]);
        assert_eq!(public[1], model[1]);
        assert_eq!(public[2], model[3]);
        assert_eq!(public[3], model[2]);
        assert_eq!(brake[0], result.model_reference_states()[0]);
    }
}
