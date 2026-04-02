use crate::car::CarConfig;
use crate::mpc_control::MpcConfig;
use wasm_bindgen::prelude::*;

use super::trajectory::{decode_trajectory, linspace, process_reference_trajectory};
use super::types::{MpcReferenceResult, PreparedTrajectory, distance, lerp};

#[wasm_bindgen]
pub struct MpcReferenceTracker {
    mpc_config: MpcConfig,
    car_config: CarConfig,
    pub(crate) prepared: PreparedTrajectory,
    cur_u: f64,
    u_limit: f64,
    brake: bool,
    braked: bool,
    brake_trajectory: Vec<[f64; 4]>,
}

#[wasm_bindgen]
impl MpcReferenceTracker {
    #[wasm_bindgen(constructor)]
    pub fn new(flat_trajectory: Vec<f64>, mpc_config: &MpcConfig, car_config: &CarConfig) -> Result<MpcReferenceTracker, JsValue> {
        let trajectory = decode_trajectory(&flat_trajectory)?;
        let prepared = process_reference_trajectory(trajectory, mpc_config, car_config).map_err(JsValue::from_str)?;
        let u_limit = *prepared.us.last().unwrap_or(&0.0);
        Ok(MpcReferenceTracker {
            mpc_config: *mpc_config,
            car_config: *car_config,
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
            model_reference_states: model_reference_states.iter().flat_map(|state| *state).collect(),
            reference_states,
            brake_trajectory,
        }
    }
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
        let brake_length = state_velocity.powi(2)
            / (2.0 * self.car_config.max_accel() * self.mpc_config.desired_max_accel_ratio);
        let brake_limit = self.u_limit.min(self.cur_u + brake_length).min(changing_point);
        let mut brake_us = Vec::new();
        let mut u = self.cur_u;
        while u <= brake_limit + self.mpc_config.motion_resolution / 2.0 {
            brake_us.push(u.min(brake_limit));
            u += self.mpc_config.motion_resolution;
        }
        self.brake_trajectory = brake_us
            .into_iter()
            .map(|brake_u| sample_state(&self.prepared, brake_u))
            .collect();
        let velocity_sign = self.brake_trajectory.iter().map(|state| state[2]).sum::<f64>().signum();
        for state in &mut self.brake_trajectory {
            state[2] = velocity_sign;
        }

        if self.brake {
            self.braked = true;
            self.u_limit = brake_limit;
        }
    }

    fn find_xref(&mut self, state_x: f64, state_y: f64, state_velocity: f64, dt: f64) -> Vec<[f64; 4]> {
        loop {
            self.find_nearest_point(state_x, state_y);

            let signed_velocity = sample_state(&self.prepared, self.cur_u)[2].signum() * state_velocity;
            let length = self
                .mpc_config
                .min_horizon_distance
                .max(signed_velocity.max(0.0) * dt * self.mpc_config.horizon_length as f64);
            let mut ref_us = linspace(self.cur_u, self.cur_u + length, self.mpc_config.horizon_length as usize + 1);
            for value in &mut ref_us {
                *value = value.min(self.u_limit);
            }

            let changing_point = self.next_direction_change();
            if ref_us.last().copied().unwrap_or(self.cur_u) >= changing_point {
                if self.cur_u + self.mpc_config.direction_change_dist >= changing_point {
                    self.cur_u = changing_point;
                    continue;
                }

                let cutoff = ref_us.partition_point(|&value| value <= changing_point);
                ref_us.truncate(cutoff);
                if ref_us.len() < self.mpc_config.horizon_length as usize + 1 {
                    ref_us.push(changing_point);
                    while ref_us.len() < self.mpc_config.horizon_length as usize + 1 {
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

pub(crate) fn sample_state(trajectory: &PreparedTrajectory, u: f64) -> [f64; 4] {
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

pub(crate) fn reorder_public_states(states: &[[f64; 4]]) -> Vec<f64> {
    states
        .iter()
        .flat_map(|state| [state[0], state[1], state[3], state[2]])
        .collect()
}

#[cfg(test)]
pub(crate) fn mpc_prepare_reference(
    flat_trajectory: Vec<f64>,
    state_x: f64,
    state_y: f64,
    state_yaw: f64,
    state_velocity: f64,
    dt: f64,
    brake: bool,
    mpc_config: &MpcConfig,
    car_config: &CarConfig,
) -> Result<MpcReferenceResult, JsValue> {
    let mut tracker = MpcReferenceTracker::new(flat_trajectory, mpc_config, car_config)?;
    if brake {
        tracker.brake();
    }
    Ok(tracker.update(state_x, state_y, state_yaw, state_velocity, dt))
}
