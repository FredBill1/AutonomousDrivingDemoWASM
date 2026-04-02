use super::builder::linear_mpc_control;
use super::config::MpcConfig;
use super::types::{Control, ModelState, NX, RollingCarState};
use crate::geometry::{clamp, wrap_angle};
use wasm_bindgen::prelude::*;

/// Runs the MPC preview using default configuration values.
///
/// For customised configuration, use [`mpc_control_preview_with_config`].
#[wasm_bindgen]
pub fn mpc_control_preview(
    flat_reference_states: Vec<f64>,
    state_x: f64,
    state_y: f64,
    state_velocity: f64,
    state_yaw: f64,
    last_steer: f64,
    dt: f64,
) -> Result<super::types::MpcControlResult, JsValue> {
    let config = MpcConfig::default();
    mpc_control_preview_with_config(
        &config,
        flat_reference_states,
        state_x,
        state_y,
        state_velocity,
        state_yaw,
        last_steer,
        dt,
    )
}

pub(crate) fn mpc_control_preview_with_config(
    config: &MpcConfig,
    flat_reference_states: Vec<f64>,
    state_x: f64,
    state_y: f64,
    state_velocity: f64,
    mut state_yaw: f64,
    last_steer: f64,
    dt: f64,
) -> Result<super::types::MpcControlResult, JsValue> {
    let xref = decode_reference(&flat_reference_states)?;
    if xref.len() < config.horizon_length + 1 {
        return Err(JsValue::from_str("Need at least HORIZON_LENGTH + 1 reference states"));
    }

    state_yaw = align_yaw(state_yaw, xref[0][3]);

    let initial_state = RollingCarState {
        x: state_x,
        y: state_y,
        velocity: state_velocity,
        yaw: state_yaw,
        steer: last_steer,
    };
    let mut controls = vec![[0.0, 0.0]; config.horizon_length];
    let mut predicted_states = vec![[0.0; NX]; config.horizon_length + 1];
    let mut iterations = 0;

    for iteration in 0..config.max_iter {
        iterations = iteration + 1;
        let xbar = predict_motion(initial_state, &controls, config, dt);
        let previous_controls = controls.clone();
        let Some((updated_controls, updated_states)) = linear_mpc_control(&xref, &xbar, last_steer, config, dt) else {
            break;
        };
        let du = control_delta(&previous_controls, &updated_controls);
        controls = updated_controls;
        predicted_states = updated_states;
        if du < config.du_th {
            break;
        }
    }

    Ok(super::types::MpcControlResult {
        controls: controls.into_iter().flatten().collect(),
        predicted_states: predicted_states.into_iter().flatten().collect(),
        iterations,
    })
}

fn decode_reference(flat: &[f64]) -> Result<Vec<ModelState>, JsValue> {
    if !flat.len().is_multiple_of(4) {
        return Err(JsValue::from_str("Reference states must be flat [x, y, v, yaw] data"));
    }

    let mut states = Vec::with_capacity(flat.len() / 4);
    for chunk in flat.chunks_exact(4) {
        states.push([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    Ok(states)
}

pub(crate) fn predict_motion(
    initial: RollingCarState,
    controls: &[Control],
    config: &MpcConfig,
    dt: f64,
) -> Vec<ModelState> {
    let mut state = initial;
    let mut out = vec![[state.x, state.y, state.velocity, state.yaw]];
    for control in controls {
        let target_velocity = state.velocity + control[0] * dt;
        state = step_state(state, target_velocity, control[1], config, dt);
        out.push([state.x, state.y, state.velocity, state.yaw]);
    }
    out
}

fn step_state(
    mut state: RollingCarState,
    target_velocity: f64,
    target_steer: f64,
    config: &MpcConfig,
    dt: f64,
) -> RollingCarState {
    state.x += state.velocity * state.yaw.cos() * dt;
    state.y += state.velocity * state.yaw.sin() * dt;
    state.yaw += state.velocity / config.wheel_base * state.steer.tan() * dt;

    let clipped_target_velocity = clamp(target_velocity, config.min_speed, config.max_speed);
    let clipped_target_steer = clamp(target_steer, -config.max_steer, config.max_steer);
    state.velocity += clamp(
        clipped_target_velocity - state.velocity,
        -config.max_accel * dt,
        config.max_accel * dt,
    );
    state.steer += clamp(
        clipped_target_steer - state.steer,
        -config.max_steer_speed * dt,
        config.max_steer_speed * dt,
    );
    state
}

fn align_yaw(yaw: f64, target_yaw: f64) -> f64 {
    target_yaw + wrap_angle(yaw - target_yaw)
}

fn control_delta(left: &[Control], right: &[Control]) -> f64 {
    left.iter()
        .zip(right.iter())
        .map(|(lhs, rhs)| (lhs[0] - rhs[0]).powi(2) + (lhs[1] - rhs[1]).powi(2))
        .sum::<f64>()
        .sqrt()
}
