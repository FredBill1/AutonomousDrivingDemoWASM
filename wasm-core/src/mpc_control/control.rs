use super::builder::linear_mpc_control;
use super::types::{
    Control, DU_TH, HORIZON_LENGTH, MAX_ACCEL, MAX_ITER, MAX_SPEED, MAX_STEER, MAX_STEER_SPEED, MIN_SPEED, ModelState,
    MpcControlResult, NX, RollingCarState, WHEEL_BASE,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn mpc_control_preview(
    flat_reference_states: Vec<f64>,
    state_x: f64,
    state_y: f64,
    state_velocity: f64,
    mut state_yaw: f64,
    last_steer: f64,
    dt: f64,
) -> Result<MpcControlResult, JsValue> {
    let xref = decode_reference(&flat_reference_states)?;
    if xref.len() < HORIZON_LENGTH + 1 {
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
    let mut controls = vec![[0.0, 0.0]; HORIZON_LENGTH];
    let mut predicted_states = vec![[0.0; NX]; HORIZON_LENGTH + 1];
    let mut iterations = 0;

    for iteration in 0..MAX_ITER {
        iterations = iteration + 1;
        let xbar = predict_motion(initial_state, &controls, dt);
        let previous_controls = controls.clone();
        let Some((updated_controls, updated_states)) = linear_mpc_control(&xref, &xbar, last_steer, dt) else {
            break;
        };
        let du = control_delta(&previous_controls, &updated_controls);
        controls = updated_controls;
        predicted_states = updated_states;
        if du < DU_TH {
            break;
        }
    }

    Ok(MpcControlResult {
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

pub(crate) fn predict_motion(initial: RollingCarState, controls: &[Control], dt: f64) -> Vec<ModelState> {
    let mut state = initial;
    let mut out = vec![[state.x, state.y, state.velocity, state.yaw]];
    for control in controls {
        let target_velocity = state.velocity + control[0] * dt;
        state = step_state(state, target_velocity, control[1], dt);
        out.push([state.x, state.y, state.velocity, state.yaw]);
    }
    out
}

fn step_state(mut state: RollingCarState, target_velocity: f64, target_steer: f64, dt: f64) -> RollingCarState {
    state.x += state.velocity * state.yaw.cos() * dt;
    state.y += state.velocity * state.yaw.sin() * dt;
    state.yaw += state.velocity / WHEEL_BASE * state.steer.tan() * dt;

    let clipped_target_velocity = clamp(target_velocity, MIN_SPEED, MAX_SPEED);
    let clipped_target_steer = clamp(target_steer, -MAX_STEER, MAX_STEER);
    state.velocity += clamp(
        clipped_target_velocity - state.velocity,
        -MAX_ACCEL * dt,
        MAX_ACCEL * dt,
    );
    state.steer += clamp(
        clipped_target_steer - state.steer,
        -MAX_STEER_SPEED * dt,
        MAX_STEER_SPEED * dt,
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

pub(crate) fn wrap_angle(mut angle: f64) -> f64 {
    while angle >= std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}

pub(crate) fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}
