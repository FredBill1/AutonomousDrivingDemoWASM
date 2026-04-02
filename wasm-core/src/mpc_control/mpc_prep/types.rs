use wasm_bindgen::prelude::*;

pub(crate) use crate::car::constants::{
    MAX_ACCEL, MAX_CENTRIPETAL_ACCEL, MAX_SPEED, MIN_SPEED, MOTION_RESOLUTION, TARGET_SPEED,
};
pub(crate) use crate::geometry::{clamp, distance, wrap_angle};
pub(crate) use crate::mpc_control::HORIZON_LENGTH;

pub(crate) const DESIRED_MAX_ACCEL_RATIO: f64 = 0.7;
pub(crate) const MIN_HORIZON_DISTANCE: f64 = 0.3;
pub(crate) const DIRECTION_CHANGE_DIST: f64 = 0.1;

#[wasm_bindgen]
pub struct MpcReferenceResult {
    pub(crate) model_reference_states: Vec<f64>,
    pub(crate) reference_states: Vec<f64>,
    pub(crate) brake_trajectory: Vec<f64>,
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

#[derive(Clone)]
pub(crate) struct PreparedTrajectory {
    pub(crate) points: Vec<[f64; 4]>,
    pub(crate) us: Vec<f64>,
    pub(crate) direction_change_us: Vec<f64>,
}

pub(crate) fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

pub(crate) fn midpoint(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        (a[0] + b[0]) / 2.0,
        (a[1] + b[1]) / 2.0,
        (a[2] + b[2]) / 2.0,
        (a[3] + b[3]) / 2.0,
    ]
}
