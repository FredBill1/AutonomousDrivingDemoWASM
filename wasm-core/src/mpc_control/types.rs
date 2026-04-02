use wasm_bindgen::prelude::*;

pub(crate) use crate::car::constants::{MAX_ACCEL, MAX_SPEED, MAX_STEER, MAX_STEER_SPEED, MIN_SPEED, WHEEL_BASE};

pub(crate) const HORIZON_LENGTH: usize = 5;
pub(crate) const MAX_ITER: usize = 5;
pub(crate) const DU_TH: f64 = 0.1;
pub(crate) const NX: usize = 4;
pub(crate) const NU: usize = 2;

pub(crate) const R_ACCEL: f64 = 0.01;
pub(crate) const R_STEER: f64 = 0.005;
pub(crate) const RD_ACCEL: f64 = 1e-5;
pub(crate) const RD_STEER: f64 = 1e-3;
pub(crate) const Q_X: f64 = 1.1;
pub(crate) const Q_Y: f64 = 1.1;
pub(crate) const Q_V: f64 = 0.05;
pub(crate) const Q_YAW: f64 = 1.1;
pub(crate) const QF_SCALE: f64 = 2.0;

pub(crate) type ModelState = [f64; 4];
pub(crate) type Control = [f64; 2];

#[derive(Clone, Copy)]
pub(crate) struct RollingCarState {
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) velocity: f64,
    pub(crate) yaw: f64,
    pub(crate) steer: f64,
}

#[wasm_bindgen]
pub struct MpcControlResult {
    pub(crate) controls: Vec<f64>,
    pub(crate) predicted_states: Vec<f64>,
    pub(crate) iterations: usize,
}

#[wasm_bindgen]
impl MpcControlResult {
    #[wasm_bindgen(getter)]
    pub fn controls(&self) -> Vec<f64> {
        self.controls.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn predicted_states(&self) -> Vec<f64> {
        self.predicted_states.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn iterations(&self) -> usize {
        self.iterations
    }
}
