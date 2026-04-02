use wasm_bindgen::prelude::*;

pub(crate) const NX: usize = 4;
pub(crate) const NU: usize = 2;

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
