use wasm_bindgen::prelude::*;

pub(crate) const TARGET_SPEED: f64 = 40.0 / 3.6;
pub(crate) const MAX_SPEED: f64 = 55.0 / 3.6;
pub(crate) const MIN_SPEED: f64 = -30.0 / 3.6;
pub(crate) const MAX_ACCEL: f64 = 15.0;
pub(crate) const MAX_CENTRIPETAL_ACCEL: f64 = 16.0;
pub(crate) const DESIRED_MAX_ACCEL_RATIO: f64 = 0.7;
pub(crate) const HORIZON_LENGTH: usize = 5;
pub(crate) const MIN_HORIZON_DISTANCE: f64 = 0.3;
pub(crate) const MOTION_RESOLUTION: f64 = 0.5;
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

pub(crate) fn distance(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    (ax - bx).hypot(ay - by)
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
