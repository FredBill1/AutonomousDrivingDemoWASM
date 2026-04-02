use wasm_bindgen::prelude::*;

use super::constants::{
    MAX_ACCEL, MAX_CENTRIPETAL_ACCEL, MAX_SPEED, MAX_STEER, MAX_STEER_SPEED, MIN_SPEED, TARGET_SPEED, WHEEL_BASE,
};

#[wasm_bindgen]
#[derive(Clone, Copy, Debug)]
pub struct CarConfig {
    wheel_base: f64,
    length: f64,
    width: f64,
    back_to_wheel: f64,
    target_max_steer: f64,
    max_steer: f64,
    max_steer_speed: f64,
    max_speed: f64,
    min_speed: f64,
    max_accel: f64,
    max_centripetal_accel: f64,
    target_speed: f64,
    scan_radius: f64,
}

#[wasm_bindgen]
impl CarConfig {
    #[wasm_bindgen(constructor)]
    pub fn new() -> CarConfig {
        Self::default()
    }

    #[wasm_bindgen(getter)]
    pub fn wheel_base(&self) -> f64 {
        self.wheel_base
    }

    #[wasm_bindgen(getter)]
    pub fn length(&self) -> f64 {
        self.length
    }

    #[wasm_bindgen(getter)]
    pub fn width(&self) -> f64 {
        self.width
    }

    #[wasm_bindgen(getter)]
    pub fn back_to_wheel(&self) -> f64 {
        self.back_to_wheel
    }

    #[wasm_bindgen(getter)]
    pub fn wheel_length(&self) -> f64 {
        0.8
    }

    #[wasm_bindgen(getter)]
    pub fn wheel_width(&self) -> f64 {
        0.5
    }

    #[wasm_bindgen(getter)]
    pub fn wheel_spacing(&self) -> f64 {
        1.4
    }

    #[wasm_bindgen(getter)]
    pub fn target_max_steer(&self) -> f64 {
        self.target_max_steer
    }

    #[wasm_bindgen(getter)]
    pub fn max_steer(&self) -> f64 {
        self.max_steer
    }

    #[wasm_bindgen(getter)]
    pub fn max_steer_speed(&self) -> f64 {
        self.max_steer_speed
    }

    #[wasm_bindgen(getter)]
    pub fn max_speed(&self) -> f64 {
        self.max_speed
    }

    #[wasm_bindgen(getter)]
    pub fn min_speed(&self) -> f64 {
        self.min_speed
    }

    #[wasm_bindgen(getter)]
    pub fn max_accel(&self) -> f64 {
        self.max_accel
    }

    #[wasm_bindgen(getter)]
    pub fn max_centripetal_accel(&self) -> f64 {
        self.max_centripetal_accel
    }

    #[wasm_bindgen(getter)]
    pub fn target_speed(&self) -> f64 {
        self.target_speed
    }

    #[wasm_bindgen(getter)]
    pub fn scan_radius(&self) -> f64 {
        self.scan_radius
    }

    #[wasm_bindgen(getter)]
    pub fn back_to_center(&self) -> f64 {
        self.length / 2.0 - self.back_to_wheel
    }

    #[wasm_bindgen(getter)]
    pub fn collision_length(&self) -> f64 {
        self.length + 0.5
    }

    #[wasm_bindgen(getter)]
    pub fn collision_width(&self) -> f64 {
        self.width + 0.5
    }

    #[wasm_bindgen(getter)]
    pub fn collision_radius(&self) -> f64 {
        (self.collision_width() / 2.0).hypot(self.collision_length() / 2.0)
    }

    #[wasm_bindgen(getter)]
    pub fn target_min_turning_radius(&self) -> f64 {
        self.wheel_base / self.target_max_steer.tan()
    }
}

impl Default for CarConfig {
    fn default() -> Self {
        Self {
            wheel_base: WHEEL_BASE,
            length: 4.5,
            width: 2.0,
            back_to_wheel: 1.0,
            target_max_steer: 35.0_f64.to_radians(),
            max_steer: MAX_STEER,
            max_steer_speed: MAX_STEER_SPEED,
            max_speed: MAX_SPEED,
            min_speed: MIN_SPEED,
            max_accel: MAX_ACCEL,
            max_centripetal_accel: MAX_CENTRIPETAL_ACCEL,
            target_speed: TARGET_SPEED,
            scan_radius: 15.0,
        }
    }
}
