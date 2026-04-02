use crate::car::CarConfig;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Clone, Copy, Debug)]
pub struct MpcConfig {
    pub(crate) horizon_length: u32,
    pub(crate) max_iter: u32,
    pub(crate) du_th: f64,
    pub(crate) r_accel: f64,
    pub(crate) r_steer: f64,
    pub(crate) rd_accel: f64,
    pub(crate) rd_steer: f64,
    pub(crate) q_x: f64,
    pub(crate) q_y: f64,
    pub(crate) q_v: f64,
    pub(crate) q_yaw: f64,
    pub(crate) qf_scale: f64,
    pub(crate) max_speed: f64,
    pub(crate) min_speed: f64,
    pub(crate) max_accel: f64,
    pub(crate) max_steer: f64,
    pub(crate) max_steer_speed: f64,
    pub(crate) wheel_base: f64,
    pub(crate) dt: f64,
}

#[wasm_bindgen]
impl MpcConfig {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self::default()
    }

    #[wasm_bindgen(getter)]
    pub fn horizon_length(&self) -> u32 {
        self.horizon_length
    }

    #[wasm_bindgen(getter)]
    pub fn max_iter(&self) -> u32 {
        self.max_iter
    }

    #[wasm_bindgen(getter)]
    pub fn du_th(&self) -> f64 {
        self.du_th
    }

    #[wasm_bindgen(getter)]
    pub fn r_accel(&self) -> f64 {
        self.r_accel
    }

    #[wasm_bindgen(getter)]
    pub fn r_steer(&self) -> f64 {
        self.r_steer
    }

    #[wasm_bindgen(getter)]
    pub fn rd_accel(&self) -> f64 {
        self.rd_accel
    }

    #[wasm_bindgen(getter)]
    pub fn rd_steer(&self) -> f64 {
        self.rd_steer
    }

    #[wasm_bindgen(getter)]
    pub fn q_x(&self) -> f64 {
        self.q_x
    }

    #[wasm_bindgen(getter)]
    pub fn q_y(&self) -> f64 {
        self.q_y
    }

    #[wasm_bindgen(getter)]
    pub fn q_v(&self) -> f64 {
        self.q_v
    }

    #[wasm_bindgen(getter)]
    pub fn q_yaw(&self) -> f64 {
        self.q_yaw
    }

    #[wasm_bindgen(getter)]
    pub fn qf_scale(&self) -> f64 {
        self.qf_scale
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
    pub fn max_steer(&self) -> f64 {
        self.max_steer
    }

    #[wasm_bindgen(getter)]
    pub fn max_steer_speed(&self) -> f64 {
        self.max_steer_speed
    }

    #[wasm_bindgen(getter)]
    pub fn wheel_base(&self) -> f64 {
        self.wheel_base
    }

    #[wasm_bindgen(getter)]
    pub fn dt(&self) -> f64 {
        self.dt
    }
}

impl Default for MpcConfig {
    fn default() -> Self {
        let car = CarConfig::default();
        Self {
            horizon_length: 5,
            max_iter: 5,
            du_th: 0.1,
            r_accel: 0.01,
            r_steer: 0.005,
            rd_accel: 1e-5,
            rd_steer: 1e-3,
            q_x: 1.1,
            q_y: 1.1,
            q_v: 0.05,
            q_yaw: 1.1,
            qf_scale: 2.0,
            max_speed: car.max_speed(),
            min_speed: car.min_speed(),
            max_accel: car.max_accel(),
            max_steer: car.max_steer(),
            max_steer_speed: car.max_steer_speed(),
            wheel_base: car.wheel_base(),
            dt: 0.07,
        }
    }
}
