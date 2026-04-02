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
    /// MPC time step in seconds (dt used for both linearisation and integration).
    pub(crate) dt: f64,
    pub(crate) desired_max_accel_ratio: f64,
    pub(crate) min_horizon_distance: f64,
    pub(crate) direction_change_dist: f64,
    pub(crate) motion_resolution: f64,
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
    pub fn dt(&self) -> f64 {
        self.dt
    }

    #[wasm_bindgen(getter)]
    pub fn desired_max_accel_ratio(&self) -> f64 {
        self.desired_max_accel_ratio
    }

    #[wasm_bindgen(getter)]
    pub fn min_horizon_distance(&self) -> f64 {
        self.min_horizon_distance
    }

    #[wasm_bindgen(getter)]
    pub fn direction_change_dist(&self) -> f64 {
        self.direction_change_dist
    }

    #[wasm_bindgen(getter)]
    pub fn motion_resolution(&self) -> f64 {
        self.motion_resolution
    }
}

impl Default for MpcConfig {
    fn default() -> Self {
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
            dt: 0.07,
            desired_max_accel_ratio: 0.7,
            min_horizon_distance: 0.3,
            direction_change_dist: 0.1,
            motion_resolution: 0.5,
        }
    }
}
