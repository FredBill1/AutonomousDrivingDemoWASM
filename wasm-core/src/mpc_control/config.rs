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
}

wasm_getters!(MpcConfig {
    horizon_length(this) -> u32 => this.horizon_length;
    max_iter(this) -> u32 => this.max_iter;
    du_th(this) -> f64 => this.du_th;
    r_accel(this) -> f64 => this.r_accel;
    r_steer(this) -> f64 => this.r_steer;
    rd_accel(this) -> f64 => this.rd_accel;
    rd_steer(this) -> f64 => this.rd_steer;
    q_x(this) -> f64 => this.q_x;
    q_y(this) -> f64 => this.q_y;
    q_v(this) -> f64 => this.q_v;
    q_yaw(this) -> f64 => this.q_yaw;
    qf_scale(this) -> f64 => this.qf_scale;
    desired_max_accel_ratio(this) -> f64 => this.desired_max_accel_ratio;
    min_horizon_distance(this) -> f64 => this.min_horizon_distance;
    direction_change_dist(this) -> f64 => this.direction_change_dist;
    motion_resolution(this) -> f64 => this.motion_resolution;
});

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
            desired_max_accel_ratio: 0.7,
            min_horizon_distance: 0.3,
            direction_change_dist: 0.1,
            motion_resolution: 0.5,
        }
    }
}
