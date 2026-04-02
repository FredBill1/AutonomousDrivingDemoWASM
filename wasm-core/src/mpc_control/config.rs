use crate::car::CarConfig;

#[derive(Clone, Copy, Debug)]
pub struct MpcConfig {
    pub(crate) horizon_length: usize,
    pub(crate) max_iter: usize,
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
        }
    }
}
