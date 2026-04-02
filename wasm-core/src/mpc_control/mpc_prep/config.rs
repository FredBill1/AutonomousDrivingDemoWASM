use crate::car::CarConfig;
use crate::mpc_control::MpcConfig;

#[derive(Clone, Copy, Debug)]
pub struct MpcPrepConfig {
    pub(crate) max_accel: f64,
    pub(crate) max_centripetal_accel: f64,
    pub(crate) max_speed: f64,
    pub(crate) min_speed: f64,
    pub(crate) target_speed: f64,
    pub(crate) desired_max_accel_ratio: f64,
    pub(crate) min_horizon_distance: f64,
    pub(crate) direction_change_dist: f64,
    pub(crate) motion_resolution: f64,
    pub(crate) horizon_length: usize,
}

impl Default for MpcPrepConfig {
    fn default() -> Self {
        let car = CarConfig::default();
        let mpc = MpcConfig::default();
        Self {
            max_accel: car.max_accel(),
            max_centripetal_accel: car.max_centripetal_accel(),
            max_speed: car.max_speed(),
            min_speed: car.min_speed(),
            target_speed: car.target_speed(),
            desired_max_accel_ratio: 0.7,
            min_horizon_distance: 0.3,
            direction_change_dist: 0.1,
            motion_resolution: 0.5,
            horizon_length: mpc.horizon_length,
        }
    }
}
