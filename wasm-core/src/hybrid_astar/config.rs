#[derive(Clone, Copy, Debug)]
pub struct HybridAStarConfig {
    pub(crate) xy_grid_resolution: f64,
    pub(crate) yaw_grid_resolution: f64,
    pub(crate) motion_distance: f64,
    pub(crate) motion_resolution: f64,
    pub(crate) num_steer_commands: usize,
    pub(crate) reeds_shepp_max_distance: f64,
    pub(crate) switch_direction_cost: f64,
    pub(crate) backwards_cost: f64,
    pub(crate) steer_change_cost: f64,
    pub(crate) steer_cost: f64,
    pub(crate) h_dist_cost: f64,
    pub(crate) h_yaw_cost: f64,
}

impl Default for HybridAStarConfig {
    fn default() -> Self {
        let xy_grid_resolution = 1.0;
        Self {
            xy_grid_resolution,
            yaw_grid_resolution: 15.0_f64.to_radians(),
            motion_distance: xy_grid_resolution * 1.5,
            motion_resolution: 0.5,
            num_steer_commands: 10,
            reeds_shepp_max_distance: 10.0,
            switch_direction_cost: 25.0,
            backwards_cost: 4.0,
            steer_change_cost: 3.0,
            steer_cost: 1.5,
            h_dist_cost: 2.0,
            h_yaw_cost: 3.0 / 45.0_f64.to_radians(),
        }
    }
}
