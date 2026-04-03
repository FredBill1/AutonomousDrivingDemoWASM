use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Clone, Copy, Debug)]
pub struct HybridAStarConfig {
    pub(crate) xy_grid_resolution: f64,
    pub(crate) yaw_grid_resolution: f64,
    pub(crate) motion_distance: f64,
    pub(crate) motion_resolution: f64,
    pub(crate) num_steer_commands: u32,
    pub(crate) reeds_shepp_max_distance: f64,
    pub(crate) switch_direction_cost: f64,
    pub(crate) backwards_cost: f64,
    pub(crate) steer_change_cost: f64,
    pub(crate) steer_cost: f64,
    pub(crate) h_dist_cost: f64,
    pub(crate) h_yaw_cost: f64,
}

#[wasm_bindgen]
impl HybridAStarConfig {
    #[wasm_bindgen(constructor)]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        xy_grid_resolution: f64,
        yaw_grid_resolution: f64,
        motion_distance: f64,
        motion_resolution: f64,
        num_steer_commands: u32,
        reeds_shepp_max_distance: f64,
        switch_direction_cost: f64,
        backwards_cost: f64,
        steer_change_cost: f64,
        steer_cost: f64,
        h_dist_cost: f64,
        h_yaw_cost: f64,
    ) -> Self {
        Self {
            xy_grid_resolution,
            yaw_grid_resolution,
            motion_distance,
            motion_resolution,
            num_steer_commands,
            reeds_shepp_max_distance,
            switch_direction_cost,
            backwards_cost,
            steer_change_cost,
            steer_cost,
            h_dist_cost,
            h_yaw_cost,
        }
    }
}

wasm_getters!(HybridAStarConfig {
    xy_grid_resolution(this) -> f64 => this.xy_grid_resolution;
    yaw_grid_resolution(this) -> f64 => this.yaw_grid_resolution;
    motion_distance(this) -> f64 => this.motion_distance;
    motion_resolution(this) -> f64 => this.motion_resolution;
    num_steer_commands(this) -> u32 => this.num_steer_commands;
    reeds_shepp_max_distance(this) -> f64 => this.reeds_shepp_max_distance;
    switch_direction_cost(this) -> f64 => this.switch_direction_cost;
    backwards_cost(this) -> f64 => this.backwards_cost;
    steer_change_cost(this) -> f64 => this.steer_change_cost;
    steer_cost(this) -> f64 => this.steer_cost;
    h_dist_cost(this) -> f64 => this.h_dist_cost;
    h_yaw_cost(this) -> f64 => this.h_yaw_cost;
});

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
