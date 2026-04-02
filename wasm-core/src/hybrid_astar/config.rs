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
    pub fn new() -> Self {
        Self::default()
    }

    #[wasm_bindgen(getter)]
    pub fn xy_grid_resolution(&self) -> f64 {
        self.xy_grid_resolution
    }

    #[wasm_bindgen(getter)]
    pub fn yaw_grid_resolution(&self) -> f64 {
        self.yaw_grid_resolution
    }

    #[wasm_bindgen(getter)]
    pub fn motion_distance(&self) -> f64 {
        self.motion_distance
    }

    #[wasm_bindgen(getter)]
    pub fn motion_resolution(&self) -> f64 {
        self.motion_resolution
    }

    #[wasm_bindgen(getter)]
    pub fn num_steer_commands(&self) -> u32 {
        self.num_steer_commands
    }

    #[wasm_bindgen(getter)]
    pub fn reeds_shepp_max_distance(&self) -> f64 {
        self.reeds_shepp_max_distance
    }

    #[wasm_bindgen(getter)]
    pub fn switch_direction_cost(&self) -> f64 {
        self.switch_direction_cost
    }

    #[wasm_bindgen(getter)]
    pub fn backwards_cost(&self) -> f64 {
        self.backwards_cost
    }

    #[wasm_bindgen(getter)]
    pub fn steer_change_cost(&self) -> f64 {
        self.steer_change_cost
    }

    #[wasm_bindgen(getter)]
    pub fn steer_cost(&self) -> f64 {
        self.steer_cost
    }

    #[wasm_bindgen(getter)]
    pub fn h_dist_cost(&self) -> f64 {
        self.h_dist_cost
    }

    #[wasm_bindgen(getter)]
    pub fn h_yaw_cost(&self) -> f64 {
        self.h_yaw_cost
    }
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
