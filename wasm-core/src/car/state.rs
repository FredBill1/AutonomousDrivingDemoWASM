use wasm_bindgen::prelude::*;

use crate::geometry::{clamp, rotate, wrap_angle};

use super::config::CarConfig;

pub(super) fn check_collision_against_points(
    center_x: f64,
    center_y: f64,
    yaw: f64,
    collision_radius: f64,
    half_length: f64,
    half_width: f64,
    obstacle_coordinates: &[f64],
) -> bool {
    let local_yaw = -yaw;

    let mut index = 0usize;
    while index + 1 < obstacle_coordinates.len() {
        let x = obstacle_coordinates[index];
        let y = obstacle_coordinates[index + 1];
        index += 2;

        if (x - center_x).hypot(y - center_y) > collision_radius {
            continue;
        }

        let (local_x, local_y) = rotate(x - center_x, y - center_y, local_yaw);
        if local_x.abs() < half_length && local_y.abs() < half_width {
            return true;
        }
    }

    false
}

#[wasm_bindgen]
#[derive(Clone, Copy, Debug)]
pub struct CarState {
    x: f64,
    y: f64,
    yaw: f64,
    velocity: f64,
    steer: f64,
}

#[wasm_bindgen]
impl CarState {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, yaw: f64, velocity: f64, steer: f64) -> CarState {
        Self {
            x,
            y,
            yaw,
            velocity,
            steer,
        }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.x
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.y
    }

    #[wasm_bindgen(getter)]
    pub fn yaw(&self) -> f64 {
        self.yaw
    }

    #[wasm_bindgen(getter)]
    pub fn velocity(&self) -> f64 {
        self.velocity
    }

    #[wasm_bindgen(getter)]
    pub fn steer(&self) -> f64 {
        self.steer
    }

    pub fn align_yaw(&mut self, target_yaw: f64) {
        self.yaw = target_yaw + wrap_angle(self.yaw - target_yaw);
    }

    pub fn update(&mut self, config: &CarConfig, dt: f64) {
        self.x += self.velocity * self.yaw.cos() * dt;
        self.y += self.velocity * self.yaw.sin() * dt;
        self.yaw =
            wrap_angle(self.yaw + self.velocity / config.wheel_base() * self.steer.tan() * dt);
    }

    pub fn update_with_control(
        &mut self,
        config: &CarConfig,
        target_velocity: f64,
        target_steer: f64,
        dt: f64,
    ) {
        self.update(config, dt);

        let clipped_velocity = clamp(target_velocity, config.min_speed(), config.max_speed());
        let clipped_steer = clamp(target_steer, -config.max_steer(), config.max_steer());

        self.velocity += clamp(
            clipped_velocity - self.velocity,
            -config.max_accel() * dt,
            config.max_accel() * dt,
        );
        self.steer += clamp(
            clipped_steer - self.steer,
            -config.max_steer_speed() * dt,
            config.max_steer_speed() * dt,
        );
        self.steer = clamp(self.steer, -config.max_steer(), config.max_steer());
    }

    pub fn stepped(
        &self,
        config: &CarConfig,
        target_velocity: f64,
        target_steer: f64,
        dt: f64,
    ) -> CarUpdateResult {
        let mut next = *self;
        next.update_with_control(config, target_velocity, target_steer, dt);
        CarUpdateResult::from_state(next)
    }

    pub fn collision_center_x(&self, config: &CarConfig) -> f64 {
        self.x + config.back_to_center() * self.yaw.cos()
    }

    pub fn collision_center_y(&self, config: &CarConfig) -> f64 {
        self.y + config.back_to_center() * self.yaw.sin()
    }

    pub fn check_collision(&self, config: &CarConfig, obstacle_coordinates: Vec<f64>) -> bool {
        let center_x = self.collision_center_x(config);
        let center_y = self.collision_center_y(config);
        check_collision_against_points(
            center_x,
            center_y,
            self.yaw,
            config.collision_radius(),
            config.collision_length() / 2.0,
            config.collision_width() / 2.0,
            &obstacle_coordinates,
        )
    }
}

#[wasm_bindgen]
#[derive(Clone, Copy, Debug)]
pub struct CarUpdateResult {
    x: f64,
    y: f64,
    yaw: f64,
    velocity: f64,
    steer: f64,
}

#[wasm_bindgen]
impl CarUpdateResult {
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.x
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.y
    }

    #[wasm_bindgen(getter)]
    pub fn yaw(&self) -> f64 {
        self.yaw
    }

    #[wasm_bindgen(getter)]
    pub fn velocity(&self) -> f64 {
        self.velocity
    }

    #[wasm_bindgen(getter)]
    pub fn steer(&self) -> f64 {
        self.steer
    }
}

impl CarUpdateResult {
    pub(super) fn from_state(state: CarState) -> Self {
        Self {
            x: state.x,
            y: state.y,
            yaw: state.yaw,
            velocity: state.velocity,
            steer: state.steer,
        }
    }
}
