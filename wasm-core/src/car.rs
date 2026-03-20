use wasm_bindgen::prelude::*;

use crate::geometry::{clamp, distance, rotate, wrap_angle};

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
            wheel_base: 2.5,
            length: 4.5,
            width: 2.0,
            back_to_wheel: 1.0,
            target_max_steer: 35.0_f64.to_radians(),
            max_steer: 40.0_f64.to_radians(),
            max_steer_speed: 360.0_f64.to_radians(),
            max_speed: 55.0 / 3.6,
            min_speed: -30.0 / 3.6,
            max_accel: 15.0,
            max_centripetal_accel: 16.0,
            target_speed: 40.0 / 3.6,
            scan_radius: 15.0,
        }
    }
}

fn check_collision_against_points(
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
        self.yaw = wrap_angle(self.yaw + self.velocity / config.wheel_base * self.steer.tan() * dt);
    }

    pub fn update_with_control(
        &mut self,
        config: &CarConfig,
        target_velocity: f64,
        target_steer: f64,
        dt: f64,
    ) {
        self.update(config, dt);

        let clipped_velocity = clamp(target_velocity, config.min_speed, config.max_speed);
        let clipped_steer = clamp(target_steer, -config.max_steer, config.max_steer);

        self.velocity += clamp(
            clipped_velocity - self.velocity,
            -config.max_accel * dt,
            config.max_accel * dt,
        );
        self.steer += clamp(
            clipped_steer - self.steer,
            -config.max_steer_speed * dt,
            config.max_steer_speed * dt,
        );
        self.steer = clamp(self.steer, -config.max_steer, config.max_steer);
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
pub fn path_check_collision(
    config: &CarConfig,
    flat_path: Vec<f64>,
    obstacle_coordinates: Vec<f64>,
) -> bool {
    let mut index = 0usize;
    while index + 2 < flat_path.len() {
        let state = CarState::new(
            flat_path[index],
            flat_path[index + 1],
            flat_path[index + 2],
            0.0,
            0.0,
        );
        if state.check_collision(config, obstacle_coordinates.clone()) {
            return true;
        }
        index += 3;
    }

    false
}

#[wasm_bindgen]
pub fn trajectory_check_collision(
    config: &CarConfig,
    flat_trajectory: Vec<f64>,
    obstacle_coordinates: Vec<f64>,
) -> bool {
    let collision_radius = config.collision_radius();
    let mut trajectory_index = 0usize;

    while trajectory_index + 2 < flat_trajectory.len() {
        let state = CarState::new(
            flat_trajectory[trajectory_index],
            flat_trajectory[trajectory_index + 1],
            flat_trajectory[trajectory_index + 2],
            0.0,
            0.0,
        );
        trajectory_index += 3;

        let center_x = state.collision_center_x(config);
        let center_y = state.collision_center_y(config);
        let mut nearby_obstacles = Vec::new();

        let mut obstacle_index = 0usize;
        while obstacle_index + 1 < obstacle_coordinates.len() {
            let obstacle_x = obstacle_coordinates[obstacle_index];
            let obstacle_y = obstacle_coordinates[obstacle_index + 1];
            obstacle_index += 2;

            if distance(center_x, center_y, obstacle_x, obstacle_y) <= collision_radius {
                nearby_obstacles.push(obstacle_x);
                nearby_obstacles.push(obstacle_y);
            }
        }

        if !nearby_obstacles.is_empty() && state.check_collision(config, nearby_obstacles) {
            return true;
        }
    }

    false
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
    fn from_state(state: CarState) -> Self {
        Self {
            x: state.x,
            y: state.y,
            yaw: state.yaw,
            velocity: state.velocity,
            steer: state.steer,
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::{CarConfig, CarState, path_check_collision, trajectory_check_collision};

    #[test]
    fn updates_with_control_like_python_car_model() {
        let config = CarConfig::new();
        let mut car = CarState::new(1.0, 2.0, 0.3, 4.0, 0.1);

        car.update_with_control(&config, 6.5, 0.22, 0.1);

        assert_relative_eq!(car.x(), 1.38213459565, epsilon = 1e-9);
        assert_relative_eq!(car.y(), 2.11820808266, epsilon = 1e-9);
        assert_relative_eq!(car.yaw(), 0.31605354748, epsilon = 1e-9);
        assert_relative_eq!(car.velocity(), 5.5, epsilon = 1e-12);
        assert_relative_eq!(car.steer(), 0.22, epsilon = 1e-12);
    }

    #[test]
    fn exposes_python_car_constants() {
        let config = CarConfig::new();

        assert_relative_eq!(config.wheel_base(), 2.5, epsilon = 1e-12);
        assert_relative_eq!(config.length(), 4.5, epsilon = 1e-12);
        assert_relative_eq!(config.width(), 2.0, epsilon = 1e-12);
        assert_relative_eq!(config.back_to_wheel(), 1.0, epsilon = 1e-12);
        assert_relative_eq!(config.back_to_center(), 1.25, epsilon = 1e-12);
        assert_relative_eq!(config.collision_length(), 5.0, epsilon = 1e-12);
        assert_relative_eq!(config.collision_width(), 2.5, epsilon = 1e-12);
        assert_relative_eq!(config.collision_radius(), 2.79508497187, epsilon = 1e-11);
        assert_relative_eq!(
            config.target_max_steer(),
            35.0_f64.to_radians(),
            epsilon = 1e-12
        );
        assert_relative_eq!(config.max_steer(), 40.0_f64.to_radians(), epsilon = 1e-12);
        assert_relative_eq!(
            config.max_steer_speed(),
            360.0_f64.to_radians(),
            epsilon = 1e-12
        );
        assert_relative_eq!(config.max_speed(), 55.0 / 3.6, epsilon = 1e-12);
        assert_relative_eq!(config.min_speed(), -30.0 / 3.6, epsilon = 1e-12);
        assert_relative_eq!(config.max_accel(), 15.0, epsilon = 1e-12);
        assert_relative_eq!(config.max_centripetal_accel(), 16.0, epsilon = 1e-12);
        assert_relative_eq!(config.target_speed(), 40.0 / 3.6, epsilon = 1e-12);
        assert_relative_eq!(
            config.target_min_turning_radius(),
            3.57037001686,
            epsilon = 1e-11
        );
        assert_relative_eq!(config.scan_radius(), 15.0, epsilon = 1e-12);
    }

    #[test]
    fn detects_collision_in_vehicle_frame() {
        let config = CarConfig::new();
        let car = CarState::new(5.0, 5.0, 0.0, 0.0, 0.0);
        let obstacles = [6.5, 5.0, 15.0, 15.0];

        assert!(car.check_collision(&config, obstacles.to_vec()));
        assert!(!car.check_collision(&config, vec![15.0, 15.0]));
    }

    #[test]
    fn collision_uses_rear_axle_origin_and_rotated_rectangle() {
        let config = CarConfig::new();
        let car = CarState::new(10.0, 4.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0);

        assert!(car.check_collision(&config, vec![10.0, 5.6]));
        assert!(!car.check_collision(&config, vec![12.0, 4.0]));
    }

    #[test]
    fn detects_collision_along_flat_path() {
        let config = CarConfig::new();
        let flat_path = vec![0.0, 0.0, 0.0, 4.0, 4.0, 0.0, 5.2, 5.0, 0.0];

        assert!(path_check_collision(&config, flat_path, vec![6.1, 5.0]));
    }

    #[test]
    fn detects_trajectory_collision_using_center_offset_candidates() {
        let config = CarConfig::new();
        let flat_trajectory = vec![5.2, 5.0, 0.0, 15.0, 15.0, 0.0];

        assert!(trajectory_check_collision(
            &config,
            flat_trajectory,
            vec![7.4, 5.0, 30.0, 30.0],
        ));
    }

    #[test]
    fn ignores_obstacles_outside_center_offset_collision_radius() {
        let config = CarConfig::new();
        let flat_trajectory = vec![5.2, 5.0, 0.0];

        assert!(!trajectory_check_collision(
            &config,
            flat_trajectory,
            vec![20.0, 20.0],
        ));
    }

    #[test]
    fn trajectory_collision_uses_precise_rectangle_after_coarse_screening() {
        let config = CarConfig::new();
        let flat_trajectory = vec![5.2, 5.0, 0.0];

        assert!(!trajectory_check_collision(
            &config,
            flat_trajectory,
            vec![6.45, 7.7],
        ));
    }
}
