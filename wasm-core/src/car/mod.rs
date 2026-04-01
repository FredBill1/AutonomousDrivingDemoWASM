mod collision;
mod config;
mod state;

pub use collision::{path_check_collision, trajectory_check_collision};
pub use config::CarConfig;
pub use state::{CarState, CarUpdateResult};

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
        assert_relative_eq!(config.target_max_steer(), 35.0_f64.to_radians(), epsilon = 1e-12);
        assert_relative_eq!(config.max_steer(), 40.0_f64.to_radians(), epsilon = 1e-12);
        assert_relative_eq!(config.max_steer_speed(), 360.0_f64.to_radians(), epsilon = 1e-12);
        assert_relative_eq!(config.max_speed(), 55.0 / 3.6, epsilon = 1e-12);
        assert_relative_eq!(config.min_speed(), -30.0 / 3.6, epsilon = 1e-12);
        assert_relative_eq!(config.max_accel(), 15.0, epsilon = 1e-12);
        assert_relative_eq!(config.max_centripetal_accel(), 16.0, epsilon = 1e-12);
        assert_relative_eq!(config.target_speed(), 40.0 / 3.6, epsilon = 1e-12);
        assert_relative_eq!(config.target_min_turning_radius(), 3.57037001686, epsilon = 1e-11);
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

        assert!(!trajectory_check_collision(&config, flat_trajectory, vec![20.0, 20.0],));
    }

    #[test]
    fn trajectory_collision_uses_precise_rectangle_after_coarse_screening() {
        let config = CarConfig::new();
        let flat_trajectory = vec![5.2, 5.0, 0.0];

        assert!(!trajectory_check_collision(&config, flat_trajectory, vec![6.45, 7.7],));
    }
}
