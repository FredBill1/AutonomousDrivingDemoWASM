use wasm_bindgen::prelude::*;

use crate::geometry::distance;

use super::config::CarConfig;
use super::state::CarState;

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
