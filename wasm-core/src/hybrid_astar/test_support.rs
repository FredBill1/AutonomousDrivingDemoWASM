use crate::car::CarConfig;

use super::{HybridAStarConfig, planner::HybridAStarPlanner};

pub(crate) fn build_test_planner(
    start_x: f64,
    start_y: f64,
    start_yaw: f64,
    goal_x: f64,
    goal_y: f64,
    goal_yaw: f64,
    obstacle_coordinates: Vec<f64>,
    max_iterations: usize,
) -> HybridAStarPlanner {
    let car_config = CarConfig::default();
    let ha_config = HybridAStarConfig::default();
    HybridAStarPlanner::new(
        start_x,
        start_y,
        start_yaw,
        goal_x,
        goal_y,
        goal_yaw,
        obstacle_coordinates,
        max_iterations,
        &car_config,
        &ha_config,
    )
    .expect("planner")
}

pub(crate) fn build_test_seed_planner(
    flat_start_seed: Vec<f64>,
    goal_x: f64,
    goal_y: f64,
    goal_yaw: f64,
    obstacle_coordinates: Vec<f64>,
    max_iterations: usize,
) -> HybridAStarPlanner {
    let car_config = CarConfig::default();
    let ha_config = HybridAStarConfig::default();
    HybridAStarPlanner::from_trajectory_seed(
        flat_start_seed,
        goal_x,
        goal_y,
        goal_yaw,
        obstacle_coordinates,
        max_iterations,
        &car_config,
        &ha_config,
    )
    .expect("planner")
}
