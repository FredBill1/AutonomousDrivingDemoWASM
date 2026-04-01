use approx::assert_relative_eq;

use crate::rsplan::{ReedsSheppPath, ReedsSheppSegment, SegmentKind};

use super::{
    heuristic::HeuristicGrid,
    planner::HybridAStarPlanner,
    search::{calc_rspath_cost, generate_neighbour, traceback_path},
    types::SearchNode,
    utils::{calc_ijk, python_sign},
};

#[test]
fn planner_uses_python_target_max_steer_for_rs_radius() {
    let planner = HybridAStarPlanner::new(
        2.0,
        2.0,
        0.0,
        15.0,
        15.0,
        0.0,
        vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0],
        10,
    )
    .expect("planner");

    let expected = planner.config.wheel_base() / 35.0_f64.to_radians().tan();
    let actual = planner.config.target_min_turning_radius();

    assert_relative_eq!(actual, expected, epsilon = 1e-12);
    assert!(
        planner
            .steer_commands
            .iter()
            .all(|steer: &f64| steer.abs() <= 35.0_f64.to_radians() + 1e-12)
    );
}

#[test]
fn heuristic_grid_indexes_goal_cell() {
    let obstacles = vec![0.0, 0.0, 0.0, 10.0, 10.0, 0.0, 10.0, 10.0];
    let config = crate::car::CarConfig::new();
    let grid = HeuristicGrid::from_obstacles(&obstacles, 5.0, 5.0, &config).expect("grid");
    let index = calc_ijk(5.0, 5.0, 0.0, &grid);
    assert!(grid.contains(index.0, index.1));
    assert!(grid.distance_at(index.0, index.1) <= 1.0);
}

#[test]
fn heuristic_grid_matches_python_downsampling() {
    let obstacles = vec![0.0, 0.0, 0.0, 4.0, 4.0, 0.0, 4.0, 4.0, 1.75, 1.75];
    let config = crate::car::CarConfig::new();
    let grid = HeuristicGrid::from_obstacles(&obstacles, 2.0, 2.0, &config).expect("grid");

    assert_eq!(grid.width, 5);
    assert_eq!(grid.height, 5);
    assert_eq!(
        grid.blocked,
        vec![
            true, true, false, true, true, true, true, true, false, true, false, true, true, false, false, true, false,
            false, false, true, true, true, false, true, true,
        ]
    );
}

#[test]
fn planner_returns_success_result_in_empty_box() {
    let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0];
    let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 4000).expect("path");
    while !planner.step(256).expect("step") {}
    let result = planner.take_result().expect("result");
    assert!(result.flat_path().len() > 4);
    assert!(result.explored_count() > 0);
    assert!(result.success());
}

#[test]
fn goal_collision_returns_finished_empty_result() {
    let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0, 15.0, 15.0];
    let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 4000).expect("planner");

    assert!(planner.is_finished());
    let result = planner.take_result().expect("result");
    assert!(!result.success());
    assert!(result.flat_path().is_empty());
}

#[test]
fn unreachable_search_returns_finished_empty_result() {
    let mut obstacles = box_obstacles(12.0, 12.0);
    for y in 0..=12 {
        obstacles.extend_from_slice(&[6.0, y as f64]);
    }

    let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 10.0, 2.0, 0.0, obstacles, 4000).expect("planner");
    while !planner.step(256).expect("step") {}

    let result = planner.take_result().expect("result");
    assert!(!result.success());
    assert!(result.flat_path().is_empty());
}

#[test]
fn planner_steps_emit_explored_segments_before_finish() {
    let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0];
    let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 4000).expect("planner");
    let finished = planner.step(4).expect("step");
    let explored = planner.take_explored_segments();

    assert!(!finished || planner.is_finished());
    assert!(!explored.is_empty());
}

#[test]
fn point_start_collision_keeps_first_escape_rollout() {
    let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0, 0.8, 2.0];
    let planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 1).expect("planner");
    let current = planner.nodes.values().next().expect("start node");

    let neighbour = generate_neighbour(
        current,
        1,
        0.0,
        &planner.goal,
        &planner.config,
        &planner.heuristic,
        &planner.obstacle_coordinates,
        true,
    );

    assert!(neighbour.is_some());
}

#[test]
fn neighbour_acceptance_does_not_prune_blocked_heuristic_cells() {
    let current = SearchNode {
        ijk: (1, 1, 0),
        trajectory: vec![[2.0, 2.0, 0.0, 0.0]],
        direction: 0,
        steer: 0.0,
        cost: 0.0,
        h_cost: 0.0,
        parent: None,
        analytic_path: None,
    };
    let heuristic = HeuristicGrid {
        min_x: 0.0,
        min_y: 0.0,
        width: 8,
        height: 8,
        resolution: 1.0,
        blocked: vec![true; 64],
        distances: vec![0.0; 64],
    };
    let config = crate::car::CarConfig::new();
    let goal = [7.0, 7.0, 0.0];
    let obstacles = box_obstacles(8.0, 8.0);

    let neighbour = generate_neighbour(&current, 1, 0.0, &goal, &config, &heuristic, &obstacles, false);

    assert!(neighbour.is_some());
}

#[test]
fn trajectory_seed_start_matches_python_seed_semantics() {
    let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0];
    let planner = HybridAStarPlanner::from_trajectory_seed(
        vec![1.0, 1.0, 0.0, 2.0, 2.0, 1.0, 0.2, 2.0, 3.0, 1.1, 0.3, 2.0],
        15.0,
        15.0,
        0.0,
        obstacles,
        4000,
    )
    .expect("planner");

    let start = planner.nodes.get(&(1, 3, 1)).expect("seed start node");
    assert_eq!(start.direction, 1);
    assert_eq!(start.trajectory.len(), 3);
    assert_relative_eq!(start.trajectory[0][0], 1.0, epsilon = 1e-12);
    assert_relative_eq!(start.trajectory[2][0], 3.0, epsilon = 1e-12);
    assert_relative_eq!(start.trajectory[2][1], 1.1, epsilon = 1e-12);
    assert_relative_eq!(start.trajectory[2][2], 0.3, epsilon = 1e-12);
    assert!(start.steer.is_finite());
}

#[test]
fn python_sign_keeps_zero_velocity_seed_direction() {
    assert_eq!(python_sign(2.0), 1);
    assert_eq!(python_sign(-2.0), -1);
    assert_eq!(python_sign(0.0), 0);
    assert_eq!(python_sign(-0.0), 0);
}

#[test]
fn planner_ignores_runtime_iteration_cap_parameter() {
    let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0];
    let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 1).expect("planner");

    for _ in 0..128 {
        if planner.step(1).expect("step") {
            break;
        }
    }

    assert!(planner.is_finished());
    let result = planner.take_result().expect("result");
    assert!(result.success());
}

#[test]
fn rs_cost_matches_python_segment_penalties() {
    let config = crate::car::CarConfig::new();
    let node = SearchNode {
        ijk: (0, 0, 0),
        trajectory: vec![[0.0, 0.0, 0.0, 1.0]],
        direction: 1,
        steer: 0.0,
        cost: 0.0,
        h_cost: 0.0,
        parent: None,
        analytic_path: None,
    };
    let mut path = ReedsSheppPath::new(0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 2.0, 0.5);
    path.push_segment(&ReedsSheppSegment::new(SegmentKind::Left, 1, 2.0, 2.0));
    path.push_segment(&ReedsSheppSegment::new(SegmentKind::Straight, -1, 1.0, 2.0));

    let cost = calc_rspath_cost(&node, &path, &config);
    let expected = 2.0
        + 1.0 * 4.0
        + 25.0
        + 3.0 * config.target_max_steer().abs()
        + 3.0 * config.target_max_steer().abs()
        + 1.5 * config.target_max_steer().abs() * 2.0;
    assert_relative_eq!(cost, expected, epsilon = 1e-12);
}

#[test]
fn explored_segments_follow_neighbour_trajectory_geometry() {
    let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0];
    let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 4000).expect("planner");
    planner.step(1).expect("step");
    let explored = planner.take_explored_segments();

    assert_eq!(explored.len() % 4, 0);
    assert!(explored.len() >= 8);
}

#[test]
fn traceback_matches_python_direction_cleanup() {
    let start = SearchNode {
        ijk: (0, 0, 0),
        trajectory: vec![[0.0, 0.0, 0.0, 0.0]],
        direction: 0,
        steer: 0.0,
        cost: 0.0,
        h_cost: 0.0,
        parent: None,
        analytic_path: None,
    };
    let forward = SearchNode {
        ijk: (0, 1, 0),
        trajectory: vec![[1.0, 0.0, 0.0, 1.0]],
        direction: 1,
        steer: 0.0,
        cost: 1.0,
        h_cost: 0.0,
        parent: Some(Box::new(start)),
        analytic_path: None,
    };
    let backward = SearchNode {
        ijk: (0, 2, 0),
        trajectory: vec![[2.0, 0.0, 0.0, -1.0]],
        direction: -1,
        steer: 0.0,
        cost: 2.0,
        h_cost: 0.0,
        parent: Some(Box::new(forward)),
        analytic_path: None,
    };
    let forward_again = SearchNode {
        ijk: (0, 3, 0),
        trajectory: vec![[3.0, 0.0, 0.0, 1.0]],
        direction: 1,
        steer: 0.0,
        cost: 3.0,
        h_cost: 0.0,
        parent: Some(Box::new(backward)),
        analytic_path: None,
    };

    let trajectory = traceback_path(forward_again);

    assert_eq!(trajectory.len(), 12);
    assert_eq!(trajectory[3], 1.0);
    assert_eq!(trajectory[7], 1.0);
    assert_eq!(trajectory[11], 1.0);
}

#[test]
fn traceback_keeps_exact_parent_chain_when_cells_match() {
    let start = SearchNode {
        ijk: (0, 0, 0),
        trajectory: vec![[0.0, 0.0, 0.0, 0.0]],
        direction: 0,
        steer: 0.0,
        cost: 0.0,
        h_cost: 0.0,
        parent: None,
        analytic_path: None,
    };
    let original_parent = SearchNode {
        ijk: (1, 1, 0),
        trajectory: vec![[1.0, 1.0, 0.1, 1.0]],
        direction: 1,
        steer: 0.1,
        cost: 1.0,
        h_cost: 0.0,
        parent: Some(Box::new(start)),
        analytic_path: None,
    };
    let overwritten_same_cell = SearchNode {
        ijk: (1, 1, 0),
        trajectory: vec![[1.6, 1.4, 0.4, 1.0]],
        direction: 1,
        steer: 0.4,
        cost: 0.8,
        h_cost: 0.0,
        parent: None,
        analytic_path: None,
    };
    let child = SearchNode {
        ijk: (2, 2, 0),
        trajectory: vec![[2.0, 2.0, 0.2, 1.0]],
        direction: 1,
        steer: 0.2,
        cost: 2.0,
        h_cost: 0.0,
        parent: Some(Box::new(original_parent)),
        analytic_path: None,
    };

    let trajectory = traceback_path(child);

    assert_eq!(trajectory.len(), 12);
    assert_relative_eq!(trajectory[4], 1.0, epsilon = 1e-12);
    assert_relative_eq!(trajectory[5], 1.0, epsilon = 1e-12);
    assert_relative_eq!(trajectory[6], 0.1, epsilon = 1e-12);
    assert_ne!(overwritten_same_cell.trajectory[0][0], trajectory[4]);
    assert_ne!(overwritten_same_cell.trajectory[0][1], trajectory[5]);
}

fn box_obstacles(width: f64, height: f64) -> Vec<f64> {
    let mut obstacles = Vec::new();

    for x in 0..=width as i32 {
        let xf = x as f64;
        obstacles.extend_from_slice(&[xf, 0.0, xf, height]);
    }

    for y in 1..height as i32 {
        let yf = y as f64;
        obstacles.extend_from_slice(&[0.0, yf, width, yf]);
    }

    obstacles
}
