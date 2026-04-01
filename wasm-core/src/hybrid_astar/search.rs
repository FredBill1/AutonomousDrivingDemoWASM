use crate::car::CarConfig;
use crate::car::CarState;
use crate::rsplan::{SegmentKind, solve_all_paths};

use super::heuristic::HeuristicGrid;
use super::types::SearchNode;
use super::utils::{calc_ijk, wrap_angle};
use super::{
    BACKWARDS_COST, H_DIST_COST, H_YAW_COST, MOTION_DISTANCE, MOTION_RESOLUTION, STEER_CHANGE_COST, STEER_COST,
    SWITCH_DIRECTION_COST,
};

pub(crate) fn generate_neighbour(
    current: &SearchNode,
    direction: i8,
    steer: f64,
    goal: &[f64; 3],
    config: &CarConfig,
    heuristic: &HeuristicGrid,
    obstacle_coordinates: &[f64],
    start_collided: bool,
) -> Option<SearchNode> {
    let last = current.trajectory.last()?;
    let mut car = CarState::new(last[0], last[1], last[2], direction as f64, steer);
    let mut trajectory = Vec::new();
    let steps = (MOTION_DISTANCE / MOTION_RESOLUTION) as usize;

    for _ in 0..steps {
        car.update(config, MOTION_RESOLUTION);
        if !start_collided && car.check_collision(config, obstacle_coordinates.to_vec()) {
            return None;
        }
        trajectory.push([car.x(), car.y(), car.yaw(), direction as f64]);
    }

    let ijk = calc_ijk(car.x(), car.y(), car.yaw(), heuristic);
    if !heuristic.contains(ijk.0, ijk.1) {
        return None;
    }

    let distance_cost = if direction == 1 {
        MOTION_DISTANCE
    } else {
        MOTION_DISTANCE * BACKWARDS_COST
    };
    let switch_direction_cost = if current.direction != 0 && current.direction != direction {
        SWITCH_DIRECTION_COST
    } else {
        0.0
    };
    let steer_change_cost = STEER_CHANGE_COST * (steer - current.steer).abs();
    let steer_cost = STEER_COST * steer.abs() * MOTION_DISTANCE;
    let cost = current.cost + distance_cost + switch_direction_cost + steer_change_cost + steer_cost;

    let h_cost = H_DIST_COST * heuristic.distance_at(ijk.0, ijk.1) + H_YAW_COST * wrap_angle(goal[2] - car.yaw()).abs();

    Some(SearchNode {
        ijk,
        trajectory,
        direction,
        steer,
        cost,
        h_cost,
        parent: Some(Box::new(current.clone())),
        analytic_path: None,
    })
}

pub(crate) fn generate_rspath(
    node: &SearchNode,
    goal: &[f64; 3],
    config: &CarConfig,
    obstacle_coordinates: &[f64],
) -> Option<SearchNode> {
    let tail = node.trajectory.last()?;
    let mut best_path = None;
    let mut best_cost = f64::INFINITY;

    for path in solve_all_paths(
        (tail[0], tail[1], tail[2]),
        (goal[0], goal[1], goal[2]),
        config.target_min_turning_radius(),
        0.0,
        MOTION_RESOLUTION,
    ) {
        if path_collides(&path, config, obstacle_coordinates) {
            continue;
        }

        let cost = calc_rspath_cost(node, &path, config);
        if cost < best_cost {
            best_cost = cost;
            best_path = Some(path);
        }
    }

    best_path.map(|path| SearchNode {
        ijk: node.ijk,
        trajectory: Vec::new(),
        direction: node.direction,
        steer: node.steer,
        cost: node.cost + best_cost,
        h_cost: 0.0,
        parent: Some(Box::new(node.clone())),
        analytic_path: Some(path),
    })
}

pub(crate) fn path_collides(
    path: &crate::rsplan::ReedsSheppPath,
    config: &CarConfig,
    obstacle_coordinates: &[f64],
) -> bool {
    for waypoint in path.flat_coordinates().chunks_exact(3) {
        let state = CarState::new(waypoint[0], waypoint[1], waypoint[2], 0.0, 0.0);
        if state.check_collision(config, obstacle_coordinates.to_vec()) {
            return true;
        }
    }
    false
}

pub(crate) fn calc_rspath_cost(node: &SearchNode, path: &crate::rsplan::ReedsSheppPath, config: &CarConfig) -> f64 {
    let mut last_direction = node.direction;
    let mut last_steer = node.steer;
    let mut distance_cost = 0.0;
    let mut switch_direction_cost = 0.0;
    let mut steer_change_cost = 0.0;
    let mut steer_cost = 0.0;

    for segment in path.segments() {
        let length = segment.length().abs();
        let direction = if segment.direction() < 0 { -1 } else { 1 };
        distance_cost += if direction == 1 {
            length
        } else {
            length * BACKWARDS_COST
        };
        if last_direction != 0 && direction != last_direction {
            switch_direction_cost += SWITCH_DIRECTION_COST;
        }
        last_direction = direction;

        let steer = match segment.kind() {
            SegmentKind::Left => config.target_max_steer(),
            SegmentKind::Right => -config.target_max_steer(),
            SegmentKind::Straight => 0.0,
        };
        steer_change_cost += STEER_CHANGE_COST * (steer - last_steer).abs();
        last_steer = steer;
        steer_cost += STEER_COST * steer.abs() * length;
    }

    distance_cost + switch_direction_cost + steer_change_cost + steer_cost
}

pub(crate) fn traceback_path(goal_node: SearchNode) -> Vec<f64> {
    let mut segments = Vec::<Vec<[f64; 4]>>::new();
    let mut cursor = Some(goal_node);

    while let Some(node) = cursor {
        if let Some(path) = &node.analytic_path {
            let segment = path
                .flat_waypoints()
                .chunks_exact(6)
                .skip(1)
                .map(|chunk| [chunk[0], chunk[1], chunk[2], chunk[4]])
                .collect::<Vec<_>>();
            segments.push(segment);
        } else {
            segments.push(node.trajectory.clone());
        }

        cursor = node.parent.map(|parent| *parent);
    }

    segments.reverse();
    let mut trajectory = segments.into_iter().flatten().collect::<Vec<_>>();

    if trajectory.len() > 1 {
        trajectory[0][3] = trajectory[1][3];
    } else if let Some(first) = trajectory.first_mut() {
        first[3] = 1.0;
    }

    let mut keep = vec![true; trajectory.len()];
    for index in 1..trajectory.len().saturating_sub(1) {
        let prev_changed = trajectory[index - 1][3] != trajectory[index][3];
        let next_changed = trajectory[index][3] != trajectory[index + 1][3];
        if prev_changed && next_changed {
            keep[index] = false;
        }
    }

    trajectory
        .into_iter()
        .zip(keep)
        .filter_map(|(point, keep_point)| keep_point.then_some(point))
        .flatten()
        .collect()
}

pub(crate) fn push_explored_segment(storage: &mut Vec<f64>, current: &SearchNode, neighbour: &SearchNode) {
    let mut points = Vec::<[f64; 2]>::with_capacity(neighbour.trajectory.len() + 1);
    if let Some(start) = current.trajectory.last() {
        points.push([start[0], start[1]]);
    }
    points.extend(neighbour.trajectory.iter().map(|point| [point[0], point[1]]));

    for pair in points.windows(2) {
        storage.extend_from_slice(&[pair[0][0], pair[0][1], pair[1][0], pair[1][1]]);
    }
}
