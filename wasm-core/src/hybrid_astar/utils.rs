use crate::car::CarConfig;

use super::config::HybridAStarConfig;
use super::heuristic::HeuristicGrid;
use super::types::{SearchNode, StartSeedPoint};

pub(crate) fn steer_commands(max_steer: f64, num_steer_commands: usize) -> Vec<f64> {
    let mut commands = Vec::with_capacity(num_steer_commands + 1);
    for index in 0..num_steer_commands {
        let t = index as f64 / (num_steer_commands - 1) as f64;
        commands.push(-max_steer + (2.0 * max_steer) * t);
    }
    commands.push(0.0);
    commands.sort_by(|left, right| left.total_cmp(right));
    commands.dedup_by(|left, right| (*left - *right).abs() < 1e-9);
    commands
}

pub(crate) fn calc_ijk(x: f64, y: f64, yaw: f64, heuristic: &HeuristicGrid, yaw_grid_resolution: f64) -> (i32, i32, i32) {
    let (i, j) = heuristic.calc_index(x, y);
    let yaw_bins = (2.0 * std::f64::consts::PI / yaw_grid_resolution).round() as i32;
    let wrapped = wrap_zero_to_two_pi(yaw);
    let k = ((wrapped / yaw_grid_resolution).floor() as i32).rem_euclid(yaw_bins);
    (i, j, k)
}

fn wrap_zero_to_two_pi(angle: f64) -> f64 {
    let mut value = angle % (2.0 * std::f64::consts::PI);
    if value < 0.0 {
        value += 2.0 * std::f64::consts::PI;
    }
    value
}

pub(crate) fn python_sign(value: f64) -> i8 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}

pub(crate) fn decode_start_seed(flat: &[f64]) -> Result<Vec<StartSeedPoint>, wasm_bindgen::JsValue> {
    if !flat.len().is_multiple_of(4) {
        return Err(wasm_bindgen::JsValue::from_str(
            "Start trajectory seed must be flat [x, y, yaw, velocity] data",
        ));
    }

    let mut points = Vec::with_capacity(flat.len() / 4);
    for chunk in flat.chunks_exact(4) {
        points.push(StartSeedPoint {
            x: chunk[0],
            y: chunk[1],
            yaw: chunk[2],
            velocity: chunk[3],
        });
    }
    Ok(remove_duplicate_seed_xy(points))
}

fn remove_duplicate_seed_xy(points: Vec<StartSeedPoint>) -> Vec<StartSeedPoint> {
    let mut deduped = Vec::with_capacity(points.len());
    for point in points {
        let keep = deduped
            .last()
            .map(|last: &StartSeedPoint| (last.x != point.x) || (last.y != point.y))
            .unwrap_or(true);
        if keep {
            deduped.push(point);
        }
    }
    deduped
}

pub(crate) fn build_point_start_node(
    start_x: f64,
    start_y: f64,
    start_yaw: f64,
    heuristic: &HeuristicGrid,
    ha_config: &HybridAStarConfig,
) -> SearchNode {
    let start_ijk = calc_ijk(start_x, start_y, start_yaw, heuristic, ha_config.yaw_grid_resolution);
    SearchNode {
        ijk: start_ijk,
        trajectory: vec![[start_x, start_y, start_yaw, 0.0]],
        direction: 0,
        steer: 0.0,
        cost: 0.0,
        h_cost: 0.0,
        parent: None,
        analytic_path: None,
    }
}

pub(crate) fn build_seed_start_node(
    start_seed: &[StartSeedPoint],
    heuristic: &HeuristicGrid,
    car_config: &CarConfig,
    ha_config: &HybridAStarConfig,
) -> Result<SearchNode, wasm_bindgen::JsValue> {
    let last = start_seed
        .last()
        .ok_or_else(|| wasm_bindgen::JsValue::from_str("Start trajectory seed cannot be empty"))?;
    let start_ijk = calc_ijk(last.x, last.y, last.yaw, heuristic, ha_config.yaw_grid_resolution);
    let direction = python_sign(start_seed[0].velocity);
    let steer = start_seed
        .windows(2)
        .last()
        .and_then(|window| {
            let dx = window[1].x - window[0].x;
            let dy = window[1].y - window[0].y;
            let length = dx.hypot(dy);
            (length > 0.0).then(|| {
                let delta_yaw = window[1].yaw - window[0].yaw;
                (car_config.wheel_base() * delta_yaw / length).atan()
            })
        })
        .unwrap_or(0.0);

    Ok(SearchNode {
        ijk: start_ijk,
        trajectory: start_seed
            .iter()
            .map(|point| [point.x, point.y, point.yaw, direction as f64])
            .collect(),
        direction,
        steer,
        cost: 0.0,
        h_cost: 0.0,
        parent: None,
        analytic_path: None,
    })
}
