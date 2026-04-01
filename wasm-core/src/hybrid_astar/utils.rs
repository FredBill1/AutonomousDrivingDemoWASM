use crate::car::CarConfig;

use super::heuristic::HeuristicGrid;
use super::types::{SearchNode, StartSeedPoint};
use super::{NUM_STEER_COMMANDS, YAW_GRID_RESOLUTION};

pub(crate) fn steer_commands(max_steer: f64) -> Vec<f64> {
    let mut commands = Vec::with_capacity(NUM_STEER_COMMANDS + 1);
    for index in 0..NUM_STEER_COMMANDS {
        let t = index as f64 / (NUM_STEER_COMMANDS - 1) as f64;
        commands.push(-max_steer + (2.0 * max_steer) * t);
    }
    commands.push(0.0);
    commands.sort_by(|left, right| left.total_cmp(right));
    commands.dedup_by(|left, right| (*left - *right).abs() < 1e-9);
    commands
}

pub(crate) fn calc_ijk(x: f64, y: f64, yaw: f64, heuristic: &HeuristicGrid) -> (i32, i32, i32) {
    let (i, j) = heuristic.calc_index(x, y);
    let yaw_bins = (2.0 * std::f64::consts::PI / YAW_GRID_RESOLUTION).round() as i32;
    let wrapped = wrap_zero_to_two_pi(yaw);
    let k = ((wrapped / YAW_GRID_RESOLUTION).floor() as i32).rem_euclid(yaw_bins);
    (i, j, k)
}

pub(crate) fn wrap_angle(angle: f64) -> f64 {
    let mut value = angle;
    while value >= std::f64::consts::PI {
        value -= 2.0 * std::f64::consts::PI;
    }
    while value < -std::f64::consts::PI {
        value += 2.0 * std::f64::consts::PI;
    }
    value
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

pub(crate) fn decode_start_seed(
    flat: &[f64],
) -> Result<Vec<StartSeedPoint>, wasm_bindgen::JsValue> {
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
) -> SearchNode {
    let start_ijk = calc_ijk(start_x, start_y, start_yaw, heuristic);
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
    config: &CarConfig,
) -> Result<SearchNode, wasm_bindgen::JsValue> {
    let last = start_seed
        .last()
        .ok_or_else(|| wasm_bindgen::JsValue::from_str("Start trajectory seed cannot be empty"))?;
    let start_ijk = calc_ijk(last.x, last.y, last.yaw, heuristic);
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
                (config.wheel_base() * delta_yaw / length).atan()
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
