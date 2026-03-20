use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use wasm_bindgen::prelude::*;

use crate::car::{CarConfig, CarState};
use crate::rsplan::{solve_all_paths, ReedsSheppPath, SegmentKind};

const XY_GRID_RESOLUTION: f64 = 1.0;
const YAW_GRID_RESOLUTION: f64 = 15.0_f64.to_radians();
const MOTION_DISTANCE: f64 = XY_GRID_RESOLUTION * 1.5;
const MOTION_RESOLUTION: f64 = 0.5;
const NUM_STEER_COMMANDS: usize = 10;
const REEDS_SHEPP_MAX_DISTANCE: f64 = 10.0;

const SWITCH_DIRECTION_COST: f64 = 25.0;
const BACKWARDS_COST: f64 = 4.0;
const STEER_CHANGE_COST: f64 = 3.0;
const STEER_COST: f64 = 1.5;
const H_DIST_COST: f64 = 2.0;
const H_YAW_COST: f64 = 3.0 / 45.0_f64.to_radians();

#[derive(Clone, Debug)]
struct SearchNode {
    ijk: (i32, i32, i32),
    trajectory: Vec<[f64; 4]>,
    direction: i8,
    steer: f64,
    cost: f64,
    h_cost: f64,
    parent: Option<Box<SearchNode>>,
    analytic_path: Option<crate::rsplan::ReedsSheppPath>,
}

#[derive(Clone, Copy)]
struct StartSeedPoint {
    x: f64,
    y: f64,
    yaw: f64,
    velocity: f64,
}

#[derive(Clone, Copy, Debug)]
struct QueueEntry {
    priority: f64,
    cost: f64,
    ijk: (i32, i32, i32),
}

impl PartialEq for QueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.cost == other.cost && self.ijk == other.ijk
    }
}

impl Eq for QueueEntry {}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .total_cmp(&self.priority)
            .then_with(|| other.cost.total_cmp(&self.cost))
    }
}

#[wasm_bindgen]
pub struct HybridAStarResult {
    flat_path: Vec<f64>,
    explored_segments: Vec<f64>,
    explored_count: usize,
    analytic_expansions: usize,
    success: bool,
}

#[wasm_bindgen]
impl HybridAStarResult {
    #[wasm_bindgen(getter)]
    pub fn flat_path(&self) -> Vec<f64> {
        self.flat_path.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn explored_segments(&self) -> Vec<f64> {
        self.explored_segments.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn explored_count(&self) -> usize {
        self.explored_count
    }

    #[wasm_bindgen(getter)]
    pub fn analytic_expansions(&self) -> usize {
        self.analytic_expansions
    }

    #[wasm_bindgen(getter)]
    pub fn success(&self) -> bool {
        self.success
    }
}

#[wasm_bindgen]
pub struct HybridAStarPlanner {
    config: CarConfig,
    heuristic: HeuristicGrid,
    goal: [f64; 3],
    obstacle_coordinates: Vec<f64>,
    open: BinaryHeap<QueueEntry>,
    nodes: HashMap<(i32, i32, i32), SearchNode>,
    explored_segments: Vec<f64>,
    analytic_expansions: usize,
    steer_commands: Vec<f64>,
    start_state: [f64; 3],
    start_collided: bool,
    solved_result: Option<Vec<f64>>,
    finished: bool,
}

#[wasm_bindgen]
impl HybridAStarPlanner {
    #[wasm_bindgen(constructor)]
    pub fn new(
        start_x: f64,
        start_y: f64,
        start_yaw: f64,
        goal_x: f64,
        goal_y: f64,
        goal_yaw: f64,
        obstacle_coordinates: Vec<f64>,
        _max_iterations: usize,
    ) -> Result<HybridAStarPlanner, JsValue> {
        Self::from_point_start(
            start_x,
            start_y,
            start_yaw,
            goal_x,
            goal_y,
            goal_yaw,
            obstacle_coordinates,
        )
    }

    #[wasm_bindgen(js_name = from_trajectory_seed)]
    pub fn from_trajectory_seed(
        flat_start_seed: Vec<f64>,
        goal_x: f64,
        goal_y: f64,
        goal_yaw: f64,
        obstacle_coordinates: Vec<f64>,
        _max_iterations: usize,
    ) -> Result<HybridAStarPlanner, JsValue> {
        let seed = decode_start_seed(&flat_start_seed)?;
        Self::from_seed_start(&seed, goal_x, goal_y, goal_yaw, obstacle_coordinates)
    }

    #[wasm_bindgen(getter)]
    pub fn explored_count(&self) -> usize {
        self.nodes.len()
    }

    #[wasm_bindgen(getter)]
    pub fn analytic_expansions(&self) -> usize {
        self.analytic_expansions
    }

    #[wasm_bindgen(getter)]
    pub fn is_finished(&self) -> bool {
        self.finished
    }

    pub fn step(&mut self, iteration_budget: usize) -> Result<bool, JsValue> {
        if self.finished {
            return Ok(true);
        }

        for _ in 0..iteration_budget {
            let Some(entry) = self.open.pop() else {
                self.finished = true;
                return Ok(true);
            };

            let current = match self.nodes.get(&entry.ijk).cloned() {
                Some(node) => node,
                None => continue,
            };

            if entry.cost > current.cost {
                continue;
            }

            let tail = current.trajectory.last().copied().unwrap_or([
                self.start_state[0],
                self.start_state[1],
                self.start_state[2],
                0.0,
            ]);

            if distance_xy(tail[0], tail[1], self.goal[0], self.goal[1]) <= REEDS_SHEPP_MAX_DISTANCE
            {
                if let Some(goal_node) = generate_rspath(
                    &current,
                    &self.goal,
                    &self.config,
                    &self.obstacle_coordinates,
                ) {
                    self.analytic_expansions += 1;
                    self.solved_result = Some(traceback_path(goal_node));
                    self.finished = true;
                    return Ok(true);
                }
            }

            for &direction in &[1_i8, -1_i8] {
                for &steer in &self.steer_commands {
                    if let Some(neighbour) = generate_neighbour(
                        &current,
                        direction,
                        steer,
                        &self.goal,
                        &self.config,
                        &self.heuristic,
                        &self.obstacle_coordinates,
                        self.start_collided,
                    ) {
                        push_explored_segment(&mut self.explored_segments, &current, &neighbour);
                        let needs_update = match self.nodes.get(&neighbour.ijk) {
                            Some(existing) => neighbour.cost < existing.cost,
                            None => true,
                        };

                        if needs_update {
                            self.open.push(QueueEntry {
                                priority: neighbour.cost + neighbour.h_cost,
                                cost: neighbour.cost,
                                ijk: neighbour.ijk,
                            });
                            self.nodes.insert(neighbour.ijk, neighbour);
                        }
                    }
                }
            }

            self.start_collided = false;
        }

        Ok(self.finished)
    }

    pub fn take_explored_segments(&mut self) -> Vec<f64> {
        std::mem::take(&mut self.explored_segments)
    }

    pub fn take_result(&mut self) -> Option<HybridAStarResult> {
        if !self.finished {
            return None;
        }

        let success = self.solved_result.is_some();
        let flat_path = self.solved_result.take().unwrap_or_default();

        Some(HybridAStarResult {
            flat_path,
            explored_segments: std::mem::take(&mut self.explored_segments),
            explored_count: self.nodes.len(),
            analytic_expansions: self.analytic_expansions,
            success,
        })
    }
}

impl HybridAStarPlanner {
    fn from_point_start(
        start_x: f64,
        start_y: f64,
        start_yaw: f64,
        goal_x: f64,
        goal_y: f64,
        goal_yaw: f64,
        obstacle_coordinates: Vec<f64>,
    ) -> Result<HybridAStarPlanner, JsValue> {
        let config = CarConfig::new();
        let heuristic =
            HeuristicGrid::from_obstacles(&obstacle_coordinates, goal_x, goal_y, &config)?;
        let start_state = [start_x, start_y, start_yaw];

        if CarState::new(goal_x, goal_y, goal_yaw, 0.0, 0.0)
            .check_collision(&config, obstacle_coordinates.clone())
        {
            return Ok(Self::finished_without_path(
                config,
                heuristic,
                start_state,
                [goal_x, goal_y, goal_yaw],
                obstacle_coordinates,
            ));
        }

        let start_node = build_point_start_node(start_x, start_y, start_yaw, &heuristic);
        let start_collided = CarState::new(start_x, start_y, start_yaw, 0.0, 0.0)
            .check_collision(&config, obstacle_coordinates.clone());
        Self::from_start_node(
            config,
            heuristic,
            start_state,
            start_node,
            start_collided,
            goal_x,
            goal_y,
            goal_yaw,
            obstacle_coordinates,
        )
    }

    fn from_seed_start(
        start_seed: &[StartSeedPoint],
        goal_x: f64,
        goal_y: f64,
        goal_yaw: f64,
        obstacle_coordinates: Vec<f64>,
    ) -> Result<HybridAStarPlanner, JsValue> {
        let config = CarConfig::new();
        let heuristic =
            HeuristicGrid::from_obstacles(&obstacle_coordinates, goal_x, goal_y, &config)?;

        if CarState::new(goal_x, goal_y, goal_yaw, 0.0, 0.0)
            .check_collision(&config, obstacle_coordinates.clone())
        {
            let start_state = start_seed
                .last()
                .map(|last| [last.x, last.y, last.yaw])
                .unwrap_or([0.0, 0.0, 0.0]);
            return Ok(Self::finished_without_path(
                config,
                heuristic,
                start_state,
                [goal_x, goal_y, goal_yaw],
                obstacle_coordinates,
            ));
        }

        let start_node = build_seed_start_node(start_seed, &heuristic, &config)?;
        let last = start_seed
            .last()
            .ok_or_else(|| JsValue::from_str("Start trajectory seed cannot be empty"))?;
        let start_state = [last.x, last.y, last.yaw];
        Self::from_start_node(
            config,
            heuristic,
            start_state,
            start_node,
            false,
            goal_x,
            goal_y,
            goal_yaw,
            obstacle_coordinates,
        )
    }

    fn from_start_node(
        config: CarConfig,
        heuristic: HeuristicGrid,
        start_state: [f64; 3],
        start_node: SearchNode,
        start_collided: bool,
        goal_x: f64,
        goal_y: f64,
        goal_yaw: f64,
        obstacle_coordinates: Vec<f64>,
    ) -> Result<HybridAStarPlanner, JsValue> {
        let start_ijk = start_node.ijk;
        let start_node = SearchNode {
            h_cost: H_DIST_COST * heuristic.distance_at(start_ijk.0, start_ijk.1),
            ..start_node
        };

        let mut open = BinaryHeap::new();
        open.push(QueueEntry {
            priority: start_node.cost + start_node.h_cost,
            cost: start_node.cost,
            ijk: start_ijk,
        });

        let mut nodes = HashMap::<(i32, i32, i32), SearchNode>::new();
        nodes.insert(start_ijk, start_node);

        Ok(Self {
            config,
            heuristic,
            goal: [goal_x, goal_y, goal_yaw],
            obstacle_coordinates,
            open,
            nodes,
            explored_segments: Vec::new(),
            analytic_expansions: 0,
            steer_commands: steer_commands(config.target_max_steer()),
            start_state,
            start_collided,
            solved_result: None,
            finished: false,
        })
    }

    fn finished_without_path(
        config: CarConfig,
        heuristic: HeuristicGrid,
        start_state: [f64; 3],
        goal: [f64; 3],
        obstacle_coordinates: Vec<f64>,
    ) -> HybridAStarPlanner {
        Self {
            config,
            heuristic,
            goal,
            obstacle_coordinates,
            open: BinaryHeap::new(),
            nodes: HashMap::new(),
            explored_segments: Vec::new(),
            analytic_expansions: 0,
            steer_commands: Vec::new(),
            start_state,
            start_collided: false,
            solved_result: None,
            finished: true,
        }
    }
}

fn generate_neighbour(
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
    let cost =
        current.cost + distance_cost + switch_direction_cost + steer_change_cost + steer_cost;

    let h_cost = H_DIST_COST * heuristic.distance_at(ijk.0, ijk.1)
        + H_YAW_COST * wrap_angle(goal[2] - car.yaw()).abs();

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

fn decode_start_seed(flat: &[f64]) -> Result<Vec<StartSeedPoint>, JsValue> {
    if !flat.len().is_multiple_of(4) {
        return Err(JsValue::from_str(
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

fn build_point_start_node(
    start_x: f64,
    start_y: f64,
    start_yaw: f64,
    heuristic: &HeuristicGrid,
) -> SearchNode {
    let start_ijk = calc_ijk(start_x, start_y, start_yaw, &heuristic);
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

fn build_seed_start_node(
    start_seed: &[StartSeedPoint],
    heuristic: &HeuristicGrid,
    config: &CarConfig,
) -> Result<SearchNode, JsValue> {
    let last = start_seed
        .last()
        .ok_or_else(|| JsValue::from_str("Start trajectory seed cannot be empty"))?;
    let start_ijk = calc_ijk(last.x, last.y, last.yaw, &heuristic);
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

fn traceback_path(goal_node: SearchNode) -> Vec<f64> {
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
        .flat_map(|point| point)
        .collect()
}

fn push_explored_segment(storage: &mut Vec<f64>, current: &SearchNode, neighbour: &SearchNode) {
    let mut points = Vec::<[f64; 2]>::with_capacity(neighbour.trajectory.len() + 1);
    if let Some(start) = current.trajectory.last() {
        points.push([start[0], start[1]]);
    }
    points.extend(
        neighbour
            .trajectory
            .iter()
            .map(|point| [point[0], point[1]]),
    );

    for pair in points.windows(2) {
        storage.extend_from_slice(&[pair[0][0], pair[0][1], pair[1][0], pair[1][1]]);
    }
}

fn generate_rspath(
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

fn python_sign(value: f64) -> i8 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}

fn calc_rspath_cost(node: &SearchNode, path: &ReedsSheppPath, config: &CarConfig) -> f64 {
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

fn steer_commands(max_steer: f64) -> Vec<f64> {
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

fn path_collides(
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

fn calc_ijk(x: f64, y: f64, yaw: f64, heuristic: &HeuristicGrid) -> (i32, i32, i32) {
    let (i, j) = heuristic.calc_index(x, y);
    let yaw_bins = (2.0 * std::f64::consts::PI / YAW_GRID_RESOLUTION).round() as i32;
    let wrapped = wrap_zero_to_two_pi(yaw);
    let k = ((wrapped / YAW_GRID_RESOLUTION).floor() as i32).rem_euclid(yaw_bins);
    (i, j, k)
}

fn wrap_angle(angle: f64) -> f64 {
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

fn distance_xy(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    (ax - bx).hypot(ay - by)
}

struct HeuristicGrid {
    min_x: f64,
    min_y: f64,
    width: usize,
    height: usize,
    resolution: f64,
    blocked: Vec<bool>,
    distances: Vec<f64>,
}

impl HeuristicGrid {
    fn from_obstacles(
        obstacle_coordinates: &[f64],
        goal_x: f64,
        goal_y: f64,
        config: &CarConfig,
    ) -> Result<Self, JsValue> {
        if obstacle_coordinates.len() < 4 {
            return Err(JsValue::from_str("Need boundary obstacles before planning"));
        }

        let resolution = XY_GRID_RESOLUTION;
        let half_res = resolution / 2.0;
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for chunk in obstacle_coordinates.chunks_exact(2) {
            min_x = min_x.min(chunk[0]);
            max_x = max_x.max(chunk[0]);
            min_y = min_y.min(chunk[1]);
            max_y = max_y.max(chunk[1]);
        }

        min_x -= half_res;
        min_y -= half_res;
        max_x += half_res;
        max_y += half_res;

        let width = ((max_x - min_x) / resolution).round().max(1.0) as usize;
        let height = ((max_y - min_y) / resolution).round().max(1.0) as usize;
        let max_x = min_x + width as f64 * resolution;
        let max_y = min_y + height as f64 * resolution;
        let cell_count = width * height;
        let collision_radius = config.collision_length().min(config.collision_width()) / 2.0;
        let query_radius_sq = (collision_radius + resolution).powi(2);
        let collision_radius_sq = collision_radius.powi(2);

        let mut blocked = vec![false; cell_count];
        for row in 0..height {
            for col in 0..width {
                let x = min_x + half_res + col as f64 * resolution;
                let y = min_y + half_res + row as f64 * resolution;
                if x >= max_x || y >= max_y {
                    continue;
                }

                let mut nearest_sq = f64::INFINITY;
                for obstacle in obstacle_coordinates.chunks_exact(2) {
                    let dx = x - obstacle[0];
                    let dy = y - obstacle[1];
                    let distance_sq = dx * dx + dy * dy;
                    if distance_sq <= query_radius_sq && distance_sq < nearest_sq {
                        nearest_sq = distance_sq;
                    }
                }

                blocked[row * width + col] = nearest_sq <= collision_radius_sq;
            }
        }

        let mut grid = Self {
            min_x,
            min_y,
            width,
            height,
            resolution,
            blocked,
            distances: vec![1e4; cell_count],
        };
        grid.compute_distances(goal_x, goal_y);
        Ok(grid)
    }

    fn compute_distances(&mut self, goal_x: f64, goal_y: f64) {
        let goal = self.calc_index(goal_x, goal_y);
        let goal_row = self.normalize_index(goal.0, self.height);
        let goal_col = self.normalize_index(goal.1, self.width);

        let mut open = BinaryHeap::new();
        let goal_idx = self.flat_index(goal_row, goal_col);
        self.distances[goal_idx] = 0.0;
        open.push(GridEntry {
            distance: 0.0,
            row: goal_row,
            col: goal_col,
        });

        while let Some(entry) = open.pop() {
            let idx = self.flat_index(entry.row, entry.col);
            if entry.distance > self.distances[idx] {
                continue;
            }

            for di in -1..=1 {
                for dj in -1..=1 {
                    if di == 0 && dj == 0 {
                        continue;
                    }
                    let nr = entry.row as i32 + di;
                    let nc = entry.col as i32 + dj;
                    if !self.contains(nr, nc) || self.is_blocked(nr, nc) {
                        continue;
                    }
                    let step = ((di * di + dj * dj) as f64).sqrt();
                    let next_idx = self.flat_index(nr as usize, nc as usize);
                    let next_distance = entry.distance + step;
                    if next_distance < self.distances[next_idx] {
                        self.distances[next_idx] = next_distance;
                        open.push(GridEntry {
                            distance: next_distance,
                            row: nr as usize,
                            col: nc as usize,
                        });
                    }
                }
            }
        }
    }

    fn calc_index(&self, x: f64, y: f64) -> (i32, i32) {
        (
            ((y - self.min_y) / self.resolution) as i32,
            ((x - self.min_x) / self.resolution) as i32,
        )
    }

    fn flat_index(&self, row: usize, col: usize) -> usize {
        row * self.width + col
    }

    fn normalize_index(&self, index: i32, size: usize) -> usize {
        if index >= 0 {
            let index = index as usize;
            assert!(index < size, "index out of bounds");
            return index;
        }

        let wrapped = size as i32 + index;
        assert!(wrapped >= 0, "index out of bounds");
        wrapped as usize
    }

    fn contains(&self, row: i32, col: i32) -> bool {
        row >= 0 && col >= 0 && row < self.height as i32 && col < self.width as i32
    }

    fn is_blocked(&self, row: i32, col: i32) -> bool {
        self.blocked[self.flat_index(row as usize, col as usize)]
    }

    fn distance_at(&self, row: i32, col: i32) -> f64 {
        let row = self.normalize_index(row, self.height);
        let col = self.normalize_index(col, self.width);
        self.distances[self.flat_index(row, col)]
    }
}

#[derive(Clone, Copy)]
struct GridEntry {
    distance: f64,
    row: usize,
    col: usize,
}

impl PartialEq for GridEntry {
    fn eq(&self, other: &Self) -> bool {
        self.distance == other.distance && self.row == other.row && self.col == other.col
    }
}

impl Eq for GridEntry {}

impl PartialOrd for GridEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for GridEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .distance
            .total_cmp(&self.distance)
            .then_with(|| other.row.cmp(&self.row))
            .then_with(|| other.col.cmp(&self.col))
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::rsplan::{ReedsSheppPath, ReedsSheppSegment, SegmentKind};

    use super::{
        calc_ijk, calc_rspath_cost, generate_neighbour, python_sign, traceback_path, HeuristicGrid,
        HybridAStarPlanner, SearchNode,
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
        assert!(planner
            .steer_commands
            .iter()
            .all(|steer| steer.abs() <= 35.0_f64.to_radians() + 1e-12));
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
                true, true, false, true, true, true, true, true, false, true, false, true, true,
                false, false, true, false, false, false, true, true, true, false, true, true,
            ]
        );
    }

    #[test]
    fn planner_returns_success_result_in_empty_box() {
        let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0];
        let mut planner =
            HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 4000).expect("path");
        while !planner.step(256).expect("step") {}
        let result = planner.take_result().expect("result");
        assert!(result.flat_path().len() > 4);
        assert!(result.explored_count() > 0);
        assert!(result.success());
    }

    #[test]
    fn goal_collision_returns_finished_empty_result() {
        let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0, 15.0, 15.0];
        let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 4000)
            .expect("planner");

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

        let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 10.0, 2.0, 0.0, obstacles, 4000)
            .expect("planner");
        while !planner.step(256).expect("step") {}

        let result = planner.take_result().expect("result");
        assert!(!result.success());
        assert!(result.flat_path().is_empty());
    }

    #[test]
    fn planner_steps_emit_explored_segments_before_finish() {
        let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0];
        let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 4000)
            .expect("planner");
        let finished = planner.step(4).expect("step");
        let explored = planner.take_explored_segments();

        assert!(!finished || planner.is_finished());
        assert!(!explored.is_empty());
    }

    #[test]
    fn point_start_collision_keeps_first_escape_rollout() {
        let obstacles = vec![0.0, 0.0, 0.0, 20.0, 20.0, 0.0, 20.0, 20.0, 0.8, 2.0];
        let planner =
            HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 1).expect("planner");
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

        let neighbour = generate_neighbour(
            &current, 1, 0.0, &goal, &config, &heuristic, &obstacles, false,
        );

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
        let mut planner =
            HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 1).expect("planner");

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
        let mut planner = HybridAStarPlanner::new(2.0, 2.0, 0.0, 15.0, 15.0, 0.0, obstacles, 4000)
            .expect("planner");
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
}
