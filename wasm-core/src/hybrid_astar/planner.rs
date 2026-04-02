use crate::car::CarConfig;
use crate::car::CarState;

use super::config::HybridAStarConfig;
use super::heuristic::HeuristicGrid;
use super::search::{generate_neighbour, generate_rspath, push_explored_segment, traceback_path};
use super::types::{QueueEntry, SearchNode, StartSeedPoint};
use super::utils::{build_point_start_node, build_seed_start_node, decode_start_seed, steer_commands};

use std::collections::{BinaryHeap, HashMap};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct HybridAStarPlanner {
    pub(crate) car_config: CarConfig,
    pub(crate) ha_config: HybridAStarConfig,
    pub(crate) heuristic: HeuristicGrid,
    pub(crate) goal: [f64; 3],
    pub(crate) obstacle_coordinates: Vec<f64>,
    pub(crate) open: BinaryHeap<QueueEntry>,
    pub(crate) nodes: HashMap<(i32, i32, i32), SearchNode>,
    pub(crate) explored_segments: Vec<f64>,
    pub(crate) analytic_expansions: usize,
    pub(crate) steer_commands: Vec<f64>,
    pub(crate) start_state: [f64; 3],
    pub(crate) start_collided: bool,
    pub(crate) solved_result: Option<Vec<f64>>,
    pub(crate) finished: bool,
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

            let tail_dist = (tail[0] - self.goal[0]).hypot(tail[1] - self.goal[1]);
            if tail_dist <= self.ha_config.reeds_shepp_max_distance
                && let Some(goal_node) = generate_rspath(
                    &current,
                    &self.goal,
                    &self.car_config,
                    &self.ha_config,
                    &self.obstacle_coordinates,
                )
            {
                self.analytic_expansions += 1;
                self.solved_result = Some(traceback_path(goal_node));
                self.finished = true;
                return Ok(true);
            }

            for &direction in &[1_i8, -1_i8] {
                for &steer in &self.steer_commands {
                    if let Some(neighbour) = generate_neighbour(
                        &current,
                        direction,
                        steer,
                        &self.goal,
                        &self.car_config,
                        &self.ha_config,
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

    pub fn take_result(&mut self) -> Option<super::result::HybridAStarResult> {
        if !self.finished {
            return None;
        }

        let success = self.solved_result.is_some();
        let flat_path = self.solved_result.take().unwrap_or_default();

        Some(super::result::HybridAStarResult::new(
            flat_path,
            std::mem::take(&mut self.explored_segments),
            self.nodes.len(),
            self.analytic_expansions,
            success,
        ))
    }
}

impl HybridAStarPlanner {
    fn build_shared_components(
        goal_x: f64,
        goal_y: f64,
        obstacle_coordinates: &[f64],
    ) -> Result<(CarConfig, HybridAStarConfig, HeuristicGrid), JsValue> {
        let car_config = CarConfig::new();
        let ha_config = HybridAStarConfig::default();
        let heuristic = HeuristicGrid::from_obstacles(obstacle_coordinates, goal_x, goal_y, &car_config, &ha_config)?;
        Ok((car_config, ha_config, heuristic))
    }

    fn goal_collides(car_config: &CarConfig, goal: [f64; 3], obstacle_coordinates: &[f64]) -> bool {
        CarState::new(goal[0], goal[1], goal[2], 0.0, 0.0).check_collision(car_config, obstacle_coordinates.to_vec())
    }

    fn last_seed_state(start_seed: &[StartSeedPoint]) -> Result<[f64; 3], JsValue> {
        start_seed
            .last()
            .map(|last| [last.x, last.y, last.yaw])
            .ok_or_else(|| JsValue::from_str("Start trajectory seed cannot be empty"))
    }

    pub(super) fn from_point_start(
        start_x: f64,
        start_y: f64,
        start_yaw: f64,
        goal_x: f64,
        goal_y: f64,
        goal_yaw: f64,
        obstacle_coordinates: Vec<f64>,
    ) -> Result<HybridAStarPlanner, JsValue> {
        let (car_config, ha_config, heuristic) = Self::build_shared_components(goal_x, goal_y, &obstacle_coordinates)?;
        let start_state = [start_x, start_y, start_yaw];
        let goal = [goal_x, goal_y, goal_yaw];

        if Self::goal_collides(&car_config, goal, &obstacle_coordinates) {
            return Ok(Self::finished_without_path(
                car_config,
                ha_config,
                heuristic,
                start_state,
                goal,
                obstacle_coordinates,
            ));
        }

        let start_node = build_point_start_node(start_x, start_y, start_yaw, &heuristic, &ha_config);
        let start_collided = CarState::new(start_x, start_y, start_yaw, 0.0, 0.0)
            .check_collision(&car_config, obstacle_coordinates.clone());
        Self::from_start_node(
            car_config,
            ha_config,
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

    pub(super) fn from_seed_start(
        start_seed: &[StartSeedPoint],
        goal_x: f64,
        goal_y: f64,
        goal_yaw: f64,
        obstacle_coordinates: Vec<f64>,
    ) -> Result<HybridAStarPlanner, JsValue> {
        let (car_config, ha_config, heuristic) = Self::build_shared_components(goal_x, goal_y, &obstacle_coordinates)?;
        let goal = [goal_x, goal_y, goal_yaw];
        let start_state = start_seed
            .last()
            .map_or([0.0, 0.0, 0.0], |last| [last.x, last.y, last.yaw]);

        if Self::goal_collides(&car_config, goal, &obstacle_coordinates) {
            return Ok(Self::finished_without_path(
                car_config,
                ha_config,
                heuristic,
                start_state,
                goal,
                obstacle_coordinates,
            ));
        }

        let start_node = build_seed_start_node(start_seed, &heuristic, &car_config, &ha_config)?;
        let start_state = Self::last_seed_state(start_seed)?;
        Self::from_start_node(
            car_config,
            ha_config,
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

    #[allow(clippy::too_many_arguments)]
    pub(super) fn from_start_node(
        car_config: CarConfig,
        ha_config: HybridAStarConfig,
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
            h_cost: ha_config.h_dist_cost * heuristic.distance_at(start_ijk.0, start_ijk.1),
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
            steer_commands: steer_commands(car_config.target_max_steer(), ha_config.num_steer_commands as usize),
            car_config,
            ha_config,
            heuristic,
            goal: [goal_x, goal_y, goal_yaw],
            obstacle_coordinates,
            open,
            nodes,
            explored_segments: Vec::new(),
            analytic_expansions: 0,
            start_state,
            start_collided,
            solved_result: None,
            finished: false,
        })
    }

    pub(super) fn finished_without_path(
        car_config: CarConfig,
        ha_config: HybridAStarConfig,
        heuristic: HeuristicGrid,
        start_state: [f64; 3],
        goal: [f64; 3],
        obstacle_coordinates: Vec<f64>,
    ) -> HybridAStarPlanner {
        Self {
            car_config,
            ha_config,
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
