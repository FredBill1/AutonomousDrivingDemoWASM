use std::cmp::Ordering;
use std::collections::BinaryHeap;

use wasm_bindgen::prelude::JsValue;

use crate::car::CarConfig;

use super::config::HybridAStarConfig;

pub(crate) struct HeuristicGrid {
    pub(crate) min_x: f64,
    pub(crate) min_y: f64,
    pub(crate) width: usize,
    pub(crate) height: usize,
    pub(crate) resolution: f64,
    pub(crate) blocked: Vec<bool>,
    pub(crate) distances: Vec<f64>,
}

const HEURISTIC_INITIAL_DISTANCE: f64 = 1e4;

impl HeuristicGrid {
    pub(crate) fn from_obstacles(
        obstacle_coordinates: &[f64],
        goal_x: f64,
        goal_y: f64,
        car_config: &CarConfig,
        ha_config: &HybridAStarConfig,
    ) -> Result<Self, JsValue> {
        if obstacle_coordinates.len() < 4 {
            return Err(JsValue::from_str("Need boundary obstacles before planning"));
        }

        let resolution = ha_config.xy_grid_resolution;
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
        let collision_radius = car_config.collision_length().min(car_config.collision_width()) / 2.0;
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
            distances: vec![HEURISTIC_INITIAL_DISTANCE; cell_count],
        };
        grid.compute_distances(goal_x, goal_y);
        Ok(grid)
    }

    pub(crate) fn compute_distances(&mut self, goal_x: f64, goal_y: f64) {
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

            for di in -1..=1_i32 {
                for dj in -1..=1_i32 {
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

    pub(crate) fn calc_index(&self, x: f64, y: f64) -> (i32, i32) {
        (
            ((y - self.min_y) / self.resolution) as i32,
            ((x - self.min_x) / self.resolution) as i32,
        )
    }

    pub(crate) fn flat_index(&self, row: usize, col: usize) -> usize {
        row * self.width + col
    }

    pub(crate) fn normalize_index(&self, index: i32, size: usize) -> usize {
        if index >= 0 {
            let index = index as usize;
            assert!(index < size, "index out of bounds");
            return index;
        }

        let wrapped = size as i32 + index;
        assert!(wrapped >= 0, "index out of bounds");
        wrapped as usize
    }

    pub(crate) fn contains(&self, row: i32, col: i32) -> bool {
        row >= 0 && col >= 0 && row < self.height as i32 && col < self.width as i32
    }

    pub(crate) fn is_blocked(&self, row: i32, col: i32) -> bool {
        self.blocked[self.flat_index(row as usize, col as usize)]
    }

    pub(crate) fn distance_at(&self, row: i32, col: i32) -> f64 {
        let row = self.normalize_index(row, self.height);
        let col = self.normalize_index(col, self.width);
        self.distances[self.flat_index(row, col)]
    }
}

#[derive(Clone, Copy)]
pub(crate) struct GridEntry {
    pub(super) distance: f64,
    pub(super) row: usize,
    pub(super) col: usize,
}

impl PartialEq for GridEntry {
    fn eq(&self, other: &Self) -> bool {
        self.distance == other.distance && self.row == other.row && self.col == other.col
    }
}

impl Eq for GridEntry {}

super::impl_partial_ord_from_ord!(GridEntry);

impl Ord for GridEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .distance
            .total_cmp(&self.distance)
            .then_with(|| other.row.cmp(&self.row))
            .then_with(|| other.col.cmp(&self.col))
    }
}
