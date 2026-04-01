use std::f64::consts::FRAC_PI_2;

use wasm_bindgen::prelude::*;

use super::math::py_round_to;

pub(super) type Pose = (f64, f64, f64);
pub(super) type CurveFormula = fn(f64, f64, f64, f64, f64, f64) -> Option<(f64, f64, f64)>;

pub(super) const NEAR_ZERO_TOL: f64 = 1e-12;
pub(super) const PI_DIVS: [f64; 4] = [FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2, -FRAC_PI_2];

#[wasm_bindgen]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SegmentKind {
    Left,
    Right,
    Straight,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Waypoint {
    pub(super) x: f64,
    pub(super) y: f64,
    pub(super) yaw: f64,
    pub(super) curvature: f64,
    pub(super) driving_direction: i8,
    pub(super) is_runway: bool,
}

impl Waypoint {
    pub(super) fn is_close(&self, other: &Self) -> bool {
        py_round_to(self.x, 5) == py_round_to(other.x, 5)
            && py_round_to(self.y, 5) == py_round_to(other.y, 5)
            && py_round_to(self.yaw, 5) == py_round_to(other.yaw, 5)
    }
}

#[derive(Default)]
pub(super) struct PathParameter {
    pub(super) t: f64,
    pub(super) u: f64,
    pub(super) v: f64,
    pub(super) path_ix: usize,
}

#[derive(Default)]
pub(super) struct PathParameters {
    pub(super) values: Vec<PathParameter>,
}

impl PathParameters {
    pub(super) fn push_from(&mut self, other: PathParameters) {
        self.values.extend(other.values);
    }

    pub(super) fn iter(&self) -> impl Iterator<Item = (usize, f64, f64, f64, usize)> + '_ {
        self.values
            .iter()
            .enumerate()
            .map(|(ix, params)| (ix, params.t, params.u, params.v, params.path_ix))
    }
}
