use std::f64::consts::PI;

pub(super) fn wrap_to_pi(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

pub(super) fn linspace(start: f64, end: f64, count: usize) -> Vec<f64> {
    if count == 0 {
        return Vec::new();
    }
    if count == 1 {
        return vec![end];
    }

    let mut values = Vec::with_capacity(count);
    let step = (end - start) / (count - 1) as f64;
    for index in 0..count {
        values.push(start + step * index as f64);
    }
    if let Some(last) = values.last_mut() {
        *last = end;
    }
    values
}

pub(super) fn py_round_to(value: f64, digits: i32) -> f64 {
    let factor = 10_f64.powi(digits);
    round_ties_even(value * factor) / factor
}

fn round_ties_even(value: f64) -> f64 {
    if !value.is_finite() {
        return value;
    }

    let floor = value.floor();
    let diff = value - floor;
    if diff < 0.5 {
        floor
    } else if diff > 0.5 {
        floor + 1.0
    } else if (floor / 2.0).fract() == 0.0 {
        floor
    } else {
        floor + 1.0
    }
}

use wasm_bindgen::prelude::*;

use crate::geometry::{distance, rotate};

#[wasm_bindgen]
pub fn rs_change_base(start_x: f64, start_y: f64, start_yaw: f64, end_x: f64, end_y: f64, end_yaw: f64) -> Vec<f64> {
    let dx = end_x - start_x;
    let dy = end_y - start_y;
    let (xb, yb) = rotate(dx, dy, -start_yaw);
    vec![xb, yb, end_yaw - start_yaw]
}

#[wasm_bindgen]
pub fn rs_steering_angles(phi: f64, turn_radius: f64) -> Vec<f64> {
    vec![turn_radius * (phi.cos() - 1.0), turn_radius * (phi.cos() + 1.0)]
}

pub fn rs_polar(x: f64, y: f64) -> (f64, f64) {
    (x.hypot(y), y.atan2(x))
}

#[wasm_bindgen]
pub fn rs_wrap_to_pi(angle: f64) -> f64 {
    wrap_to_pi(angle)
}

#[wasm_bindgen]
pub fn rs_euclidean_distance(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    distance(ax, ay, bx, by)
}
