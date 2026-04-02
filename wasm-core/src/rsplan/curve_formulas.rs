use std::f64::consts::{FRAC_PI_2, PI};

use super::math::rs_polar;
use super::types::NEAR_ZERO_TOL;

fn non_negative(t: f64, u: f64, v: f64) -> Option<(f64, f64, f64)> {
    if t >= 0.0 && u >= 0.0 && v >= 0.0 {
        Some((t, u, v))
    } else {
        None
    }
}

fn near_zero_ab(a: f64, b: f64) -> bool {
    a.abs() < NEAR_ZERO_TOL && b.abs() < NEAR_ZERO_TOL
}

fn polar_with_offset(x: f64, y: f64, rsin: f64, rcos: f64, x_sign: f64, y_sign: f64) -> (f64, f64) {
    rs_polar(x + x_sign * rsin, y + y_sign * rcos)
}

fn with_radius_threshold<F>(
    coords: (f64, f64),
    turn_radius: f64,
    threshold_multiplier: f64,
    compute: F,
) -> Option<(f64, f64, f64)>
where
    F: Fn(f64, f64, f64, f64) -> Option<(f64, f64, f64)>,
{
    let (r, theta) = coords;
    let threshold = threshold_multiplier * turn_radius;
    if r >= threshold {
        return compute(r, theta, threshold, turn_radius);
    }
    None
}

pub(super) fn csca(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, _turn_radius: f64) -> Option<(f64, f64, f64)> {
    let (r, theta) = rs_polar(x - rsin, y + rcos);
    let v = super::math::wrap_to_pi(phi - theta);
    non_negative(theta, r, v)
}

pub(super) fn cscb(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    with_radius_threshold(polar_with_offset(x, y, rsin, rcos, 1.0, -1.0), turn_radius, 2.0, |r, theta, turn_radius_x2, _| {
        let u = (r * r - turn_radius_x2 * turn_radius_x2).sqrt();
        let alpha = turn_radius_x2.atan2(u);
        let t = super::math::wrap_to_pi(theta + alpha);
        let v = super::math::wrap_to_pi(t - phi);
        non_negative(t, u, v)
    })
}

pub(super) fn c_c_c(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x - rsin;
    let b = y + rcos;
    if near_zero_ab(a, b) {
        return None;
    }
    let (r, theta) = rs_polar(a, b);
    if r < turn_radius_x4 {
        let alpha = (r / turn_radius_x4).acos();
        let t = super::math::wrap_to_pi(FRAC_PI_2 + alpha + theta);
        let u = super::math::wrap_to_pi(PI - 2.0 * alpha);
        let v = super::math::wrap_to_pi(phi - t - u);
        return non_negative(t, u, v);
    }
    None
}

pub(super) fn c_cc(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x - rsin;
    let b = y + rcos;
    if near_zero_ab(a, b) {
        return None;
    }
    let (r, theta) = rs_polar(a, b);
    if r <= turn_radius_x4 {
        let alpha = (r / turn_radius_x4).acos();
        let t = super::math::wrap_to_pi(FRAC_PI_2 + alpha + theta);
        let u = super::math::wrap_to_pi(PI - 2.0 * alpha);
        let v = super::math::wrap_to_pi(t + u - phi);
        return non_negative(t, u, v);
    }
    None
}

pub(super) fn cc_c(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x - rsin;
    let b = y + rcos;
    if near_zero_ab(a, b) {
        return None;
    }
    let (r, theta) = rs_polar(a, b);
    if r <= turn_radius_x4 {
        let u = ((8.0 * turn_radius * turn_radius - r * r) / (8.0 * turn_radius * turn_radius)).acos();
        let mut sin_u = u.sin();
        const SIN_NEAR_ZERO: f64 = 0.001;
        if sin_u.abs() < SIN_NEAR_ZERO {
            sin_u = 0.0;
        }
        if sin_u.abs() < SIN_NEAR_ZERO && r.abs() < SIN_NEAR_ZERO {
            return None;
        }
        let alpha = ((turn_radius_x2 * sin_u) / r).asin();
        let t = super::math::wrap_to_pi(FRAC_PI_2 - alpha + theta);
        let v = super::math::wrap_to_pi(t - u - phi);
        return non_negative(t, u, v);
    }
    None
}

pub(super) fn ccu_cuc(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let turn_radius_x4 = 4.0 * turn_radius;
    let a = x + rsin;
    let b = y - rcos;
    if near_zero_ab(a, b) {
        return None;
    }
    let (r, theta) = rs_polar(a, b);
    if r <= turn_radius_x4 {
        let (t, u, v) = if r > turn_radius_x2 {
            let alpha = ((r / 2.0 - turn_radius) / turn_radius_x2).acos();
            let t = super::math::wrap_to_pi(FRAC_PI_2 + theta - alpha);
            let u = super::math::wrap_to_pi(PI - alpha);
            let v = super::math::wrap_to_pi(phi - t + 2.0 * u);
            (t, u, v)
        } else {
            let alpha = ((r / 2.0 + turn_radius) / turn_radius_x2).acos();
            let t = super::math::wrap_to_pi(FRAC_PI_2 + theta + alpha);
            let u = super::math::wrap_to_pi(alpha);
            let v = super::math::wrap_to_pi(phi - t + 2.0 * u);
            (t, u, v)
        };
        return non_negative(t, u, v);
    }
    None
}

pub(super) fn c_cucu_c(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    let turn_radius_x2 = 2.0 * turn_radius;
    let a = x + rsin;
    let b = y - rcos;
    if near_zero_ab(a, b) {
        return None;
    }
    let (r, theta) = rs_polar(a, b);
    const SIX: f64 = 6.0;
    if r > SIX * turn_radius {
        return None;
    }
    let va = (5.0 * turn_radius * turn_radius - r * r / 4.0) / (turn_radius_x2 * turn_radius_x2);
    if !(0.0..=1.0).contains(&va) {
        return None;
    }
    let u = va.acos();
    let alpha = ((turn_radius_x2 * u.sin()) / r).asin();
    let t = super::math::wrap_to_pi(FRAC_PI_2 + theta + alpha);
    let v = super::math::wrap_to_pi(t - phi);
    non_negative(t, u, v)
}

pub(super) fn c_c2sca(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    with_radius_threshold(polar_with_offset(x, y, rsin, rcos, -1.0, 1.0), turn_radius, 2.0, |r, theta, turn_radius_x2, _| {
        let u = (r * r - turn_radius_x2 * turn_radius_x2).sqrt() - turn_radius_x2;
        if u >= 0.0 {
            let alpha = turn_radius_x2.atan2(u + turn_radius_x2);
            let t = super::math::wrap_to_pi(FRAC_PI_2 + theta + alpha);
            let v = super::math::wrap_to_pi(t + FRAC_PI_2 - phi);
            return non_negative(t, u, v);
        }
        None
    })
}

pub(super) fn c_c2scb(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    with_radius_threshold(polar_with_offset(x, y, rsin, rcos, 1.0, -1.0), turn_radius, 2.0, |r, theta, turn_radius_x2, _| {
        let t = super::math::wrap_to_pi(FRAC_PI_2 + theta);
        let u = r - turn_radius_x2;
        let v = super::math::wrap_to_pi(phi - t - FRAC_PI_2);
        non_negative(t, u, v)
    })
}

pub(super) fn csc2_ca(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    with_radius_threshold(polar_with_offset(x, y, rsin, rcos, -1.0, 1.0), turn_radius, 2.0, |r, theta, turn_radius_x2, _| {
        let u = (r * r - turn_radius_x2 * turn_radius_x2).sqrt() - turn_radius_x2;
        if u >= 0.0 {
            let alpha = (u + turn_radius_x2).atan2(turn_radius_x2);
            let t = super::math::wrap_to_pi(FRAC_PI_2 + theta - alpha);
            let v = super::math::wrap_to_pi(t - FRAC_PI_2 - phi);
            return non_negative(t, u, v);
        }
        None
    })
}

pub(super) fn csc2_cb(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    with_radius_threshold(polar_with_offset(x, y, rsin, rcos, 1.0, -1.0), turn_radius, 2.0, |r, theta, turn_radius_x2, _| {
        let t = super::math::wrap_to_pi(theta);
        let u = r - turn_radius_x2;
        let v = super::math::wrap_to_pi(-t - FRAC_PI_2 + phi);
        non_negative(t, u, v)
    })
}

pub(super) fn c_c2sc2_c(x: f64, y: f64, phi: f64, rsin: f64, rcos: f64, turn_radius: f64) -> Option<(f64, f64, f64)> {
    with_radius_threshold(polar_with_offset(x, y, rsin, rcos, 1.0, -1.0), turn_radius, 4.0, |r, theta, turn_radius_x4, turn_radius| {
        let turn_radius_x2 = 2.0 * turn_radius;
        let u = (r * r - turn_radius_x2 * turn_radius_x2).sqrt() - turn_radius_x4;
        if u >= 0.0 {
            let alpha = turn_radius_x2.atan2(u + turn_radius_x4);
            let t = super::math::wrap_to_pi(FRAC_PI_2 + theta + alpha);
            let v = super::math::wrap_to_pi(t - phi);
            return non_negative(t, u, v);
        }
        None
    })
}
