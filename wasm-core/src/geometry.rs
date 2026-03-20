pub fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

pub fn wrap_angle(radian: f64) -> f64 {
    let mut ret = radian.rem_euclid(std::f64::consts::TAU);
    if ret >= std::f64::consts::PI {
        ret -= std::f64::consts::TAU;
    }
    ret
}

pub fn distance(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    (ax - bx).hypot(ay - by)
}

pub fn rotate(x: f64, y: f64, yaw: f64) -> (f64, f64) {
    let s = yaw.sin();
    let c = yaw.cos();
    (x * c - y * s, x * s + y * c)
}
