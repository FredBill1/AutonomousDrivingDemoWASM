use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Clone, Copy, Debug)]
pub struct CarConfig {
    wheel_base: f64,
    length: f64,
    width: f64,
    back_to_wheel: f64,
    target_max_steer: f64,
    max_steer: f64,
    max_steer_speed: f64,
    max_speed: f64,
    min_speed: f64,
    max_accel: f64,
    max_centripetal_accel: f64,
    target_speed: f64,
    scan_radius: f64,
}

#[wasm_bindgen]
impl CarConfig {
    #[wasm_bindgen(constructor)]
    pub fn new() -> CarConfig {
        Self::default()
    }
}

wasm_getters!(CarConfig {
    wheel_base(this) -> f64 => this.wheel_base;
    length(this) -> f64 => this.length;
    width(this) -> f64 => this.width;
    back_to_wheel(this) -> f64 => this.back_to_wheel;
    wheel_length(_this) -> f64 => 0.8;
    wheel_width(_this) -> f64 => 0.5;
    wheel_spacing(_this) -> f64 => 1.4;
    target_max_steer(this) -> f64 => this.target_max_steer;
    max_steer(this) -> f64 => this.max_steer;
    max_steer_speed(this) -> f64 => this.max_steer_speed;
    max_speed(this) -> f64 => this.max_speed;
    min_speed(this) -> f64 => this.min_speed;
    max_accel(this) -> f64 => this.max_accel;
    max_centripetal_accel(this) -> f64 => this.max_centripetal_accel;
    target_speed(this) -> f64 => this.target_speed;
    scan_radius(this) -> f64 => this.scan_radius;
    back_to_center(this) -> f64 => this.length / 2.0 - this.back_to_wheel;
    collision_length(this) -> f64 => this.length + 0.5;
    collision_width(this) -> f64 => this.width + 0.5;
    collision_radius(this) -> f64 => (this.collision_width() / 2.0).hypot(this.collision_length() / 2.0);
    target_min_turning_radius(this) -> f64 => this.wheel_base / this.target_max_steer.tan();
});

impl Default for CarConfig {
    fn default() -> Self {
        Self {
            wheel_base: 2.5,
            length: 4.5,
            width: 2.0,
            back_to_wheel: 1.0,
            target_max_steer: 35.0_f64.to_radians(),
            max_steer: 40.0_f64.to_radians(),
            max_steer_speed: 360.0_f64.to_radians(),
            max_speed: 55.0 / 3.6,
            min_speed: -30.0 / 3.6,
            max_accel: 15.0,
            max_centripetal_accel: 16.0,
            target_speed: 40.0 / 3.6,
            scan_radius: 15.0,
        }
    }
}
