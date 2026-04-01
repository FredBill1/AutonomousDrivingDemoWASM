mod heuristic;
mod planner;
mod result;
mod search;
mod types;
mod utils;

#[cfg(test)]
mod tests;

macro_rules! impl_partial_ord_from_ord {
    ($t:ty) => {
        impl PartialOrd for $t {
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }
    };
}
pub(crate) use impl_partial_ord_from_ord;

pub use planner::HybridAStarPlanner;
pub use result::HybridAStarResult;

const XY_GRID_RESOLUTION: f64 = 1.0;
pub(crate) const YAW_GRID_RESOLUTION: f64 = 15.0_f64.to_radians();
const MOTION_DISTANCE: f64 = XY_GRID_RESOLUTION * 1.5;
pub(crate) const MOTION_RESOLUTION: f64 = 0.5;
pub(crate) const NUM_STEER_COMMANDS: usize = 10;
pub(crate) const REEDS_SHEPP_MAX_DISTANCE: f64 = 10.0;

pub(crate) const SWITCH_DIRECTION_COST: f64 = 25.0;
pub(crate) const BACKWARDS_COST: f64 = 4.0;
pub(crate) const STEER_CHANGE_COST: f64 = 3.0;
pub(crate) const STEER_COST: f64 = 1.5;
pub(crate) const H_DIST_COST: f64 = 2.0;
pub(crate) const H_YAW_COST: f64 = 3.0 / 45.0_f64.to_radians();
