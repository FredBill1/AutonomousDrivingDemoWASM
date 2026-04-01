mod car;
mod geometry;
mod hybrid_astar;
mod mpc_control;
mod mpc_prep;
mod rsplan;

use wasm_bindgen::prelude::*;

pub use car::{CarConfig, CarState, CarUpdateResult, path_check_collision, trajectory_check_collision};
pub use hybrid_astar::{HybridAStarPlanner, HybridAStarResult};
pub use mpc_control::MpcControlResult;
pub use mpc_prep::{MpcReferenceResult, MpcReferenceTracker};
pub use rsplan::{ReedsSheppPath, ReedsSheppSegment, SegmentKind};

#[wasm_bindgen(start)]
pub fn start() {
    console_error_panic_hook::set_once();
}
