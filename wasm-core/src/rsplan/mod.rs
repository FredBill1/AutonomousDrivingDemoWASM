// Copyright header preserved from original rsplan.rs:
// Derived from the rsplan Python library:
// https://github.com/builtrobotics/rsplan/tree/47b3ab572a4b7ffc314dfc2aa68b8349f061fe13
// This Rust translation was completed entirely by GPT-5.4 and is provided
// without any guarantee of correctness.
// Copyright (c) 2023 Built Robotics
// SPDX-License-Identifier: MIT

mod curve_formulas;
mod math;
mod path;
mod path_generators;
mod paths_data;
mod segment;
mod solve;
mod types;

#[cfg(test)]
mod tests;

pub use math::{rs_change_base, rs_euclidean_distance, rs_polar, rs_steering_angles, rs_wrap_to_pi};
pub use path::ReedsSheppPath;
pub use segment::ReedsSheppSegment;
pub use solve::rs_solve_path;
pub use types::SegmentKind;
pub(crate) use solve::{solve_all_paths, solve_best_path};
