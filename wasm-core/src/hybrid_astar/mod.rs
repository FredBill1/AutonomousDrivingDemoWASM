mod config;
mod heuristic;
mod planner;
mod result;
mod search;
mod types;
mod utils;

#[cfg(test)]
mod test_support;
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

pub use config::HybridAStarConfig;
pub use planner::HybridAStarPlanner;
pub use result::HybridAStarResult;
