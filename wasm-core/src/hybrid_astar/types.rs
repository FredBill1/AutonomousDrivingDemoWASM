use std::cmp::Ordering;

pub(crate) struct SearchNode {
    pub(crate) ijk: (i32, i32, i32),
    pub(crate) trajectory: Vec<[f64; 4]>,
    pub(crate) direction: i8,
    pub(crate) steer: f64,
    pub(crate) cost: f64,
    pub(crate) h_cost: f64,
    pub(crate) parent: Option<Box<SearchNode>>,
    pub(crate) analytic_path: Option<crate::rsplan::ReedsSheppPath>,
}

impl Clone for SearchNode {
    fn clone(&self) -> Self {
        Self {
            ijk: self.ijk,
            trajectory: self.trajectory.clone(),
            direction: self.direction,
            steer: self.steer,
            cost: self.cost,
            h_cost: self.h_cost,
            parent: self.parent.clone(),
            analytic_path: self.analytic_path.clone(),
        }
    }
}

impl std::fmt::Debug for SearchNode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SearchNode")
            .field("ijk", &self.ijk)
            .field("direction", &self.direction)
            .field("cost", &self.cost)
            .finish()
    }
}

pub(crate) struct StartSeedPoint {
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) yaw: f64,
    pub(crate) velocity: f64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct QueueEntry {
    pub(crate) priority: f64,
    pub(crate) cost: f64,
    pub(crate) ijk: (i32, i32, i32),
}

impl PartialEq for QueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.cost == other.cost && self.ijk == other.ijk
    }
}

impl Eq for QueueEntry {}

super::impl_partial_ord_from_ord!(QueueEntry);

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .total_cmp(&self.priority)
            .then_with(|| other.cost.total_cmp(&self.cost))
    }
}
