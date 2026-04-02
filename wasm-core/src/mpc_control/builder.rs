use super::config::MpcConfig;
use super::matrix_utils::{
    add_difference_penalty, add_quadratic_term, control_index, decode_controls, decode_states, final_state_weight,
    get_linear_model_matrix, push_difference_bound, push_entry, push_symmetric_bound, state_index, state_weight,
};
use super::types::{Control, ModelState, NU, NX};
use crate::car::CarConfig;
use clarabel::{algebra::*, solver::*};

pub(crate) fn linear_mpc_control(
    xref: &[ModelState],
    xbar: &[ModelState],
    last_steer: f64,
    dt: f64,
    mpc_config: &MpcConfig,
    car_config: &CarConfig,
) -> Option<(Vec<Control>, Vec<ModelState>)> {
    let problem = build_mpc_problem(xref, xbar, last_steer, dt, mpc_config, car_config);

    let settings = DefaultSettings::<f64> {
        verbose: false,
        max_iter: 50,
        ..Default::default()
    };

    let mut solver =
        DefaultSolver::new(&problem.p, &problem.q, &problem.a, &problem.b, &problem.cones, settings).ok()?;

    solver.solve();

    if !matches!(
        solver.solution.status,
        SolverStatus::Solved | SolverStatus::AlmostSolved
    ) {
        return None;
    }

    let horizon = mpc_config.horizon_length as usize;
    Some((
        decode_controls(&solver.solution.x, horizon),
        decode_states(&solver.solution.x, horizon),
    ))
}

struct MpcProblem {
    p: CscMatrix<f64>,
    q: Vec<f64>,
    a: CscMatrix<f64>,
    b: Vec<f64>,
    cones: Vec<SupportedConeT<f64>>,
}

struct ConstraintBuilder {
    rows: Vec<usize>,
    cols: Vec<usize>,
    vals: Vec<f64>,
    b: Vec<f64>,
    row: usize,
}

impl ConstraintBuilder {
    fn new(total_rows: usize) -> Self {
        Self {
            rows: Vec::new(),
            cols: Vec::new(),
            vals: Vec::new(),
            b: Vec::with_capacity(total_rows),
            row: 0,
        }
    }

    fn push_row_entry(&mut self, column: usize, value: f64) {
        push_entry(&mut self.rows, &mut self.cols, &mut self.vals, self.row, column, value);
    }

    fn finish_row(&mut self, rhs: f64) {
        self.b.push(rhs);
        self.row += 1;
    }

    fn push_scalar_row(&mut self, column: usize, value: f64, rhs: f64) {
        self.push_row_entry(column, value);
        self.finish_row(rhs);
    }

    fn push_symmetric_bound_row(&mut self, column: usize, bound: f64) {
        push_symmetric_bound(
            &mut self.rows,
            &mut self.cols,
            &mut self.vals,
            &mut self.b,
            &mut self.row,
            column,
            bound,
        );
    }

    fn push_difference_bound_row(&mut self, left: usize, right: usize, bound: f64) {
        push_difference_bound(
            &mut self.rows,
            &mut self.cols,
            &mut self.vals,
            &mut self.b,
            &mut self.row,
            left,
            right,
            bound,
        );
    }

    fn finish(self, total_rows: usize, nvars: usize) -> (CscMatrix<f64>, Vec<f64>) {
        (
            CscMatrix::new_from_triplets(total_rows, nvars, self.rows, self.cols, self.vals),
            self.b,
        )
    }
}

#[allow(clippy::needless_range_loop)]
fn add_dynamics_constraints(
    constraints: &mut ConstraintBuilder,
    horizon: usize,
    xbar: &[ModelState],
    last_steer: f64,
    dt: f64,
    car_config: &CarConfig,
) {
    for t in 0..horizon {
        let (a_t, b_t, c_t) =
            get_linear_model_matrix(xbar[t][2], xbar[t][3], last_steer, car_config.wheel_base(), dt);
        for state_dim in 0..NX {
            constraints.push_row_entry(state_index(t + 1, state_dim), 1.0);
            for previous_dim in 0..NX {
                constraints.push_row_entry(state_index(t, previous_dim), -a_t[state_dim][previous_dim]);
            }
            for control_dim in 0..NU {
                constraints.push_row_entry(control_index(t, control_dim, horizon), -b_t[state_dim][control_dim]);
            }
            constraints.finish_row(c_t[state_dim]);
        }
    }
}

fn add_initial_state_constraints(
    constraints: &mut ConstraintBuilder,
    horizon: usize,
    initial_state: ModelState,
    last_steer: f64,
) {
    for (state_dim, value) in initial_state.into_iter().enumerate() {
        constraints.push_scalar_row(state_index(0, state_dim), 1.0, value);
    }
    constraints.push_scalar_row(control_index(0, 1, horizon), 1.0, last_steer);
}

fn add_speed_constraints(constraints: &mut ConstraintBuilder, horizon: usize, car_config: &CarConfig) {
    for t in 0..=horizon {
        constraints.push_scalar_row(state_index(t, 2), 1.0, car_config.max_speed());
        constraints.push_scalar_row(state_index(t, 2), -1.0, -car_config.min_speed());
    }
}

fn add_control_constraints(constraints: &mut ConstraintBuilder, horizon: usize, car_config: &CarConfig) {
    for t in 0..horizon {
        constraints.push_symmetric_bound_row(control_index(t, 0, horizon), car_config.max_accel());
        constraints.push_symmetric_bound_row(control_index(t, 1, horizon), car_config.max_steer());
    }
}

fn add_steer_rate_constraints(constraints: &mut ConstraintBuilder, horizon: usize, steer_delta_limit: f64) {
    for t in 1..horizon {
        constraints.push_difference_bound_row(
            control_index(t - 1, 1, horizon),
            control_index(t, 1, horizon),
            steer_delta_limit,
        );
    }
}

fn add_control_cost_terms(
    horizon: usize,
    p_rows: &mut Vec<usize>,
    p_cols: &mut Vec<usize>,
    p_vals: &mut Vec<f64>,
    q: &mut [f64],
    mpc_config: &MpcConfig,
) {
    for t in 0..horizon {
        add_quadratic_term(
            p_rows,
            p_cols,
            p_vals,
            q,
            control_index(t, 0, horizon),
            mpc_config.r_accel,
            0.0,
        );
        add_quadratic_term(
            p_rows,
            p_cols,
            p_vals,
            q,
            control_index(t, 1, horizon),
            mpc_config.r_steer,
            0.0,
        );
    }
}

#[allow(clippy::needless_range_loop)]
fn add_state_tracking_terms(
    horizon: usize,
    xref: &[ModelState],
    p_rows: &mut Vec<usize>,
    p_cols: &mut Vec<usize>,
    p_vals: &mut Vec<f64>,
    q: &mut [f64],
    mpc_config: &MpcConfig,
) {
    for t in 1..horizon {
        for k in 0..NX {
            add_quadratic_term(
                p_rows,
                p_cols,
                p_vals,
                q,
                state_index(t, k),
                state_weight(k, mpc_config),
                xref[t][k],
            );
        }
    }

    for k in 0..NX {
        add_quadratic_term(
            p_rows,
            p_cols,
            p_vals,
            q,
            state_index(horizon, k),
            final_state_weight(k, mpc_config),
            xref[horizon][k],
        );
    }
}

fn add_control_smoothness_terms(
    horizon: usize,
    dt: f64,
    p_rows: &mut Vec<usize>,
    p_cols: &mut Vec<usize>,
    p_vals: &mut Vec<f64>,
    mpc_config: &MpcConfig,
) {
    let accel_diff_weight = mpc_config.rd_accel / (dt * dt);
    let steer_diff_weight = mpc_config.rd_steer / (dt * dt);
    for t in 1..horizon {
        add_difference_penalty(
            p_rows,
            p_cols,
            p_vals,
            control_index(t - 1, 0, horizon),
            control_index(t, 0, horizon),
            accel_diff_weight,
        );
        add_difference_penalty(
            p_rows,
            p_cols,
            p_vals,
            control_index(t - 1, 1, horizon),
            control_index(t, 1, horizon),
            steer_diff_weight,
        );
    }
}

#[allow(clippy::needless_range_loop)]
fn build_mpc_problem(
    xref: &[ModelState],
    xbar: &[ModelState],
    last_steer: f64,
    dt: f64,
    mpc_config: &MpcConfig,
    car_config: &CarConfig,
) -> MpcProblem {
    let horizon = mpc_config.horizon_length as usize;
    let nvars = NX * (horizon + 1) + NU * horizon;
    let mut q = vec![0.0; nvars];
    let mut p_rows = Vec::new();
    let mut p_cols = Vec::new();
    let mut p_vals = Vec::new();

    add_control_cost_terms(horizon, &mut p_rows, &mut p_cols, &mut p_vals, &mut q, mpc_config);
    add_state_tracking_terms(horizon, xref, &mut p_rows, &mut p_cols, &mut p_vals, &mut q, mpc_config);
    add_control_smoothness_terms(horizon, dt, &mut p_rows, &mut p_cols, &mut p_vals, mpc_config);

    let p = CscMatrix::new_from_triplets(nvars, nvars, p_rows, p_cols, p_vals);

    let equality_rows = NX * horizon + NX + 1;
    let inequality_rows = 2 * (horizon + 1) + 4 * horizon + 2 * (horizon - 1);
    let total_rows = equality_rows + inequality_rows;

    let mut constraints = ConstraintBuilder::new(total_rows);
    add_dynamics_constraints(&mut constraints, horizon, xbar, last_steer, dt, car_config);
    add_initial_state_constraints(&mut constraints, horizon, xbar[0], last_steer);
    add_speed_constraints(&mut constraints, horizon, car_config);
    add_control_constraints(&mut constraints, horizon, car_config);
    add_steer_rate_constraints(&mut constraints, horizon, car_config.max_steer_speed() * dt);

    let (a, b) = constraints.finish(total_rows, nvars);
    let cones = vec![ZeroConeT(equality_rows), NonnegativeConeT(inequality_rows)];

    MpcProblem { p, q, a, b, cones }
}
