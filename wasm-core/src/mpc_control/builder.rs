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
    mpc_config: &MpcConfig,
    car_config: &CarConfig,
) -> Option<(Vec<Control>, Vec<ModelState>)> {
    let problem = build_mpc_problem(xref, xbar, last_steer, mpc_config, car_config);

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

#[allow(clippy::needless_range_loop)]
fn build_mpc_problem(
    xref: &[ModelState],
    xbar: &[ModelState],
    last_steer: f64,
    mpc_config: &MpcConfig,
    car_config: &CarConfig,
) -> MpcProblem {
    let dt = mpc_config.dt;
    let horizon = mpc_config.horizon_length as usize;
    let nvars = NX * (horizon + 1) + NU * horizon;
    let mut q = vec![0.0; nvars];
    let mut p_rows = Vec::new();
    let mut p_cols = Vec::new();
    let mut p_vals = Vec::new();

    for t in 0..horizon {
        add_quadratic_term(
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            &mut q,
            control_index(t, 0, horizon),
            mpc_config.r_accel,
            0.0,
        );
        add_quadratic_term(
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            &mut q,
            control_index(t, 1, horizon),
            mpc_config.r_steer,
            0.0,
        );
    }

    for t in 1..horizon {
        for k in 0..NX {
            add_quadratic_term(
                &mut p_rows,
                &mut p_cols,
                &mut p_vals,
                &mut q,
                state_index(t, k),
                state_weight(k, mpc_config),
                xref[t][k],
            );
        }
    }

    for k in 0..NX {
        add_quadratic_term(
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            &mut q,
            state_index(horizon, k),
            final_state_weight(k, mpc_config),
            xref[horizon][k],
        );
    }

    let accel_diff_weight = mpc_config.rd_accel / (dt * dt);
    let steer_diff_weight = mpc_config.rd_steer / (dt * dt);
    for t in 1..horizon {
        add_difference_penalty(
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            control_index(t - 1, 0, horizon),
            control_index(t, 0, horizon),
            accel_diff_weight,
        );
        add_difference_penalty(
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            control_index(t - 1, 1, horizon),
            control_index(t, 1, horizon),
            steer_diff_weight,
        );
    }

    let p = CscMatrix::new_from_triplets(nvars, nvars, p_rows, p_cols, p_vals);

    let equality_rows = NX * horizon + NX + 1;
    let inequality_rows = 2 * (horizon + 1) + 4 * horizon + 2 * (horizon - 1);
    let total_rows = equality_rows + inequality_rows;

    let mut a_rows = Vec::new();
    let mut a_cols = Vec::new();
    let mut a_vals = Vec::new();
    let mut b = Vec::with_capacity(total_rows);
    let mut row = 0;

    for t in 0..horizon {
        let (a_t, b_t, c_t) =
            get_linear_model_matrix(xbar[t][2], xbar[t][3], last_steer, car_config.wheel_base(), dt);
        for state_dim in 0..NX {
            push_entry(
                &mut a_rows,
                &mut a_cols,
                &mut a_vals,
                row,
                state_index(t + 1, state_dim),
                1.0,
            );
            for previous_dim in 0..NX {
                let value = -a_t[state_dim][previous_dim];
                if value != 0.0 {
                    push_entry(
                        &mut a_rows,
                        &mut a_cols,
                        &mut a_vals,
                        row,
                        state_index(t, previous_dim),
                        value,
                    );
                }
            }
            for control_dim in 0..NU {
                let value = -b_t[state_dim][control_dim];
                if value != 0.0 {
                    push_entry(
                        &mut a_rows,
                        &mut a_cols,
                        &mut a_vals,
                        row,
                        control_index(t, control_dim, horizon),
                        value,
                    );
                }
            }
            b.push(c_t[state_dim]);
            row += 1;
        }
    }

    for state_dim in 0..NX {
        push_entry(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            row,
            state_index(0, state_dim),
            1.0,
        );
        b.push(xbar[0][state_dim]);
        row += 1;
    }

    push_entry(
        &mut a_rows,
        &mut a_cols,
        &mut a_vals,
        row,
        control_index(0, 1, horizon),
        1.0,
    );
    b.push(last_steer);
    row += 1;

    for t in 0..=horizon {
        push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, state_index(t, 2), 1.0);
        b.push(car_config.max_speed());
        row += 1;

        push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, state_index(t, 2), -1.0);
        b.push(-car_config.min_speed());
        row += 1;
    }

    for t in 0..horizon {
        push_symmetric_bound(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            &mut b,
            &mut row,
            control_index(t, 0, horizon),
            car_config.max_accel(),
        );
        push_symmetric_bound(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            &mut b,
            &mut row,
            control_index(t, 1, horizon),
            car_config.max_steer(),
        );
    }

    let steer_delta_limit = car_config.max_steer_speed() * dt;
    for t in 1..horizon {
        push_difference_bound(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            &mut b,
            &mut row,
            control_index(t - 1, 1, horizon),
            control_index(t, 1, horizon),
            steer_delta_limit,
        );
    }

    let a = CscMatrix::new_from_triplets(total_rows, nvars, a_rows, a_cols, a_vals);
    let cones = vec![ZeroConeT(equality_rows), NonnegativeConeT(inequality_rows)];

    MpcProblem { p, q, a, b, cones }
}
