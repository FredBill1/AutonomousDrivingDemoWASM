use clarabel::{algebra::*, solver::*};
use super::types::{
    ModelState, Control,
    HORIZON_LENGTH, NX, NU,
    R_ACCEL, R_STEER, RD_ACCEL, RD_STEER,
    Q_X, Q_Y, Q_V, Q_YAW, QF_SCALE,
    WHEEL_BASE, MAX_STEER, MAX_STEER_SPEED, MAX_SPEED, MIN_SPEED, MAX_ACCEL,
};

pub(crate) fn linear_mpc_control(
    xref: &[ModelState],
    xbar: &[ModelState],
    last_steer: f64,
    dt: f64,
) -> Option<(Vec<Control>, Vec<ModelState>)> {
    let problem = build_mpc_problem(xref, xbar, last_steer, dt);

    let mut settings = DefaultSettings::default();
    settings.verbose = false;
    settings.max_iter = 50;

    let mut solver = DefaultSolver::new(
        &problem.p,
        &problem.q,
        &problem.a,
        &problem.b,
        &problem.cones,
        settings,
    )
    .ok()?;

    solver.solve();

    if !matches!(
        solver.solution.status,
        SolverStatus::Solved | SolverStatus::AlmostSolved
    ) {
        return None;
    }

    Some((
        decode_controls(&solver.solution.x),
        decode_states(&solver.solution.x),
    ))
}

struct MpcProblem {
    p: CscMatrix<f64>,
    q: Vec<f64>,
    a: CscMatrix<f64>,
    b: Vec<f64>,
    cones: Vec<SupportedConeT<f64>>,
}

fn build_mpc_problem(
    xref: &[ModelState],
    xbar: &[ModelState],
    last_steer: f64,
    dt: f64,
) -> MpcProblem {
    let nvars = NX * (HORIZON_LENGTH + 1) + NU * HORIZON_LENGTH;
    let mut q = vec![0.0; nvars];
    let mut p_rows = Vec::new();
    let mut p_cols = Vec::new();
    let mut p_vals = Vec::new();

    for t in 0..HORIZON_LENGTH {
        add_quadratic_term(
            &mut p_rows, &mut p_cols, &mut p_vals, &mut q,
            control_index(t, 0), R_ACCEL, 0.0,
        );
        add_quadratic_term(
            &mut p_rows, &mut p_cols, &mut p_vals, &mut q,
            control_index(t, 1), R_STEER, 0.0,
        );
    }

    for t in 1..HORIZON_LENGTH {
        for k in 0..NX {
            add_quadratic_term(
                &mut p_rows, &mut p_cols, &mut p_vals, &mut q,
                state_index(t, k), state_weight(k), xref[t][k],
            );
        }
    }

    for k in 0..NX {
        add_quadratic_term(
            &mut p_rows, &mut p_cols, &mut p_vals, &mut q,
            state_index(HORIZON_LENGTH, k), final_state_weight(k), xref[HORIZON_LENGTH][k],
        );
    }

    let accel_diff_weight = RD_ACCEL / (dt * dt);
    let steer_diff_weight = RD_STEER / (dt * dt);
    for t in 1..HORIZON_LENGTH {
        add_difference_penalty(
            &mut p_rows, &mut p_cols, &mut p_vals,
            control_index(t - 1, 0), control_index(t, 0), accel_diff_weight,
        );
        add_difference_penalty(
            &mut p_rows, &mut p_cols, &mut p_vals,
            control_index(t - 1, 1), control_index(t, 1), steer_diff_weight,
        );
    }

    let p = CscMatrix::new_from_triplets(nvars, nvars, p_rows, p_cols, p_vals);

    let equality_rows = NX * HORIZON_LENGTH + NX + 1;
    let inequality_rows = 2 * (HORIZON_LENGTH + 1) + 4 * HORIZON_LENGTH + 2 * (HORIZON_LENGTH - 1);
    let total_rows = equality_rows + inequality_rows;

    let mut a_rows = Vec::new();
    let mut a_cols = Vec::new();
    let mut a_vals = Vec::new();
    let mut b = Vec::with_capacity(total_rows);
    let mut row = 0;

    for t in 0..HORIZON_LENGTH {
        let (a_t, b_t, c_t) = get_linear_model_matrix(xbar[t][2], xbar[t][3], last_steer, dt);
        for state_dim in 0..NX {
            push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, state_index(t + 1, state_dim), 1.0);
            for previous_dim in 0..NX {
                let value = -a_t[state_dim][previous_dim];
                if value != 0.0 {
                    push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, state_index(t, previous_dim), value);
                }
            }
            for control_dim in 0..NU {
                let value = -b_t[state_dim][control_dim];
                if value != 0.0 {
                    push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, control_index(t, control_dim), value);
                }
            }
            b.push(c_t[state_dim]);
            row += 1;
        }
    }

    for state_dim in 0..NX {
        push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, state_index(0, state_dim), 1.0);
        b.push(xbar[0][state_dim]);
        row += 1;
    }

    push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, control_index(0, 1), 1.0);
    b.push(last_steer);
    row += 1;

    for t in 0..=HORIZON_LENGTH {
        push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, state_index(t, 2), 1.0);
        b.push(MAX_SPEED);
        row += 1;

        push_entry(&mut a_rows, &mut a_cols, &mut a_vals, row, state_index(t, 2), -1.0);
        b.push(-MIN_SPEED);
        row += 1;
    }

    for t in 0..HORIZON_LENGTH {
        push_symmetric_bound(&mut a_rows, &mut a_cols, &mut a_vals, &mut b, &mut row, control_index(t, 0), MAX_ACCEL);
        push_symmetric_bound(&mut a_rows, &mut a_cols, &mut a_vals, &mut b, &mut row, control_index(t, 1), MAX_STEER);
    }

    let steer_delta_limit = MAX_STEER_SPEED * dt;
    for t in 1..HORIZON_LENGTH {
        push_difference_bound(
            &mut a_rows, &mut a_cols, &mut a_vals, &mut b, &mut row,
            control_index(t - 1, 1), control_index(t, 1), steer_delta_limit,
        );
    }

    let a = CscMatrix::new_from_triplets(total_rows, nvars, a_rows, a_cols, a_vals);
    let cones = vec![ZeroConeT(equality_rows), NonnegativeConeT(inequality_rows)];

    MpcProblem { p, q, a, b, cones }
}

fn decode_controls(solution: &[f64]) -> Vec<Control> {
    (0..HORIZON_LENGTH)
        .map(|t| [solution[control_index(t, 0)], solution[control_index(t, 1)]])
        .collect()
}

fn decode_states(solution: &[f64]) -> Vec<ModelState> {
    (0..=HORIZON_LENGTH)
        .map(|t| {
            [
                solution[state_index(t, 0)],
                solution[state_index(t, 1)],
                solution[state_index(t, 2)],
                solution[state_index(t, 3)],
            ]
        })
        .collect()
}

fn add_quadratic_term(
    rows: &mut Vec<usize>, cols: &mut Vec<usize>, vals: &mut Vec<f64>,
    q: &mut [f64], index: usize, weight: f64, reference: f64,
) {
    if weight == 0.0 { return; }
    rows.push(index);
    cols.push(index);
    vals.push(2.0 * weight);
    q[index] += -2.0 * weight * reference;
}

fn add_difference_penalty(
    rows: &mut Vec<usize>, cols: &mut Vec<usize>, vals: &mut Vec<f64>,
    left: usize, right: usize, weight: f64,
) {
    if weight == 0.0 { return; }
    rows.push(left); cols.push(left); vals.push(2.0 * weight);
    rows.push(right); cols.push(right); vals.push(2.0 * weight);
    rows.push(left.min(right)); cols.push(left.max(right)); vals.push(-2.0 * weight);
}

fn get_linear_model_matrix(
    velocity: f64, yaw: f64, steer: f64, dt: f64,
) -> ([[f64; NX]; NX], [[f64; NU]; NX], [f64; NX]) {
    let sy = yaw.sin();
    let cy = yaw.cos();
    let cs = steer.cos();

    let mut a = [[0.0; NX]; NX];
    a[0][0] = 1.0; a[1][1] = 1.0; a[2][2] = 1.0; a[3][3] = 1.0;
    a[0][2] = dt * cy; a[0][3] = -dt * velocity * sy;
    a[1][2] = dt * sy; a[1][3] = dt * velocity * cy;
    a[3][2] = dt * steer.tan() / WHEEL_BASE;

    let mut b = [[0.0; NU]; NX];
    b[2][0] = dt;
    b[3][1] = dt * velocity / (WHEEL_BASE * cs * cs);

    let mut c = [0.0; NX];
    c[0] = dt * velocity * sy * yaw;
    c[1] = -dt * velocity * cy * yaw;
    c[3] = -dt * velocity * steer / (WHEEL_BASE * cs * cs);

    (a, b, c)
}

fn state_index(step: usize, component: usize) -> usize {
    step * NX + component
}

fn control_index(step: usize, component: usize) -> usize {
    NX * (HORIZON_LENGTH + 1) + step * NU + component
}

fn state_weight(component: usize) -> f64 {
    match component {
        0 => Q_X, 1 => Q_Y, 2 => Q_V, 3 => Q_YAW, _ => 0.0,
    }
}

fn final_state_weight(component: usize) -> f64 {
    QF_SCALE * state_weight(component)
}

fn push_entry(
    rows: &mut Vec<usize>, cols: &mut Vec<usize>, vals: &mut Vec<f64>,
    row: usize, col: usize, value: f64,
) {
    if value == 0.0 { return; }
    rows.push(row); cols.push(col); vals.push(value);
}

fn push_symmetric_bound(
    rows: &mut Vec<usize>, cols: &mut Vec<usize>, vals: &mut Vec<f64>,
    b: &mut Vec<f64>, row: &mut usize, index: usize, limit: f64,
) {
    push_entry(rows, cols, vals, *row, index, 1.0);
    b.push(limit);
    *row += 1;

    push_entry(rows, cols, vals, *row, index, -1.0);
    b.push(limit);
    *row += 1;
}

fn push_difference_bound(
    rows: &mut Vec<usize>, cols: &mut Vec<usize>, vals: &mut Vec<f64>,
    b: &mut Vec<f64>, row: &mut usize, previous: usize, current: usize, limit: f64,
) {
    push_entry(rows, cols, vals, *row, current, 1.0);
    push_entry(rows, cols, vals, *row, previous, -1.0);
    b.push(limit);
    *row += 1;

    push_entry(rows, cols, vals, *row, current, -1.0);
    push_entry(rows, cols, vals, *row, previous, 1.0);
    b.push(limit);
    *row += 1;
}
