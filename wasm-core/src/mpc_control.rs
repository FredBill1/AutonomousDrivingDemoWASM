use clarabel::{algebra::*, solver::*};
use wasm_bindgen::prelude::*;

const HORIZON_LENGTH: usize = 5;
const MAX_ITER: usize = 5;
const DU_TH: f64 = 0.1;
const NX: usize = 4;
const NU: usize = 2;

const R_ACCEL: f64 = 0.01;
const R_STEER: f64 = 0.005;
const RD_ACCEL: f64 = 1e-5;
const RD_STEER: f64 = 1e-3;
const Q_X: f64 = 1.1;
const Q_Y: f64 = 1.1;
const Q_V: f64 = 0.05;
const Q_YAW: f64 = 1.1;
const QF_SCALE: f64 = 2.0;

const WHEEL_BASE: f64 = 2.5;
const MAX_STEER: f64 = 40.0_f64.to_radians();
const MAX_STEER_SPEED: f64 = 360.0_f64.to_radians();
const MAX_SPEED: f64 = 55.0 / 3.6;
const MIN_SPEED: f64 = -30.0 / 3.6;
const MAX_ACCEL: f64 = 15.0;

type ModelState = [f64; 4];
type Control = [f64; 2];

#[derive(Clone, Copy)]
struct RollingCarState {
    x: f64,
    y: f64,
    velocity: f64,
    yaw: f64,
    steer: f64,
}

#[wasm_bindgen]
pub struct MpcControlResult {
    controls: Vec<f64>,
    predicted_states: Vec<f64>,
    iterations: usize,
}

#[wasm_bindgen]
impl MpcControlResult {
    #[wasm_bindgen(getter)]
    pub fn controls(&self) -> Vec<f64> {
        self.controls.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn predicted_states(&self) -> Vec<f64> {
        self.predicted_states.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn iterations(&self) -> usize {
        self.iterations
    }
}

#[wasm_bindgen]
pub fn mpc_control_preview(
    flat_reference_states: Vec<f64>,
    state_x: f64,
    state_y: f64,
    state_velocity: f64,
    mut state_yaw: f64,
    last_steer: f64,
    dt: f64,
) -> Result<MpcControlResult, JsValue> {
    let xref = decode_reference(&flat_reference_states)?;
    if xref.len() < HORIZON_LENGTH + 1 {
        return Err(JsValue::from_str(
            "Need at least HORIZON_LENGTH + 1 reference states",
        ));
    }

    state_yaw = align_yaw(state_yaw, xref[0][3]);

    let initial_state = RollingCarState {
        x: state_x,
        y: state_y,
        velocity: state_velocity,
        yaw: state_yaw,
        steer: last_steer,
    };
    let mut controls = vec![[0.0, 0.0]; HORIZON_LENGTH];
    let mut predicted_states = vec![[0.0; NX]; HORIZON_LENGTH + 1];
    let mut iterations = 0;

    for iteration in 0..MAX_ITER {
        iterations = iteration + 1;
        let xbar = predict_motion(initial_state, &controls, dt);
        let previous_controls = controls.clone();
        let Some((updated_controls, updated_states)) =
            linear_mpc_control(&xref, &xbar, last_steer, dt)
        else {
            break;
        };
        let du = control_delta(&previous_controls, &updated_controls);
        controls = updated_controls;
        predicted_states = updated_states;
        if du < DU_TH {
            break;
        }
    }

    Ok(MpcControlResult {
        controls: controls.into_iter().flat_map(|control| control).collect(),
        predicted_states: predicted_states
            .into_iter()
            .flat_map(|state| state)
            .collect(),
        iterations,
    })
}

fn decode_reference(flat: &[f64]) -> Result<Vec<ModelState>, JsValue> {
    if !flat.len().is_multiple_of(4) {
        return Err(JsValue::from_str(
            "Reference states must be flat [x, y, v, yaw] data",
        ));
    }

    let mut states = Vec::with_capacity(flat.len() / 4);
    for chunk in flat.chunks_exact(4) {
        states.push([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    Ok(states)
}

fn predict_motion(initial: RollingCarState, controls: &[Control], dt: f64) -> Vec<ModelState> {
    let mut state = initial;
    let mut out = vec![[state.x, state.y, state.velocity, state.yaw]];
    for control in controls {
        let target_velocity = state.velocity + control[0] * dt;
        state = step_state(state, target_velocity, control[1], dt);
        out.push([state.x, state.y, state.velocity, state.yaw]);
    }
    out
}

fn step_state(
    mut state: RollingCarState,
    target_velocity: f64,
    target_steer: f64,
    dt: f64,
) -> RollingCarState {
    state.x += state.velocity * state.yaw.cos() * dt;
    state.y += state.velocity * state.yaw.sin() * dt;
    state.yaw += state.velocity / WHEEL_BASE * state.steer.tan() * dt;

    let clipped_target_velocity = clamp(target_velocity, MIN_SPEED, MAX_SPEED);
    let clipped_target_steer = clamp(target_steer, -MAX_STEER, MAX_STEER);
    state.velocity += clamp(
        clipped_target_velocity - state.velocity,
        -MAX_ACCEL * dt,
        MAX_ACCEL * dt,
    );
    state.steer += clamp(
        clipped_target_steer - state.steer,
        -MAX_STEER_SPEED * dt,
        MAX_STEER_SPEED * dt,
    );
    state
}

fn linear_mpc_control(
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
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            &mut q,
            control_index(t, 0),
            R_ACCEL,
            0.0,
        );
        add_quadratic_term(
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            &mut q,
            control_index(t, 1),
            R_STEER,
            0.0,
        );
    }

    for t in 1..HORIZON_LENGTH {
        for k in 0..NX {
            add_quadratic_term(
                &mut p_rows,
                &mut p_cols,
                &mut p_vals,
                &mut q,
                state_index(t, k),
                state_weight(k),
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
            state_index(HORIZON_LENGTH, k),
            final_state_weight(k),
            xref[HORIZON_LENGTH][k],
        );
    }

    let accel_diff_weight = RD_ACCEL / (dt * dt);
    let steer_diff_weight = RD_STEER / (dt * dt);
    for t in 1..HORIZON_LENGTH {
        add_difference_penalty(
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            control_index(t - 1, 0),
            control_index(t, 0),
            accel_diff_weight,
        );
        add_difference_penalty(
            &mut p_rows,
            &mut p_cols,
            &mut p_vals,
            control_index(t - 1, 1),
            control_index(t, 1),
            steer_diff_weight,
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
                        control_index(t, control_dim),
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
        control_index(0, 1),
        1.0,
    );
    b.push(last_steer);
    row += 1;

    for t in 0..=HORIZON_LENGTH {
        push_entry(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            row,
            state_index(t, 2),
            1.0,
        );
        b.push(MAX_SPEED);
        row += 1;

        push_entry(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            row,
            state_index(t, 2),
            -1.0,
        );
        b.push(-MIN_SPEED);
        row += 1;
    }

    for t in 0..HORIZON_LENGTH {
        push_symmetric_bound(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            &mut b,
            &mut row,
            control_index(t, 0),
            MAX_ACCEL,
        );
        push_symmetric_bound(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            &mut b,
            &mut row,
            control_index(t, 1),
            MAX_STEER,
        );
    }

    let steer_delta_limit = MAX_STEER_SPEED * dt;
    for t in 1..HORIZON_LENGTH {
        push_difference_bound(
            &mut a_rows,
            &mut a_cols,
            &mut a_vals,
            &mut b,
            &mut row,
            control_index(t - 1, 1),
            control_index(t, 1),
            steer_delta_limit,
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
    rows: &mut Vec<usize>,
    cols: &mut Vec<usize>,
    vals: &mut Vec<f64>,
    q: &mut [f64],
    index: usize,
    weight: f64,
    reference: f64,
) {
    if weight == 0.0 {
        return;
    }

    rows.push(index);
    cols.push(index);
    vals.push(2.0 * weight);
    q[index] += -2.0 * weight * reference;
}

fn add_difference_penalty(
    rows: &mut Vec<usize>,
    cols: &mut Vec<usize>,
    vals: &mut Vec<f64>,
    left: usize,
    right: usize,
    weight: f64,
) {
    if weight == 0.0 {
        return;
    }

    rows.push(left);
    cols.push(left);
    vals.push(2.0 * weight);
    rows.push(right);
    cols.push(right);
    vals.push(2.0 * weight);
    rows.push(left.min(right));
    cols.push(left.max(right));
    vals.push(-2.0 * weight);
}

fn get_linear_model_matrix(
    velocity: f64,
    yaw: f64,
    steer: f64,
    dt: f64,
) -> ([[f64; NX]; NX], [[f64; NU]; NX], [f64; NX]) {
    let sy = yaw.sin();
    let cy = yaw.cos();
    let cs = steer.cos();

    let mut a = [[0.0; NX]; NX];
    a[0][0] = 1.0;
    a[1][1] = 1.0;
    a[2][2] = 1.0;
    a[3][3] = 1.0;
    a[0][2] = dt * cy;
    a[0][3] = -dt * velocity * sy;
    a[1][2] = dt * sy;
    a[1][3] = dt * velocity * cy;
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
        0 => Q_X,
        1 => Q_Y,
        2 => Q_V,
        3 => Q_YAW,
        _ => 0.0,
    }
}

fn final_state_weight(component: usize) -> f64 {
    QF_SCALE * state_weight(component)
}

fn push_entry(
    rows: &mut Vec<usize>,
    cols: &mut Vec<usize>,
    vals: &mut Vec<f64>,
    row: usize,
    col: usize,
    value: f64,
) {
    if value == 0.0 {
        return;
    }

    rows.push(row);
    cols.push(col);
    vals.push(value);
}

fn push_symmetric_bound(
    rows: &mut Vec<usize>,
    cols: &mut Vec<usize>,
    vals: &mut Vec<f64>,
    b: &mut Vec<f64>,
    row: &mut usize,
    index: usize,
    limit: f64,
) {
    push_entry(rows, cols, vals, *row, index, 1.0);
    b.push(limit);
    *row += 1;

    push_entry(rows, cols, vals, *row, index, -1.0);
    b.push(limit);
    *row += 1;
}

fn push_difference_bound(
    rows: &mut Vec<usize>,
    cols: &mut Vec<usize>,
    vals: &mut Vec<f64>,
    b: &mut Vec<f64>,
    row: &mut usize,
    previous: usize,
    current: usize,
    limit: f64,
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

fn align_yaw(yaw: f64, target_yaw: f64) -> f64 {
    target_yaw + wrap_angle(yaw - target_yaw)
}

fn control_delta(left: &[Control], right: &[Control]) -> f64 {
    left.iter()
        .zip(right.iter())
        .map(|(lhs, rhs)| (lhs[0] - rhs[0]).powi(2) + (lhs[1] - rhs[1]).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn wrap_angle(mut angle: f64) -> f64 {
    while angle >= std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}

fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

#[cfg(test)]
mod tests {
    use super::{MAX_ACCEL, MAX_STEER, linear_mpc_control, mpc_control_preview, predict_motion};

    fn assert_close(actual: &[f64], expected: &[f64], tolerance: f64) {
        assert_eq!(actual.len(), expected.len());
        for (index, (lhs, rhs)) in actual.iter().zip(expected.iter()).enumerate() {
            assert!(
                (lhs - rhs).abs() <= tolerance,
                "value mismatch at {index}: actual={lhs}, expected={rhs}, tolerance={tolerance}"
            );
        }
    }

    #[test]
    fn generates_control_and_prediction_preview() {
        let xref = vec![
            0.0, 0.0, 6.0, 0.0, 1.0, 0.0, 6.0, 0.0, 2.0, 0.0, 6.0, 0.0, 3.0, 0.0, 6.0, 0.0, 4.0,
            0.0, 6.0, 0.0, 5.0, 0.0, 0.0, 0.0,
        ];
        let result = mpc_control_preview(xref, 0.0, 0.0, 5.0, 0.0, 0.0, 0.07).expect("preview");
        assert_eq!(result.controls().len(), 10);
        assert_eq!(result.predicted_states().len(), 24);
        assert!(result.iterations() >= 1);
    }

    #[test]
    fn keeps_first_steer_fixed_and_controls_within_limits() {
        let xref = vec![
            0.0, 0.0, 4.0, 0.0, 1.0, 0.1, 4.0, 0.08, 2.0, 0.35, 4.0, 0.18, 2.8, 0.8, 3.5, 0.3, 3.1,
            1.4, 3.0, 0.4, 3.2, 2.1, 2.5, 0.45,
        ];
        let last_steer = 0.12;

        let result =
            mpc_control_preview(xref, 0.0, 0.0, 3.0, 0.02, last_steer, 0.1).expect("preview");
        let controls = result.controls();

        assert!((controls[1] - last_steer).abs() < 1e-6);

        for pair in controls.chunks_exact(2) {
            assert!(pair[0].abs() <= MAX_ACCEL + 1e-6);
            assert!(pair[1].abs() <= MAX_STEER + 1e-6);
        }
    }

    #[test]
    fn aligns_vehicle_yaw_to_wrapped_reference() {
        let xref = vec![
            0.0,
            0.0,
            3.0,
            std::f64::consts::PI - 0.02,
            -0.8,
            0.0,
            3.0,
            std::f64::consts::PI - 0.01,
            -1.6,
            0.0,
            3.0,
            std::f64::consts::PI,
            -2.4,
            0.0,
            3.0,
            -std::f64::consts::PI + 0.01,
            -3.2,
            0.0,
            2.5,
            -std::f64::consts::PI + 0.02,
            -4.0,
            0.0,
            0.0,
            -std::f64::consts::PI + 0.02,
        ];

        let result =
            mpc_control_preview(xref, 0.0, 0.0, 2.5, -std::f64::consts::PI + 0.01, 0.0, 0.1)
                .expect("preview");
        let predicted = result.predicted_states();
        let initial_yaw = predicted[3];

        assert!(initial_yaw > 3.0);
    }

    #[test]
    fn matches_python_fixture_on_straight_reference() {
        let straight_xref = vec![
            0.0, 0.0, 4.0, 0.0, 1.0, 0.0, 4.0, 0.0, 2.0, 0.0, 4.0, 0.0, 3.0, 0.0, 4.0, 0.0, 4.0,
            0.0, 4.0, 0.0, 5.0, 0.0, 0.0, 0.0,
        ];
        let straight = mpc_control_preview(straight_xref, 0.0, 0.0, 3.0, 0.0, 0.0, 0.07)
            .expect("straight fixture preview");

        let expected_controls = vec![
            14.999998614949373,
            0.0,
            10.571998681548704,
            0.0,
            4.895701473376704,
            0.0,
            0.19554004160815755,
            0.0,
            -2.8430650137075264,
            0.0,
        ];
        let expected_states = vec![
            0.0,
            0.0,
            3.0,
            0.0,
            0.21,
            0.0,
            4.049999903046456,
            0.0,
            0.4934999932132519,
            0.0,
            4.790039810754866,
            0.0,
            0.8288027799660924,
            0.0,
            5.1327389138912345,
            0.0,
            1.188094503938479,
            0.0,
            5.146426716803806,
            0.0,
            1.5483443741147453,
            0.0,
            4.947412165844279,
            0.0,
        ];

        assert_close(&straight.controls(), &expected_controls, 1e-4);
        assert_close(&straight.predicted_states(), &expected_states, 1e-4);
    }

    #[test]
    fn stays_close_to_python_fixture_on_turning_reference() {
        let turning_xref = vec![
            0.0, 0.0, 3.0, 0.15, 0.7, 0.15, 3.2, 0.18, 1.35, 0.45, 3.4, 0.24, 1.9, 0.95, 3.1, 0.33,
            2.25, 1.55, 2.7, 0.41, 2.45, 2.2, 0.0, 0.46,
        ];
        let turning = mpc_control_preview(turning_xref, 0.1, -0.05, 2.2, 0.12, 0.08, 0.07)
            .expect("turning fixture preview");

        let expected_controls = vec![
            8.750088, 0.08, 5.475858, 0.519823, 2.239496, 0.698132, -0.308206, 0.698132, -1.910972,
            0.698131,
        ];
        let expected_states = vec![
            0.1, -0.05, 2.2, 0.12, 0.252893, -0.031564, 2.812506, 0.124939, 0.448233, -0.007031,
            3.195816, 0.166107, 0.668996, 0.029976, 3.352581, 0.228939, 0.897802, 0.083283,
            3.331007, 0.29485, 1.121038, 0.151074, 3.197238, 0.360333,
        ];

        assert_close(&turning.controls(), &expected_controls, 0.3);
        assert_close(&turning.predicted_states(), &expected_states, 0.06);
    }

    #[test]
    fn linear_solver_path_returns_controls_without_fallback() {
        let xref = [
            [0.0, 0.0, 4.0, 0.0],
            [1.0, 0.0, 4.0, 0.0],
            [2.0, 0.0, 4.0, 0.0],
            [3.0, 0.0, 4.0, 0.0],
            [4.0, 0.0, 4.0, 0.0],
            [5.0, 0.0, 0.0, 0.0],
        ];
        let xbar = predict_motion(
            super::RollingCarState {
                x: 0.0,
                y: 0.0,
                velocity: 3.0,
                yaw: 0.0,
                steer: 0.0,
            },
            &vec![[0.0, 0.0]; 5],
            0.07,
        );
        let (controls, _) = linear_mpc_control(&xref, &xbar, 0.0, 0.07).expect("solver result");

        assert_eq!(controls.len(), 5);
        assert!(
            controls
                .iter()
                .all(|pair| pair[0].is_finite() && pair[1].is_finite())
        );
    }
}
