use super::types::{
    Control, HORIZON_LENGTH, ModelState, NU, NX, Q_V, Q_X, Q_Y, Q_YAW, QF_SCALE, WHEEL_BASE,
};

pub(super) fn decode_controls(solution: &[f64]) -> Vec<Control> {
    (0..HORIZON_LENGTH)
        .map(|t| [solution[control_index(t, 0)], solution[control_index(t, 1)]])
        .collect()
}

pub(super) fn decode_states(solution: &[f64]) -> Vec<ModelState> {
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

pub(super) fn get_linear_model_matrix(
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

pub(super) fn state_index(step: usize, component: usize) -> usize {
    step * NX + component
}

pub(super) fn control_index(step: usize, component: usize) -> usize {
    NX * (HORIZON_LENGTH + 1) + step * NU + component
}

pub(super) fn state_weight(component: usize) -> f64 {
    match component {
        0 => Q_X,
        1 => Q_Y,
        2 => Q_V,
        3 => Q_YAW,
        _ => 0.0,
    }
}

pub(super) fn final_state_weight(component: usize) -> f64 {
    QF_SCALE * state_weight(component)
}

pub(super) fn push_entry(
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

pub(super) fn add_quadratic_term(
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
    vals.push(weight);
    q[index] -= weight * reference;
}

pub(super) fn add_difference_penalty(
    rows: &mut Vec<usize>,
    cols: &mut Vec<usize>,
    vals: &mut Vec<f64>,
    prev: usize,
    curr: usize,
    weight: f64,
) {
    rows.push(prev);
    cols.push(prev);
    vals.push(weight);

    rows.push(prev);
    cols.push(curr);
    vals.push(-weight);

    rows.push(curr);
    cols.push(prev);
    vals.push(-weight);

    rows.push(curr);
    cols.push(curr);
    vals.push(weight);
}

pub(super) fn push_symmetric_bound(
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

pub(super) fn push_difference_bound(
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
