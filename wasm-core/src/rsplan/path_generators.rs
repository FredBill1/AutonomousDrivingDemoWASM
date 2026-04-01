use super::curve_formulas::{
    c_c_c, c_c2sc2_c, c_c2sca, c_c2scb, c_cc, c_cucu_c, cc_c, ccu_cuc, csc2_ca, csc2_cb, csca, cscb,
};
use super::math::rs_steering_angles;
use super::path::ReedsSheppPath;
use super::paths_data::{PATH_TYPE_INDICES, PATHS};
use super::segment::ReedsSheppSegment;
use super::types::{CurveFormula, PI_DIVS, PathParameter, PathParameters, Pose, SegmentKind};

pub(super) fn gen_path_parameters(
    curve_func: CurveFormula,
    path_type_indices: (usize, usize, usize, usize),
    x: f64,
    y: f64,
    phi: f64,
    cos_phi_param: f64,
    turn_radius: f64,
) -> PathParameters {
    let pos_sin_phi = turn_radius * phi.sin();
    let neg_sin_phi = -turn_radius * phi.sin();

    let standard = curve_func(x, y, phi, pos_sin_phi, cos_phi_param, turn_radius);
    let reflection = curve_func(x, -y, -phi, neg_sin_phi, cos_phi_param, turn_radius);
    let time_flip = curve_func(-x, y, -phi, neg_sin_phi, cos_phi_param, turn_radius).map(|(t, u, v)| (-t, -u, -v));
    let reflection_time_flip =
        curve_func(-x, -y, phi, pos_sin_phi, cos_phi_param, turn_radius).map(|(t, u, v)| (-t, -u, -v));

    let candidates = [
        (standard, path_type_indices.0),
        (reflection, path_type_indices.1),
        (time_flip, path_type_indices.2),
        (reflection_time_flip, path_type_indices.3),
    ];

    let mut params = PathParameters::default();
    for (candidate, path_ix) in candidates {
        if let Some((t, u, v)) = candidate {
            params.values.push(PathParameter { t, u, v, path_ix });
        }
    }
    params
}

pub(super) fn create_path(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    segment_params: &[f64],
    path_segment_types: &[(SegmentKind, i8)],
    turn_radius: f64,
) -> ReedsSheppPath {
    let segments = segment_params
        .iter()
        .zip(path_segment_types.iter())
        .map(|(segment_param, (kind, direction))| create_segment(*segment_param, *kind, *direction, turn_radius))
        .collect();

    ReedsSheppPath::from_segments(start_pose, end_pose, segments, turn_radius, step_size)
}

fn create_segment(segment_param: f64, kind: SegmentKind, direction: i8, turn_radius: f64) -> ReedsSheppSegment {
    let length = if kind == SegmentKind::Straight {
        segment_param
    } else {
        segment_param * turn_radius
    };
    ReedsSheppSegment {
        kind,
        direction,
        length,
        turn_radius,
    }
}

fn collect_tuv_paths(
    all: super::types::PathParameters,
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    all.iter()
        .map(|(_, t, u, v, path_ix)| {
            create_path(start_pose, end_pose, step_size, &[t, u, v], PATHS[path_ix], turn_radius)
        })
        .collect()
}

pub(super) fn csc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let csca_params = gen_path_parameters(csca, PATH_TYPE_INDICES[0], x, y, phi, steering[0], turn_radius);
    let cscb_params = gen_path_parameters(cscb, PATH_TYPE_INDICES[1], x, y, phi, steering[1], turn_radius);

    let mut all = super::types::PathParameters::default();
    all.push_from(csca_params);
    all.push_from(cscb_params);

    collect_tuv_paths(all, start_pose, end_pose, step_size, turn_radius)
}

pub(super) fn ccc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let gp = |f: CurveFormula, idx: usize| {
        gen_path_parameters(f, PATH_TYPE_INDICES[idx], x, y, phi, steering[0], turn_radius)
    };
    let mut all = super::types::PathParameters::default();
    all.push_from(gp(c_c_c, 2));
    all.push_from(gp(c_cc, 3));
    all.push_from(gp(cc_c, 4));

    collect_tuv_paths(all, start_pose, end_pose, step_size, turn_radius)
}

pub(super) fn cccc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let gp = |f: CurveFormula, idx: usize| {
        gen_path_parameters(f, PATH_TYPE_INDICES[idx], x, y, phi, steering[1], turn_radius)
    };
    let mut all = super::types::PathParameters::default();
    all.push_from(gp(ccu_cuc, 5));
    all.push_from(gp(c_cucu_c, 6));

    all.iter()
        .map(|(_, t, u, v, path_ix)| {
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, u, u, v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

pub(super) fn ccsc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let gp = |f: CurveFormula, idx: usize, si: usize| {
        gen_path_parameters(f, PATH_TYPE_INDICES[idx], x, y, phi, steering[si], turn_radius)
    };
    let mut all = super::types::PathParameters::default();
    all.push_from(gp(c_c2sca, 7, 0));
    all.push_from(gp(c_c2scb, 8, 1));

    all.iter()
        .map(|(ix, t, u, v, path_ix)| {
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, PI_DIVS[ix], u, v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

pub(super) fn cscc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let gp = |f: CurveFormula, idx: usize, si: usize| {
        gen_path_parameters(f, PATH_TYPE_INDICES[idx], x, y, phi, steering[si], turn_radius)
    };
    let mut all = super::types::PathParameters::default();
    all.push_from(gp(csc2_ca, 9, 0));
    all.push_from(gp(csc2_cb, 10, 1));

    all.iter()
        .map(|(ix, t, u, v, path_ix)| {
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, u, PI_DIVS[ix], v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}

pub(super) fn ccscc(
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
) -> Vec<ReedsSheppPath> {
    let steering = rs_steering_angles(phi, turn_radius);
    let params = gen_path_parameters(c_c2sc2_c, PATH_TYPE_INDICES[11], x, y, phi, steering[1], turn_radius);

    params
        .iter()
        .map(|(ix, t, u, v, path_ix)| {
            let pi_div = PI_DIVS[ix];
            create_path(
                start_pose,
                end_pose,
                step_size,
                &[t, pi_div, u, pi_div, v],
                PATHS[path_ix],
                turn_radius,
            )
        })
        .collect()
}
