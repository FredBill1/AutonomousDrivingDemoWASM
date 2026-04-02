use super::curve_formulas::{
    c_c_c, c_c2sc2_c, c_c2sca, c_c2scb, c_cc, c_cucu_c, cc_c, ccu_cuc, csc2_ca, csc2_cb, csca, cscb,
};
use super::math::rs_steering_angles;
use super::path::ReedsSheppPath;
use super::paths_data::{PATH_TYPE_INDICES, PATHS};
use super::segment::ReedsSheppSegment;
use super::types::{CurveFormula, PI_DIVS, PathParameter, PathParameters, Pose, SegmentKind};

fn collect_path_parameters(
    x: f64,
    y: f64,
    phi: f64,
    turn_radius: f64,
    steering: &[f64],
    generators: &[(CurveFormula, usize, usize)],
) -> PathParameters {
    let mut params = PathParameters::default();
    for (curve, path_type_index, steering_index) in generators {
        params.push_from(gen_path_parameters(
            *curve,
            PATH_TYPE_INDICES[*path_type_index],
            x,
            y,
            phi,
            steering[*steering_index],
            turn_radius,
        ));
    }
    params
}

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

fn collect_paths<F>(
    params: PathParameters,
    start_pose: Pose,
    end_pose: Pose,
    step_size: f64,
    turn_radius: f64,
    build_segments: F,
) -> Vec<ReedsSheppPath>
where
    F: Fn(usize, f64, f64, f64) -> Vec<f64>,
{
    params
        .iter()
        .map(|(ix, t, u, v, path_ix)| {
            let segment_params = build_segments(ix, t, u, v);
            create_path(
                start_pose,
                end_pose,
                step_size,
                &segment_params,
                PATHS[path_ix],
                turn_radius,
            )
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
    let params = collect_path_parameters(x, y, phi, turn_radius, &steering, &[(csca, 0, 0), (cscb, 1, 1)]);
    collect_paths(params, start_pose, end_pose, step_size, turn_radius, |_, t, u, v| {
        vec![t, u, v]
    })
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
    let params = collect_path_parameters(
        x,
        y,
        phi,
        turn_radius,
        &steering,
        &[(c_c_c, 2, 0), (c_cc, 3, 0), (cc_c, 4, 0)],
    );
    collect_paths(params, start_pose, end_pose, step_size, turn_radius, |_, t, u, v| {
        vec![t, u, v]
    })
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
    let params = collect_path_parameters(x, y, phi, turn_radius, &steering, &[(ccu_cuc, 5, 1), (c_cucu_c, 6, 1)]);
    collect_paths(params, start_pose, end_pose, step_size, turn_radius, |_, t, u, v| {
        vec![t, u, u, v]
    })
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
    let params = collect_path_parameters(x, y, phi, turn_radius, &steering, &[(c_c2sca, 7, 0), (c_c2scb, 8, 1)]);
    collect_paths(params, start_pose, end_pose, step_size, turn_radius, |ix, t, u, v| {
        vec![t, PI_DIVS[ix], u, v]
    })
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
    let params = collect_path_parameters(x, y, phi, turn_radius, &steering, &[(csc2_ca, 9, 0), (csc2_cb, 10, 1)]);
    collect_paths(params, start_pose, end_pose, step_size, turn_radius, |ix, t, u, v| {
        vec![t, u, PI_DIVS[ix], v]
    })
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
    let params = collect_path_parameters(x, y, phi, turn_radius, &steering, &[(c_c2sc2_c, 11, 1)]);
    collect_paths(params, start_pose, end_pose, step_size, turn_radius, |ix, t, u, v| {
        let pi_div = PI_DIVS[ix];
        vec![t, pi_div, u, pi_div, v]
    })
}
