use approx::assert_relative_eq;

use super::math::{rs_change_base, rs_euclidean_distance, rs_polar, rs_steering_angles};
use super::path::ReedsSheppPath;
use super::segment::ReedsSheppSegment;
use super::solve::rs_solve_path;
use super::types::SegmentKind;

#[test]
fn helper_functions_match_python_reference_behavior() {
    let changed = rs_change_base(1.0, 2.0, std::f64::consts::FRAC_PI_2, 3.0, 2.0, 0.25);
    assert_relative_eq!(changed[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(changed[1], -2.0, epsilon = 1e-12);
    assert_relative_eq!(
        changed[2],
        0.25 - std::f64::consts::FRAC_PI_2,
        epsilon = 1e-12
    );

    let steering = rs_steering_angles(0.4, 3.0);
    assert_relative_eq!(steering[0], 3.0 * (0.4_f64.cos() - 1.0), epsilon = 1e-12);
    assert_relative_eq!(steering[1], 3.0 * (0.4_f64.cos() + 1.0), epsilon = 1e-12);

    let polar = rs_polar(3.0, 4.0);
    assert_relative_eq!(polar[0], 5.0, epsilon = 1e-12);
    assert_relative_eq!(polar[1], 4.0_f64.atan2(3.0), epsilon = 1e-12);
}

#[test]
fn straight_path_waypoints_follow_python_discretization() {
    let mut path = ReedsSheppPath::new(0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.5);
    path.push_segment(&ReedsSheppSegment::new(SegmentKind::Straight, 1, 2.0, 1.0));

    assert_relative_eq!(path.total_length(), 2.0, epsilon = 1e-12);
    assert_eq!(path.segment_count(), 1);
    assert_eq!(path.waypoint_count(), 6);

    let coordinates = path.flat_coordinates();
    assert_relative_eq!(coordinates[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(coordinates[1], 0.0, epsilon = 1e-12);
    assert_relative_eq!(coordinates[coordinates.len() - 3], 2.0, epsilon = 1e-12);
    assert_relative_eq!(coordinates[coordinates.len() - 2], 0.0, epsilon = 1e-12);
    assert_relative_eq!(coordinates[coordinates.len() - 1], 0.0, epsilon = 1e-12);
}

#[test]
fn curved_and_runway_segments_produce_expected_metrics() {
    let mut path = ReedsSheppPath::new(
        0.0,
        0.0,
        0.0,
        2.0,
        3.5,
        std::f64::consts::FRAC_PI_2,
        2.0,
        0.5,
    );
    path.push_segment(&ReedsSheppSegment::new(
        SegmentKind::Left,
        1,
        std::f64::consts::PI,
        2.0,
    ));
    path.push_segment(&ReedsSheppSegment::new(SegmentKind::Straight, -1, 1.5, 2.0));

    assert_relative_eq!(
        path.total_length(),
        std::f64::consts::PI + 1.5,
        epsilon = 1e-12
    );
    assert_relative_eq!(path.runway_length(), 1.5, epsilon = 1e-12);
    assert_eq!(path.number_of_cusp_points(), 1);

    let coordinates = path.flat_coordinates();
    let last_x = coordinates[coordinates.len() - 3];
    let last_y = coordinates[coordinates.len() - 2];
    let last_yaw = coordinates[coordinates.len() - 1];

    assert_relative_eq!(last_x, 2.0, epsilon = 1e-12);
    assert_relative_eq!(last_y, 0.5, epsilon = 1e-12);
    assert_relative_eq!(last_yaw, std::f64::consts::FRAC_PI_2, epsilon = 1e-12);
}

#[test]
fn solved_straight_path_matches_euclidean_distance() {
    let path = rs_solve_path(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 2.0, 0.0, 0.05, 2.0)
        .expect("path should exist");

    assert_relative_eq!(
        path.total_length(),
        rs_euclidean_distance(0.0, 0.0, 10.0, 0.0),
        epsilon = 1e-9
    );

    let coordinates = path.flat_coordinates();
    assert_relative_eq!(coordinates[coordinates.len() - 3], 10.0, epsilon = 1e-9);
    assert_relative_eq!(coordinates[coordinates.len() - 2], 0.0, epsilon = 1e-9);
    assert_relative_eq!(coordinates[coordinates.len() - 1], 0.0, epsilon = 1e-9);
}

#[test]
fn solved_path_with_runway_reports_runway_length() {
    let path = rs_solve_path(
        0.0,
        0.0,
        0.0,
        -4.0,
        -5.0,
        std::f64::consts::PI / 10.0,
        2.0,
        2.0,
        0.05,
        2.0,
    )
    .expect("path should exist");

    assert!(path.total_length() >= rs_euclidean_distance(0.0, 0.0, -4.0, -5.0));
    assert_relative_eq!(path.runway_length(), 2.0, epsilon = 1e-9);
    assert!(path.segment_count() >= 2);
}
