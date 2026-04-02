mod config;
mod tracker;
mod trajectory;
mod types;

#[cfg(test)]
pub(crate) use config::MpcPrepConfig;
pub use tracker::MpcReferenceTracker;
pub use types::MpcReferenceResult;

#[cfg(test)]
pub(crate) use tracker::mpc_prepare_reference;
#[cfg(test)]
pub(crate) use trajectory::{process_reference_trajectory, smooth_yaws};

#[cfg(test)]
mod tests {
    use super::{MpcPrepConfig, MpcReferenceTracker, mpc_prepare_reference, process_reference_trajectory, smooth_yaws};

    #[test]
    fn prepares_reference_and_brake_preview() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 1.0, 15.0, 0.0, 0.0, 1.0,
        ];

        let result = mpc_prepare_reference(trajectory, 1.0, 0.0, 0.0, 6.0, 0.07, false).expect("mpc prep");
        assert_eq!(result.model_reference_states().len(), 24);
        assert_eq!(result.reference_states().len(), 24);
        assert!(!result.brake_trajectory().is_empty());
    }

    #[test]
    fn handles_direction_change_stop_points() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 4.0, 0.0, 0.0, 1.0, 4.0, -2.0, -1.57, -1.0, 4.0, -5.0, -1.57, -1.0,
        ];

        let tracker = MpcReferenceTracker::new(trajectory).expect("tracker");
        assert!(!tracker.prepared.direction_change_us.is_empty());
    }

    #[test]
    fn smooth_yaws_preserves_first_heading_like_python() {
        let mut points = vec![[0.0, 0.0, 1.2, 1.0], [1.0, 0.0, 1.25, 1.0], [2.0, 0.0, 1.3, 1.0]];

        smooth_yaws(&mut points);

        assert!((points[0][2] - 1.2).abs() < 1e-9);
        assert!((points[1][2] - 1.25).abs() < 1e-9);
        assert!((points[2][2] - 1.3).abs() < 1e-9);
    }

    #[test]
    fn tracker_searches_forward_from_current_progress() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 1.0, 15.0, 0.0, 0.0, 1.0,
        ];

        let mut tracker = MpcReferenceTracker::new(trajectory).expect("tracker");
        let _ = tracker.update(8.0, 0.0, 0.0, 4.0, 0.07);
        let progressed_u = tracker.current_progress();
        let _ = tracker.update(1.0, 0.0, 0.0, 4.0, 0.07);

        assert!(tracker.current_progress() >= progressed_u - 1e-9);
        assert!(tracker.current_progress() > 5.0);
    }

    #[test]
    fn tracker_freezes_brake_limit_after_brake() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 1.0, 15.0, 0.0, 0.0, 1.0, 20.0, 0.0, 0.0, 1.0,
        ];

        let mut tracker = MpcReferenceTracker::new(trajectory).expect("tracker");
        let _ = tracker.update(2.0, 0.0, 0.0, 6.0, 0.07);
        tracker.brake();
        let first = tracker.update(2.5, 0.0, 0.0, 6.0, 0.07);
        let frozen_limit = tracker.progress_limit();
        let second = tracker.update(3.0, 0.0, 0.0, 5.0, 0.07);

        assert!(frozen_limit < 20.0);
        assert!((tracker.progress_limit() - frozen_limit).abs() < 1e-9);
        assert_eq!(first.brake_trajectory(), second.brake_trajectory());
    }

    #[test]
    fn rejects_zero_direction_input_like_python_assertion() {
        let config = MpcPrepConfig::default();
        let error = process_reference_trajectory(vec![[0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0]], &config)
            .err()
            .expect("zero direction must fail");

        assert_eq!(
            error,
            "the direction on each point of the trajectory should not be zero"
        );
    }

    #[test]
    fn removes_only_exact_adjacent_duplicate_xy_points() {
        let config = MpcPrepConfig::default();
        let prepared = process_reference_trajectory(
            vec![
                [0.0, 0.0, 0.1, 1.0],
                [0.0, 0.0, 0.2, 1.0],
                [1e-12, 0.0, 0.3, 1.0],
                [1.0, 0.0, 0.4, 1.0],
            ],
            &config,
        )
        .expect("prepared trajectory");

        assert_eq!(prepared.points.len(), 3);
        assert_eq!(prepared.points[0][0], 0.0);
        assert_eq!(prepared.points[1][0], 1e-12);
    }

    #[test]
    fn exposes_public_reference_and_brake_states_in_python_order() {
        let trajectory = vec![
            0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 1.0, 15.0, 0.0, 0.0, 1.0,
        ];

        let result = mpc_prepare_reference(trajectory, 1.0, 0.0, 0.0, 6.0, 0.07, false).expect("mpc prep");
        let model = result.model_reference_states();
        let public = result.reference_states();
        let brake = result.brake_trajectory();

        assert_eq!(model.len(), public.len());
        assert!(!brake.is_empty());
        assert_eq!(public[0], model[0]);
        assert_eq!(public[1], model[1]);
        assert_eq!(public[2], model[3]);
        assert_eq!(public[3], model[2]);
        assert_eq!(brake[0], result.model_reference_states()[0]);
    }
}
