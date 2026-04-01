mod builder;
mod control;
mod matrix_utils;
mod types;

pub use types::MpcControlResult;

#[cfg(test)]
mod tests {
    use super::builder::linear_mpc_control;
    use super::control::{mpc_control_preview, predict_motion};
    use super::types::{MAX_ACCEL, MAX_STEER, RollingCarState};

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
            RollingCarState {
                x: 0.0,
                y: 0.0,
                velocity: 3.0,
                yaw: 0.0,
                steer: 0.0,
            },
            &[[0.0, 0.0]; 5],
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
