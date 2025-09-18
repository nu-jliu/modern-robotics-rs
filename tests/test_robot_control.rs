use assert_float_eq::assert_float_absolute_eq;
use modern_robotics;
use nalgebra;

const TOLERANCE: f64 = 1e-6;

#[test]
fn test_compute_torque() {
    let thetalist = nalgebra::dvector![0.1, 0.1, 0.1];
    let dthetalist = nalgebra::dvector![0.1, 0.2, 0.3];
    let eint = nalgebra::dvector![0.2, 0.2, 0.2];
    let g = nalgebra::Vector3::new(0.0, 0.0, -9.8);

    let m01 = nalgebra::matrix![
        1.0, 0.0, 0.0, 0.0;
        0.0, 1.0, 0.0, 0.0;
        0.0, 0.0, 1.0, 0.089159;
        0.0, 0.0, 0.0, 1.0
    ];
    let m12 = nalgebra::matrix![
        0.0, 0.0, 1.0, 0.28;
        0.0, 1.0, 0.0, 0.13585;
        -1.0, 0.0, 0.0, 0.0;
        0.0, 0.0, 0.0, 1.0
    ];
    let m23 = nalgebra::matrix![
        1.0, 0.0, 0.0, 0.0;
        0.0, 1.0, 0.0, -0.1197;
        0.0, 0.0, 1.0, 0.395;
        0.0, 0.0, 0.0, 1.0
    ];
    let m34 = nalgebra::matrix![
        1.0, 0.0, 0.0, 0.0;
        0.0, 1.0, 0.0, 0.0;
        0.0, 0.0, 1.0, 0.14225;
        0.0, 0.0, 0.0, 1.0
    ];
    let g1 = nalgebra::Matrix6::from_diagonal(&nalgebra::Vector6::new(
        0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7,
    ));
    let g2 = nalgebra::Matrix6::from_diagonal(&nalgebra::Vector6::new(
        0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393,
    ));
    let g3 = nalgebra::Matrix6::from_diagonal(&nalgebra::Vector6::new(
        0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275,
    ));
    let s1 = nalgebra::Vector6::new(1.0, 0.0, 1.0, 0.0, 1.0, 0.0);
    let s2 = nalgebra::Vector6::new(0.0, 1.0, 0.0, -0.089, 0.0, 0.0);
    let s3 = nalgebra::Vector6::new(0.0, 1.0, 0.0, -0.089, 0.0, 0.425);
    let mlist = vec![m01, m12, m23, m34];
    let glist = vec![g1, g2, g3];
    let slist = vec![s1, s2, s3];

    let thetalistd = nalgebra::dvector![1.0, 1.0, 1.0];
    let dthetalistd = nalgebra::dvector![2.0, 1.2, 2.0];
    let ddthetalistd = nalgebra::dvector![0.1, 0.1, 0.1];

    let kp = 1.3;
    let ki = 1.2;
    let kd = 1.1;

    let taulist = modern_robotics::compute_torque(
        &thetalist,
        &dthetalist,
        &eint,
        &g,
        &mlist,
        &glist,
        &slist,
        &thetalistd,
        &dthetalistd,
        &ddthetalistd,
        kp,
        ki,
        kd,
    );

    assert_float_absolute_eq!(taulist[0], 133.00525246, TOLERANCE);
    assert_float_absolute_eq!(taulist[1], -29.94223324, TOLERANCE);
    assert_float_absolute_eq!(taulist[2], -3.03276856, TOLERANCE);
}
