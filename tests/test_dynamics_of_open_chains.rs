use assert_float_eq::assert_float_absolute_eq;
use modern_robotics;
use nalgebra;

const TOLERANCE: f64 = 1e-4;

#[test]
fn test_ad() {
    let v = nalgebra::Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    let adv = modern_robotics::ad(&v);

    assert_float_absolute_eq!(adv[(0, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(0, 1)], -3.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(0, 2)], 2.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(0, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(0, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(0, 5)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(1, 0)], 3.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(1, 2)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(1, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(1, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(1, 5)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(2, 0)], -2.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(2, 1)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(2, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(2, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(2, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(2, 5)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(3, 1)], -6.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(3, 2)], 5.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(3, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(3, 4)], -3.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(3, 5)], 2.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(4, 0)], 6.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(4, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(4, 2)], -4.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(4, 3)], 3.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(4, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(4, 5)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(5, 0)], -5.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(5, 1)], 4.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(5, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(5, 3)], -2.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(5, 4)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(adv[(5, 5)], 0.0, TOLERANCE);
}

#[test]
fn test_inverse_dynamics() {
    let thetalist = nalgebra::dvector![0.1, 0.1, 0.1];
    let dthetalist = nalgebra::dvector![0.1, 0.2, 0.3];
    let ddthetalist = nalgebra::dvector![2.0, 1.5, 1.0];
    let g = nalgebra::Vector3::new(0.0, 0.0, -9.8);
    let ftip = nalgebra::Vector6::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
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

    let taulist = modern_robotics::inverse_dynamics(
        &thetalist,
        &dthetalist,
        &ddthetalist,
        &g,
        &ftip,
        &mlist,
        &glist,
        &slist,
    );
    println!("{taulist}");

    assert!(taulist.len() == 3);
    assert_float_absolute_eq!(taulist[0], 74.69616155, TOLERANCE);
    assert_float_absolute_eq!(taulist[1], -33.06766016, TOLERANCE);
    assert_float_absolute_eq!(taulist[2], -3.23057314, TOLERANCE);
}

#[test]
fn test_mass_matrix() {
    let thetalist = nalgebra::dvector![0.1, 0.1, 0.1];
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

    let m = modern_robotics::mass_matrix(&thetalist, &mlist, &glist, &slist);

    assert!(m.shape().0 == 3);
    assert!(m.shape().1 == 3);
    assert_float_absolute_eq!(m[(0, 0)], 2.25433380e+01, TOLERANCE);
    assert_float_absolute_eq!(m[(0, 1)], -3.07146754e-01, TOLERANCE);
    assert_float_absolute_eq!(m[(0, 2)], -7.18426391e-03, TOLERANCE);
    assert_float_absolute_eq!(m[(1, 0)], -3.07146754e-01, TOLERANCE);
    assert_float_absolute_eq!(m[(1, 1)], 1.96850717e+00, TOLERANCE);
    assert_float_absolute_eq!(m[(1, 2)], 4.32157368e-01, TOLERANCE);
    assert_float_absolute_eq!(m[(2, 0)], -7.18426391e-03, TOLERANCE);
    assert_float_absolute_eq!(m[(2, 1)], 4.32157368e-01, TOLERANCE);
    assert_float_absolute_eq!(m[(2, 2)], 1.91630858e-01, TOLERANCE);
}

#[test]
fn test_vel_quadratic_forces() {
    let thetalist = nalgebra::dvector![0.1, 0.1, 0.1];
    let dthetalist = nalgebra::dvector![0.1, 0.2, 0.3];
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

    let c = modern_robotics::vel_quadratic_forces(&thetalist, &dthetalist, &mlist, &glist, &slist);

    assert_float_absolute_eq!(c[0], 0.26453118, TOLERANCE);
    assert_float_absolute_eq!(c[1], -0.05505157, TOLERANCE);
    assert_float_absolute_eq!(c[2], -0.00689132, TOLERANCE);
}

#[test]
fn test_gravity_forces() {
    let thetalist = nalgebra::dvector![0.1, 0.1, 0.1];
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

    let grav = modern_robotics::gravity_forces(&thetalist, &g, &mlist, &glist, &slist);

    assert!(grav.len() == 3);
    assert_float_absolute_eq!(grav[0], 28.40331262, TOLERANCE);
    assert_float_absolute_eq!(grav[1], -37.64094817, TOLERANCE);
    assert_float_absolute_eq!(grav[2], -5.4415892, TOLERANCE);
}

#[test]
fn test_end_effector_forces() {
    let thetalist = nalgebra::dvector![0.1, 0.1, 0.1];
    let ftip = nalgebra::Vector6::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
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

    let jt_ftip = modern_robotics::end_effector_forces(&thetalist, &ftip, &mlist, &glist, &slist);

    assert!(jt_ftip.len() == 3);
    assert_float_absolute_eq!(jt_ftip[0], 1.40954608, TOLERANCE);
    assert_float_absolute_eq!(jt_ftip[1], 1.85771497, TOLERANCE);
    assert_float_absolute_eq!(jt_ftip[2], 1.392409, TOLERANCE);
}

#[test]
fn test_forward_dynamics() {
    let thetalist = nalgebra::dvector![0.1, 0.1, 0.1];
    let dthetalist = nalgebra::dvector![0.1, 0.2, 0.3];
    let taulist = nalgebra::dvector![0.5, 0.6, 0.7];
    let g = nalgebra::Vector3::new(0.0, 0.0, -9.8);
    let ftip = nalgebra::Vector6::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
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

    let ddthetalist = modern_robotics::forward_dynamics(
        &thetalist,
        &dthetalist,
        &taulist,
        &g,
        &ftip,
        &mlist,
        &glist,
        &slist,
    );

    assert!(ddthetalist.len() == 3);
    assert_float_absolute_eq!(ddthetalist[0], -0.97392907, TOLERANCE);
    assert_float_absolute_eq!(ddthetalist[1], 25.58466784, TOLERANCE);
    assert_float_absolute_eq!(ddthetalist[2], -32.91499212, TOLERANCE);
}

#[test]
fn test_euler_step() {
    let thetalist = nalgebra::dvector![0.1, 0.1, 0.1];
    let dthetalist = nalgebra::dvector![0.1, 0.2, 0.3];
    let ddthetalist = nalgebra::dvector![2.0, 1.5, 1.0];
    let dt = 0.1;

    let (thetalist_next, dthetalist_next) =
        modern_robotics::euler_step(&thetalist, &dthetalist, &ddthetalist, dt);

    assert!(thetalist_next.len() == 3);
    assert!(dthetalist_next.len() == 3);
    assert_float_absolute_eq!(thetalist_next[0], 0.11, TOLERANCE);
    assert_float_absolute_eq!(thetalist_next[1], 0.12, TOLERANCE);
    assert_float_absolute_eq!(thetalist_next[2], 0.13, TOLERANCE);
    assert_float_absolute_eq!(dthetalist_next[0], 0.3, TOLERANCE);
    assert_float_absolute_eq!(dthetalist_next[1], 0.35, TOLERANCE);
    assert_float_absolute_eq!(dthetalist_next[2], 0.4, TOLERANCE);
}
