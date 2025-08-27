use assert_float_eq::assert_float_absolute_eq;
use modern_robotics;

const TOLERANCE: f64 = 1e-6;

#[test]
fn test_cubic_time_scaling() {
    let tf = 2.0;
    let t = 0.6;

    let s = modern_robotics::cubic_time_scaling(tf, t);

    assert_float_absolute_eq!(s, 0.2160, TOLERANCE);
}

#[test]
fn test_qintic_time_scaling() {
    let tf = 2.0;
    let t = 0.6;

    let s = modern_robotics::quintic_time_scaling(tf, t);

    assert_float_absolute_eq!(s, 0.16308, TOLERANCE);
}

#[test]
fn test_joint_trajectory() {
    let thetastart = nalgebra::dvector![1.0, 0.0, 0.0, 1.0, 1.0, 0.2, 0.0, 1.0];
    let thetaend = nalgebra::dvector![1.2, 0.5, 0.6, 1.1, 2.0, 2.0, 0.9, 1.0];
    let tf = 4.0;
    let n = 6;
    let method = modern_robotics::Method::Cubic;

    let traj = modern_robotics::joint_trajectory(&thetastart, &thetaend, tf, n, method);

    assert!(traj.len() == 6);
    assert_float_absolute_eq!(traj[0][0], 1.0, TOLERANCE);
    assert_float_absolute_eq!(traj[0][1], 0.0, TOLERANCE);
    assert_float_absolute_eq!(traj[0][2], 0.0, TOLERANCE);
    assert_float_absolute_eq!(traj[0][3], 1.0, TOLERANCE);
    assert_float_absolute_eq!(traj[0][4], 1.0, TOLERANCE);
    assert_float_absolute_eq!(traj[0][5], 0.2, TOLERANCE);
    assert_float_absolute_eq!(traj[0][6], 0.0, TOLERANCE);
    assert_float_absolute_eq!(traj[0][7], 1.0, TOLERANCE);
    assert_float_absolute_eq!(traj[1][0], 1.0208, TOLERANCE);
    assert_float_absolute_eq!(traj[1][1], 0.052, TOLERANCE);
    assert_float_absolute_eq!(traj[1][2], 0.0624, TOLERANCE);
    assert_float_absolute_eq!(traj[1][3], 1.0104, TOLERANCE);
    assert_float_absolute_eq!(traj[1][4], 1.104, TOLERANCE);
    assert_float_absolute_eq!(traj[1][5], 0.3872, TOLERANCE);
    assert_float_absolute_eq!(traj[1][6], 0.0936, TOLERANCE);
    assert_float_absolute_eq!(traj[1][7], 1.0, TOLERANCE);
    assert_float_absolute_eq!(traj[2][0], 1.0704, TOLERANCE);
    assert_float_absolute_eq!(traj[2][1], 0.176, TOLERANCE);
    assert_float_absolute_eq!(traj[2][2], 0.2112, TOLERANCE);
    assert_float_absolute_eq!(traj[2][3], 1.0352, TOLERANCE);
    assert_float_absolute_eq!(traj[2][4], 1.352, TOLERANCE);
    assert_float_absolute_eq!(traj[2][5], 0.8336, TOLERANCE);
    assert_float_absolute_eq!(traj[2][6], 0.3168, TOLERANCE);
    assert_float_absolute_eq!(traj[2][7], 1.0, TOLERANCE);
    assert_float_absolute_eq!(traj[3][0], 1.1296, TOLERANCE);
    assert_float_absolute_eq!(traj[3][1], 0.324, TOLERANCE);
    assert_float_absolute_eq!(traj[3][2], 0.3888, TOLERANCE);
    assert_float_absolute_eq!(traj[3][3], 1.0648, TOLERANCE);
    assert_float_absolute_eq!(traj[3][4], 1.648, TOLERANCE);
    assert_float_absolute_eq!(traj[3][5], 1.3664, TOLERANCE);
    assert_float_absolute_eq!(traj[3][6], 0.5832, TOLERANCE);
    assert_float_absolute_eq!(traj[3][7], 1.0, TOLERANCE);
    assert_float_absolute_eq!(traj[4][0], 1.1792, TOLERANCE);
    assert_float_absolute_eq!(traj[4][1], 0.448, TOLERANCE);
    assert_float_absolute_eq!(traj[4][2], 0.5376, TOLERANCE);
    assert_float_absolute_eq!(traj[4][3], 1.0896, TOLERANCE);
    assert_float_absolute_eq!(traj[4][4], 1.896, TOLERANCE);
    assert_float_absolute_eq!(traj[4][5], 1.8128, TOLERANCE);
    assert_float_absolute_eq!(traj[4][6], 0.8064, TOLERANCE);
    assert_float_absolute_eq!(traj[4][7], 1.0, TOLERANCE);
    assert_float_absolute_eq!(traj[5][0], 1.2, TOLERANCE);
    assert_float_absolute_eq!(traj[5][1], 0.5, TOLERANCE);
    assert_float_absolute_eq!(traj[5][2], 0.6, TOLERANCE);
    assert_float_absolute_eq!(traj[5][3], 1.1, TOLERANCE);
    assert_float_absolute_eq!(traj[5][4], 2.0, TOLERANCE);
    assert_float_absolute_eq!(traj[5][5], 2.0, TOLERANCE);
    assert_float_absolute_eq!(traj[5][6], 0.9, TOLERANCE);
    assert_float_absolute_eq!(traj[5][7], 1.0, TOLERANCE);
}
