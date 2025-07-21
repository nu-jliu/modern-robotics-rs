use assert_float_eq::assert_float_absolute_eq;
use modern_robotics::rigid_body_motions;
use nalgebra;

const TOLERANCE: f64 = 1e-6;

#[test]
fn test_rot_inv() {
    let r = nalgebra::Matrix3::new(0., 0., 1., 1., 0., 0., 0., 1., 0.);
    let r_inv = rigid_body_motions::rot_inv(r);

    assert_eq!(r_inv[(0, 0)], 0.);
    assert_eq!(r_inv[(0, 1)], 1.);
    assert_eq!(r_inv[(0, 2)], 0.);
    assert_eq!(r_inv[(1, 0)], 0.);
    assert_eq!(r_inv[(1, 1)], 0.);
    assert_eq!(r_inv[(1, 2)], 1.);
    assert_eq!(r_inv[(2, 0)], 1.);
    assert_eq!(r_inv[(2, 1)], 0.);
    assert_eq!(r_inv[(2, 2)], 0.);
}

#[test]
fn test_vec_to_so3() {
    let omg = nalgebra::Vector3::new(1., 2., 3.);
    let so3mat = rigid_body_motions::vec_to_so3(omg);

    assert_eq!(so3mat[(0, 0)], 0.);
    assert_eq!(so3mat[(0, 1)], -3.);
    assert_eq!(so3mat[(0, 2)], 2.);
    assert_eq!(so3mat[(1, 0)], 3.);
    assert_eq!(so3mat[(1, 1)], 0.);
    assert_eq!(so3mat[(1, 2)], -1.);
    assert_eq!(so3mat[(2, 0)], -2.);
    assert_eq!(so3mat[(2, 1)], 1.);
    assert_eq!(so3mat[(2, 2)], 0.);
}

#[test]
fn test_so3_to_vec() {
    let so3mat = nalgebra::Matrix3::new(0., -3., 2., 3., 0., -1., -2., 1., 0.);
    let omg = rigid_body_motions::so3_to_vec(so3mat);

    assert_eq!(omg[0], 1.);
    assert_eq!(omg[1], 2.);
    assert_eq!(omg[2], 3.);
}

#[test]
fn test_axis_ang() {
    let expc3 = nalgebra::Vector3::new(1., 2., 3.);
    let (omghat, theta) = rigid_body_motions::axis_ang3(expc3);

    assert_float_absolute_eq!(omghat[0], 0.26726124, TOLERANCE);
    assert_float_absolute_eq!(omghat[1], 0.53452248, TOLERANCE);
    assert_float_absolute_eq!(omghat[2], 0.80178373, TOLERANCE);

    assert_float_absolute_eq!(theta, 3.7416573867739413, TOLERANCE);
}

#[test]
fn test_matrix_exp3() {
    let so3mat = nalgebra::Matrix3::new(0., -3., 2., 3., 0., -1., -2., 1., 0.);
    let r = rigid_body_motions::matrix_exp3(so3mat);

    assert_float_absolute_eq!(r[(0, 0)], -0.69492056, TOLERANCE);
    assert_float_absolute_eq!(r[(0, 1)], 0.71352099, TOLERANCE);
    assert_float_absolute_eq!(r[(0, 2)], 0.08929286, TOLERANCE);
    assert_float_absolute_eq!(r[(1, 0)], -0.19200697, TOLERANCE);
    assert_float_absolute_eq!(r[(1, 1)], -0.30378504, TOLERANCE);
    assert_float_absolute_eq!(r[(1, 2)], 0.93319235, TOLERANCE);
    assert_float_absolute_eq!(r[(2, 0)], 0.69297817, TOLERANCE);
    assert_float_absolute_eq!(r[(2, 1)], 0.6313497, TOLERANCE);
    assert_float_absolute_eq!(r[(2, 2)], 0.34810748, TOLERANCE);
}
