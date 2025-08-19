use assert_float_eq::assert_float_absolute_eq;
use modern_robotics;
use nalgebra;

const TOLERANCE: f64 = 1e-6;

#[test]
fn test_rot_inv() {
    let r = nalgebra::Matrix3::new(0., 0., 1., 1., 0., 0., 0., 1., 0.);
    let r_inv = modern_robotics::rot_inv(&r);

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
    let so3mat = modern_robotics::vec_to_so3(&omg);

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
    let omg = modern_robotics::so3_to_vec(&so3mat);

    assert_eq!(omg[0], 1.);
    assert_eq!(omg[1], 2.);
    assert_eq!(omg[2], 3.);
}

#[test]
fn test_axis_ang3() {
    let expc3 = nalgebra::Vector3::new(1., 2., 3.);
    let (omghat, theta) = modern_robotics::axis_ang3(&expc3);

    assert_float_absolute_eq!(omghat[0], 0.26726124, TOLERANCE);
    assert_float_absolute_eq!(omghat[1], 0.53452248, TOLERANCE);
    assert_float_absolute_eq!(omghat[2], 0.80178373, TOLERANCE);

    assert_float_absolute_eq!(theta, 3.7416573867739413, TOLERANCE);
}

#[test]
fn test_matrix_exp3() {
    let so3mat = nalgebra::Matrix3::new(0., -3., 2., 3., 0., -1., -2., 1., 0.);
    let r = modern_robotics::matrix_exp3(&so3mat);

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

#[test]
fn test_matrix_log3() {
    let r = nalgebra::Matrix3::new(0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    let so3mat = modern_robotics::matrix_log3(&r);

    assert_float_absolute_eq!(so3mat[(0, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(so3mat[(0, 1)], -1.20919958, TOLERANCE);
    assert_float_absolute_eq!(so3mat[(0, 2)], 1.20919958, TOLERANCE);
    assert_float_absolute_eq!(so3mat[(1, 0)], 1.20919958, TOLERANCE);
    assert_float_absolute_eq!(so3mat[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(so3mat[(1, 2)], -1.20919958, TOLERANCE);
    assert_float_absolute_eq!(so3mat[(2, 0)], -1.20919958, TOLERANCE);
    assert_float_absolute_eq!(so3mat[(2, 1)], 1.20919958, TOLERANCE);
    assert_float_absolute_eq!(so3mat[(2, 2)], 0.0, TOLERANCE);
}

#[test]

fn test_rp_to_trans() {
    let r = nalgebra::Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
    let p = nalgebra::Vector3::new(1.0, 2.0, 5.0);
    let t = modern_robotics::rp_to_trans(&r, &p);

    assert_float_absolute_eq!(t[(0, 0)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 3)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 2)], -1., TOLERANCE);
    assert_float_absolute_eq!(t[(1, 3)], 2.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 1)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 3)], 5.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 3)], 1.0, TOLERANCE);
}

#[test]
fn test_trans_to_rp() {
    let t = nalgebra::Matrix4::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0,
    );
    let (r, p) = modern_robotics::trans_to_rp(&t);

    assert_float_absolute_eq!(r[(0, 0)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(r[(0, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(r[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(r[(1, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(r[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(r[(1, 2)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(r[(2, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(r[(2, 1)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(r[(2, 2)], 0.0, TOLERANCE);

    assert_float_absolute_eq!(p[0], 0.0, TOLERANCE);
    assert_float_absolute_eq!(p[1], 0.0, TOLERANCE);
    assert_float_absolute_eq!(p[2], 3.0, TOLERANCE);
}

#[test]
fn test_trans_inv() {
    let t = nalgebra::Matrix4::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0,
    );
    let t_inv = modern_robotics::trans_inv(&t);

    assert_float_absolute_eq!(t_inv[(0, 0)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(0, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(0, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(1, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(1, 2)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(1, 3)], -3.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(2, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(2, 1)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(2, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(2, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(3, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(3, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t_inv[(3, 3)], 1.0, TOLERANCE);
}

#[test]
fn test_vec_to_se3() {
    let v = nalgebra::Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    let se3mat = modern_robotics::vec_to_se3(&v);

    assert_float_absolute_eq!(se3mat[(0, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(0, 1)], -3.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(0, 2)], 2.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(0, 3)], 4.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(1, 0)], 3.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(1, 2)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(1, 3)], 5.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(2, 0)], -2.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(2, 1)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(2, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(2, 3)], 6.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(3, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(3, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(3, 3)], 0.0, TOLERANCE);
}

#[test]
fn test_se3_to_vec() {
    let se3mat = nalgebra::Matrix4::new(
        0.0, -3.0, 2.0, 4.0, 3.0, 0.0, -1.0, 5.0, -2.0, 1.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    );
    let v = modern_robotics::se3_to_vec(&se3mat);

    assert_float_absolute_eq!(v[0], 1.0, TOLERANCE);
    assert_float_absolute_eq!(v[1], 2.0, TOLERANCE);
    assert_float_absolute_eq!(v[2], 3.0, TOLERANCE);
    assert_float_absolute_eq!(v[3], 4.0, TOLERANCE);
    assert_float_absolute_eq!(v[4], 5.0, TOLERANCE);
    assert_float_absolute_eq!(v[5], 6.0, TOLERANCE);
}

#[test]
fn test_adjoint() {
    let t = nalgebra::Matrix4::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0,
    );
    let ad_t = modern_robotics::adjoint(&t);

    assert_float_absolute_eq!(ad_t[(0, 0)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(0, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(0, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(0, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(0, 5)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(1, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(1, 2)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(1, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(1, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(1, 5)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(2, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(2, 1)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(2, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(2, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(2, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(2, 5)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(3, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(3, 2)], 3.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(3, 3)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(3, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(3, 5)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(4, 0)], 3.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(4, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(4, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(4, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(4, 4)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(4, 5)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(5, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(5, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(5, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(5, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(5, 4)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(ad_t[(5, 5)], 0.0, TOLERANCE);
}

#[test]
fn test_screw_to_axis() {
    let q = nalgebra::Vector3::new(3.0, 0.0, 0.0);
    let s = nalgebra::Vector3::new(0.0, 0.0, 1.0);
    let h = 2.0;
    let v = modern_robotics::screw_to_axis(&q, &s, h);

    assert_float_absolute_eq!(v[0], 0.0, TOLERANCE);
    assert_float_absolute_eq!(v[1], 0.0, TOLERANCE);
    assert_float_absolute_eq!(v[2], 1.0, TOLERANCE);
    assert_float_absolute_eq!(v[3], 0.0, TOLERANCE);
    assert_float_absolute_eq!(v[4], -3.0, TOLERANCE);
    assert_float_absolute_eq!(v[5], 2.0, TOLERANCE);
}

#[test]
fn test_axis_ang6() {
    let expc6 = nalgebra::Vector6::new(1.0, 0.0, 0.0, 1.0, 2.0, 3.0);
    let (s, theta) = modern_robotics::axis_ang6(&expc6);

    assert_float_absolute_eq!(s[0], 1.0, TOLERANCE);
    assert_float_absolute_eq!(s[1], 0.0, TOLERANCE);
    assert_float_absolute_eq!(s[2], 0.0, TOLERANCE);
    assert_float_absolute_eq!(s[3], 1.0, TOLERANCE);
    assert_float_absolute_eq!(s[4], 2.0, TOLERANCE);
    assert_float_absolute_eq!(s[5], 3.0, TOLERANCE);

    assert_float_absolute_eq!(theta, 1.0, TOLERANCE)
}

#[test]
fn test_matrix_exp6() {
    let se3mat = nalgebra::Matrix4::new(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -1.57079632,
        2.35619449,
        0.0,
        1.57079632,
        0.0,
        2.35619449,
        0.0,
        0.0,
        0.0,
        0.0,
    );
    let t = modern_robotics::matrix_exp6(&se3mat);

    assert_float_absolute_eq!(t[(0, 0)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 2)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 1)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 3)], 3.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 3)], 1.0, TOLERANCE);
}

#[test]
fn test_matrix_log6() {
    let t = nalgebra::Matrix4::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0,
    );
    let se3mat = modern_robotics::matrix_log6(&t);

    assert_float_absolute_eq!(se3mat[(0, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(0, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(0, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(1, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(1, 2)], -1.57079633, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(1, 3)], 2.35619449, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(2, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(2, 1)], 1.57079633, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(2, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(2, 3)], 2.35619449, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(3, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(3, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(se3mat[(3, 3)], 0.0, TOLERANCE);
}
