use assert_float_eq;
use modern_robotics;
use nalgebra;

const TOLERANCE: f64 = 1e-10;

#[test]
fn test_near_zero() {
    assert!(modern_robotics::near_zero(1e-7));
    assert!(modern_robotics::near_zero(0.0));
    assert!(!modern_robotics::near_zero(1e-5));
    assert!(!modern_robotics::near_zero(1.0));
}

#[test]
fn test_normalize() {
    let v = nalgebra::DVector::from_vec(vec![1.0, 2.0, 3.0]);
    let norm_v = modern_robotics::normalize(v);

    assert_float_eq::assert_float_absolute_eq!(norm_v[0], 1.0 / 14.0_f64.sqrt(), TOLERANCE);
    assert_float_eq::assert_float_absolute_eq!(norm_v[1], 2.0 / 14.0_f64.sqrt(), TOLERANCE);
    assert_float_eq::assert_float_absolute_eq!(norm_v[2], 3.0 / 14.0_f64.sqrt(), TOLERANCE);
}
