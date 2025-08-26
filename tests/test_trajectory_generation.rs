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
