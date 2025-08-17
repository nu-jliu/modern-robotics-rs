use assert_float_eq::assert_float_absolute_eq;
use modern_robotics::{self, ikin_space};
use nalgebra;

const TOLERANCE: f64 = 5e-4;

#[test]
fn test_ikin_body() {
    let blist = vec![
        nalgebra::Vector6::new(0.0, 0.0, -1.0, 2.0, 0.0, 0.0),
        nalgebra::Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        nalgebra::Vector6::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.1),
    ];
    let m = nalgebra::Matrix4::new(
        -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, -1.0, 2.0, 0.0, 0.0, 0.0, 1.0,
    );
    let t = nalgebra::Matrix4::new(
        0.0, 1.0, 0.0, -5.0, 1.0, 0.0, 0.0, 4.0, 0.0, 0.0, -1.0, 1.6858, 0.0, 0.0, 0.0, 1.0,
    );
    let thetalist0 = nalgebra::dvector![1.5, 2.5, 3.0];
    let emog = 0.01;
    let ev = 0.001;

    let (thetalist, success) = modern_robotics::ikin_body(&blist, &m, &t, &thetalist0, emog, ev);
    println!("{thetalist}");
    println!("{success}");

    assert!(success);
    assert_float_absolute_eq!(thetalist[0], 1.57073819, TOLERANCE);
    assert_float_absolute_eq!(thetalist[1], 2.999667, TOLERANCE);
    assert_float_absolute_eq!(thetalist[2], 3.14153913, TOLERANCE);
}

#[test]
fn test_ikin_space() {
    let slist = vec![
        nalgebra::Vector6::new(0.0, 0.0, 1.0, 4.0, 0.0, 0.0),
        nalgebra::Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        nalgebra::Vector6::new(0.0, 0.0, -1.0, -6.0, 0.0, -0.1),
    ];
    let m = nalgebra::Matrix4::new(
        -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, -1.0, 2.0, 0.0, 0.0, 0.0, 1.0,
    );
    let t = nalgebra::Matrix4::new(
        0.0, 1.0, 0.0, -5.0, 1.0, 0.0, 0.0, 4.0, 0.0, 0.0, -1.0, 1.6858, 0.0, 0.0, 0.0, 1.0,
    );
    let thetalist0 = nalgebra::dvector![1.5, 2.5, 3.0];
    let emog = 0.01;
    let ev = 0.001;

    let (thetalist, success) = ikin_space(&slist, &m, &t, &thetalist0, emog, ev);

    assert!(success);
    assert_float_absolute_eq!(thetalist[0], 1.57073819, TOLERANCE);
    assert_float_absolute_eq!(thetalist[1], 2.999667, TOLERANCE);
    assert_float_absolute_eq!(thetalist[2], 3.14153913, TOLERANCE);
}
