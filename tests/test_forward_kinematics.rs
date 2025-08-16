use std::f64::consts::PI;

use assert_float_eq::assert_float_absolute_eq;
use modern_robotics::{self, fkin_space};
use nalgebra;

const TOLERANCE: f64 = 1e-6;

#[test]
fn test_fkin_body() {
    let m = nalgebra::Matrix4::new(
        -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, -1.0, 2.0, 0.0, 0.0, 0.0, 1.0,
    );
    let blist = vec![
        nalgebra::Vector6::new(0.0, 0.0, -1.0, 2.0, 0.0, 0.0),
        nalgebra::Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        nalgebra::Vector6::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.1),
    ];
    let thetalist: Vec<f64> = vec![PI / 2.0, 3.0, PI];

    let t = modern_robotics::fkin_body(m, blist, thetalist);

    assert_float_absolute_eq!(t[(0, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 1)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 3)], -5.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 0)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 3)], 4.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 2)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 3)], 1.68584073, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 3)], 1.0, TOLERANCE);
}

#[test]
fn test_fkin_space() {
    let m = nalgebra::Matrix4::new(
        -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, -1.0, 2.0, 0.0, 0.0, 0.0, 1.0,
    );
    let slist = vec![
        nalgebra::Vector6::new(0.0, 0.0, 1.0, 4.0, 0.0, 0.0),
        nalgebra::Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        nalgebra::Vector6::new(0.0, 0.0, -1.0, -6.0, 0.0, -0.1),
    ];
    let thetalist = vec![PI / 2.0, 3.0, PI];

    let t = fkin_space(m, slist, thetalist);

    assert_float_absolute_eq!(t[(0, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 1)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(0, 3)], -5.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 0)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(1, 3)], 4.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 2)], -1.0, TOLERANCE);
    assert_float_absolute_eq!(t[(2, 3)], 1.68584073, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(t[(3, 3)], 1.0, TOLERANCE);
}
