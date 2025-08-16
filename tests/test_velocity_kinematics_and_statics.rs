use assert_float_eq::assert_float_absolute_eq;
use modern_robotics;
use nalgebra;

const TOLERANCE: f64 = 1e-6;

#[test]
fn test_jacobian_body() {
    let blist = vec![
        nalgebra::Vector6::new(0.0, 0.0, 1.0, 0.0, 0.2, 0.2),
        nalgebra::Vector6::new(1.0, 0.0, 0.0, 2.0, 0.0, 3.0),
        nalgebra::Vector6::new(0.0, 1.0, 0.0, 0.0, 2.0, 1.0),
        nalgebra::Vector6::new(1.0, 0.0, 0.0, 0.2, 0.3, 0.4),
    ];
    let thetalist = vec![0.2, 1.1, 0.1, 1.2];

    let jb = modern_robotics::jacobian_body(blist, thetalist);

    assert_float_absolute_eq!(jb[(0, 0)], -0.04528405, TOLERANCE);
    assert_float_absolute_eq!(jb[(1, 0)], 0.74359313, TOLERANCE);
    assert_float_absolute_eq!(jb[(2, 0)], -0.66709716, TOLERANCE);
    assert_float_absolute_eq!(jb[(3, 0)], 2.32586047, TOLERANCE);
    assert_float_absolute_eq!(jb[(4, 0)], -1.44321167, TOLERANCE);
    assert_float_absolute_eq!(jb[(5, 0)], -2.06639565, TOLERANCE);
    assert_float_absolute_eq!(jb[(0, 1)], 0.99500417, TOLERANCE);
    assert_float_absolute_eq!(jb[(1, 1)], 0.09304865, TOLERANCE);
    assert_float_absolute_eq!(jb[(2, 1)], 0.03617541, TOLERANCE);
    assert_float_absolute_eq!(jb[(3, 1)], 1.66809, TOLERANCE);
    assert_float_absolute_eq!(jb[(4, 1)], 2.94561275, TOLERANCE);
    assert_float_absolute_eq!(jb[(5, 1)], 1.82881722, TOLERANCE);
    assert_float_absolute_eq!(jb[(0, 2)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(jb[(1, 2)], 0.36235775, TOLERANCE);
    assert_float_absolute_eq!(jb[(2, 2)], -0.93203909, TOLERANCE);
    assert_float_absolute_eq!(jb[(3, 2)], 0.56410831, TOLERANCE);
    assert_float_absolute_eq!(jb[(4, 2)], 1.43306521, TOLERANCE);
    assert_float_absolute_eq!(jb[(5, 2)], -1.58868628, TOLERANCE);
    assert_float_absolute_eq!(jb[(0, 3)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(jb[(1, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(jb[(2, 3)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(jb[(3, 3)], 0.2, TOLERANCE);
    assert_float_absolute_eq!(jb[(4, 3)], 0.3, TOLERANCE);
    assert_float_absolute_eq!(jb[(5, 3)], 0.4, TOLERANCE);
}

#[test]
fn test_jacobian_space() {
    let slist = vec![
        nalgebra::Vector6::new(0.0, 0.0, 1.0, 0.0, 0.2, 0.2),
        nalgebra::Vector6::new(1.0, 0.0, 0.0, 2.0, 0.0, 3.0),
        nalgebra::Vector6::new(0.0, 1.0, 0.0, 0.0, 2.0, 1.0),
        nalgebra::Vector6::new(1.0, 0.0, 0.0, 0.2, 0.3, 0.4),
    ];
    let thetalist = vec![0.2, 1.1, 0.1, 1.2];

    let js = modern_robotics::jacobian_space(slist, thetalist);
    println!("{js}");

    assert_float_absolute_eq!(js[(0, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(js[(1, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(js[(2, 0)], 1.0, TOLERANCE);
    assert_float_absolute_eq!(js[(3, 0)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(js[(4, 0)], 0.2, TOLERANCE);
    assert_float_absolute_eq!(js[(5, 0)], 0.2, TOLERANCE);
    assert_float_absolute_eq!(js[(0, 1)], 0.98006658, TOLERANCE);
    assert_float_absolute_eq!(js[(1, 1)], 0.19866933, TOLERANCE);
    assert_float_absolute_eq!(js[(2, 1)], 0.0, TOLERANCE);
    assert_float_absolute_eq!(js[(3, 1)], 1.95218638, TOLERANCE);
    assert_float_absolute_eq!(js[(4, 1)], 0.43654132, TOLERANCE);
    assert_float_absolute_eq!(js[(5, 1)], 2.96026613, TOLERANCE);
    assert_float_absolute_eq!(js[(0, 2)], -0.09011564, TOLERANCE);
    assert_float_absolute_eq!(js[(1, 2)], 0.4445544, TOLERANCE);
    assert_float_absolute_eq!(js[(2, 2)], 0.89120736, TOLERANCE);
    assert_float_absolute_eq!(js[(3, 2)], -2.21635216, TOLERANCE);
    assert_float_absolute_eq!(js[(4, 2)], -2.43712573, TOLERANCE);
    assert_float_absolute_eq!(js[(5, 2)], 3.23573065, TOLERANCE);
    assert_float_absolute_eq!(js[(0, 3)], 0.95749426, TOLERANCE);
    assert_float_absolute_eq!(js[(1, 3)], 0.28487557, TOLERANCE);
    assert_float_absolute_eq!(js[(2, 3)], -0.04528405, TOLERANCE);
    assert_float_absolute_eq!(js[(3, 3)], -0.51161537, TOLERANCE);
    assert_float_absolute_eq!(js[(4, 3)], 2.77535713, TOLERANCE);
    assert_float_absolute_eq!(js[(5, 3)], 2.22512443, TOLERANCE);
}
