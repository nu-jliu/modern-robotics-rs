extern crate modern_robotics;
extern crate nalgebra;

#[test]
fn test_near_zero() {
    assert!(modern_robotics::modern_robotics::near_zero(1e-7));
    assert!(modern_robotics::modern_robotics::near_zero(0.0));
    assert!(!modern_robotics::modern_robotics::near_zero(1e-5));
    assert!(!modern_robotics::modern_robotics::near_zero(1.0));
}

#[test]
fn test_normalize_3d() {
    let v = nalgebra::DVector::from_vec(vec![1.0, 1.0, 1.0]);
    let norm_v = modern_robotics::modern_robotics::normalize(v);
    
    let expected_val = 1.0 / 3.0_f64.sqrt();
    assert!((norm_v[0] - expected_val).abs() < 1e-10);
    assert!((norm_v[1] - expected_val).abs() < 1e-10);
    assert!((norm_v[2] - expected_val).abs() < 1e-10);
}

#[test]
fn test_normalize_2d() {
    let v = nalgebra::DVector::from_vec(vec![3.0, 4.0]);
    let norm_v = modern_robotics::modern_robotics::normalize(v);
    
    assert!((norm_v[0] - 0.6).abs() < 1e-10);
    assert!((norm_v[1] - 0.8).abs() < 1e-10);
}

#[test]
fn test_normalize_4d() {
    let v = nalgebra::DVector::from_vec(vec![1.0, 2.0, 2.0, 1.0]);
    let norm_v = modern_robotics::modern_robotics::normalize(v);
    
    let magnitude = (1.0 + 4.0 + 4.0 + 1.0_f64).sqrt();
    assert!((norm_v[0] - 1.0 / magnitude).abs() < 1e-10);
    assert!((norm_v[1] - 2.0 / magnitude).abs() < 1e-10);
    assert!((norm_v[2] - 2.0 / magnitude).abs() < 1e-10);
    assert!((norm_v[3] - 1.0 / magnitude).abs() < 1e-10);
}

#[test]
fn test_normalize_magnitude() {
    let v = nalgebra::DVector::from_vec(vec![5.0, 12.0]);
    let norm_v = modern_robotics::modern_robotics::normalize(v);
    
    let magnitude = norm_v.magnitude();
    assert!((magnitude - 1.0).abs() < 1e-10);
}
