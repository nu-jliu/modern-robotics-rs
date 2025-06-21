extern crate modern_robotics;
extern crate nalgebra;

#[test]
fn test_normalize() {
    let v: nalgebra::Vector3<f64> = nalgebra::Vector3::new(1.0, 1.0, 1.0);
    let norm_v = modern_robotics::modern_robotics::normalize(v);

    println!("The result vector is {norm_v}");
    let sum: f64 = 3.0;

    assert_eq!(norm_v[0], 1.0 / sum.sqrt());
}
