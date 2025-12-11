use nalgebra;

use crate::{matrix_exp6, vec_to_se3};

/// Computes forward kinematics in the body frame for an open chain robot.
///
/// # Arguments
///
/// * `m` - The home configuration (position and orientation) of the end-effector
/// * `blist` - The joint screw axes in the end-effector frame when the manipulator
///   is at the home position
/// * `thetalist` - A list of joint coordinates
///
/// # Returns
///
/// T in SE(3) representing the end-effector frame when the joints are
/// at the specified coordinates
///
/// # Example
///
/// ```
/// use nalgebra::{Matrix4, Vector6, DVector};
/// use modern_robotics::fkin_body;
///
/// let m = Matrix4::new(
///     -1.0, 0.0,  0.0, 0.0,
///      0.0, 1.0,  0.0, 6.0,
///      0.0, 0.0, -1.0, 2.0,
///      0.0, 0.0,  0.0, 1.0
/// );
/// let blist = vec![
///     Vector6::new(0.0, 0.0, -1.0, 2.0, 0.0, 0.0),
///     Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
///     Vector6::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.1)
/// ];
/// let thetalist = DVector::from_vec(vec![
///     std::f64::consts::FRAC_PI_2, 3.0, std::f64::consts::PI
/// ]);
/// let t = fkin_body(&m, &blist, &thetalist);
/// ```
pub fn fkin_body(
    m: &nalgebra::Matrix4<f64>,
    blist: &Vec<nalgebra::Vector6<f64>>,
    thetalist: &nalgebra::DVector<f64>,
) -> nalgebra::Matrix4<f64> {
    let mut t = m.clone();

    for i in 0..thetalist.len() {
        let bvec = blist[i];
        let theta = thetalist[i];
        let se3mat = vec_to_se3(&(bvec * theta));
        let t_ij = matrix_exp6(&se3mat);
        t = t * t_ij;
    }

    return t;
}

/// Computes forward kinematics in the space frame for an open chain robot.
///
/// # Arguments
///
/// * `m` - The home configuration (position and orientation) of the end-effector
/// * `slist` - The joint screw axes in the space frame when the manipulator
///   is at the home position
/// * `thetalist` - A list of joint coordinates
///
/// # Returns
///
/// T in SE(3) representing the end-effector frame when the joints are
/// at the specified coordinates
///
/// # Example
///
/// ```
/// use nalgebra::{Matrix4, Vector6, DVector};
/// use modern_robotics::fkin_space;
///
/// let m = Matrix4::new(
///     -1.0, 0.0,  0.0, 0.0,
///      0.0, 1.0,  0.0, 6.0,
///      0.0, 0.0, -1.0, 2.0,
///      0.0, 0.0,  0.0, 1.0
/// );
/// let slist = vec![
///     Vector6::new(0.0, 0.0, 1.0, 4.0, 0.0, 0.0),
///     Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
///     Vector6::new(0.0, 0.0, -1.0, -6.0, 0.0, -0.1)
/// ];
/// let thetalist = DVector::from_vec(vec![
///     std::f64::consts::FRAC_PI_2, 3.0, std::f64::consts::PI
/// ]);
/// let t = fkin_space(&m, &slist, &thetalist);
/// ```
pub fn fkin_space(
    m: &nalgebra::Matrix4<f64>,
    slist: &Vec<nalgebra::Vector6<f64>>,
    thetalist: &nalgebra::DVector<f64>,
) -> nalgebra::Matrix4<f64> {
    let mut t = m.clone();

    for i in 0..thetalist.len() {
        let j = thetalist.len() - 1 - i;
        let svec = slist[j];
        let theta = thetalist[j];
        let se3mat = vec_to_se3(&(svec * theta));
        let t_ij = matrix_exp6(&se3mat);
        t = t_ij * t;
    }

    return t;
}
