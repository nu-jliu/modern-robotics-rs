use nalgebra;

use crate::{adjoint, matrix_exp6, vec_to_se3};

/// Computes the body Jacobian Jb(theta) for an open chain robot.
///
/// # Arguments
///
/// * `blist` - The joint screw axes in the end-effector frame when the
///   manipulator is at the home position, in the format of a matrix with the
///   screw axes as the columns
/// * `thetalist` - A list of joint coordinates
///
/// # Returns
///
/// The body Jacobian Jb(theta): the 6 x n Jacobian matrix relating the joint
/// velocities to the body twist
///
/// # Example
///
/// ```
/// use nalgebra::{Vector6, DVector};
/// use modern_robotics::jacobian_body;
///
/// let blist = vec![
///     Vector6::new(0.0, 0.0, 1.0, 0.0, 2.0, 0.0),
///     Vector6::new(0.0, 0.0, 0.0, 2.0, 0.0, 0.0)
/// ];
/// let thetalist = DVector::from_vec(vec![0.2, 1.1]);
/// let jb = jacobian_body(&blist, &thetalist);
/// ```
pub fn jacobian_body(
    blist: &Vec<nalgebra::Vector6<f64>>,
    thetalist: &nalgebra::DVector<f64>,
) -> nalgebra::Matrix6xX<f64> {
    let n = blist.len();
    let mut jb: nalgebra::Matrix6xX<f64> = nalgebra::Matrix6xX::zeros(n);
    let mut t: nalgebra::Matrix4<f64> = nalgebra::Matrix4::identity();

    let bvec = blist[n - 1];
    jb.set_column(n - 1, &bvec);

    for j in 0..(n - 1) {
        let i = n - 2 - j;
        let bvec = blist[i + 1];
        let theta = thetalist[i + 1];

        let v = -bvec * theta;
        let se3mat = vec_to_se3(&v);
        let tij = matrix_exp6(&se3mat);
        t = t * tij;

        let adt = adjoint(&t);
        let jbvec = adt * blist[i];
        jb.set_column(i, &jbvec);
    }

    return jb;
}

/// Computes the space Jacobian Js(theta) for an open chain robot.
///
/// # Arguments
///
/// * `slist` - The joint screw axes in the space frame when the manipulator
///   is at the home position, in the format of a matrix with the screw axes
///   as the columns
/// * `thetalist` - A list of joint coordinates
///
/// # Returns
///
/// The space Jacobian Js(theta): the 6 x n Jacobian matrix relating the joint
/// velocities to the space twist
///
/// # Example
///
/// ```
/// use nalgebra::{Vector6, DVector};
/// use modern_robotics::jacobian_space;
///
/// let slist = vec![
///     Vector6::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
///     Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
/// ];
/// let thetalist = DVector::from_vec(vec![0.2, 1.1]);
/// let js = jacobian_space(&slist, &thetalist);
/// ```
pub fn jacobian_space(
    slist: &Vec<nalgebra::Vector6<f64>>,
    thetalist: &nalgebra::DVector<f64>,
) -> nalgebra::Matrix6xX<f64> {
    let n = slist.len();
    let mut js: nalgebra::Matrix6xX<f64> = nalgebra::Matrix6xX::zeros(n);
    let mut t = nalgebra::Matrix4::identity();

    let svec = slist[0];
    js.set_column(0, &svec);

    for i in 1..n {
        let svec = slist[i - 1];
        let theta = thetalist[i - 1];
        let v = svec * theta;
        let se3mat = vec_to_se3(&v);
        let tij = matrix_exp6(&se3mat);
        t = t * tij;

        let adt = adjoint(&t);
        let jsvec = adt * slist[i];
        js.set_column(i, &jsvec);
    }

    return js;
}
