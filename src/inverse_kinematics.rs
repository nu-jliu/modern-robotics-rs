use nalgebra;

use crate::{
    adjoint, fkin_body, fkin_space, jacobian_body, jacobian_space, matrix_log6, se3_to_vec,
    trans_inv,
};

/// Computes inverse kinematics in the body frame for an open chain robot.
///
/// # Arguments
///
/// * `blist` - The joint screw axes in the end-effector frame when the
///   manipulator is at the home position
/// * `m` - The home configuration of the end-effector
/// * `t` - The desired end-effector configuration T
/// * `thetalist0` - An initial guess of joint angles that are close to satisfying T
/// * `emog` - A small positive tolerance on the end-effector orientation error.
///   The returned joint angles must give an end-effector orientation error
///   less than emog
/// * `ev` - A small positive tolerance on the end-effector linear position error.
///   The returned joint angles must give an end-effector position error less
///   than ev
///
/// # Returns
///
/// A tuple containing:
/// * Joint angles that achieve T within the specified tolerances
/// * A boolean indicating whether the algorithm converged (true) or ran for the
///   maximum number of iterations without converging (false)
///
/// Uses an iterative Newton-Raphson root-finding method. The maximum number of
/// iterations before the algorithm is terminated has been hardcoded.
///
/// # Example
///
/// ```
/// use nalgebra::{Matrix4, Vector6, DVector};
/// use modern_robotics::ikin_body;
///
/// let blist = vec![
///     Vector6::new(0.0, 0.0, -1.0, 2.0, 0.0, 0.0),
///     Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
///     Vector6::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.1)
/// ];
/// let m = Matrix4::new(
///     -1.0, 0.0,  0.0, 0.0,
///      0.0, 1.0,  0.0, 6.0,
///      0.0, 0.0, -1.0, 2.0,
///      0.0, 0.0,  0.0, 1.0
/// );
/// let t = Matrix4::new(
///      0.0, 1.0,  0.0,     -5.0,
///      1.0, 0.0,  0.0,      4.0,
///      0.0, 0.0, -1.0,      1.6,
///      0.0, 0.0,  0.0,      1.0
/// );
/// let thetalist0 = DVector::from_vec(vec![1.5, 2.5, 3.0]);
/// let emog = 0.01;
/// let ev = 0.001;
/// let (thetalist, success) = ikin_body(&blist, &m, &t, &thetalist0, emog, ev);
/// ```
pub fn ikin_body(
    blist: &Vec<nalgebra::Vector6<f64>>,
    m: &nalgebra::Matrix4<f64>,
    t: &nalgebra::Matrix4<f64>,
    thetalist0: &nalgebra::DVector<f64>,
    emog: f64,
    ev: f64,
) -> (nalgebra::DVector<f64>, bool) {
    const MAX_ITER: i32 = 20;
    let mut thetalist = thetalist0.clone();
    let mut err: bool = true;

    for _ in 0..MAX_ITER {
        let t_sb = fkin_body(m, blist, &thetalist);
        let t_sb_inv = trans_inv(&t_sb);
        let se3mat = matrix_log6(&(t_sb_inv * t));
        let vb = se3_to_vec(&se3mat);

        let jb = jacobian_body(blist, &thetalist);
        let jb_inv = jb.pseudo_inverse(0.0);
        match jb_inv {
            Ok(val) => {
                let dthetalist = val * vb;
                thetalist += dthetalist;
            }
            Err(_) => return (thetalist, false),
        }

        let omg: nalgebra::Vector3<f64> = vb.fixed_view::<3, 1>(0, 0).into();
        let v: nalgebra::Vector3<f64> = vb.fixed_view::<3, 1>(3, 0).into();
        err = omg.norm() > emog || v.norm() > ev;

        if !err {
            break;
        }
    }

    return (thetalist, !err);
}

/// Computes inverse kinematics in the space frame for an open chain robot.
///
/// # Arguments
///
/// * `slist` - The joint screw axes in the space frame when the manipulator
///   is at the home position
/// * `m` - The home configuration of the end-effector
/// * `t` - The desired end-effector configuration T
/// * `thetalist0` - An initial guess of joint angles that are close to satisfying T
/// * `emog` - A small positive tolerance on the end-effector orientation error.
///   The returned joint angles must give an end-effector orientation error
///   less than emog
/// * `ev` - A small positive tolerance on the end-effector linear position error.
///   The returned joint angles must give an end-effector position error less
///   than ev
///
/// # Returns
///
/// A tuple containing:
/// * Joint angles that achieve T within the specified tolerances
/// * A boolean indicating whether the algorithm converged (true) or ran for the
///   maximum number of iterations without converging (false)
///
/// Uses an iterative Newton-Raphson root-finding method. The maximum number of
/// iterations before the algorithm is terminated has been hardcoded.
///
/// # Example
///
/// ```
/// use nalgebra::{Matrix4, Vector6, DVector};
/// use modern_robotics::ikin_space;
///
/// let slist = vec![
///     Vector6::new(0.0, 0.0, 1.0, 4.0, 0.0, 0.0),
///     Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
///     Vector6::new(0.0, 0.0, -1.0, -6.0, 0.0, -0.1)
/// ];
/// let m = Matrix4::new(
///     -1.0, 0.0,  0.0, 0.0,
///      0.0, 1.0,  0.0, 6.0,
///      0.0, 0.0, -1.0, 2.0,
///      0.0, 0.0,  0.0, 1.0
/// );
/// let t = Matrix4::new(
///      0.0, 1.0,  0.0,     -5.0,
///      1.0, 0.0,  0.0,      4.0,
///      0.0, 0.0, -1.0,      1.6,
///      0.0, 0.0,  0.0,      1.0
/// );
/// let thetalist0 = DVector::from_vec(vec![1.5, 2.5, 3.0]);
/// let emog = 0.01;
/// let ev = 0.001;
/// let (thetalist, success) = ikin_space(&slist, &m, &t, &thetalist0, emog, ev);
/// ```
pub fn ikin_space(
    slist: &Vec<nalgebra::Vector6<f64>>,
    m: &nalgebra::Matrix4<f64>,
    t: &nalgebra::Matrix4<f64>,
    thetalist0: &nalgebra::DVector<f64>,
    emog: f64,
    ev: f64,
) -> (nalgebra::DVector<f64>, bool) {
    const MAX_ITER: i32 = 20;
    let mut thetalist = thetalist0.clone();
    let mut err = true;

    for _ in 0..MAX_ITER {
        let t_sb = fkin_space(m, slist, &thetalist);
        let t_sb_inv = trans_inv(&t_sb);
        let se3mat = matrix_log6(&(&t_sb_inv * t));
        let adt_sb = adjoint(&t_sb);
        let vs = adt_sb * se3_to_vec(&se3mat);

        let js = jacobian_space(slist, &thetalist);
        let js_inv = js.pseudo_inverse(0.0);
        match js_inv {
            Ok(val) => {
                let dthetalist = val * vs;
                thetalist += dthetalist;
            }
            Err(_) => return (thetalist, false),
        }

        let omg: nalgebra::Vector3<f64> = vs.fixed_view::<3, 1>(0, 0).into();
        let v: nalgebra::Vector3<f64> = vs.fixed_view::<3, 1>(3, 0).into();
        err = omg.norm() > emog || v.norm() > ev;

        if !err {
            break;
        }
    }

    return (thetalist, !err);
}
