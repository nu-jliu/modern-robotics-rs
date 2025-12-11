use nalgebra;

use crate::{inverse_dynamics, mass_matrix};

/// Computes the joint control torques at a particular time instant using computed torque control.
///
/// # Arguments
///
/// * `thetalist` - n-vector of joint variables
/// * `dthetalist` - n-vector of joint rates
/// * `eint` - n-vector of the time-integral of joint errors
/// * `g` - Gravity vector g
/// * `mlist` - List of link frames {i} relative to {i-1} at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
/// * `thetalistd` - n-vector of reference joint variables
/// * `dthetalistd` - n-vector of reference joint velocities
/// * `ddthetalistd` - n-vector of reference joint accelerations
/// * `kp` - The feedback proportional gain (often denoted Kp)
/// * `ki` - The feedback integral gain (often denoted Ki)
/// * `kd` - The feedback derivative gain (often denoted Kd)
///
/// # Returns
///
/// The vector of joint forces/torques computed by the feedback linearization
/// plus feed-forward controller
///
/// This function computes:
/// tau = M(thetalist)(Kp*e + Ki*integral(e) + Kd*edot) + h(thetalist,dthetalist)
///
/// where e = thetalistd - thetalist is the joint error
///
/// Kp, Ki, and Kd are chosen such that the linearized dynamics are stable.
/// Feed-forward forces/torques are added to improve tracking accuracy.
///
/// # Example
///
/// ```
/// use nalgebra::{Vector3, Vector6, DVector, Matrix4, Matrix6};
/// use modern_robotics::compute_torque;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// let dthetalist = DVector::from_vec(vec![0.1, 0.2, 0.3]);
/// let eint = DVector::zeros(3);
/// let g = Vector3::new(0.0, 0.0, -9.8);
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let thetalistd = DVector::from_vec(vec![1.0, 1.0, 1.0]);
/// let dthetalistd = DVector::from_vec(vec![2.0, 1.2, 2.0]);
/// let ddthetalistd = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// let kp = 1.3;
/// let ki = 1.2;
/// let kd = 1.1;
/// let tau = compute_torque(&thetalist, &dthetalist, &eint, &g,
///                          &mlist, &glist, &slist,
///                          &thetalistd, &dthetalistd, &ddthetalistd,
///                          kp, ki, kd);
/// ```
pub fn compute_torque(
    thetalist: &nalgebra::DVector<f64>,
    dthetalist: &nalgebra::DVector<f64>,
    eint: &nalgebra::DVector<f64>,
    g: &nalgebra::Vector3<f64>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
    thetalistd: &nalgebra::DVector<f64>,
    dthetalistd: &nalgebra::DVector<f64>,
    ddthetalistd: &nalgebra::DVector<f64>,
    kp: f64,
    ki: f64,
    kd: f64,
) -> nalgebra::DVector<f64> {
    let n = thetalist.len();
    let i_nn = nalgebra::DMatrix::identity(n, n);
    let kp_mat = kp * i_nn.clone();
    let ki_mat = ki * i_nn.clone();
    let kd_mat = kd * i_nn.clone();

    let ep = thetalistd - thetalist;
    let ei = eint + ep.clone();
    let ed = dthetalistd - dthetalist;

    let ftip = nalgebra::Vector6::zeros();
    let m_mat = mass_matrix(thetalist, mlist, glist, slist);

    let taulist_1 = m_mat * (kp_mat * ep + ki_mat * ei + kd_mat * ed);
    let taulist_2 = inverse_dynamics(
        thetalist,
        dthetalist,
        ddthetalistd,
        g,
        &ftip,
        mlist,
        glist,
        slist,
    );

    return taulist_1 + taulist_2;
}
