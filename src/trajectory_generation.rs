use nalgebra;

use crate::{
    matrix_exp3, matrix_exp6, matrix_log3, matrix_log6, rot_inv, rp_to_trans, trans_inv,
    trans_to_rp,
};

/// Time scaling method for trajectory generation.
///
/// # Variants
///
/// * `Cubic` - Use cubic polynomial time scaling (zero start/end velocity)
/// * `Quintic` - Use quintic polynomial time scaling (zero start/end velocity and acceleration)
pub enum Method {
    Cubic,
    Quintic,
}

/// Computes s(t) for a cubic time scaling.
///
/// # Arguments
///
/// * `tf` - Total time of the motion in seconds from rest to rest
/// * `t` - The current time t satisfying 0 < t < Tf
///
/// # Returns
///
/// The path parameter s(t) corresponding to a third-order polynomial motion
/// that begins and ends at zero velocity
///
/// # Example
///
/// ```
/// use modern_robotics::cubic_time_scaling;
///
/// let tf = 2.0;
/// let t = 0.6;
/// let s = cubic_time_scaling(tf, t);
/// // s is approximately 0.216
/// ```
pub fn cubic_time_scaling(tf: f64, t: f64) -> f64 {
    let a0 = 0.0;
    let a1 = 0.0;
    let a2 = 3.0 / tf.powi(2);
    let a3 = -2.0 / tf.powi(3);

    let s = a0 + a1 * t + a2 * t.powi(2) + a3 * t.powi(3);
    return s;
}

/// Computes s(t) for a quintic time scaling.
///
/// # Arguments
///
/// * `tf` - Total time of the motion in seconds from rest to rest
/// * `t` - The current time t satisfying 0 < t < Tf
///
/// # Returns
///
/// The path parameter s(t) corresponding to a fifth-order polynomial motion
/// that begins and ends at zero velocity and zero acceleration
///
/// # Example
///
/// ```
/// use modern_robotics::quintic_time_scaling;
///
/// let tf = 2.0;
/// let t = 0.6;
/// let s = quintic_time_scaling(tf, t);
/// // s is approximately 0.16308
/// ```
pub fn quintic_time_scaling(tf: f64, t: f64) -> f64 {
    let a0 = 0.0;
    let a1 = 0.0;
    let a2 = 0.0;
    let a3 = 10.0 / tf.powi(3);
    let a4 = -15.0 / tf.powi(4);
    let a5 = 6.0 / tf.powi(5);

    let s = a0 + a1 * t + a2 * t.powi(2) + a3 * t.powi(3) + a4 * t.powi(4) + a5 * t.powi(5);
    return s;
}

/// Computes a straight-line trajectory in joint space.
///
/// # Arguments
///
/// * `thetastart` - The initial joint variables
/// * `thetaend` - The final joint variables
/// * `tf` - Total time of the motion in seconds from rest to rest
/// * `n` - The number of points N > 1 (Start and stop) in the discrete
///   representation of the trajectory
/// * `method` - The time-scaling method, where Method::Cubic uses a third-order
///   polynomial and Method::Quintic uses a fifth-order polynomial
///
/// # Returns
///
/// A trajectory as a list of N points, where each point is a vector of joint
/// coordinates. The first point is thetastart and the Nth point is thetaend.
///
/// This function is similar to the MATLAB/Octave function jtraj.
///
/// # Example
///
/// ```
/// use nalgebra::DVector;
/// use modern_robotics::{joint_trajectory, Method};
///
/// let thetastart = DVector::from_vec(vec![1.0, 0.0, 0.0, 1.0, 1.5, 2.5, 3.0, 0.0]);
/// let thetaend = DVector::from_vec(vec![1.2, 0.5, 0.6, 1.1, 2.0, 3.1, 3.2, 0.9]);
/// let tf = 4.0;
/// let n = 6;
/// let method = Method::Cubic;
/// let traj = joint_trajectory(&thetastart, &thetaend, tf, n, method);
/// ```
pub fn joint_trajectory(
    thetastart: &nalgebra::DVector<f64>,
    thetaend: &nalgebra::DVector<f64>,
    tf: f64,
    n: usize,
    method: Method,
) -> Vec<nalgebra::DVector<f64>> {
    let mut traj = Vec::new();
    let timegap = tf / (n as f64 - 1.0);

    for i in 0..n {
        let t = timegap * i as f64;
        let s: f64;
        match method {
            Method::Cubic => s = cubic_time_scaling(tf, t),
            Method::Quintic => s = quintic_time_scaling(tf, t),
        }
        let theta = s * thetaend + (1.0 - s) * thetastart;
        traj.push(theta);
    }

    return traj;
}

/// Computes a trajectory as a list of N SE(3) matrices corresponding to the screw motion.
///
/// # Arguments
///
/// * `xstart` - The initial end-effector configuration
/// * `xend` - The final end-effector configuration
/// * `tf` - Total time of the motion in seconds from rest to rest
/// * `n` - The number of points N > 1 (Start and stop) in the discrete
///   representation of the trajectory
/// * `method` - The time-scaling method, where Method::Cubic uses a third-order
///   polynomial and Method::Quintic uses a fifth-order polynomial
///
/// # Returns
///
/// The discretized trajectory as a list of N matrices in SE(3) separated in
/// time by Tf/(N-1). The first point is Xstart and the Nth point is Xend.
///
/// This function calculates a trajectory corresponding to the screw motion about
/// a space screw axis.
///
/// # Example
///
/// ```
/// use nalgebra::Matrix4;
/// use modern_robotics::{screw_trajectory, Method};
///
/// let xstart = Matrix4::new(
///     1.0, 0.0, 0.0, 1.0,
///     0.0, 1.0, 0.0, 0.0,
///     0.0, 0.0, 1.0, 1.0,
///     0.0, 0.0, 0.0, 1.0
/// );
/// let xend = Matrix4::new(
///     0.0, 0.0, 1.0, 0.1,
///     1.0, 0.0, 0.0, 0.0,
///     0.0, 1.0, 0.0, 4.1,
///     0.0, 0.0, 0.0, 1.0
/// );
/// let tf = 5.0;
/// let n = 4;
/// let method = Method::Cubic;
/// let traj = screw_trajectory(&xstart, &xend, tf, n, method);
/// ```
pub fn screw_trajectory(
    xstart: &nalgebra::Matrix4<f64>,
    xend: &nalgebra::Matrix4<f64>,
    tf: f64,
    n: usize,
    method: Method,
) -> Vec<nalgebra::Matrix4<f64>> {
    let timegap = tf / (n as f64 - 1.0);
    let mut traj = Vec::new();

    for i in 0..n {
        let s: f64;
        let t = timegap * i as f64;
        match method {
            Method::Cubic => s = cubic_time_scaling(tf, t),
            Method::Quintic => s = quintic_time_scaling(tf, t),
        }

        let xstart_inv = trans_inv(&xstart);
        let tse = xstart_inv * xend;
        let se3mat = matrix_log6(&tse) * s;
        let t = xstart * matrix_exp6(&se3mat);

        traj.push(t);
    }

    return traj;
}

/// Computes a trajectory as a list of N SE(3) matrices with Cartesian motion.
///
/// # Arguments
///
/// * `xstart` - The initial end-effector configuration
/// * `xend` - The final end-effector configuration
/// * `tf` - Total time of the motion in seconds from rest to rest
/// * `n` - The number of points N > 1 (Start and stop) in the discrete
///   representation of the trajectory
/// * `method` - The time-scaling method, where Method::Cubic uses a third-order
///   polynomial and Method::Quintic uses a fifth-order polynomial
///
/// # Returns
///
/// The discretized trajectory as a list of N matrices in SE(3) separated in
/// time by Tf/(N-1). The first point is Xstart and the Nth point is Xend.
///
/// This function is similar to ScrewTrajectory, except the origin of the
/// end-effector frame follows a straight line, decoupled from the rotational
/// motion.
///
/// # Example
///
/// ```
/// use nalgebra::Matrix4;
/// use modern_robotics::{cartesian_trajectory, Method};
///
/// let xstart = Matrix4::new(
///     1.0, 0.0, 0.0, 1.0,
///     0.0, 1.0, 0.0, 0.0,
///     0.0, 0.0, 1.0, 1.0,
///     0.0, 0.0, 0.0, 1.0
/// );
/// let xend = Matrix4::new(
///     0.0, 0.0, 1.0, 0.1,
///     1.0, 0.0, 0.0, 0.0,
///     0.0, 1.0, 0.0, 4.1,
///     0.0, 0.0, 0.0, 1.0
/// );
/// let tf = 5.0;
/// let n = 4;
/// let method = Method::Quintic;
/// let traj = cartesian_trajectory(&xstart, &xend, tf, n, method);
/// ```
pub fn cartesian_trajectory(
    xstart: &nalgebra::Matrix4<f64>,
    xend: &nalgebra::Matrix4<f64>,
    tf: f64,
    n: usize,
    method: Method,
) -> Vec<nalgebra::Matrix4<f64>> {
    let timegap = tf / (n as f64 - 1.0);
    let mut traj = Vec::new();

    let (rstart, pstart) = trans_to_rp(xstart);
    let (rend, pend) = trans_to_rp(xend);

    for i in 0..n {
        let t = timegap * i as f64;
        let s: f64;
        match method {
            Method::Cubic => s = cubic_time_scaling(tf, t),
            Method::Quintic => s = quintic_time_scaling(tf, t),
        }

        let rstart_inv = rot_inv(&rstart);
        let rse = rstart_inv * rend;
        let so3mat = matrix_log3(&rse) * s;
        let r = matrix_exp3(&so3mat);

        let p = pstart + (pend - pstart) * s;
        let t = rp_to_trans(&r, &p);

        traj.push(t);
    }

    return traj;
}
