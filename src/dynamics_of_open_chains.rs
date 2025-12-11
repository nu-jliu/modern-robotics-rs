use nalgebra;

use crate::{adjoint, matrix_exp6, trans_inv, vec_to_se3, vec_to_so3};

/// Calculates the 6x6 matrix \[adV\] of a spatial velocity vector.
///
/// Used to calculate the Lie bracket \[V1, V2\] = \[adV1\]V2.
///
/// # Arguments
///
/// * `v` - A 6-vector spatial velocity
///
/// # Returns
///
/// The 6x6 matrix \[adV\]
///
/// # Example
///
/// ```
/// use nalgebra::Vector6;
/// use modern_robotics::ad;
///
/// let v = Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
/// let adv = ad(&v);
/// ```
pub fn ad(v: &nalgebra::Vector6<f64>) -> nalgebra::Matrix6<f64> {
    let omg = nalgebra::Vector3::new(v[0], v[1], v[2]);
    let v3 = nalgebra::Vector3::new(v[3], v[4], v[5]);

    let omgmat = vec_to_so3(&omg);
    let vmat = vec_to_so3(&v3);
    let zeros = nalgebra::Matrix3::zeros();

    return nalgebra::stack![omgmat, zeros; vmat, omgmat];
}

/// Computes inverse dynamics in the space frame for an open chain robot.
///
/// # Arguments
///
/// * `thetalist` - n-vector of joint variables
/// * `dthetalist` - n-vector of joint rates
/// * `ddthetalist` - n-vector of joint accelerations
/// * `g` - Gravity vector g
/// * `ftip` - Spatial force applied by the end-effector expressed in frame {n+1}
/// * `mlist` - List of link frames {i} relative to {i-1} at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
///
/// # Returns
///
/// The n-vector of required joint forces/torques
///
/// This function uses the Newton-Euler inverse dynamics algorithm.
///
/// # Example
///
/// ```
/// use nalgebra::{Vector3, Vector6, DVector, Matrix4, Matrix6};
/// use modern_robotics::inverse_dynamics;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// let dthetalist = DVector::from_vec(vec![0.1, 0.2, 0.3]);
/// let ddthetalist = DVector::from_vec(vec![2.0, 1.5, 1.0]);
/// let g = Vector3::new(0.0, 0.0, -9.8);
/// let ftip = Vector6::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let taulist = inverse_dynamics(&thetalist, &dthetalist, &ddthetalist,
///                                  &g, &ftip, &mlist, &glist, &slist);
/// ```
pub fn inverse_dynamics(
    thetalist: &nalgebra::DVector<f64>,
    dthetalist: &nalgebra::DVector<f64>,
    ddthetalist: &nalgebra::DVector<f64>,
    g: &nalgebra::Vector3<f64>,
    ftip: &nalgebra::Vector6<f64>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
) -> nalgebra::DVector<f64> {
    let n = thetalist.len();
    let mut mi: nalgebra::Matrix4<f64> = nalgebra::Matrix4::identity();
    let mut ai: Vec<nalgebra::Vector6<f64>> = Vec::new();
    let mut adti: Vec<nalgebra::Matrix6<f64>> = Vec::new();
    let mut vi: Vec<nalgebra::Vector6<f64>> = Vec::new();
    let mut vdi: Vec<nalgebra::Vector6<f64>> = Vec::new();
    let mut fi = ftip.clone();

    vi.push(nalgebra::Vector6::zeros());
    vdi.push(nalgebra::Vector6::new(0.0, 0.0, 0.0, -g[0], -g[1], -g[2]));

    for i in 0..n {
        let m = mlist[i];
        let s = slist[i];
        let theta = thetalist[i];
        let dtheta = dthetalist[i];
        let ddtheta = ddthetalist[i];
        let v_curr = vi[i];
        let vd_curr = vdi[i];

        mi = mi * m;
        let m_inv = trans_inv(&m);
        let mi_inv = trans_inv(&mi);
        let admi_inv = adjoint(&mi_inv);
        let a = admi_inv * s;
        let se3mat = vec_to_se3(&(a * -theta));
        let t = matrix_exp6(&se3mat);
        let adt = adjoint(&(t * m_inv));
        let v = adt * v_curr + a * dtheta;
        let vd = adt * vd_curr + a * ddtheta + ad(&v) * a * dtheta;

        ai.push(a);
        adti.push(adt);
        vi.push(v);
        vdi.push(vd);
    }

    let m = mlist[n];
    let m_inv = trans_inv(&m);
    let adm_inv = adjoint(&m_inv);
    adti.push(adm_inv);

    let mut taulist = nalgebra::DVector::zeros(n);

    for j in 0..n {
        let i = n - 1 - j;
        let gmat = glist[i];
        let a = ai[i];
        let adt = adti[i + 1];
        let v = vi[i + 1];
        let vd = vdi[i + 1];

        fi = adt.transpose() * fi + gmat * vd - ad(&v).transpose() * (gmat * v);
        let tau = fi.dot(&a);

        taulist[i] = tau;
    }

    return taulist;
}

/// Computes the mass matrix of an open chain robot.
///
/// # Arguments
///
/// * `thetalist` - A list of joint variables
/// * `mlist` - List of link frames i relative to i-1 at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
///
/// # Returns
///
/// The numerical inertia matrix M(thetalist) of an n-joint serial chain at
/// the given configuration thetalist
///
/// This function calls inverse_dynamics n times, each time passing a
/// ddthetalist vector with a single element equal to one and all other
/// inputs set to zero.
///
/// # Example
///
/// ```
/// use nalgebra::{DVector, Matrix4, Matrix6, Vector6};
/// use modern_robotics::mass_matrix;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let m = mass_matrix(&thetalist, &mlist, &glist, &slist);
/// ```
pub fn mass_matrix(
    thetalist: &nalgebra::DVector<f64>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
) -> nalgebra::DMatrix<f64> {
    let n = thetalist.len();
    let mut m = nalgebra::DMatrix::zeros(n, n);

    for i in 0..n {
        let dthetalist = nalgebra::DVector::zeros(n);
        let mut ddthetalist = nalgebra::DVector::zeros(n);
        ddthetalist[i] = 1.0;
        let g = nalgebra::Vector3::zeros();
        let ftip = nalgebra::Vector6::zeros();
        let taulist = inverse_dynamics(
            thetalist,
            &dthetalist,
            &ddthetalist,
            &g,
            &ftip,
            mlist,
            glist,
            slist,
        );

        m.set_column(i, &taulist);
    }

    return m;
}

/// Computes the Coriolis and centripetal terms in the inverse dynamics of an open chain robot.
///
/// # Arguments
///
/// * `thetalist` - A list of joint variables
/// * `dthetalist` - A list of joint rates
/// * `mlist` - List of link frames i relative to i-1 at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
///
/// # Returns
///
/// The vector c(thetalist,dthetalist) of Coriolis and centripetal terms for
/// a given thetalist and dthetalist
///
/// This function calls inverse_dynamics with g = 0, ftip = 0, and
/// ddthetalist = 0.
///
/// # Example
///
/// ```
/// use nalgebra::{DVector, Matrix4, Matrix6, Vector6};
/// use modern_robotics::vel_quadratic_forces;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// let dthetalist = DVector::from_vec(vec![0.1, 0.2, 0.3]);
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let c = vel_quadratic_forces(&thetalist, &dthetalist, &mlist, &glist, &slist);
/// ```
pub fn vel_quadratic_forces(
    thetalist: &nalgebra::DVector<f64>,
    dthetalist: &nalgebra::DVector<f64>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
) -> nalgebra::DVector<f64> {
    let n = thetalist.len();
    let ddthetalist = nalgebra::DVector::zeros(n);
    let g = nalgebra::Vector3::zeros();
    let ftip = nalgebra::Vector6::zeros();

    let c = inverse_dynamics(
        thetalist,
        dthetalist,
        &ddthetalist,
        &g,
        &ftip,
        mlist,
        glist,
        slist,
    );

    return c;
}

/// Computes the joint forces/torques required to overcome gravity for an open chain robot.
///
/// # Arguments
///
/// * `thetalist` - A list of joint variables
/// * `g` - 3-vector for gravitational acceleration
/// * `mlist` - List of link frames i relative to i-1 at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
///
/// # Returns
///
/// The joint forces/torques required to overcome gravity at thetalist
///
/// This function calls inverse_dynamics with ftip = 0, dthetalist = 0, and
/// ddthetalist = 0.
///
/// # Example
///
/// ```
/// use nalgebra::{Vector3, DVector, Matrix4, Matrix6, Vector6};
/// use modern_robotics::gravity_forces;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// let g = Vector3::new(0.0, 0.0, -9.8);
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let grav = gravity_forces(&thetalist, &g, &mlist, &glist, &slist);
/// ```
pub fn gravity_forces(
    thetalist: &nalgebra::DVector<f64>,
    g: &nalgebra::Vector3<f64>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
) -> nalgebra::DVector<f64> {
    let n = thetalist.len();
    let dthetalist = nalgebra::DVector::zeros(n);
    let ddthetalist = nalgebra::DVector::zeros(n);
    let ftip = nalgebra::Vector6::zeros();

    let grav = inverse_dynamics(
        thetalist,
        &dthetalist,
        &ddthetalist,
        g,
        &ftip,
        mlist,
        glist,
        slist,
    );

    return grav;
}

/// Computes the joint forces/torques required to create the end-effector force Ftip.
///
/// # Arguments
///
/// * `thetalist` - A list of joint variables
/// * `ftip` - Spatial force applied by the end-effector expressed in frame {n+1}
/// * `mlist` - List of link frames i relative to i-1 at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
///
/// # Returns
///
/// The joint forces/torques required to create the end-effector force Ftip
///
/// This function calls inverse_dynamics with g = 0, dthetalist = 0, and
/// ddthetalist = 0.
///
/// # Example
///
/// ```
/// use nalgebra::{Vector6, DVector, Matrix4, Matrix6};
/// use modern_robotics::end_effector_forces;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// let ftip = Vector6::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let jt_ftip = end_effector_forces(&thetalist, &ftip, &mlist, &glist, &slist);
/// ```
pub fn end_effector_forces(
    thetalist: &nalgebra::DVector<f64>,
    ftip: &nalgebra::Vector6<f64>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
) -> nalgebra::DVector<f64> {
    let n = thetalist.len();
    let dthetalist = nalgebra::DVector::zeros(n);
    let ddthetalist = nalgebra::DVector::zeros(n);
    let g = nalgebra::Vector3::zeros();

    let jt_ftip = inverse_dynamics(
        thetalist,
        &dthetalist,
        &ddthetalist,
        &g,
        ftip,
        mlist,
        glist,
        slist,
    );

    return jt_ftip;
}

/// Computes forward dynamics in the space frame for an open chain robot.
///
/// # Arguments
///
/// * `thetalist` - A list of joint variables
/// * `dthetalist` - A list of joint rates
/// * `taulist` - An n-vector of joint forces/torques
/// * `g` - Gravity vector g
/// * `ftip` - Spatial force applied by the end-effector expressed in frame {n+1}
/// * `mlist` - List of link frames {i} relative to {i-1} at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
///
/// # Returns
///
/// The resulting joint accelerations
///
/// This function computes ddthetalist by solving:
/// M(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist)
///                                      - g(thetalist) - Jtr(thetalist) * Ftip
///
/// # Example
///
/// ```
/// use nalgebra::{Vector3, Vector6, DVector, Matrix4, Matrix6};
/// use modern_robotics::forward_dynamics;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// let dthetalist = DVector::from_vec(vec![0.1, 0.2, 0.3]);
/// let taulist = DVector::from_vec(vec![0.5, 0.6, 0.7]);
/// let g = Vector3::new(0.0, 0.0, -9.8);
/// let ftip = Vector6::zeros();
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let ddthetalist = forward_dynamics(&thetalist, &dthetalist, &taulist,
///                                     &g, &ftip, &mlist, &glist, &slist);
/// ```
pub fn forward_dynamics(
    thetalist: &nalgebra::DVector<f64>,
    dthetalist: &nalgebra::DVector<f64>,
    taulist: &nalgebra::DVector<f64>,
    g: &nalgebra::Vector3<f64>,
    ftip: &nalgebra::Vector6<f64>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
) -> nalgebra::DVector<f64> {
    let m = mass_matrix(thetalist, mlist, glist, slist);
    let c = vel_quadratic_forces(thetalist, dthetalist, mlist, glist, slist);
    let grav = gravity_forces(thetalist, g, mlist, glist, slist);
    let jt_ftip = end_effector_forces(thetalist, ftip, mlist, glist, slist);

    let rhs = taulist - c - grav - jt_ftip;

    match m.try_inverse() {
        Some(val) => {
            return val * rhs;
        }
        None => {
            let n = thetalist.len();
            return nalgebra::DVector::zeros(n);
        }
    }
}

/// Computes the joint angles and velocities at the next timestep using first order Euler integration.
///
/// # Arguments
///
/// * `thetalist` - n-vector of joint variables
/// * `dthetalist` - n-vector of joint rates
/// * `ddthetalist` - n-vector of joint accelerations
/// * `dt` - The timestep delta t
///
/// # Returns
///
/// A tuple containing:
/// * thetalist[k+1] - Joint variables after dt from now
/// * dthetalist[k+1] - Joint rates after dt from now
///
/// # Example
///
/// ```
/// use nalgebra::DVector;
/// use modern_robotics::euler_step;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.2, 0.3]);
/// let dthetalist = DVector::from_vec(vec![0.1, 0.2, 0.3]);
/// let ddthetalist = DVector::from_vec(vec![2.0, 1.5, 1.0]);
/// let dt = 0.01;
/// let (thetalist_next, dthetalist_next) = euler_step(&thetalist, &dthetalist, &ddthetalist, dt);
/// ```
pub fn euler_step(
    thetalist: &nalgebra::DVector<f64>,
    dthetalist: &nalgebra::DVector<f64>,
    ddthetalist: &nalgebra::DVector<f64>,
    dt: f64,
) -> (nalgebra::DVector<f64>, nalgebra::DVector<f64>) {
    let thetslist_next = thetalist + dt * dthetalist;
    let dthetalist_next = dthetalist + dt * ddthetalist;
    return (thetslist_next, dthetalist_next);
}

/// Calculates the required joint forces/torques to achieve a path with given accelerations.
///
/// # Arguments
///
/// * `thetamat` - A trajectory of joint variables, each row is a point in the trajectory
/// * `dthetamat` - A trajectory of joint velocities
/// * `ddthetamat` - A trajectory of joint accelerations
/// * `g` - Gravity vector g
/// * `ftipmat` - A trajectory of spatial forces applied by the end-effector
///   (i-th element is the spatial force applied by the end-effector at the i-th
///   time step)
/// * `mlist` - List of link frames {i} relative to {i-1} at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
///
/// # Returns
///
/// The N x n matrix of joint forces/torques for the specified trajectory,
/// where each of the N rows is the vector of joint forces/torques at each time step
///
/// This function uses inverse_dynamics to calculate the joint forces/torques
/// required to move along the given trajectory.
///
/// # Example
///
/// ```
/// use nalgebra::{Vector3, Vector6, DVector, Matrix4, Matrix6};
/// use modern_robotics::inverse_dynamics_trajectory;
///
/// let thetamat = vec![DVector::from_vec(vec![0.1, 0.1, 0.1])];
/// let dthetamat = vec![DVector::from_vec(vec![0.1, 0.2, 0.3])];
/// let ddthetamat = vec![DVector::from_vec(vec![2.0, 1.5, 1.0])];
/// let g = Vector3::new(0.0, 0.0, -9.8);
/// let ftipmat = vec![Vector6::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)];
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let taumat = inverse_dynamics_trajectory(&thetamat, &dthetamat, &ddthetamat,
///                                           &g, &ftipmat, &mlist, &glist, &slist);
/// ```
pub fn inverse_dynamics_trajectory(
    thetamat: &Vec<nalgebra::DVector<f64>>,
    dthetamat: &Vec<nalgebra::DVector<f64>>,
    ddthetamat: &Vec<nalgebra::DVector<f64>>,
    g: &nalgebra::Vector3<f64>,
    ftipmat: &Vec<nalgebra::Vector6<f64>>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
) -> Vec<nalgebra::DVector<f64>> {
    let n = thetamat.len();
    let mut taumat = Vec::new();

    for i in 0..n {
        let thetalist = &thetamat[i];
        let dthetalist = &dthetamat[i];
        let ddthetalist = &ddthetamat[i];
        let ftip = &ftipmat[i];

        let taulist = inverse_dynamics(
            thetalist,
            dthetalist,
            ddthetalist,
            g,
            ftip,
            mlist,
            glist,
            slist,
        );
        taumat.push(taulist);
    }

    return taumat;
}

/// Simulates the motion of a serial chain given an open-loop history of joint forces/torques.
///
/// # Arguments
///
/// * `thetalist` - n-vector of initial joint variables
/// * `dthetalist` - n-vector of initial joint rates
/// * `taumat` - An N x n matrix of joint forces/torques, where each row is the
///   joint effort at a point in the trajectory
/// * `g` - Gravity vector g
/// * `ftipmat` - An N x 6 matrix of spatial forces applied by the end-effector
///   (i-th row is the spatial force applied at the i-th time step)
/// * `mlist` - List of link frames {i} relative to {i-1} at the home position
/// * `glist` - Spatial inertia matrices Gi of the links
/// * `slist` - Screw axes Si of the joints in a space frame
/// * `dt` - The timestep between consecutive joint forces/torques
/// * `int_res` - Integration resolution: the number of Euler steps during each
///   timestep (typically set to 1-10)
///
/// # Returns
///
/// A tuple containing:
/// * The resulting joint angle trajectory
/// * The resulting joint velocity trajectory
///
/// Uses forward dynamics and Euler integration.
///
/// # Example
///
/// ```
/// use nalgebra::{Vector3, Vector6, DVector, Matrix4, Matrix6};
/// use modern_robotics::forward_dynamics_trajectory;
///
/// let thetalist = DVector::from_vec(vec![0.1, 0.1, 0.1]);
/// let dthetalist = DVector::from_vec(vec![0.1, 0.2, 0.3]);
/// let taumat = vec![DVector::from_vec(vec![0.1, 0.1, 0.1])];
/// let g = Vector3::new(0.0, 0.0, -9.8);
/// let ftipmat = vec![Vector6::zeros()];
/// // ... setup mlist, glist, slist ...
/// # let mlist = vec![Matrix4::identity(); 4];
/// # let glist = vec![Matrix6::identity(); 3];
/// # let slist = vec![Vector6::zeros(); 3];
/// let dt = 0.01;
/// let int_res = 8;
/// let (thetamat, dthetamat) = forward_dynamics_trajectory(&thetalist, &dthetalist,
///                                                          &taumat, &g, &ftipmat,
///                                                          &mlist, &glist, &slist,
///                                                          dt, int_res);
/// ```
pub fn forward_dynamics_trajectory(
    thetalist: &nalgebra::DVector<f64>,
    dthetalist: &nalgebra::DVector<f64>,
    taumat: &Vec<nalgebra::DVector<f64>>,
    g: &nalgebra::Vector3<f64>,
    ftipmat: &Vec<nalgebra::Vector6<f64>>,
    mlist: &Vec<nalgebra::Matrix4<f64>>,
    glist: &Vec<nalgebra::Matrix6<f64>>,
    slist: &Vec<nalgebra::Vector6<f64>>,
    dt: f64,
    int_res: i32,
) -> (Vec<nalgebra::DVector<f64>>, Vec<nalgebra::DVector<f64>>) {
    let n = taumat.len();
    let mut theta = thetalist.clone();
    let mut dtheta = dthetalist.clone();
    let mut thetamat = Vec::new();
    let mut dthetamat = Vec::new();

    thetamat.push(theta.clone());
    dthetamat.push(dtheta.clone());

    for i in 0..n {
        let taulist = &taumat[i];
        let ftip = &ftipmat[i];

        for _ in 0..int_res {
            let ddthetalist =
                forward_dynamics(thetalist, dthetalist, taulist, g, ftip, mlist, glist, slist);

            let (theta_next, dtheta_next) = euler_step(&theta, &dtheta, &ddthetalist, dt);
            theta = theta_next;
            dtheta = dtheta_next;

            thetamat.push(theta.clone());
            dthetamat.push(dtheta.clone());
        }
    }

    return (thetamat, dthetamat);
}
