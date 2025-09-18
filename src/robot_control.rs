use nalgebra;

use crate::{inverse_dynamics, mass_matrix};

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
