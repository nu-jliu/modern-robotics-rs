use nalgebra;

use crate::{adjoint, matrix_exp6, trans_inv, vec_to_se3, vec_to_so3};

pub fn ad(v: &nalgebra::Vector6<f64>) -> nalgebra::Matrix6<f64> {
    let omg = nalgebra::Vector3::new(v[0], v[1], v[2]);
    let v3 = nalgebra::Vector3::new(v[3], v[4], v[5]);

    let omgmat = vec_to_so3(&omg);
    let vmat = vec_to_so3(&v3);
    let zeros = nalgebra::Matrix3::zeros();

    return nalgebra::stack![omgmat, zeros; vmat, omgmat];
}

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
