use crate::utils;
use nalgebra;

pub fn rot_inv(r: nalgebra::Matrix3<f64>) -> nalgebra::Matrix3<f64> {
    r.transpose()
}

pub fn vec_to_so3(omg: nalgebra::Vector3<f64>) -> nalgebra::Matrix3<f64> {
    let so3mat = nalgebra::Matrix3::new(
        0.0, -omg[2], omg[1], omg[2], 0.0, -omg[0], -omg[1], omg[0], 0.0,
    );
    so3mat
}

pub fn so3_to_vec(so3mat: nalgebra::Matrix3<f64>) -> nalgebra::Vector3<f64> {
    let omg = nalgebra::Vector3::new(so3mat[(2, 1)], so3mat[(0, 2)], so3mat[(1, 0)]);
    omg
}

pub fn axis_ang3(expc3: nalgebra::Vector3<f64>) -> (nalgebra::Vector3<f64>, f64) {
    let exp3d = nalgebra::DVector::from_row_slice(expc3.as_slice());
    let omghatd = utils::normalize(exp3d);

    let omghat = nalgebra::Vector3::new(omghatd[0], omghatd[1], omghatd[2]);
    let theta = expc3.norm();

    (omghat, theta)
}

pub fn matrix_exp3(so3mat: nalgebra::Matrix3<f64>) -> nalgebra::Matrix3<f64> {
    let omgtheta = so3_to_vec(so3mat);
    if utils::near_zero(omgtheta.norm()) {
        nalgebra::Matrix3::identity()
    } else {
        let (_, theta) = axis_ang3(omgtheta);
        let omgmat = so3mat / theta;
        nalgebra::Matrix3::identity() + theta.sin() * omgmat + (1.0 - theta.cos()) * omgmat * omgmat
    }
}

pub fn matrix_log3(r: nalgebra::Matrix3<f64>) -> nalgebra::Matrix3<f64> {
    let acosinput = (r.trace() - 1.0) / 2.0;
    if acosinput >= 1.0 {
        nalgebra::Matrix3::zeros()
    } else if acosinput <= -1.0 {
        let omg;
        if !utils::near_zero(1.0 + r[(2, 2)]) {
            omg = (1.0 / (2.0 * (1.0 + r[(2, 2)]).sqrt()))
                * nalgebra::Vector3::new(r[(0, 2)], r[(1, 2)], 1.0 + r[(2, 2)]);
        } else if !utils::near_zero(1.0 + r[(1, 1)]) {
            omg = (1.0 / (2.0 * (1.0 + r[(1, 1)]).sqrt()))
                * nalgebra::Vector3::new(r[(0, 1)], 1.0 + r[(1, 1)], r[(2, 1)]);
        } else {
            omg = (1.0 / (2.0 * (1.0 + r[(0, 0)]).sqrt()))
                * nalgebra::Vector3::new(1.0 + r[(0, 0)], r[(1, 2)], r[(2, 2)]);
        }
        vec_to_so3(omg)
    } else {
        let theta = acosinput.acos();
        theta / 2.0 / theta.sin() * (r - r.transpose())
    }
}
