use nalgebra;

use crate::*;

pub fn fkin_body(
    m: nalgebra::Matrix4<f64>,
    blist: Vec<nalgebra::Vector6<f64>>,
    thetalist: Vec<f64>,
) -> nalgebra::Matrix4<f64> {
    let mut t = m.clone();

    for i in 0..thetalist.len() {
        let bvec = blist[i];
        let theta = thetalist[i];
        let se3mat = vec_to_se3(bvec * theta);
        let tij = matrix_exp6(se3mat);
        t = t * tij;
    }

    return t;
}

pub fn fkin_space(
    m: nalgebra::Matrix4<f64>,
    slist: Vec<nalgebra::Vector6<f64>>,
    thetalist: Vec<f64>,
) -> nalgebra::Matrix4<f64> {
    let mut t = m.clone();

    for i in 0..thetalist.len() {
        let j = thetalist.len() - 1 - i;
        let svec = slist[j];
        let theta = thetalist[j];
        let se3mat = vec_to_se3(svec * theta);
        let tji = matrix_exp6(se3mat);
        t = tji * t;
    }

    return t;
}
