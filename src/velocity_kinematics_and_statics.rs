use nalgebra;

use crate::{adjoint, matrix_exp6, vec_to_se3};

pub fn jacobian_body(
    blist: Vec<nalgebra::Vector6<f64>>,
    thetalist: Vec<f64>,
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

        let se3mat = vec_to_se3(-bvec * theta);
        let tij = matrix_exp6(se3mat);
        t = t * tij;

        let adt = adjoint(t);
        let jbvec = adt * blist[i];
        jb.set_column(i, &jbvec);
    }

    return jb;
}

pub fn jacobian_space(
    slist: Vec<nalgebra::Vector6<f64>>,
    thetalist: Vec<f64>,
) -> nalgebra::Matrix6xX<f64> {
    let n = slist.len();
    let mut js: nalgebra::Matrix6xX<f64> = nalgebra::Matrix6xX::zeros(n);
    let mut t = nalgebra::Matrix4::identity();

    let svec = slist[0];
    js.set_column(0, &svec);

    for i in 1..n {
        let svec = slist[i - 1];
        let theta = thetalist[i - 1];
        let se3mat = vec_to_se3(svec * theta);
        let tij = matrix_exp6(se3mat);
        t = t * tij;

        let adt = adjoint(t);
        let jsvec = adt * slist[i];
        js.set_column(i, &jsvec);
    }

    return js;
}
