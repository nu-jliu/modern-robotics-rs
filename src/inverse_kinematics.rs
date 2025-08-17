use nalgebra;

use crate::{
    adjoint, fkin_body, fkin_space, jacobian_body, jacobian_space, matrix_log6, se3_to_vec,
    trans_inv,
};

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
