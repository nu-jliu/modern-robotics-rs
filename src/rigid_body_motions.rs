use crate::{near_zero, normalize};
use nalgebra;

/// Inverts a rotation matrix.
///
/// # Arguments
///
/// - `r` (`nalgebra::Matrix3<f64>`) - A rotation matrix
///
/// # Returns
///
/// - `nalgebra::Matrix3<f64>` - The inverse of r
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let r = nalgebra::matrix![
///     0.0, 0.0, 1.0;
///     1.0, 0.0, 0.0;
///     0.0, 1.0, 0.0
/// ];
/// let r_inv = modern_robotics::rot_inv(&r);
/// ```
pub fn rot_inv(r: &nalgebra::Matrix3<f64>) -> nalgebra::Matrix3<f64> {
    r.transpose()
}

/// Converts a 3-vector to an so(3) representation
///
/// # Arguments
///
/// - `omg` (`nalgebra::Vector3<f64>`) - A 3-vector
///
/// # Returns
///
/// - `nalgebra::Matrix3<f64>` - The skew symmetric representation of omg
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let omg = nalgebra::Vector3::new(1.0, 2.0, 3.0);
/// let so3mat = modern_robotics::vec_to_so3(&omg);
/// ```
pub fn vec_to_so3(omg: &nalgebra::Vector3<f64>) -> nalgebra::Matrix3<f64> {
    let so3mat = nalgebra::matrix![
        0.0, -omg[2], omg[1];
        omg[2], 0.0, -omg[0];
        -omg[1], omg[0], 0.0
    ];
    return so3mat;
}

/// Converts an so(3) representation to a 3-vector
///
/// # Arguments
///
/// - `so3mat` (`&nalgebra::Matrix3<f64>`) - A 3x3 skew-symmetric matrix
///
/// # Returns
///
/// - `nalgebra::Vector3<f64>` - The 3-vector corresponding to so3mat
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let so3mat = nalgebra::matrix![
///     0.0, -3.0, 2.0;
///     3.0, 0.0, -1.0;
///     -2.0, 1.0, 0.0
/// ];
/// let omg = modern_robotics::so3_to_vec(&so3mat);
/// ```
pub fn so3_to_vec(so3mat: &nalgebra::Matrix3<f64>) -> nalgebra::Vector3<f64> {
    let omg = nalgebra::Vector3::new(so3mat[(2, 1)], so3mat[(0, 2)], so3mat[(1, 0)]);
    return omg;
}

/// Converts a 3-vector of exponential coordinates for rotation into axis-angle form
///
/// # Arguments
///
/// - `expc3` (`&nalgebra::Vector3<f64>`) - A 3-vector of exponential coordinates for rotation
///
/// # Returns
///
/// - `(nalgebra::Vector3<f64>, f64)` - A unit rotation axis, The corresponding rotation angle
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let expc3 = nalgebra::Vector3::new(1.0, 2.0, 3.0);
/// let (omghat, theta) = modern_robotics::axis_ang3(&expc3);
/// ```
pub fn axis_ang3(expc3: &nalgebra::Vector3<f64>) -> (nalgebra::Vector3<f64>, f64) {
    let exp3d = nalgebra::DVector::from_row_slice(expc3.as_slice());
    let omghatd = normalize(&exp3d);

    let omghat = nalgebra::Vector3::new(omghatd[0], omghatd[1], omghatd[2]);
    let theta = expc3.norm();

    return (omghat, theta);
}

/// Computes the matrix exponential of a matrix in so(3)
///
/// # Arguments
///
/// - `so3mat` (`&nalgebra::Matrix3<f64>`) - A 3x3 skew-symmetric matrix
///
/// # Returns
///
/// - `nalgebra::Matrix3<f64>` - The matrix exponential of so3mat
///
/// # Examples
///
/// ```
/// use crate::modern_robotics;
///
/// let so3mat = nalgebra::matrix![
///     0.0, -3.0, 2.0;
///     3.0, 0.0, -1.0;
///     -2.0, 1.0, 0.0
/// ];
/// let r = modern_robotics::matrix_exp3(&so3mat);
/// ```
pub fn matrix_exp3(so3mat: &nalgebra::Matrix3<f64>) -> nalgebra::Matrix3<f64> {
    let omgtheta = so3_to_vec(so3mat);

    if near_zero(omgtheta.norm()) {
        return nalgebra::Matrix3::identity();
    } else {
        let (_, theta) = axis_ang3(&omgtheta);
        let omgmat = so3mat / theta;

        return nalgebra::Matrix3::identity()
            + theta.sin() * omgmat
            + (1.0 - theta.cos()) * omgmat * omgmat;
    }
}

/// Computes the matrix logarithm of a rotation matrix
///
/// # Arguments
///
/// - `r` (`&nalgebra::Matrix3<f64>`) - A 3x3 rotation matrix
///
/// # Returns
///
/// - `nalgebra::Matrix3<f64>` - The matrix logarithm of R
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let r = nalgebra::matrix![
///     0.0, 0.0, 1.0;
///     1.0, 0.0, 0.0;
///     0.0, 1.0 ,0.0    
/// ];
/// let _ = modern_robotics::matrix_log3(&r);
/// ```
pub fn matrix_log3(r: &nalgebra::Matrix3<f64>) -> nalgebra::Matrix3<f64> {
    let acosinput = (r.trace() - 1.0) / 2.0;

    if acosinput >= 1.0 {
        return nalgebra::Matrix3::zeros();
    } else if acosinput <= -1.0 {
        let omg;
        if !near_zero(1.0 + r[(2, 2)]) {
            omg = (1.0 / (2.0 * (1.0 + r[(2, 2)]).sqrt()))
                * nalgebra::vector![r[(0, 2)], r[(1, 2)], 1.0 + r[(2, 2)]];
        } else if !near_zero(1.0 + r[(1, 1)]) {
            omg = (1.0 / (2.0 * (1.0 + r[(1, 1)]).sqrt()))
                * nalgebra::vector![r[(0, 1)], 1.0 + r[(1, 1)], r[(2, 1)]];
        } else {
            omg = (1.0 / (2.0 * (1.0 + r[(0, 0)]).sqrt()))
                * nalgebra::vector![1.0 + r[(0, 0)], r[(1, 2)], r[(2, 2)]];
        }

        return vec_to_so3(&omg);
    } else {
        let theta = acosinput.acos();
        return theta / 2.0 / theta.sin() * (r - r.transpose());
    }
}

/// Converts a rotation matrix and a position vector into homogeneous transformation matrix
///
/// # Arguments
///
/// - `r` (`&nalgebra::Matrix3<f64>`) - A 3x3 rotation matrix
/// - `p` (`&nalgebra::Vector3<f64>`) - A 3-vector
///
/// # Returns
///
/// - `nalgebra::Matrix4<f64>` - A homogeneous transformation matrix corresponding to the inputs
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let r = nalgebra::matrix![
///     1.0, 0.0, 0.0;
///     0.0, 0.0, -1.0;
///     0.0, 1.0, 0.0
/// ];
/// let p = nalgebra::Vector3::new(1.0, 2.0, 5.0);
/// let t = modern_robotics::rp_to_trans(&r, &p);
/// ```
pub fn rp_to_trans(
    r: &nalgebra::Matrix3<f64>,
    p: &nalgebra::Vector3<f64>,
) -> nalgebra::Matrix4<f64> {
    let bottom = nalgebra::Matrix1x3::zeros();
    let one = nalgebra::Vector1::new(1.0);

    return nalgebra::stack![r, p; bottom, one];
}

/// Converts a homogeneous transformation matrix into a rotation matrix and position vector
///
/// # Arguments
///
/// - `t` (`&nalgebra::Matrix4<f64>`) - A homogeneous transformation matrix
///
/// # Returns
///
/// - `(nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>)` - The corresponding rotation matrix, The corresponding position vector.
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let t = nalgebra::matrix![
///     1.0, 0.0, 0.0, 0.0;
///     0.0, 0.0, -1.0, 0.0;
///     0.0, 1.0, 0.0, 3.0;
///     0.0, 0.0, 0.0, 1.0
/// ];
/// let (r, p) = modern_robotics::trans_to_rp(&t);
/// ```
pub fn trans_to_rp(t: &nalgebra::Matrix4<f64>) -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
    let r = t.fixed_view::<3, 3>(0, 0).into();
    let p = nalgebra::Vector3::new(t[(0, 3)], t[(1, 3)], t[(2, 3)]);

    return (r, p);
}

/// Inverts a homogeneous transformation matrix
///
/// # Arguments
///
/// - `t` (`&nalgebra::Matrix4<f64>`) - A homogeneous transformation matrix
///
/// # Returns
///
/// - `nalgebra::Matrix4<f64>` - The inverse of T
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let t = nalgebra::matrix![
///     1.0, 0.0, 0.0, 0.0;
///     0.0, 0.0, -1.0, 0.0;
///     0.0, 1.0, 0.0, 3.0;
///     0.0, 0.0, 0.0, 1.0
/// ];
/// let t_inv = modern_robotics::trans_inv(&t);
/// ```
pub fn trans_inv(t: &nalgebra::Matrix4<f64>) -> nalgebra::Matrix4<f64> {
    let (r, p) = trans_to_rp(t);
    let rt = rot_inv(&r);
    let n_rt_p = -rt * p;
    let bottom = nalgebra::Matrix1x3::zeros();
    let one = nalgebra::Vector1::new(1.0);

    let t_inv = nalgebra::stack![rt, n_rt_p; bottom, one];
    return t_inv;
}

/// Converts a spatial velocity vector into a 4x4 matrix in se3
///
/// # Arguments
///
/// - `v` (`&nalgebra::Vector6<f64>`) - A 6-vector representing a spatial velocity
///
/// # Returns
///
/// - `nalgebra::Matrix4<f64>` - The 4x4 se3 representation of V
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let v = nalgebra::Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
/// let se3mat = modern_robotics::vec_to_se3(&v);
/// ```
pub fn vec_to_se3(v: &nalgebra::Vector6<f64>) -> nalgebra::Matrix4<f64> {
    let omg = nalgebra::Vector3::new(v[0], v[1], v[2]);
    let upper_left = vec_to_so3(&omg);
    let upper_right = nalgebra::Vector3::new(v[3], v[4], v[5]);
    let bottom_left = nalgebra::Matrix1x3::zeros();
    let bottom_right = nalgebra::Vector1::zeros();

    return nalgebra::stack![upper_left, upper_right; bottom_left, bottom_right];
}

/// Converts an se3 matrix into a spatial velocity vector
///
/// # Arguments
///
/// - `se3mat` (`&nalgebra::Matrix4<f64>`) - A 4x4 matrix in se3
///
/// # Returns
///
/// - `nalgebra::Vector6<f64>` - The spatial velocity 6-vector corresponding to se3mat
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let se3mat = nalgebra::matrix![
///     0.0, -3.0, 2.0, 4.0;
///     3.0, 0.0, -1.0, 5.0;
///     -2.0, 1.0, 0.0, 6.0;
///     0.0, 0.0, 0.0, 0.0
/// ];
/// let v = modern_robotics::se3_to_vec(&se3mat);
/// ```
pub fn se3_to_vec(se3mat: &nalgebra::Matrix4<f64>) -> nalgebra::Vector6<f64> {
    nalgebra::Vector6::new(
        se3mat[(2, 1)],
        se3mat[(0, 2)],
        se3mat[(1, 0)],
        se3mat[(0, 3)],
        se3mat[(1, 3)],
        se3mat[(2, 3)],
    )
}

/// Computes the adjoint representation of a homogeneous transformation matrix
///
/// # Arguments
///
/// - `t` (`&nalgebra::Matrix4<f64>`) - A homogeneous transformation matrix.
///
/// # Returns
///
/// - `nalgebra::Matrix6<f64>` - The 6x6 adjoint representation \[AdT\] of T.
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let t = nalgebra::matrix![
///     1.0, 0.0, 0.0, 0.0;
///     0.0, 0.0, -1.0, 0.0;
///     0.0, 1.0, 0.0, 3.0;
///     0.0, 0.0, 0.0, 1.0
/// ];
/// let adt = modern_robotics::adjoint(&t);
/// ```
pub fn adjoint(t: &nalgebra::Matrix4<f64>) -> nalgebra::Matrix6<f64> {
    let (r, p) = trans_to_rp(t);
    let upper_right = nalgebra::Matrix3::zeros();
    let bottom_left = vec_to_so3(&p) * r;

    return nalgebra::stack![r, upper_right; bottom_left, r];
}

/// Takes a parametric description of a screw axis and converts it to a normalized screw axis.
///
/// # Arguments
///
/// - `q` (`&nalgebra::Vector3<f64>`) - A point lying on the screw axis
/// - `s` (`&nalgebra::Vector3<f64>`) - A unit vector in the direction of the screw axis
/// - `h` (`f64`) - The pitch of the screw axis
///
/// # Returns
///
/// - `nalgebra::Vector6<f64>` - A normalized screw axis described by the inputs
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let q = nalgebra::Vector3::new(3.0, 0.0, 0.0);
/// let s = nalgebra::Vector3::new(0.0, 0.0, 1.0);
/// let h = 2.0;
/// let v = modern_robotics::screw_to_axis(&q, &s, h);
/// ```
pub fn screw_to_axis(
    q: &nalgebra::Vector3<f64>,
    s: &nalgebra::Vector3<f64>,
    h: f64,
) -> nalgebra::Vector6<f64> {
    let omg = s;
    let v = q.cross(&s) + h * s;

    return nalgebra::Vector6::new(omg[0], omg[1], omg[2], v[0], v[1], v[2]);
}

/// Converts a 6-vector of exponential coordinates into screw axis-angleform
///
/// # Arguments
///
/// - `expc6` (`&nalgebra::Vector6<f64>`) - A 6-vector of exponential coordinates for rigid-body motion S*theta
///
/// # Returns
///
/// - `(nalgebra::Vector6<f64>, f64)` - The corresponding normalized screw axis, The distance traveled along/about S.
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let expc6 = nalgebra::Vector6::new(1.0, 0.0, 0.0, 1.0, 2.0, 3.0);
/// let (s, theta) = modern_robotics::axis_ang6(&expc6);
/// ```
pub fn axis_ang6(expc6: &nalgebra::Vector6<f64>) -> (nalgebra::Vector6<f64>, f64) {
    let omg = nalgebra::Vector3::new(expc6[0], expc6[1], expc6[2]);
    let mut theta = omg.norm();

    if near_zero(theta) {
        let v = nalgebra::Vector3::new(expc6[3], expc6[4], expc6[5]);
        theta = v.norm();
    }

    return (expc6 / theta, theta);
}

/// Computes the matrix exponential of an se3 representation of exponential coordinates
///
/// # Arguments
///
/// - `se3mat` (`nalgebra::Matrix4<f64>`) - A matrix in se3
///
/// # Returns
///
/// - `nalgebra::Matrix4<f64>` - The matrix exponential of se3mat
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let se3mat = nalgebra::matrix![
///     0.0, 0.0, 0.0, 0.0;
///     0.0, 0.0, -1.57079632, 2.35619449;
///     0.0, 1.57079632, 0.0, 2.35619449;
///     0.0, 0.0, 0.0, 0.0
/// ];
/// let t = modern_robotics::matrix_exp6(&se3mat);
/// ```
pub fn matrix_exp6(se3mat: &nalgebra::Matrix4<f64>) -> nalgebra::Matrix4<f64> {
    let so3mat: nalgebra::Matrix3<f64> = se3mat.fixed_view::<3, 3>(0, 0).into();
    let omgtheta = so3_to_vec(&so3mat);

    if near_zero(omgtheta.norm()) {
        let r = nalgebra::Matrix3::identity();
        let p: nalgebra::Vector3<f64> = se3mat.fixed_view::<3, 1>(0, 3).into();
        let bottom_left = nalgebra::Matrix1x3::zeros();
        let bottom_right = nalgebra::Vector1::new(1.0);

        return nalgebra::stack![r, p; bottom_left, bottom_right];
    } else {
        let (_, theta) = axis_ang3(&omgtheta);
        let omgmat = so3mat / theta;
        let r = matrix_exp3(&so3mat);
        let i_33 = nalgebra::Matrix3::identity();
        let pp: nalgebra::Vector3<f64> = se3mat.fixed_view::<3, 1>(0, 3).into();
        let p = (i_33 * theta
            + (1.0 - theta.cos()) * omgmat
            + (theta - theta.sin()) * (omgmat * omgmat))
            * pp
            / theta;
        let bottom_left = nalgebra::Matrix1x3::zeros();
        let bottom_right = nalgebra::Vector1::new(1.0);

        return nalgebra::stack![r, p; bottom_left, bottom_right];
    }
}

/// Computes the matrix logarithm of a homogeneous transformation matrix
///
/// # Arguments
///
/// - `t` (`&nalgebra::Matrix4<f64>`) - A matrix in SE3
///
/// # Returns
///
/// - `nalgebra::Matrix4<f64>` - The matrix logarithm of T
///
/// # Examples
///
/// ```
/// use nalgebra;
/// use crate::modern_robotics;
///
/// let t = nalgebra::matrix![
///     1.0, 0.0, 0.0, 0.0;
///     0.0, 0.0, -1.0, 0.0;
///     0.0, 1.0, 0.0, 3.0;
///     0.0, 0.0, 0.0, 1.0
/// ];
/// let se3mat = modern_robotics::matrix_log6(&t);
/// ```
pub fn matrix_log6(t: &nalgebra::Matrix4<f64>) -> nalgebra::Matrix4<f64> {
    let (r, _) = trans_to_rp(t);
    let omgmat = matrix_log3(&r);

    if omgmat.iter().all(|x| near_zero(*x)) {
        let upper_left = nalgebra::Matrix3::zeros();
        let upper_right = nalgebra::Vector3::new(t[(0, 3)], t[(1, 3)], t[(2, 3)]);
        let bottom_left = nalgebra::Matrix1x3::zeros();
        let bottom_right = nalgebra::Vector1::zeros();

        return nalgebra::stack![upper_left, upper_right; bottom_left, bottom_right];
    } else {
        let theta = ((r.trace() - 1.0) / 2.0).acos();
        let i_33 = nalgebra::Matrix3::identity();
        let upper_right = (i_33 - omgmat / 2.0
            + (1.0 / theta - 1.0 / (theta / 2.0).tan() / 2.0) * (omgmat * omgmat) / theta)
            * nalgebra::Vector3::new(t[(0, 3)], t[(1, 3)], t[(2, 3)]);
        let bottom_left = nalgebra::Matrix1x3::zeros();
        let bottom_right = nalgebra::Vector1::zeros();

        return nalgebra::stack![omgmat, upper_right; bottom_left, bottom_right];
    }
}
