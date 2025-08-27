use nalgebra;

pub enum Method {
    Cubic,
    Quintic,
}

pub fn cubic_time_scaling(tf: f64, t: f64) -> f64 {
    let a0 = 0.0;
    let a1 = 0.0;
    let a2 = 3.0 / tf.powi(2);
    let a3 = -2.0 / tf.powi(3);

    let s = a0 + a1 * t + a2 * t.powi(2) + a3 * t.powi(3);
    return s;
}

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

pub fn joint_trajectory(
    thetastart: &nalgebra::DVector<f64>,
    thetaend: &nalgebra::DVector<f64>,
    tf: f64,
    n: i32,
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
