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
