use nalgebra;

const TOLERANCE: f64 = 1e-6;

pub fn near_zero(x: f64) -> bool {
    return x < TOLERANCE;
}

pub fn normalize(v: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
    return v.normalize();
}
