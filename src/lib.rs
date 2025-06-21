extern crate nalgebra;

pub mod modern_robotics {
    pub fn near_zero(x: f64) -> bool {
        return x < 1e-6;
    }

    pub fn normalize(v: nalgebra::Vector3<f64>) -> nalgebra::Vector3<f64> {
        return v.normalize();
    }
}
