extern crate nalgebra;

mod utils;

pub mod modern_robotics {
    pub fn near_zero(x: f64) -> bool {
        crate::utils::near_zero(x)
    }
    
    pub fn normalize(v: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        crate::utils::normalize(v)
    }
}
