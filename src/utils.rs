use nalgebra;

const TOLERANCE: f64 = 1e-6;

/// Determines whether a scalar is small enough to be treated as zero
///
/// # Arguments
///
/// - `z` (`f64`) - A scalar input to check
///
/// # Returns
///
/// - `bool` - True if z is close to zero, false otherwise
///
/// # Examples
///
/// ```
/// use crate::modern_robotics;
/// let z = -1e-7;
/// let _ = modern_robotics::near_zero(z);
/// ```
pub fn near_zero(z: f64) -> bool {
    return z < TOLERANCE;
}

/// Normalizes a vector
///
/// # Arguments
///
/// - `v` (`nalgebra`) - A vector
///
/// # Returns
///
/// - `nalgebra::DVector<f64>` - A unit vector pointing in the same direction as z
///
/// # Examples
///
/// ```
/// use crate::modern_robotics;
/// use nalgebra;
///
/// let v = nalgebra::DVector::from_vec(vec![1.0, 2.0, 3.0]);
/// let _ = modern_robotics::normalize(v);
/// ```
pub fn normalize(v: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
    return v.normalize();
}
