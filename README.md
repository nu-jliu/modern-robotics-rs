# Modern Robotics - Rust Implementation

**Author**: Jingkun Liu

![build](https://github.com/nu-jliu/modern-robotics-rs/actions/workflows/build-rust.yml/badge.svg)

A Rust implementation of fundamental robotics algorithms and utilities based on modern robotics principles.

## Overview

This library provides essential mathematical utilities commonly used in robotics applications, with a focus on rigid body motions, SO(3) operations, and numerical computations. The implementation leverages the `nalgebra` linear algebra library for efficient mathematical operations and follows modern robotics theory.

## Features

- **Rigid Body Motions**: SO(3) operations including rotation matrix inverse, matrix exponential, and Rodrigues' rotation formula
- **Skew-Symmetric Operations**: Convert between 3D vectors and skew-symmetric matrices
- **Axis-Angle Representations**: Convert between exponential coordinates and axis-angle form
- **Numerical Tolerance Checking**: Check if values are near zero within a specified tolerance
- **Vector Normalization**: Normalize vectors to unit length using nalgebra's DVector
- **Robust Testing**: Comprehensive test suite covering all mathematical operations

## Dependencies

- `nalgebra` (v0.33.2) - Linear algebra library for Rust
- `assert_float_eq` (v1.1.4) - Floating point assertions for testing

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
modern-robotics = "0.1.1"
```

## Usage

```rust
use modern_robotics::{utils::{near_zero, normalize}, rigid_body_motions::*};
use nalgebra::{DVector, Vector3, Matrix3};

// Check if a value is near zero
let is_zero = near_zero(1e-7); // true
let not_zero = near_zero(1e-5); // false

// Normalize a vector
let v = DVector::from_vec(vec![3.0, 4.0]);
let normalized = normalize(v);
// Result: [0.6, 0.8] (unit vector)

// Rigid body motion operations
let omega = Vector3::new(1.0, 2.0, 3.0);

// Convert vector to skew-symmetric matrix
let so3_matrix = vec_to_so3(omega);

// Matrix exponential (Rodrigues' rotation formula)
let rotation_matrix = matrix_exp3(so3_matrix);

// Convert exponential coordinates to axis-angle
let (axis, angle) = axis_ang3(omega);
```

## API Reference

### Utility Functions (`utils` module)

#### `near_zero(x: f64) -> bool`

Checks if a floating-point value is considered "near zero" within a predefined tolerance.

- **Parameters**: `x` - The value to check
- **Returns**: `true` if the absolute value is less than the tolerance (1e-6), `false` otherwise
- **Location**: `src/utils.rs:5`

#### `normalize(v: nalgebra::DVector<f64>) -> nalgebra::DVector<f64>`

Normalizes a vector to unit length.

- **Parameters**: `v` - The input vector to normalize
- **Returns**: A new vector with the same direction but unit magnitude
- **Location**: `src/utils.rs:9`

### Rigid Body Motions (`rigid_body_motions` module)

#### `rot_inv(r: Matrix3<f64>) -> Matrix3<f64>`

Computes the inverse of a rotation matrix (transpose).

- **Parameters**: `r` - A 3×3 rotation matrix
- **Returns**: The inverse rotation matrix (transpose of input)
- **Location**: `src/rigid_body_motions.rs:8`

#### `vec_to_so3(omg: Vector3<f64>) -> Matrix3<f64>`

Converts a 3D vector to its skew-symmetric matrix representation.

- **Parameters**: `omg` - A 3D vector [ωₓ, ωᵧ, ωᵤ]
- **Returns**: The corresponding 3×3 skew-symmetric matrix
- **Location**: `src/rigid_body_motions.rs:16`

#### `so3_to_vec(so3mat: Matrix3<f64>) -> Vector3<f64>`

Extracts a 3D vector from its skew-symmetric matrix representation.

- **Parameters**: `so3mat` - A 3×3 skew-symmetric matrix
- **Returns**: The corresponding 3D vector [ωₓ, ωᵧ, ωᵤ]
- **Location**: `src/rigid_body_motions.rs:29`

#### `axis_ang3(expc3: Vector3<f64>) -> (Vector3<f64>, f64)`

Converts exponential coordinates to axis-angle representation.

- **Parameters**: `expc3` - Exponential coordinates (3D vector)
- **Returns**: Tuple of (unit axis vector, rotation angle in radians)
- **Location**: `src/rigid_body_motions.rs:39`

#### `matrix_exp3(so3mat: Matrix3<f64>) -> Matrix3<f64>`

Computes the matrix exponential for SO(3) using Rodrigues' rotation formula.

- **Parameters**: `so3mat` - A 3×3 skew-symmetric matrix in so(3)
- **Returns**: The corresponding rotation matrix in SO(3)
- **Location**: `src/rigid_body_motions.rs:53`

## Project Structure

```
modern-robotics-rs/
├── .github/
│   └── workflows/
│       └── build-rust.yml           # CI/CD workflow for automated builds and tests
├── src/
│   ├── lib.rs                       # Main library interface
│   ├── utils.rs                     # Core utility functions
│   └── rigid_body_motions.rs        # SO(3) operations and rigid body motions
├── tests/
│   ├── test_utils.rs                # Tests for utility functions
│   └── test_rigid_body_motions.rs   # Tests for rigid body motion functions
├── target/                          # Build artifacts (generated)
├── Cargo.toml                       # Package configuration
├── Cargo.lock                       # Dependency lock file
└── README.md                        # This file
```

## Testing

The project includes comprehensive tests covering:

- Near-zero tolerance checking with various input values
- Vector normalization for 2D, 3D, and 4D vectors
- SO(3) operations and rigid body motions
- Matrix exponential computations
- Skew-symmetric matrix conversions
- Numerical precision and edge cases

Run tests with:

```bash
cargo test
```

### Test Coverage

#### Utils Module Tests (`test_utils.rs`)
- **`test_near_zero`**: Validates tolerance checking for values like 1e-7, 0.0, 1e-5, and 1.0
- **`test_normalize_3d`**: Tests normalization of 3D vectors (e.g., [1,1,1] → [0.577, 0.577, 0.577])
- **`test_normalize_2d`**: Tests normalization of 2D vectors (e.g., [3,4] → [0.6, 0.8])
- **`test_normalize_4d`**: Tests normalization of higher-dimensional vectors
- **`test_normalize_magnitude`**: Verifies that normalized vectors have unit magnitude

#### Rigid Body Motions Tests (`test_rigid_body_motions.rs`)
- **`test_rot_inv`**: Validates rotation matrix inverse operations
- **`test_vec_to_so3`**: Tests conversion from 3D vectors to skew-symmetric matrices
- **`test_so3_to_vec`**: Tests extraction of vectors from skew-symmetric matrices
- **`test_axis_ang3`**: Tests conversion to axis-angle representation
- **`test_matrix_exp3`**: Tests matrix exponential using Rodrigues' rotation formula

## Constants

- **`TOLERANCE`**: 1e-6 - The threshold used for near-zero comparisons

## Continuous Integration

The project uses GitHub Actions for automated building and testing. The workflow:

- Triggers on pushes and pull requests to the `master` branch
- Runs on Ubuntu 22.04
- Sets up the Rust toolchain automatically
- Builds the package in release mode
- Runs all tests
- Uploads build artifacts

## Development

### Building

```bash
cargo build
```

### Running Tests

```bash
cargo test
```

### Documentation

Generate documentation with:

```bash
cargo doc --open
```

## Version History

- **v0.1.1**: Current version with basic vector operations and tolerance checking

## License

This project follows standard Rust packaging conventions. Check the repository for specific license information.

## Contributing

This library serves as a foundation for robotics applications in Rust. Contributions that expand the mathematical utilities while maintaining performance and correctness are welcome.

## Future Enhancements

Potential areas for expansion include:
- SE(3) operations and homogeneous transformations
- Quaternion operations and conversions
- Forward and inverse kinematics
- Jacobian calculations
- Dynamics and trajectory planning
- Additional Lie group operations
- Path planning utilities