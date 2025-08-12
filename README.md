# Modern Robotics - Rust Implementation

**Author**: Jingkun Liu

![build](https://github.com/nu-jliu/modern-robotics-rs/actions/workflows/build-rust.yml/badge.svg)

A comprehensive Rust implementation of fundamental robotics algorithms and utilities based on modern robotics principles.

## Overview

This library provides essential mathematical utilities commonly used in robotics applications, with a focus on rigid body motions, SO(3) and SE(3) operations, and numerical computations. The implementation leverages the `nalgebra` linear algebra library for efficient mathematical operations and follows modern robotics theory.

## Features

- **SO(3) Operations**: Rotation matrix operations, matrix exponential and logarithm, Rodrigues' rotation formula
- **SE(3) Operations**: Homogeneous transformations, rigid body motions, matrix exponential and logarithm, adjoint representations
- **Skew-Symmetric Operations**: Convert between 3D/6D vectors and skew-symmetric matrices (so(3)/se(3))
- **Axis-Angle Representations**: Convert between exponential coordinates and axis-angle form for both SO(3) and SE(3)
- **Screw Theory**: Screw axis representations for robotic joint motions
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
use nalgebra::{DVector, Vector3, Vector6, Matrix3, Matrix4};

// Check if a value is near zero
let is_zero = near_zero(1e-7); // true
let not_zero = near_zero(1e-5); // false

// Normalize a vector
let v = DVector::from_vec(vec![3.0, 4.0]);
let normalized = normalize(v);
// Result: [0.6, 0.8] (unit vector)

// SO(3) operations
let omega = Vector3::new(1.0, 2.0, 3.0);

// Convert vector to skew-symmetric matrix
let so3_matrix = vec_to_so3(omega);

// Matrix exponential (Rodrigues' rotation formula)
let rotation_matrix = matrix_exp3(so3_matrix);

// Matrix logarithm
let so3_log = matrix_log3(rotation_matrix);

// Convert exponential coordinates to axis-angle
let (axis, angle) = axis_ang3(omega);

// SE(3) operations
let r = Matrix3::identity();
let p = Vector3::new(1.0, 2.0, 3.0);

// Create homogeneous transformation matrix
let transform = rp_to_trans(r, p);

// Extract rotation and translation
let (rotation, translation) = trans_to_rp(transform);

// Transform inverse
let transform_inv = trans_inv(transform);

// SE(3) vector and matrix conversions
let twist = Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
let se3_matrix = vec_to_se3(twist);
let twist_back = se3_to_vec(se3_matrix);

// SE(3) matrix exponential and logarithm
let transform_exp = matrix_exp6(se3_matrix);
let se3_log = matrix_log6(transform_exp);

// Adjoint representation
let adj_t = adjoint(transform);

// Screw axis from point, direction, and pitch
let q = Vector3::new(0.0, 0.0, 0.0);
let s = Vector3::new(0.0, 0.0, 1.0);
let h = 1.0;
let screw_axis = screw_to_axis(q, s, h);

// SE(3) axis-angle representation
let (screw_axis_unit, theta) = axis_ang6(twist);
```

## API Reference

### Utility Functions (`utils` module)

#### `near_zero(z: f64) -> bool`

Checks if a floating-point value is considered "near zero" within a predefined tolerance.

- **Parameters**: `z` - The value to check
- **Returns**: `true` if the value is less than the tolerance (1e-6), `false` otherwise
- **Location**: `src/utils.rs:22`

#### `normalize(v: nalgebra::DVector<f64>) -> nalgebra::DVector<f64>`

Normalizes a vector to unit length.

- **Parameters**: `v` - The input vector to normalize
- **Returns**: A new vector with the same direction but unit magnitude
- **Location**: `src/utils.rs:45`

### Rigid Body Motions (`rigid_body_motions` module)

#### SO(3) Operations

#### `rot_inv(r: Matrix3<f64>) -> Matrix3<f64>`

Computes the inverse of a rotation matrix (transpose).

- **Parameters**: `r` - A 3×3 rotation matrix
- **Returns**: The inverse rotation matrix (transpose of input)
- **Location**: `src/rigid_body_motions.rs:4`

#### `vec_to_so3(omg: Vector3<f64>) -> Matrix3<f64>`

Converts a 3D vector to its skew-symmetric matrix representation.

- **Parameters**: `omg` - A 3D vector [ωₓ, ωᵧ, ωᵤ]
- **Returns**: The corresponding 3×3 skew-symmetric matrix
- **Location**: `src/rigid_body_motions.rs:8`

#### `so3_to_vec(so3mat: Matrix3<f64>) -> Vector3<f64>`

Extracts a 3D vector from its skew-symmetric matrix representation.

- **Parameters**: `so3mat` - A 3×3 skew-symmetric matrix
- **Returns**: The corresponding 3D vector [ωₓ, ωᵧ, ωᵤ]
- **Location**: `src/rigid_body_motions.rs:15`

#### `axis_ang3(expc3: Vector3<f64>) -> (Vector3<f64>, f64)`

Converts exponential coordinates to axis-angle representation.

- **Parameters**: `expc3` - Exponential coordinates (3D vector)
- **Returns**: Tuple of (unit axis vector, rotation angle in radians)
- **Location**: `src/rigid_body_motions.rs:20`

#### `matrix_exp3(so3mat: Matrix3<f64>) -> Matrix3<f64>`

Computes the matrix exponential for SO(3) using Rodrigues' rotation formula.

- **Parameters**: `so3mat` - A 3×3 skew-symmetric matrix in so(3)
- **Returns**: The corresponding rotation matrix in SO(3)
- **Location**: `src/rigid_body_motions.rs:30`

#### `matrix_log3(r: Matrix3<f64>) -> Matrix3<f64>`

Computes the matrix logarithm for SO(3), the inverse of matrix exponential.

- **Parameters**: `r` - A 3×3 rotation matrix in SO(3)
- **Returns**: The corresponding skew-symmetric matrix in so(3)
- **Location**: `src/rigid_body_motions.rs:41`

#### SE(3) Operations

#### `rp_to_trans(r: Matrix3<f64>, p: Vector3<f64>) -> Matrix4<f64>`

Converts rotation matrix and position vector to homogeneous transformation matrix.

- **Parameters**: `r` - A 3×3 rotation matrix, `p` - A 3D position vector
- **Returns**: A 4×4 homogeneous transformation matrix
- **Location**: `src/rigid_body_motions.rs:64`

#### `trans_to_rp(t: Matrix4<f64>) -> (Matrix3<f64>, Vector3<f64>)`

Extracts rotation matrix and position vector from homogeneous transformation matrix.

- **Parameters**: `t` - A 4×4 homogeneous transformation matrix
- **Returns**: Tuple of (rotation matrix, position vector)
- **Location**: `src/rigid_body_motions.rs:70`

#### `trans_inv(t: Matrix4<f64>) -> Matrix4<f64>`

Computes the inverse of a homogeneous transformation matrix.

- **Parameters**: `t` - A 4×4 homogeneous transformation matrix
- **Returns**: The inverse transformation matrix
- **Location**: `src/rigid_body_motions.rs:86`

#### `vec_to_se3(v: Vector6<f64>) -> Matrix4<f64>`

Converts a 6D twist vector to its se(3) matrix representation.

- **Parameters**: `v` - A 6D twist vector [ωₓ, ωᵧ, ωᵤ, vₓ, vᵧ, vᵤ]
- **Returns**: The corresponding 4×4 se(3) matrix
- **Location**: `src/rigid_body_motions.rs:95`

#### `se3_to_vec(se3mat: Matrix4<f64>) -> Vector6<f64>`

Extracts a 6D twist vector from its se(3) matrix representation.

- **Parameters**: `se3mat` - A 4×4 se(3) matrix
- **Returns**: The corresponding 6D twist vector [ωₓ, ωᵧ, ωᵤ, vₓ, vᵧ, vᵤ]
- **Location**: `src/rigid_body_motions.rs:104`

#### `adjoint(t: Matrix4<f64>) -> Matrix6<f64>`

Computes the adjoint representation of a homogeneous transformation matrix.

- **Parameters**: `t` - A 4×4 homogeneous transformation matrix
- **Returns**: The 6×6 adjoint matrix
- **Location**: `src/rigid_body_motions.rs:115`

#### `screw_to_axis(q: Vector3<f64>, s: Vector3<f64>, h: f64) -> Vector6<f64>`

Converts screw axis parameters to a screw axis representation.

- **Parameters**: `q` - A point on the screw axis, `s` - Unit direction vector, `h` - Pitch of the screw
- **Returns**: The 6D screw axis vector
- **Location**: `src/rigid_body_motions.rs:122`

#### `axis_ang6(expc6: Vector6<f64>) -> (Vector6<f64>, f64)`

Converts 6D exponential coordinates to axis-angle representation.

- **Parameters**: `expc6` - 6D exponential coordinates (twist vector)
- **Returns**: Tuple of (unit screw axis, rotation/translation magnitude)
- **Location**: `src/rigid_body_motions.rs:132`

#### `matrix_exp6(se3mat: Matrix4<f64>) -> Matrix4<f64>`

Computes the matrix exponential for SE(3), converting a twist to a transformation matrix.

- **Parameters**: `se3mat` - A 4×4 se(3) matrix representation of a twist
- **Returns**: The corresponding 4×4 homogeneous transformation matrix
- **Location**: `src/rigid_body_motions.rs:142`

#### `matrix_log6(t: Matrix4<f64>) -> Matrix4<f64>`

Computes the matrix logarithm for SE(3), the inverse of matrix exponential.

- **Parameters**: `t` - A 4×4 homogeneous transformation matrix
- **Returns**: The corresponding 4×4 se(3) matrix representation
- **Location**: `src/rigid_body_motions.rs:168`

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
- Vector normalization for multi-dimensional vectors
- SO(3) operations: rotations, matrix exponentials, logarithms
- SE(3) operations: transformations, inversions, adjoint representations
- Skew-symmetric matrix conversions for both so(3) and se(3)
- Screw theory operations
- Numerical precision and edge cases

Run tests with:

```bash
cargo test
```

### Test Coverage

#### Utils Module Tests (`test_utils.rs`)
- **`test_near_zero`**: Validates tolerance checking for values like 1e-7, 0.0, 1e-5, and 1.0
- **`test_normalize`**: Tests normalization of 3D vectors (e.g., [1,2,3] → normalized unit vector)

#### Rigid Body Motions Tests (`test_rigid_body_motions.rs`)
- **`test_rot_inv`**: Validates rotation matrix inverse operations
- **`test_vec_to_so3`**: Tests conversion from 3D vectors to skew-symmetric matrices
- **`test_so3_to_vec`**: Tests extraction of vectors from skew-symmetric matrices
- **`test_axis_ang`**: Tests conversion to axis-angle representation for SO(3)
- **`test_matrix_exp3`**: Tests matrix exponential using Rodrigues' rotation formula
- **`test_matrix_log3`**: Tests matrix logarithm, inverse of matrix exponential for SO(3)
- **`test_rp_to_trans`**: Tests conversion from rotation and position to transformation matrix
- **`test_trans_to_rp`**: Tests extraction of rotation and position from transformation matrix
- **`test_trans_inv`**: Tests transformation matrix inverse
- **`test_vec_to_se3`**: Tests conversion from 6D twist to se(3) matrix
- **`test_se3_to_vec`**: Tests extraction of 6D twist from se(3) matrix
- **`test_adjoint`**: Tests adjoint representation computation
- **`test_screw_to_axis`**: Tests screw axis conversion from geometric parameters
- **`test_axis_ang6`**: Tests conversion to axis-angle representation for SE(3)
- **`test_matrix_exp6`**: Tests matrix exponential for SE(3) transformations
- **`test_matrix_log6`**: Tests matrix logarithm for SE(3), inverse of matrix exponential

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
- Quaternion operations and conversions
- Forward and inverse kinematics for robot manipulators
- Jacobian calculations for velocity kinematics
- Dynamics and trajectory planning algorithms
- Additional Lie group operations
- Path planning utilities
- Support for different robot configurations
- Integration with robot simulation frameworks