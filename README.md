# Modern Robotics - Rust Implementation

**Author**: Jingkun Liu

![build](https://github.com/nu-jliu/modern-robotics-rs/actions/workflows/build-rust.yml/badge.svg)

A comprehensive Rust implementation of fundamental robotics algorithms and utilities based on modern robotics principles.

## Overview

This library provides essential mathematical utilities commonly used in robotics applications, with a focus on rigid body motions, SO(3) and SE(3) operations, forward and inverse kinematics, robot dynamics, and numerical computations. The implementation leverages the `nalgebra` linear algebra library for efficient mathematical operations and follows modern robotics theory.

## Features

- **SO(3) Operations**: Rotation matrix operations, matrix exponential and logarithm, Rodrigues' rotation formula
- **SE(3) Operations**: Homogeneous transformations, rigid body motions, matrix exponential and logarithm, adjoint representations
- **Skew-Symmetric Operations**: Convert between 3D/6D vectors and skew-symmetric matrices (so(3)/se(3))
- **Axis-Angle Representations**: Convert between exponential coordinates and axis-angle form for both SO(3) and SE(3)
- **Screw Theory**: Screw axis representations for robotic joint motions
- **Forward Kinematics**: Body frame and space frame forward kinematics calculations
- **Inverse Kinematics**: Newton-Raphson method for body and space frame inverse kinematics
- **Velocity Kinematics**: Jacobian calculations for both body and space frames
- **Dynamics of Open Chains**: Robot dynamics calculations including inverse dynamics, forward dynamics, mass matrix computation, gravity forces, and trajectory simulation
- **Trajectory Generation**: Time scaling functions for smooth trajectory generation including cubic and quintic polynomial scaling
- **Numerical Tolerance Checking**: Check if values are near zero within a specified tolerance
- **Vector Normalization**: Normalize vectors to unit length using nalgebra's DVector
- **Robust Testing**: Comprehensive test suite covering all mathematical operations

## Dependencies

- `nalgebra` (v0.34.0) - Linear algebra library for Rust
- `assert_float_eq` (v1.1.4) - Floating point assertions for testing

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
modern-robotics = "0.1.2"
```

## Usage

```rust
use modern_robotics::*;
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

// Forward kinematics
let m = Matrix4::identity(); // End-effector configuration at zero position
let blist = vec![Vector6::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)]; // Body screw axes
let thetalist = vec![1.57]; // Joint angles in radians
let t_body = fkin_body(m, blist.clone(), thetalist.clone());

let slist = vec![Vector6::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)]; // Space screw axes
let t_space = fkin_space(m, slist.clone(), thetalist.clone());

// Jacobian calculations
let jb = jacobian_body(blist, thetalist.clone());
let js = jacobian_space(slist, thetalist.clone());

// Inverse kinematics
use nalgebra::DVector;
let thetalist0 = DVector::from_vec(vec![0.0; slist.len()]); // Initial guess
let emog = 0.01; // Angular error tolerance
let ev = 0.01; // Linear error tolerance
let target_pose = Matrix4::identity(); // Target end-effector pose

let (theta_body, success_body) = ikin_body(&blist, &m, &target_pose, &thetalist0, emog, ev);
let (theta_space, success_space) = ikin_space(&slist, &m, &target_pose, &thetalist0, emog, ev);

// Dynamics calculations
let dthetalist = DVector::from_vec(vec![0.1, 0.2, 0.3]); // Joint velocities
let ddthetalist = DVector::from_vec(vec![1.0, 2.0, 3.0]); // Joint accelerations
let g = Vector3::new(0.0, 0.0, -9.8); // Gravity vector
let ftip = Vector6::zeros(); // End-effector wrench
let mlist = vec![Matrix4::identity(); 4]; // Link transformations
let glist = vec![Matrix6::identity(); 3]; // Link inertia matrices

// Inverse dynamics (compute required joint torques)
let taulist = inverse_dynamics(&thetalist, &dthetalist, &ddthetalist, &g, &ftip, &mlist, &glist, &slist);

// Mass matrix computation
let mass_matrix_m = mass_matrix(&thetalist, &mlist, &glist, &slist);

// Velocity-dependent forces (Coriolis and centrifugal forces)
let c_forces = vel_quadratic_forces(&thetalist, &dthetalist, &mlist, &glist, &slist);

// Gravity forces
let grav_forces = gravity_forces(&thetalist, &g, &mlist, &glist, &slist);

// End-effector forces
let ee_forces = end_effector_forces(&thetalist, &ftip, &mlist, &glist, &slist);

// Forward dynamics (compute joint accelerations from torques)
let taulist = DVector::from_vec(vec![1.0, 2.0, 3.0]); // Applied joint torques
let joint_accelerations = forward_dynamics(&thetalist, &dthetalist, &taulist, &g, &ftip, &mlist, &glist, &slist);

// Euler integration step for trajectory simulation
let dt = 0.01; // Time step
let (theta_next, dtheta_next) = euler_step(&thetalist, &dthetalist, &joint_accelerations, dt);

// Trajectory generation with time scaling
let tf = 2.0; // Total time
let t = 0.6; // Current time
let s_cubic = cubic_time_scaling(tf, t); // Cubic polynomial scaling
let s_quintic = quintic_time_scaling(tf, t); // Quintic polynomial scaling

// Adjoint representation of twist
let twist = Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
let ad_twist = ad(&twist);
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

### Forward Kinematics (`forward_kinematics` module)

#### `fkin_body(m: Matrix4<f64>, blist: Vec<Vector6<f64>>, thetalist: Vec<f64>) -> Matrix4<f64>`

Computes forward kinematics using the body frame representation.

- **Parameters**: `m` - End-effector configuration at zero position, `blist` - Body screw axes, `thetalist` - Joint angles
- **Returns**: The end-effector configuration
- **Location**: `src/forward_kinematics.rs:5`

#### `fkin_space(m: Matrix4<f64>, slist: Vec<Vector6<f64>>, thetalist: Vec<f64>) -> Matrix4<f64>`

Computes forward kinematics using the space frame representation.

- **Parameters**: `m` - End-effector configuration at zero position, `slist` - Space screw axes, `thetalist` - Joint angles
- **Returns**: The end-effector configuration
- **Location**: `src/forward_kinematics.rs:23`

### Inverse Kinematics (`inverse_kinematics` module)

#### `ikin_body(blist: &Vec<Vector6<f64>>, m: &Matrix4<f64>, t: &Matrix4<f64>, thetalist0: &DVector<f64>, emog: f64, ev: f64) -> (DVector<f64>, bool)`

Computes inverse kinematics using the body frame representation with Newton-Raphson method.

- **Parameters**: `blist` - Body screw axes, `m` - End-effector at zero position, `t` - Target pose, `thetalist0` - Initial joint angle guess, `emog` - Angular error tolerance, `ev` - Linear error tolerance
- **Returns**: Tuple of (joint angles, convergence success flag)
- **Location**: `src/inverse_kinematics.rs:8`

#### `ikin_space(slist: &Vec<Vector6<f64>>, m: &Matrix4<f64>, t: &Matrix4<f64>, thetalist0: &DVector<f64>, emog: f64, ev: f64) -> (DVector<f64>, bool)`

Computes inverse kinematics using the space frame representation with Newton-Raphson method.

- **Parameters**: `slist` - Space screw axes, `m` - End-effector at zero position, `t` - Target pose, `thetalist0` - Initial joint angle guess, `emog` - Angular error tolerance, `ev` - Linear error tolerance
- **Returns**: Tuple of (joint angles, convergence success flag)
- **Location**: `src/inverse_kinematics.rs:48`

### Dynamics of Open Chains (`dynamics_of_open_chains` module)

#### `ad(v: &Vector6<f64>) -> Matrix6<f64>`

Computes the adjoint representation of a 6D twist vector.

- **Parameters**: `v` - A 6D twist vector [ωₓ, ωᵧ, ωᵤ, vₓ, vᵧ, vᵤ]
- **Returns**: The 6×6 adjoint matrix representation
- **Location**: `src/dynamics_of_open_chains.rs:5`

#### `inverse_dynamics(thetalist, dthetalist, ddthetalist, g, ftip, mlist, glist, slist) -> DVector<f64>`

Computes the joint torques required for given joint positions, velocities, and accelerations using the Newton-Euler inverse dynamics algorithm.

- **Parameters**: `thetalist` - Joint angles, `dthetalist` - Joint velocities, `ddthetalist` - Joint accelerations, `g` - Gravity vector, `ftip` - End-effector wrench, `mlist` - Link transformations, `glist` - Link inertia matrices, `slist` - Space screw axes
- **Returns**: Vector of required joint torques
- **Location**: `src/dynamics_of_open_chains.rs:16`

#### `mass_matrix(thetalist, mlist, glist, slist) -> DMatrix<f64>`

Computes the mass matrix (inertia matrix) of the robot at a given configuration.

- **Parameters**: `thetalist` - Joint angles, `mlist` - Link transformations, `glist` - Link inertia matrices, `slist` - Space screw axes
- **Returns**: The n×n mass matrix where n is the number of joints
- **Location**: `src/dynamics_of_open_chains.rs:87`

#### `vel_quadratic_forces(thetalist, dthetalist, mlist, glist, slist) -> DVector<f64>`

Computes the velocity-dependent forces (Coriolis and centrifugal forces) acting on the robot.

- **Parameters**: `thetalist` - Joint angles, `dthetalist` - Joint velocities, `mlist` - Link transformations, `glist` - Link inertia matrices, `slist` - Space screw axes
- **Returns**: Vector of velocity-dependent forces
- **Location**: `src/dynamics_of_open_chains.rs:119`

#### `gravity_forces(thetalist, g, mlist, glist, slist) -> DVector<f64>`

Computes the gravitational forces acting on the robot joints.

- **Parameters**: `thetalist` - Joint angles, `g` - Gravity vector, `mlist` - Link transformations, `glist` - Link inertia matrices, `slist` - Space screw axes
- **Returns**: Vector of gravitational forces
- **Location**: `src/dynamics_of_open_chains.rs:145`

#### `end_effector_forces(thetalist, ftip, mlist, glist, slist) -> DVector<f64>`

Computes the joint torques required to generate specified end-effector forces.

- **Parameters**: `thetalist` - Joint angles, `ftip` - End-effector wrench, `mlist` - Link transformations, `glist` - Link inertia matrices, `slist` - Space screw axes
- **Returns**: Vector of joint torques due to end-effector forces
- **Location**: `src/dynamics_of_open_chains.rs:171`

#### `forward_dynamics(thetalist, dthetalist, taulist, g, ftip, mlist, glist, slist) -> DVector<f64>`

Computes joint accelerations from applied joint torques using forward dynamics.

- **Parameters**: `thetalist` - Joint angles, `dthetalist` - Joint velocities, `taulist` - Applied joint torques, `g` - Gravity vector, `ftip` - End-effector wrench, `mlist` - Link transformations, `glist` - Link inertia matrices, `slist` - Space screw axes
- **Returns**: Vector of joint accelerations
- **Location**: `src/dynamics_of_open_chains.rs:197`

#### `euler_step(thetalist, dthetalist, ddthetalist, dt) -> (DVector<f64>, DVector<f64>)`

Performs one Euler integration step for trajectory simulation.

- **Parameters**: `thetalist` - Current joint angles, `dthetalist` - Current joint velocities, `ddthetalist` - Joint accelerations, `dt` - Time step
- **Returns**: Tuple of (next joint angles, next joint velocities)
- **Location**: `src/dynamics_of_open_chains.rs:225`

#### `inverse_dynamics_trajectory(thetamat, dthetamat, ddthetamat, g, ftipmat, mlist, glist, slist) -> Vec<DVector<f64>>`

Computes inverse dynamics for an entire trajectory of robot configurations.

- **Parameters**: `thetamat` - Trajectory of joint angles, `dthetamat` - Trajectory of joint velocities, `ddthetamat` - Trajectory of joint accelerations, `g` - Gravity vector, `ftipmat` - Trajectory of end-effector wrenches, `mlist` - Link transformations, `glist` - Link inertia matrices, `slist` - Space screw axes
- **Returns**: Vector of required joint torque trajectories
- **Location**: `src/dynamics_of_open_chains.rs:236`

#### `forward_dynamics_trajectory(thetalist, dthetalist, taumat, g, ftipmat, mlist, glist, slist, dt, int_res) -> (Vec<DVector<f64>>, Vec<DVector<f64>>)`

Simulates robot motion forward in time given torque inputs using forward dynamics.

- **Parameters**: `thetalist` - Initial joint angles, `dthetalist` - Initial joint velocities, `taumat` - Trajectory of applied torques, `g` - Gravity vector, `ftipmat` - Trajectory of end-effector wrenches, `mlist` - Link transformations, `glist` - Link inertia matrices, `slist` - Space screw axes, `dt` - Time step, `int_res` - Integration resolution
- **Returns**: Tuple of (joint angle trajectory, joint velocity trajectory)
- **Location**: `src/dynamics_of_open_chains.rs:271`

### Trajectory Generation (`trajectory_generation` module)

#### `cubic_time_scaling(tf: f64, t: f64) -> f64`

Generates smooth time scaling using a cubic polynomial that starts and ends at zero velocity.

- **Parameters**: `tf` - Total time duration, `t` - Current time
- **Returns**: Scaling factor s(t) between 0 and 1
- **Location**: `src/trajectory_generation.rs:1`

#### `quintic_time_scaling(tf: f64, t: f64) -> f64`

Generates smooth time scaling using a quintic polynomial that starts and ends with zero velocity and acceleration.

- **Parameters**: `tf` - Total time duration, `t` - Current time  
- **Returns**: Scaling factor s(t) between 0 and 1
- **Location**: `src/trajectory_generation.rs:11`

### Velocity Kinematics (`velocity_kinematics_and_statics` module)

#### `jacobian_body(blist: Vec<Vector6<f64>>, thetalist: Vec<f64>) -> Matrix6xX<f64>`

Computes the body Jacobian matrix.

- **Parameters**: `blist` - Body screw axes, `thetalist` - Joint angles
- **Returns**: The 6×n body Jacobian matrix
- **Location**: `src/velocity_kinematics_and_statics.rs:5`

#### `jacobian_space(slist: Vec<Vector6<f64>>, thetalist: Vec<f64>) -> Matrix6xX<f64>`

Computes the space Jacobian matrix.

- **Parameters**: `slist` - Space screw axes, `thetalist` - Joint angles
- **Returns**: The 6×n space Jacobian matrix
- **Location**: `src/velocity_kinematics_and_statics.rs:33`

## Project Structure

```
modern-robotics-rs/
├── .github/
│   └── workflows/
│       └── build-rust.yml                        # CI/CD workflow for automated builds and tests
├── src/
│   ├── lib.rs                                     # Main library interface
│   ├── utils.rs                                   # Core utility functions
│   ├── rigid_body_motions.rs                      # SO(3) and SE(3) operations
│   ├── forward_kinematics.rs                      # Forward kinematics calculations
│   ├── inverse_kinematics.rs                      # Inverse kinematics calculations
│   ├── velocity_kinematics_and_statics.rs         # Jacobian and velocity kinematics
│   ├── dynamics_of_open_chains.rs                 # Robot dynamics calculations
│   └── trajectory_generation.rs                   # Time scaling for trajectory generation
├── tests/
│   ├── test_utils.rs                              # Tests for utility functions
│   ├── test_rigid_body_motions.rs                 # Tests for rigid body motion functions
│   ├── test_forward_kinematics.rs                 # Tests for forward kinematics
│   ├── test_inverse_kinematics.rs                 # Tests for inverse kinematics
│   ├── test_velocity_kinematics_and_statics.rs    # Tests for velocity kinematics
│   ├── test_dynamics_of_open_chains.rs            # Tests for dynamics calculations
│   └── test_trajectory_generation.rs              # Tests for trajectory generation
├── target/                                        # Build artifacts (generated)
├── Cargo.toml                                     # Package configuration
├── Cargo.lock                                     # Dependency lock file
├── CLAUDE.md                                      # Claude Code guidance
└── README.md                                      # This file
```

## Testing

The project includes comprehensive tests covering:

- Near-zero tolerance checking with various input values
- Vector normalization for multi-dimensional vectors
- SO(3) operations: rotations, matrix exponentials, logarithms
- SE(3) operations: transformations, inversions, adjoint representations
- Skew-symmetric matrix conversions for both so(3) and se(3)
- Screw theory operations
- Forward kinematics for both body and space frames
- Inverse kinematics using Newton-Raphson method
- Jacobian calculations for velocity kinematics
- Robot dynamics including inverse dynamics and mass matrix computation
- Trajectory generation with time scaling functions
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

#### Forward Kinematics Tests (`test_forward_kinematics.rs`)
- **`test_fkin_body`**: Tests body frame forward kinematics calculations
- **`test_fkin_space`**: Tests space frame forward kinematics calculations

#### Inverse Kinematics Tests (`test_inverse_kinematics.rs`)
- **`test_ikin_body`**: Tests body frame inverse kinematics using Newton-Raphson method
- **`test_ikin_space`**: Tests space frame inverse kinematics using Newton-Raphson method

#### Velocity Kinematics Tests (`test_velocity_kinematics_and_statics.rs`)
- **`test_jacobian_body`**: Tests body Jacobian matrix computation
- **`test_jacobian_space`**: Tests space Jacobian matrix computation

#### Dynamics of Open Chains Tests (`test_dynamics_of_open_chains.rs`)
- **`test_ad`**: Tests adjoint representation computation for 6D twist vectors
- **`test_inverse_dynamics`**: Tests inverse dynamics algorithm for computing joint torques
- **`test_mass_matrix`**: Tests mass matrix computation for robot configurations
- **`test_vel_quadratic_forces`**: Tests velocity-dependent forces (Coriolis and centrifugal) computation
- **`test_gravity_forces`**: Tests gravitational forces computation
- **`test_end_effector_forces`**: Tests end-effector force mapping to joint torques
- **`test_forward_dynamics`**: Tests forward dynamics for computing joint accelerations
- **`test_euler_step`**: Tests Euler integration step for trajectory simulation
- **`test_inverse_dynamics_trajectory`**: Tests inverse dynamics for trajectory computation
- **`test_forward_dynamics_trajectory`**: Tests forward dynamics trajectory simulation

#### Trajectory Generation Tests (`test_trajectory_generation.rs`)
- **`test_cubic_time_scaling`**: Tests cubic polynomial time scaling for smooth trajectory generation
- **`test_qintic_time_scaling`**: Tests quintic polynomial time scaling for smooth trajectory generation with zero acceleration endpoints

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

- **v0.1.2**: Current version with expanded dynamics module including forward dynamics, gravity forces, trajectory simulation, and comprehensive robot dynamics algorithms
- **v0.1.1**: Previous version with basic vector operations and tolerance checking

## License

This project follows standard Rust packaging conventions. Check the repository for specific license information.

## Contributing

This library serves as a foundation for robotics applications in Rust. Contributions that expand the mathematical utilities while maintaining performance and correctness are welcome.

## Future Enhancements

Potential areas for expansion include:
- Quaternion operations and conversions
- Advanced trajectory planning algorithms
- Additional Lie group operations
- Path planning utilities
- Support for different robot configurations (parallel robots, mobile robots)
- Integration with robot simulation frameworks
- Optimization-based inverse kinematics solvers
- Robot control algorithms and closed-loop dynamics
- Support for flexible joints and link compliance
- Multi-body system dynamics