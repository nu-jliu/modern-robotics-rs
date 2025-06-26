# Modern Robotics - Rust Implementation

**Author**: Jingkun Liu

![build](https://github.com/nu-jliu/modern-robotics-rs/actions/workflows/build-rust.yml/badge.svg)

A Rust implementation of fundamental robotics algorithms and utilities based on modern robotics principles.

## Overview

This library provides essential mathematical utilities commonly used in robotics applications, including vector normalization and numerical tolerance checking. The implementation leverages the `nalgebra` linear algebra library for efficient mathematical operations.

## Features

- **Numerical Tolerance Checking**: Check if values are near zero within a specified tolerance
- **Vector Normalization**: Normalize vectors to unit length using nalgebra's DVector
- **Robust Testing**: Comprehensive test suite covering various dimensional vectors

## Dependencies

- `nalgebra` (v0.33.2) - Linear algebra library for Rust

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
modern-robotics = "0.1.1"
```

## Usage

```rust
use modern_robotics::modern_robotics::{near_zero, normalize};
use nalgebra::DVector;

// Check if a value is near zero
let is_zero = near_zero(1e-7); // true
let not_zero = near_zero(1e-5); // false

// Normalize a vector
let v = DVector::from_vec(vec![3.0, 4.0]);
let normalized = normalize(v);
// Result: [0.6, 0.8] (unit vector)
```

## API Reference

### Functions

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

## Project Structure

```
modern-robotics-rs/
├── src/
│   ├── lib.rs          # Main library interface
│   └── utils.rs        # Core utility functions
├── tests/
│   └── test_utils.rs   # Comprehensive test suite
├── scripts/
│   └── rustup.sh       # Rust toolchain installation script
├── Cargo.toml          # Package configuration
├── Cargo.lock          # Dependency lock file
└── README.md           # This file
```

## Testing

The project includes comprehensive tests covering:

- Near-zero tolerance checking with various input values
- Vector normalization for 2D, 3D, and 4D vectors
- Magnitude verification for normalized vectors
- Edge cases and numerical precision

Run tests with:

```bash
cargo test
```

### Test Coverage

- **`test_near_zero`**: Validates tolerance checking for values like 1e-7, 0.0, 1e-5, and 1.0
- **`test_normalize_3d`**: Tests normalization of 3D vectors (e.g., [1,1,1] → [0.577, 0.577, 0.577])
- **`test_normalize_2d`**: Tests normalization of 2D vectors (e.g., [3,4] → [0.6, 0.8])
- **`test_normalize_4d`**: Tests normalization of higher-dimensional vectors
- **`test_normalize_magnitude`**: Verifies that normalized vectors have unit magnitude

## Constants

- **`TOLERANCE`**: 1e-6 - The threshold used for near-zero comparisons

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
- Rotation matrices and quaternion operations
- Homogeneous transformations
- Kinematics and dynamics calculations
- Path planning utilities
- Additional numerical methods for robotics