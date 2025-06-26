# AGENT Instructions

This file defines the guidelines for contributing to HexaMotion.

## Objective

This library provides locomotion control for a hexapod robot based on the Arduino Giga R1. The robot body forms a hexagon with legs spaced 60° apart, each leg having three joints for 3DOF. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling. The interfaces `IIMUInterface`, `IFSRInterface` and `IServoInterface` must be implemented to connect the IMU, FSR sensors and smart servos.

**New Feature**: HexaMotion now includes **smooth trajectory interpolation** that uses current servo positions as starting points for pose calculations, equivalent to OpenSHC's smooth movement approach. This is the default behavior and provides natural, smooth robot movements that optimize from the current state rather than theoretical positions.

**Recent Updates**:

-   **Enhanced Kinematic Accuracy**: Fixed pose calculations to achieve correct angles (coxa ≈ 0°, femur ≈ -19°, tibia ≈ 0°) for 100mm height using analytical planar geometry instead of fixed extension factors.
-   **Configurable Joint Sign Control**: Added independent sign multipliers (`angle_sign_coxa`, `angle_sign_femur`, `angle_sign_tibia`) to correct joint angle directions without affecting kinematic calculations.
-   **Code Optimization**: Consolidated redundant pose calculation code using helper functions to improve maintainability.

## Key Features

-   **Smooth Movement System**: Uses current servo positions as trajectory starting points (OpenSHC-style)
-   **Configurable Interpolation**: Adjustable speed and precision for different use cases
-   **Backward Compatibility**: Original immediate movement behavior available when needed
-   **Automatic Optimization**: Movements are inherently smooth and natural by default
-   **Accurate Kinematics**: Corrected pose calculations using proper analytical geometry for expected joint angles
-   **Hardware Adaptation**: Independent joint angle sign control for different servo orientations

## Code Style

-   Use C++17.
-   Four-space indentation with no tabs.
-   Place the opening brace on the same line as the declaration.
-   Document public functions using Doxygen-style comments (`/** ... */`) and in English.

## Development

-   Clone the repository with all submodules.
-   Implementation files live exclusively in the `src` and `include` directories.
-   The `tests` folder only contains code for validating fixes.
-   The `OpenSHC` directory holds the reference code used as a base for HexaMotion.
-   When implementing or modifying functionality, review the `OpenSHC` folder first so the implementation remains equivalent.
-   To test changes, run the tests inside the `tests` folder.

## Testing

-   Run the unit tests before submitting changes.
-   Install the Eigen dependency by running `tests/setup.sh` if needed.
-   Build the tests with `make` inside the `tests` directory.

```bash
cd tests
./setup.sh
make
```

Each test executable can be run individually.

### Test Parameters

robot height: 100 mm
robot weight: 6.5 Kg
body hexagon radius: 200 mm.
coxa length: 50 mm
coxa weight: 54 g
femur length: 101 mm
femur weight: 150 g
tibia length: 208 mm
tibia weight: 200 g

Use the following `Parameters` configuration in the test files:

```cpp
Parameters p{};
p.hexagon_radius = 200;
p.coxa_length = 50;
p.femur_length = 101;
p.tibia_length = 208;
p.robot_height = 100;
p.control_frequency = 50;
p.coxa_angle_limits[0] = -65;
p.coxa_angle_limits[1] = 65;
p.femur_angle_limits[0] = -75;
p.femur_angle_limits[1] = 75;
p.tibia_angle_limits[0] = -45;
p.tibia_angle_limits[1] = 45;
```

## Commit Messages

-   Use imperative mood in English. Example: "Add new gait option".
-   Keep the summary under 72 characters.

## Pull Requests

-   Include a summary of the changes made.
-   Mention any known limitations or additional steps.
