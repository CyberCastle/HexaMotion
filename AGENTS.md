# AGENT Instructions

This file defines the guidelines for contributing to HexaMotion.

## Objective

This library provides locomotion control for a hexapod robot based on the Arduino Giga R1. The robot body forms a hexagon with legs spaced 60° apart, each leg having three joints for 3DOF. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling. The interfaces `IIMUInterface`, `IFSRInterface` and `IServoInterface` must be implemented to connect the IMU, FSR sensors and smart servos.

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

## Physical characteristics of the robot

These are the characteristics of a real robot, used to test this library.

-   robot height: 100 mm
-   robot weight: 6.5 Kg
-   body hexagon radius: 200 mm.
-   coxa length: 50 mm
-   coxa weight: 54 g
-   femur length: 101 mm
-   femur weight: 150 g
-   tibia length: 208 mm
-   tibia weight: 200 g

**Note:** Physically, the robot, with the angles coxa-femur = 0º and femur-tibia = 0º, the femur remains horizontal, while the tibia remains vertical, standing stably.

### Test Parameters

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
