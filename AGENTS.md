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

-   Don't create arduino examples.
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

-   robot height: 208 mm (with all angles in local position equals to 0º)
-   default standing height 150 mm
-   robot weight: 6.5 Kg
-   body hexagon radius: 200 mm
-   coxa length: 50 mm
-   coxa weight: 54 g
-   femur length: 101 mm
-   femur weight: 150 g
-   tibia length: 208 mm
-   tibia weight: 200 g

**Note:** Physically, if the robot has all servo angles at 0°, the femur remains horizontal, in line with the coxa. The tibia, on the other hand, remains vertical, perpendicular to the ground. This allows the robot to stand stably by default when all angles are at 0°. Due to the aforementioned peculiarity, the robot's body will be positioned at z = -208, this value being the length of the tibia equal to the default height.

**Default Height Configuration:** The library now supports configurable default height through the `default_height_offset` parameter. This parameter defines the Z-axis offset when all servo angles are at 0°. If set to 0.0 (default), the system falls back to using `-tibia_length` for backward compatibility. For explicit configuration, set it to `-208.0` (or `-tibia_length`) to match the physical robot characteristics.

### Leg base orientation

The internal DH/base orientation offsets are (degrees): leg0 = -30, leg1 = -90, leg2 = -150, leg3 = +150, leg4 = +90, leg5 = +30. This preserves 60° separation while rotating the frame so opposing pairs are symmetric ((0,3), (1,4), (2,5)). Forward remains +X; external tools assuming leg0 at 0° should account for this -30° global rotation.

### Velocity and acceleration units

All internal kinematic/dynamic calculations use millimeters (mm), millimeters per second (mm/s) and millimeters per second squared (mm/s²). Convert to SI only at integration boundaries if required.

### Stance and workspace radii

Stance and walkspace radii are now derived from the kinematics of the configured standing pose, not from the theoretical flat extension (femur + tibia) nor the naïve sum (coxa + femur + tibia).

Standing pose definition:

-   Tibia is vertical (femur_angle + tibia_angle = 0)
-   Body height = `standing_height`

Given femur_length = 101 mm, tibia_length = 208 mm, standing_height = 150 mm:

```
target_z = -standing_height
target_z = -femur_length * sin(femur_angle) - tibia_length
sin(femur_angle) = -(target_z + tibia_length) / femur_length
```

Solving yields femur_angle ≈ -35° and horizontal femur projection = femur_length \* cos(femur_angle) ≈ 82.8 mm.
The tibia contributes no horizontal reach in this pose (vertical), so horizontal reach from the coxa pivot is:

```
standing_horizontal_reach = coxa_length + horizontal_femur ≈ 50 + 82.8 = 132.8 mm
```

Thus the default stance radial distance from body center (hexagon radius + standing horizontal reach) is about:

```
hexagon_radius + standing_horizontal_reach ≈ 200 + 132.8 = 332.8 mm
```

Implementation details:

-   `BodyPoseConfiguration::standing_horizontal_reach` stores this value (computed from the configured standing pose joints).
-   Fallback stance initialization and walkspace generation now use this standing horizontal reach directly (no additional 0.65 scaling) because it is already conservative relative to maximum flat extension.
-   Previous documentation claiming (coxa + femur + tibia = 359 mm) overstated usable horizontal reach; tibia length is mostly vertical in standing pose and should not be added for horizontal radius computation.

This change ensures gait planning, stride limits, and circular trajectory validation remain consistent with the maintained body height, preventing overestimation that could command unreachable lateral positions at higher angular velocities.

Symmetry requirement:
All calculations (standing horizontal reach, stance positions, walkspace radii and gait phase assumptions) currently assume a symmetric standing pose across opposing leg pairs (0↔3, 1↔4, 2↔5) with identical joint solutions. Per‑leg asymmetric standing poses or individualized reach scaling are NOT supported at this time. Providing asymmetric joint angles will produce undefined or inconsistent workspace/velocity limits.

### Test Parameters

Use the following `Parameters` configuration in the test files:

```cpp
Parameters p{};
p.hexagon_radius = 200;
p.coxa_length = 50;
p.femur_length = 101;
p.tibia_length = 208;
p.default_height_offset = -208.0;  // Set to -tibia_length for explicit configuration
p.robot_height = 208;
p.standing_height = 150;
p.time_delta = 1.0 / 50.0; // 50 Hz
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
