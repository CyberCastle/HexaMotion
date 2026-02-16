# AGENT Instructions

This file defines the guidelines for contributing to HexaMotion.

## Objective

HexaMotion is a 1:1 port of OpenSHC without ROS support. It brings OpenSHC's locomotion logic to MCU targets such as the STM32H7 (Arduino Giga R1) for a hexapod robot with a hexagonal body, six legs spaced 60 degrees apart, and three joints per leg. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling. The interfaces `IIMUInterface`, `IFSRInterface` and `IServoInterface` must be implemented to connect the IMU, FSR sensors and smart servos. HexaMotion does NOT support ROS natively.

Key differences from OpenSHC:

- Supports only 3DOF per leg.
- Supports only six legs.
- AMBLE_GAIT is not supported with current morphology/constraints.
- `StateController` must preserve OpenSHC's functional orchestration 1:1 (state transitions, running loop sequencing, gait/pose/manual leg coordination), excluding ROS transport details.
- `LocomotionSystem` acts as a ROS-less wrapper/facade around `StateController` for integration: it replaces the external ROS script/graph role, routes external inputs into `StateController`, and executes low-level hardware/control pipeline steps (sensors, walk update, IK, servo output).
- `LocomotionSystem` should expose predefined high-level robot actions (e.g., forward, backward, turn left/right, stop) as convenience APIs; these are equivalent to common ROS command patterns but are provided directly as library methods.
- Planner mode is intentionally not ported to HexaMotion; equivalent planning behavior should be implemented by an external software component using the API exposed by `LocomotionSystem`.
- Conceptually, OpenSHC's external ROS graph/script that reads subscriptions and writes publishers is replaced by `LocomotionSystem` + direct API calls in HexaMotion.
- No YAML configuration files; everything is configured through the `Parameters` structure.
- OpenSHC logic is split into specific classes so the code is more readable and maintainable; the current HexaMotion organization follows this.
- Class/data structures and naming (classes, constants, globals, locals) follow a semantic, self-documenting pattern, so some names differ from OpenSHC while keeping 1:1 logic.
- Includes tests to verify hexapod kinematics and dynamics logic.
- Certain configurations are handled via factory patterns.
- No dynamic configuration support.

## Code Style

- Use C++11.
- Four-space indentation with no tabs.
- Place the opening brace on the same line as the declaration.
- Document public functions using Doxygen-style comments (`/** ... */`) and in English.

## Development

- Don't create arduino examples.
- Clone the repository with all submodules.
- Implementation files live exclusively in the `src` and `include` directories.
- The `tests` folder only contains code for validating fixes.
- The `OpenSHC` directory holds the reference code used as a base for HexaMotion.
- When implementing or modifying functionality, review the `OpenSHC` folder first so the implementation remains equivalent.
- To test changes, run the tests inside the `tests` folder.

## Testing

- Run the unit tests before submitting changes.
- Install the Eigen dependency by running `tests/setup.sh` if needed.
- Build the tests with `make` inside the `tests` directory.

```bash
cd tests
./setup.sh
make
```

### Test execution policy

- Run only the tests related to the components you changed.
- Do not run the full suite for every small change; prefer focused validation first.
- To run the complete suite, use only:

```bash
cd tests
bash run_all_tests.sh
```

### Runtime expectations

- Some integration/stress tests can take more than 7 minutes to finish, depending on machine performance.
- Longer tests usually include: `walk_controller_test`, `tripod_walk_visualization_test`, `virtual_hardware_sim_test`, and `hexapod_trajectory_analysis_test`.

### Test coverage matrix

| Test                                 | What it covers                                        | Main software pieces                                                                                                                                        |
| ------------------------------------ | ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `math_utils_test`                    | Math helpers and numeric primitives                   | `math_utils`                                                                                                                                                |
| `quaternion_functions_test`          | Quaternion utility behavior                           | `math_utils`                                                                                                                                                |
| `simple_dh_test`                     | Basic DH forward kinematics sanity                    | `robot_model`, `math_utils`, `workspace_analyzer`                                                                                                           |
| `simple_ik_test`                     | Basic inverse kinematics sanity                       | `robot_model`, `math_utils`, `workspace_analyzer`                                                                                                           |
| `simple_advanced_ik_test`            | Advanced IK scenarios with leg-level checks           | `robot_model`, `leg`, `math_utils`, `workspace_analyzer`                                                                                                    |
| `kinematics_validation_test`         | FK/IK consistency and kinematic validation            | `robot_model`, `math_utils`, `workspace_analyzer`, `body_pose_config_factory`                                                                               |
| `jacobian_validation_test`           | Jacobian validation and local differential behavior   | `robot_model`, `math_utils`, `workspace_analyzer`                                                                                                           |
| `dh_vs_analytic_test`                | DH model equivalence against analytic model           | `robot_model`, `analytic_robot_model`, `math_utils`, `workspace_analyzer`                                                                                   |
| `finetune_angles_test`               | Angle fine-tuning and kinematic calibration flow      | `robot_model`, `analytic_robot_model`, `math_utils`, `body_pose_config_factory`, `workspace_analyzer`                                                       |
| `brute_force_workspace_test`         | Reachability/workspace brute-force validation         | `robot_model`, `math_utils`, `workspace_analyzer`                                                                                                           |
| `workspace_analyzer_fusion_test`     | Workspace analysis integrated with locomotion stack   | `workspace_analyzer`, `walk_controller`, `locomotion_system`, `state_controller`, `velocity_limits`                                                         |
| `pose_controller_test`               | Body pose control behavior and leg posing integration | `body_pose_controller`, `robot_model`, `body_pose_config_factory`, `leg`, `leg_poser`, `workspace_analyzer`                                                 |
| `pose_gait_integration_test`         | Pose + gait end-to-end integration                    | `locomotion_system`, `state_controller`, `walk_controller`, `body_pose_controller`, `leg_stepper`, `cartesian_velocity_controller`, `admittance_controller` |
| `walk_controller_test`               | Core gait update/orchestration logic                  | `walk_controller`, `leg_stepper`, `terrain_adaptation`, `velocity_limits`, `gait_config_factory`, `body_pose_controller`                                    |
| `trajectory_tip_position_test`       | Single-foot tip trajectory correctness                | `walk_controller`, `leg_stepper`, `robot_model`, `velocity_limits`, `terrain_adaptation`                                                                    |
| `trajectory_all_legs_test`           | Multi-leg synchronized trajectory generation          | `walk_controller`, `leg_stepper`, `robot_model`, `velocity_limits`, `terrain_adaptation`                                                                    |
| `hexapod_trajectory_analysis_test`   | Global hexapod trajectory constraints and continuity  | `walk_controller`, `leg_stepper`, `robot_model`, `velocity_limits`, `gait_config_factory`                                                                   |
| `tripod_walk_visualization_test`     | Tripod gait progression in full control stack         | `walk_controller`, `leg_stepper`, `state_controller`, `locomotion_system`, `cartesian_velocity_controller`, `admittance_controller`                         |
| `tripod_linearity_test`              | Tripod gait linearity and path consistency            | `walk_controller`, `leg_stepper`, `state_controller`, `locomotion_system`                                                                                   |
| `virtual_hardware_sim_test`          | Virtual hardware loop and full-stack integration      | `locomotion_system`, `state_controller`, `walk_controller`, `admittance_controller`, `cartesian_velocity_controller`, `velocity_limits`                     |
| `bezier_validation_test`             | Bezier support checks and workspace compatibility     | `robot_model`, `math_utils`, `workspace_analyzer`                                                                                                           |
| `bezier_transition_single_leg_test`  | Single-leg Bezier transition continuity               | `walk_controller`, `leg_stepper`, `robot_model`, `state_controller`, `locomotion_system`, `velocity_limits`                                                 |
| `bezier_transition_all_legs_test`    | All-legs Bezier transition coherence                  | `walk_controller`, `leg_stepper`, `robot_model`, `state_controller`, `locomotion_system`, `velocity_limits`                                                 |
| `coxa_phase_transition_test`         | Coxa phase transitions across gait cycle              | `walk_controller`, `leg_stepper`, `gait_config_factory`, `state_controller`, `locomotion_system`                                                            |
| `coxa_stride_decomposition_test`     | Coxa stride decomposition and component isolation     | `walk_controller`, `leg_stepper`, `gait_config_factory`, `state_controller`, `locomotion_system`                                                            |
| `stride_vector_validation_test`      | Stride vector generation and normalization            | `leg_stepper`, `velocity_limits`, `gait_config_factory`, `robot_model`, `body_pose_controller`                                                              |
| `swing_coxa_orientation_test`        | Coxa orientation during swing phase                   | `leg_stepper`, `gait_config_factory`, `velocity_limits`, `robot_model`                                                                                      |
| `coxa_tripod_symmetry_analytic_test` | Analytic symmetry of opposite tripod coxa behavior    | `leg_stepper`, `walk_controller`, `terrain_adaptation`, `state_controller`, `locomotion_system`, `cartesian_velocity_controller`                            |
| `step_frequency_regeneration_test`   | Step frequency regeneration from gait parameters      | `gait_config_factory`, `robot_model`, `math_utils`, `workspace_analyzer`                                                                                    |
| `ik_tracking_diagnostic_test`        | IK tracking diagnostics under gait progression        | `leg_stepper`, `walk_controller`, `robot_model`, `state_controller`, `locomotion_system`, `velocity_limits`                                                 |

## Physical characteristics of the robot

These are the characteristics of a real robot, used to test this library.

- robot height: 208 mm (with all angles in local position equals to 0º)
- default standing height 150 mm
- robot weight: 6.5 Kg
- body hexagon radius: 200 mm
- coxa length: 50 mm
- coxa weight: 54 g
- femur length: 101 mm
- femur weight: 150 g
- tibia length: 208 mm
- tibia weight: 200 g

**Note:** Physically, if the robot has all servo angles at 0°, the femur remains horizontal, in line with the coxa. The tibia, on the other hand, remains vertical, perpendicular to the ground. This allows the robot to stand stably by default when all angles are at 0°. Due to the aforementioned peculiarity, the robot's body will be positioned at z = -208, this value being the length of the tibia equal to the default height.

**Default Height Configuration:** The library now supports configurable default height through the `default_height_offset` parameter. This parameter defines the Z-axis offset when all servo angles are at 0°. If set to 0.0 (default), the system falls back to using `-tibia_length` for backward compatibility. For explicit configuration, set it to `-208.0` (or `-tibia_length`) to match the physical robot characteristics.

### Leg base orientation

Current internal DH/base orientation offsets (degrees) come directly from `BASE_THETA_OFFSETS` in `hexamotion_constants.h` and match OpenSHC's default DH base link `theta` values. They are ordered by internal leg index / mnemonic:

| Index | Name | Meaning        | Angle (°) |
| ----- | ---- | -------------- | --------- |
| 0     | AR   | Anterior Right | -30       |
| 1     | BR   | Back Right     | -90       |
| 2     | CR   | Center Right   | -150      |
| 3     | CL   | Center Left    | +150      |
| 4     | BL   | Back Left      | +90       |
| 5     | AL   | Anterior Left  | +30       |

Forward still points along +X. External tools assuming leg0 at 0° should account for the global -30° rotation.

Opposite leg pairs now cancel their base offsets: (AR + AL) = 0°, (BR + BL) = 0° and (CR + CL) = 0°. This matches
OpenSHC's DH conventions and keeps tripod symmetry expectations intact after the correction.

### Velocity and acceleration units

All internal kinematic/dynamic calculations use millimeters (mm), millimeters per second (mm/s) and millimeters per second squared (mm/s²). Convert to SI only at integration boundaries if required.

### Stance and workspace radii

Stance and walkspace radii are now derived from the kinematics of the configured standing pose, not from the theoretical flat extension (femur + tibia) nor the naïve sum (coxa + femur + tibia).

Standing pose definition:

- Tibia is vertical (femur_angle + tibia_angle = 0)
- Body height = `standing_height`

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

- `BodyPoseConfiguration::standing_horizontal_reach` stores this value (computed from the configured standing pose joints).
- Fallback stance initialization and walkspace generation now use this standing horizontal reach directly (no additional 0.65 scaling) because it is already conservative relative to maximum flat extension.
- Previous documentation claiming (coxa + femur + tibia = 359 mm) overstated usable horizontal reach; tibia length is mostly vertical in standing pose and should not be added for horizontal radius computation.

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

- Use imperative mood in English. Example: "Add new gait option".
- Keep the summary under 72 characters.

## Pull Requests

- Include a summary of the changes made.
- Mention any known limitations or additional steps.
