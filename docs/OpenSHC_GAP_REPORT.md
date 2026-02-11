# OpenSHC Parity Gap Report (HexaMotion)

> **Last updated**: 2026-02-10 — Full method-level audit against OpenSHC source.
> **Validation pass**: 2026-02-10 — Cross-verified all OpenSHC symbols (14 enums, 6 structs, 14 classes, ~290 public methods, ~200 member variables) against HexaMotion (24 classes, 3 interfaces, ~20 enums, ~50 structs, ~500+ public methods).

## Context (from AGENTS.md)

HexaMotion is a 1:1 port of OpenSHC without ROS support. The goal is to run OpenSHC's locomotion logic on MCU targets such as the STM32H7 (Arduino Giga R1) for a hexapod robot with a hexagonal body, six legs spaced 60 degrees apart, and three joints per leg. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling. The interfaces `IIMUInterface`, `IFSRInterface` and `IServoInterface` must be implemented to connect the IMU, FSR sensors and smart servos. HexaMotion does NOT support ROS natively.

Key differences from OpenSHC:

- **Orchestration**: `LocomotionSystem` orchestrates control classes (`WalkController`, `BodyPoseController`, etc.), analogous to the ROS-side orchestration done by OpenSHC's `StateController`.
- **State Machine**: HexaMotion's `StateController` focuses on FSM logic; OpenSHC's `StateController` handled FSM logic plus ROS orchestration, publishers/subscribers, and param dynamics.
- **Model Structure**: `RobotModel` in HexaMotion is primarily a kinematics/parameters provider and does not own `Leg` objects directly. `Leg` objects are owned by `LocomotionSystem`.
- **Configuration**: No YAML; everything is configured through the `Parameters` structure and factories.
- **ROS**: Completely removed. No `ros::Publisher`, `ros::Subscriber`, or `tf` dependencies. Debugging hooks (Visualiser) are stripped or replaced with telemetry.
- **Constraints**: Supports only 3DOF per leg and only six legs.

Development and testing constraints:

- Use C++11, 4-space indentation, brace on same line, and Doxygen-style docs for public functions.
- Implementation files live exclusively in `src` and `include`. Do not add Arduino examples.
- Review `OpenSHC` first when modifying functionality to keep logic equivalent.
- Run tests with `tests/setup.sh` (Eigen install) and `make`.

Physical parameters and conventions:

- Default robot dimensions: hexagon radius 200 mm; coxa 50 mm; femur 101 mm; tibia 208 mm; robot height 208 mm; standing height 150 mm.
- All internal kinematics use mm, mm/s, mm/s^2.
- Leg base orientation offsets are defined in `BASE_THETA_OFFSETS` (AR +30°, BR +90°, CR +150°, CL -150°, BL -90°, AL -30°).
- Default height is configured through `default_height_offset` (0.0 uses `-tibia_length`, explicit `-208.0` recommended for physical robot).
- Stance/walkspace radii are derived from standing pose horizontal reach (coxa + femur projection), not `coxa + femur + tibia`.
- Symmetry requirement: standing pose and gait assumptions require opposing leg pairs to be symmetric.

---

## Gap Checklist by Module

### 1. parameters_and_states.h (OpenSHC) vs HexaMotion enums/config

**Status**: Near Full Parity (renamed, restructured)

#### Enums — Detailed Mapping

| OpenSHC Enum         | OpenSHC Values                                                                                             | HexaMotion Equivalent                                 | HexaMotion Values                                                                                                                          | Status                                                                                                                                                                                  |
| :------------------- | :--------------------------------------------------------------------------------------------------------- | :---------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SystemState`        | `SUSPENDED`, `OPERATIONAL`                                                                                 | Merged into `SystemState` in `hexamotion_constants.h` | `SYSTEM_UNKNOWN`, `SYSTEM_PACKED`, `SYSTEM_READY`, `SYSTEM_RUNNING`                                                                        | **Redesigned**: OpenSHC's two-state toggle replaced by four-state lifecycle. SUSPENDED/OPERATIONAL semantics handled by `system_enabled` flag in `LocomotionSystem`.                    |
| `RobotState`         | `PACKED`, `READY`, `RUNNING`, `UNKNOWN=-1`, `OFF=-2`                                                       | `RobotState` in `state_controller.h`                  | `ROBOT_PACKED`, `ROBOT_READY`, `ROBOT_RUNNING`, `ROBOT_UNKNOWN=-1`, `ROBOT_OFF=-2`                                                         | ✅ 1:1 (renamed)                                                                                                                                                                        |
| `GaitDesignation`    | `WAVE_GAIT(0)`, `AMBLE_GAIT(1)`, `RIPPLE_GAIT(2)`, `TRIPOD_GAIT(3)`, `GAIT_UNDESIGNATED(-1)`               | `GaitType` in `gait_types.h`                          | `NO_GAIT(0)`, `TRIPOD_GAIT(1)`, `WAVE_GAIT(2)`, `RIPPLE_GAIT(3)`, `METACHRONAL_GAIT(4)`, `ADAPTIVE_GAIT(5)`                                | `AMBLE_GAIT` not supported. `NO_GAIT` replaces `GAIT_UNDESIGNATED` at value 0 (was -1). **Ordinal values differ.** `METACHRONAL_GAIT` and `ADAPTIVE_GAIT` are HexaMotion extensions.    |
| `PosingMode`         | `NO_POSING`, `X_Y_POSING`, `PITCH_ROLL_POSING`, `Z_YAW_POSING`, `EXTERNAL_POSING`                          | `PosingMode` in `state_controller.h`                  | `POSING_NONE`, `POSING_X_Y`, `POSING_PITCH_ROLL`, `POSING_Z_YAW`, `POSING_EXTERNAL`                                                        | ✅ 1:1 (renamed)                                                                                                                                                                        |
| `CruiseControlMode`  | `CRUISE_CONTROL_OFF`, `CRUISE_CONTROL_ON`, `CRUISE_CONTROL_EXTERNAL=-1`                                    | `CruiseControlMode` in `state_controller.h`           | Same names                                                                                                                                 | ✅ 1:1                                                                                                                                                                                  |
| `PlannerMode`        | `PLANNER_MODE_OFF`, `PLANNER_MODE_ON`                                                                      | `PlannerMode` in `state_controller.h`                 | `PLANNER_MODE_OFF`, `PLANNER_MODE_ON`                                                                                                      | ✅ Stubbed (returns not supported on MCU).                                                                                                                                              |
| `LegState`           | `WALKING`, `MANUAL`, `WALKING_TO_MANUAL=-1`, `MANUAL_TO_WALKING=-2`                                        | `LegState` in `gait_types.h`                          | `LEG_WALKING`, `LEG_MANUAL`, `LEG_WALKING_TO_MANUAL=-1`, `LEG_MANUAL_TO_WALKING=-2`                                                        | ✅ 1:1 (renamed)                                                                                                                                                                        |
| `WalkState`          | `STARTING`, `MOVING`, `STOPPING`, `STOPPED`                                                                | `WalkState` in `gait_types.h`                         | `WALK_STARTING`, `WALK_MOVING`, `WALK_STOPPING`, `WALK_STOPPED`                                                                            | ✅ 1:1 (renamed)                                                                                                                                                                        |
| `StepState`          | `SWING`, `STANCE`, `FORCE_STANCE`, `FORCE_STOP`                                                            | `StepState` in `leg_stepper.h`                        | `STEP_SWING`, `STEP_STANCE`, `STEP_FORCE_STANCE`, `STEP_FORCE_STOP`                                                                        | ✅ 1:1 (renamed). Note: lives in `leg_stepper.h`, not `gait_types.h`. HexaMotion also has a parallel `StepPhase` enum (`STANCE_PHASE`, `SWING_PHASE`) in `robot_model.h` used by `Leg`. |
| `PosingState`        | `POSING`, `STOP_POSING`, `POSING_COMPLETE`                                                                 | `PosingState` in `state_controller.h`                 | `POSE_POSING`, `POSE_STOP_POSING`, `POSE_POSING_COMPLETE`                                                                                  | ✅ 1:1 (renamed)                                                                                                                                                                        |
| `PoseResetMode`      | `NO_RESET`, `Z_AND_YAW_RESET`, `X_AND_Y_RESET`, `PITCH_AND_ROLL_RESET`, `ALL_RESET`, `IMMEDIATE_ALL_RESET` | `PoseResetMode` in `state_controller.h`               | `POSE_RESET_NONE`, `POSE_RESET_Z_AND_YAW`, `POSE_RESET_X_AND_Y`, `POSE_RESET_PITCH_AND_ROLL`, `POSE_RESET_ALL`, `POSE_RESET_IMMEDIATE_ALL` | ✅ 1:1 (renamed)                                                                                                                                                                        |
| `LegDesignation`     | `LEG_0`–`LEG_7`, `LEG_UNDESIGNATED=-1`                                                                     | `LegDesignation` in `hexamotion_constants.h`          | `LEG_0`–`LEG_5`, `LEG_UNDESIGNATED=-1`                                                                                                     | ✅ Reduced to 6 (by design).                                                                                                                                                            |
| `ParameterSelection` | 10 values (step_freq, swing_height, etc.)                                                                  | —                                                     | —                                                                                                                                          | **Partial**: `LocomotionSystem::setParameter()` supports a limited subset.                                                                                                              |
| `SequenceSelection`  | `START_UP`, `SHUT_DOWN`                                                                                    | `SequenceType` in `state_controller.h`                | `SEQUENCE_START_UP`, `SEQUENCE_SHUT_DOWN`, `SEQUENCE_PACK`, `SEQUENCE_UNPACK`                                                              | ✅ Extended (pack/unpack added).                                                                                                                                                        |

#### Structs — Detailed Mapping

| OpenSHC                                              | HexaMotion                                                   | Status                                                                      |
| :--------------------------------------------------- | :----------------------------------------------------------- | :-------------------------------------------------------------------------- |
| `Parameter<T>` (ROS param server binding)            | —                                                            | **Removed** (by design: no ROS).                                            |
| `AdjustableParameter` (current/min/max/step + ROS)   | —                                                            | **Partial**: Lightweight runtime `setParameter()` for core gait parameters. |
| `Parameters` struct (~70 fields, all `Parameter<T>`) | `Parameters` struct in `robot_model.h` (flat numeric fields) | ✅ Equivalent data, different storage format.                               |

#### Constants — Detailed Mapping

| OpenSHC Constant                   | HexaMotion Equivalent                      | Location                                                                                  |
| :--------------------------------- | :----------------------------------------- | :---------------------------------------------------------------------------------------- |
| `UNASSIGNED_VALUE`                 | —                                          | **Gap**: No global sentinel constant; uses validity flags or local `1e9` large sentinels. |
| `UNDEFINED_POSITION`               | `Pose::Undefined()`                        | Uses NaN sentinel in `Pose` (no global constant).                                         |
| `UNDEFINED_ROTATION`               | `Pose::Undefined()`                        | Uses NaN quaternion sentinel (no global constant).                                        |
| `GRAVITY_ACCELERATION`             | Computed from IMU or hardcoded per context | Distributed.                                                                              |
| `PROGRESS_COMPLETE` (100)          | `PROGRESS_COMPLETE` (100)                  | `state_controller.h` ✅                                                                   |
| `THROTTLE_PERIOD` (5)              | `THROTTLE_PERIOD` (1.0f)                   | `state_controller.h` — value changed.                                                     |
| `IK_TOLERANCE` (0.005)             | `IK_TOLERANCE`                             | `hexamotion_constants.h` ✅                                                               |
| `DLS_COEFFICIENT` (0.02)           | `IK_DLS_COEFFICIENT`                       | `hexamotion_constants.h` ✅                                                               |
| `BEARING_STEP` (45)                | `BEARING_STEP`                             | `hexamotion_constants.h` ✅                                                               |
| `MAX_POSITION_DELTA` (0.002)       | `MAX_POSITION_DELTA`                       | `hexamotion_constants.h` ✅                                                               |
| `MAX_WORKSPACE_RADIUS` (1.0)       | `MAX_WORKSPACE_RADIUS`                     | `hexamotion_constants.h` ✅                                                               |
| `WORKSPACE_LAYERS` (10)            | `WORKSPACE_LAYERS`                         | `hexamotion_constants.h` ✅                                                               |
| `JOINT_TOLERANCE` (0.01)           | `JOINT_TOLERANCE` (0.1f)                   | `state_controller.h` — value relaxed.                                                     |
| `TIP_TOLERANCE` (0.01)             | `TIP_TOLERANCE`                            | `hexamotion_constants.h` ✅                                                               |
| `SAFETY_FACTOR` (0.15)             | `SAFETY_FACTOR`                            | `hexamotion_constants.h` ✅                                                               |
| `PACK_TIME` (2.0)                  | `PACK_TIME` (2.0f)                         | `state_controller.h` ✅                                                                   |
| `MAX_MANUAL_LEGS` (2)              | `MAX_MANUAL_LEGS` (2)                      | `state_controller.h` ✅                                                                   |
| `HORIZONTAL_TRANSITION_TIME` (1.0) | `HORIZONTAL_TRANSITION_TIME`               | `hexamotion_constants.h` ✅                                                               |
| `VERTICAL_TRANSITION_TIME` (3.0)   | `VERTICAL_TRANSITION_TIME`                 | `hexamotion_constants.h` ✅                                                               |
| `STABILITY_THRESHOLD` (100)        | `DEFAULT_STABILITY_THRESHOLD`              | `hexamotion_constants.h` ✅                                                               |
| `ADMITTANCE_DEADBAND` (0.0)        | Inline in `AdmittanceController`           | ✅                                                                                        |

### 2. standard_includes.h (OpenSHC) vs HexaMotion utility/constants

**Status**: Refactored — Logic Preserved

| OpenSHC Function                | HexaMotion Equivalent       | Location       | Status                                                                    |
| :------------------------------ | :-------------------------- | :------------- | :------------------------------------------------------------------------ |
| `degreesToRadians()`            | `degreesToRadians()`        | `math_utils.h` | ✅                                                                        |
| `radiansToDegrees()`            | `radiansToDegrees()`        | `math_utils.h` | ✅                                                                        |
| `mod<T>()`                      | `mod<T>()`                  | `math_utils.h` | ✅                                                                        |
| `sqr<T>()`                      | `sqr<T>()`                  | `math_utils.h` | ✅                                                                        |
| `sign<T>()`                     | `sign<T>()`                 | `math_utils.h` | ✅                                                                        |
| `roundToInt()`                  | `roundToInt()`              | `math_utils.h` | ✅                                                                        |
| `roundToEvenInt()`              | `roundToEvenInt()`          | `math_utils.h` | ✅                                                                        |
| `clamped<T>(value, min, max)`   | `clamped()`                 | `math_utils.h` | ✅                                                                        |
| `clamped<T>(vector, magnitude)` | `clamped()`                 | `math_utils.h` | ✅                                                                        |
| `setPrecision(double, int)`     | `setPrecision()`            | `math_utils.h` | ✅                                                                        |
| `setPrecision(Vector3d, int)`   | `setPrecision()`            | `math_utils.h` | ✅                                                                        |
| `smoothStep()`                  | `smoothStep()`              | `math_utils.h` | ✅                                                                        |
| `getProjection()`               | `getProjection()`           | `math_utils.h` | ✅                                                                        |
| `getRejection()`                | `getRejection()`            | `math_utils.h` | ✅                                                                        |
| `interpolate<T>()`              | `interpolate<T>()`          | `math_utils.h` | ✅                                                                        |
| `correctRotation()`             | `correctRotation()`         | `math_utils.h` | ✅                                                                        |
| `eulerAnglesToQuaternion()`     | `eulerAnglesToQuaternion()` | `math_utils.h` | ✅                                                                        |
| `quaternionToEulerAngles()`     | `quaternionToEulerAngles()` | `math_utils.h` | ✅                                                                        |
| `numberToString<T>()`           | —                           | —              | **Gap** (minor utility, `String()` used).                                 |
| `stringFormat<Args>()`          | —                           | —              | **Gap** (minor utility, snprintf-based format; no HexaMotion equivalent). |

#### Bezier / DH Functions (OpenSHC standard_includes.h) — Detailed Mapping

| OpenSHC Function                               | HexaMotion Equivalent   | Location       | Status                                                                            |
| :--------------------------------------------- | :---------------------- | :------------- | :-------------------------------------------------------------------------------- |
| `quadraticBezier<T>()`                         | `quadraticBezier<T>()`  | `math_utils.h` | ✅                                                                                |
| `quadraticBezierCurveThroughControlPoint<T>()` | —                       | —              | **Gap**: Adjusts control point so curve passes _through_ it (not just toward it). |
| `cubicBezier<T>()`                             | `cubicBezier<T>()`      | `math_utils.h` | ✅                                                                                |
| `cubicBezierDot<T>()`                          | `cubicBezierDot<T>()`   | `math_utils.h` | ✅                                                                                |
| `cubicBezierCurveThroughControlPoint<T>()`     | —                       | —              | **Gap**: Through-control-point variant (parameterized by node index).             |
| `quarticBezier<T>()`                           | `quarticBezier<T>()`    | `math_utils.h` | ✅                                                                                |
| `quarticBezierDot<T>()`                        | `quarticBezierDot<T>()` | `math_utils.h` | ✅                                                                                |
| `quarticBezierCurveThroughControlPoint<T>()`   | —                       | —              | **Gap**: Through-control-point variant (parameterized by node index).             |
| `createDHMatrix(theta, d, r, alpha)`           | `dhTransform<T>()`      | `math_utils.h` | ✅ (renamed, templated).                                                          |

### 3. Pose class (OpenSHC pose.h) vs HexaMotion Pose

**Status**: Functional Equivalent

| OpenSHC Pose Method                   | HexaMotion Equivalent         | Status                                             |
| :------------------------------------ | :---------------------------- | :------------------------------------------------- |
| `Pose(Vector3d, Quaterniond)`         | `Pose(Vector3d, Quaterniond)` | ✅                                                 |
| `Pose(geometry_msgs::Pose)`           | —                             | **Removed** (no ROS).                              |
| `Pose(geometry_msgs::Transform)`      | —                             | **Removed** (no ROS).                              |
| `isValid()`                           | `Pose::isValid()`             | ✅                                                 |
| `toPoseMessage()`                     | —                             | **Removed** (no ROS).                              |
| `toTransformMessage()`                | —                             | **Removed** (no ROS).                              |
| `operator==` / `operator!=`           | `operator==` / `operator!=`   | ✅                                                 |
| `operator~()` (inverse)               | `inverse()`                   | ✅ (named method, comment references `operator~`). |
| `transform(geometry_msgs::Transform)` | —                             | **Removed** (no ROS).                              |
| `transform(Matrix4d)`                 | `transform(Matrix4d)`         | ✅                                                 |
| `transformVector()`                   | `transformVector()`           | ✅                                                 |
| `inverseTransformVector()`            | `inverseTransformVector()`    | ✅                                                 |
| `addPose()`                           | `addPose()`                   | ✅                                                 |
| `removePose()`                        | `removePose()`                | ✅                                                 |
| `interpolate()`                       | `interpolate()`               | ✅                                                 |
| `Identity()`                          | `Identity()`                  | ✅                                                 |
| `Undefined()`                         | `Pose::Undefined()`           | ✅                                                 |

### 4. Model / Leg / Joint / Link / Tip

**Status**: Architectural Divergence — Core Logic Preserved

#### Model → RobotModel

| OpenSHC Model Method                    | HexaMotion Equivalent                                          | Location                           | Status                                           |
| :-------------------------------------- | :------------------------------------------------------------- | :--------------------------------- | :----------------------------------------------- |
| `Model(params, debug_visualiser)`       | `RobotModel(const Parameters&)`                                | `robot_model.h`                    | ✅ (no debug param)                              |
| `Model(model)` (copy ctor)              | —                                                              | —                                  | **Gap** (not needed: stateless + external legs). |
| `getLegContainer()`                     | — (array in `LocomotionSystem`)                                | —                                  | **Redesigned**: Legs external.                   |
| `getLegCount()`                         | `NUM_LEGS` constant (6)                                        | `hexamotion_constants.h`           | ✅ (hardcoded).                                  |
| `getCurrentPose()` / `setCurrentPose()` | `BodyPoseController::getCurrentBodyPose()`                     | `body_pose_controller.h`           | ✅ (moved).                                      |
| `getDefaultPose()` / `setDefaultPose()` | `BodyPoseController::walk_plane_pose_`                         | `body_pose_controller.h`           | ✅ (moved).                                      |
| `getTimeDelta()`                        | `getTimeDelta()`                                               | `robot_model.h`                    | ✅                                               |
| `generate()`                            | — (distributed initialization)                                 | —                                  | **Redesigned**.                                  |
| `initLegs()`                            | `LocomotionSystem::initialize()` + per-leg `Leg::initialize()` | `locomotion_system.cpp`, `leg.cpp` | ✅                                               |
| `legsBearingLoad()`                     | `LocomotionSystem::legsBearingLoad()`                          | `locomotion_system.h`              | ✅ (public API).                                 |
| `getLegByIDNumber()`                    | `LocomotionSystem::getLeg(int)`                                | `locomotion_system.h`              | ✅                                               |
| `getLegByIDName()`                      | —                                                              | —                                  | **Gap** (minor: index-only access).              |
| `getImuData()` / `setImuData()`         | `BodyPoseController::setIMUData()` / `IMUAutoPose`             | `body_pose_controller.h`           | ✅ (moved).                                      |
| `updateDefaultConfiguration()`          | —                                                              | —                                  | **Gap** (no dynamic reconfiguration).            |
| `generateWorkspaces()`                  | `WorkspaceAnalyzer::generateWorkspace()`                       | `workspace_analyzer.h`             | ✅                                               |
| `updateModel()`                         | `LocomotionSystem::applyInverseKinematicsToAllLegs()`          | `locomotion_system.cpp`            | ✅                                               |
| `estimateGravity()`                     | `TerrainAdaptation::estimateGravity()`                         | `terrain_adaptation.h`             | ✅ (moved).                                      |

#### Leg

| OpenSHC Leg Feature                                               | HexaMotion Equivalent                                              | Status                                                                                                            |
| :---------------------------------------------------------------- | :----------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------- |
| `Joint`/`Link`/`Tip` sub-objects                                  | —                                                                  | **Partial**: Flattened into `JointAngles` + `Point3D`; per-joint velocity/effort tracked but no sub-object model. |
| `joint_container_` / `link_container_` / `tip_`                   | —                                                                  | **Gap**: No sub-object containers.                                                                                |
| `workspace_` (per-leg)                                            | `WorkspaceAnalyzer` (centralized)                                  | ✅ (moved).                                                                                                       |
| `group_` (stepping coordination)                                  | `BodyPoseController::tripod_leg_groups`                            | ✅ (moved).                                                                                                       |
| `leg_stepper_` / `leg_poser_`                                     | Separate classes, owned by `WalkController` / `BodyPoseController` | ✅ (moved).                                                                                                       |
| `leg_state_`                                                      | `leg_state_`                                                       | ✅                                                                                                                |
| `admittance_delta_`                                               | `LegPoser::admittance_delta_`                                      | ✅ (moved).                                                                                                       |
| `virtual_mass_` / `virtual_stiffness_` / `virtual_damping_ratio_` | `AdmittanceController::LegAdmittanceState`                         | ✅ (moved to centralized controller).                                                                             |
| `admittance_state_` (ODE state vector)                            | `AdmittanceController::leg_dynamics_state_[]`                      | ✅ (moved).                                                                                                       |
| `tip_force_calculated_` / `tip_torque_calculated_`                | `Leg::calculateTipForce()` + `tip_force_calculated_`               | **Partial**: Force estimation implemented; torque vector not tracked.                                             |
| `tip_force_measured_` / `tip_torque_measured_`                    | `Leg::contact_force_` (scalar)                                     | **Partial**: Scalar FSR only, not 3D force/torque vectors.                                                        |
| `desired_tip_velocity_` / `current_tip_velocity_`                 | — in `Leg` (in `LegStepper::current_tip_velocity_`)                | **Partial**: Present in `LegStepper`, not in `Leg` directly.                                                      |
| `step_plane_pose_`                                                | `TerrainAdaptation::step_planes_[]`                                | ✅ (moved).                                                                                                       |
| `desired_tip_pose_` / `current_tip_pose_`                         | `desired_tip_position_` / `tip_position_` (Point3D)                | ✅ (simplified: 3DOF needs position only, not full Pose).                                                         |
| `generate()` / `init()`                                           | `Leg::initialize(const Pose&)`                                     | ✅                                                                                                                |
| `generateWorkspace()`                                             | `WorkspaceAnalyzer::generateWalkspaceForLeg()`                     | ✅ (moved).                                                                                                       |
| `getWorkplane()`                                                  | `WorkspaceAnalyzer::getWorkplane()`                                | ✅ (moved).                                                                                                       |
| `makeReachable()`                                                 | `RobotModel::makeReachable()`                                      | ✅ (moved).                                                                                                       |
| `setDesiredTipPose(apply_delta)`                                  | `Leg::setDesiredTipPosition()` + `LegPoser::admittance_delta_`     | ✅ (split).                                                                                                       |
| `setDesiredTipVelocity()`                                         | — on Leg                                                           | **Gap** on Leg (tracked in `LegStepper`).                                                                         |
| `calculateTipForce()`                                             | `Leg::calculateTipForce()`                                         | ✅                                                                                                                |
| `touchdownDetection()`                                            | `TerrainAdaptation::detectTouchdownEvents()`                       | ✅ (moved).                                                                                                       |
| `solveIK()`                                                       | `RobotModel::solveIK()` / `solveDeltaIK()`                         | ✅ (centralized).                                                                                                 |
| `updateJointPositions()`                                          | `Leg::setJointAngles()` + `updateTipPosition()`                    | ✅                                                                                                                |
| `applyIK()`                                                       | `Leg::applyIK()` / `applyAdvancedIK()`                             | ✅                                                                                                                |
| `applyFK()`                                                       | `Leg::updateTipPosition()`                                         | ✅                                                                                                                |
| `generateDesiredJointStateMsg()`                                  | —                                                                  | **Removed** (no ROS messages).                                                                                    |
| ROS publishers                                                    | —                                                                  | **Removed** (no ROS).                                                                                             |

**HexaMotion Leg additions** (not in OpenSHC):

- `applyAdvancedIK()` — nullspace-projected joint-limit cost gradient IK.
- FSR history circular buffer API (`updateFSRHistory`, `getFilteredContactState`, `getAverageContactValue`).
- Gait phase helpers (`calculateLegPhase`, `shouldBeInStance/Swing`, `setPhaseOffset`).
- `isInDefaultStance()`, `getLegDirection()`.

### 5. StateController (OpenSHC) vs HexaMotion StateController / LocomotionSystem

**Status**: Split into two classes — Logic Preserved

#### Method Mapping

| OpenSHC StateController Method       | HexaMotion Equivalent                                           | Location                     | Status                                                       |
| :----------------------------------- | :-------------------------------------------------------------- | :--------------------------- | :----------------------------------------------------------- |
| `StateController()` (ctor)           | `StateController(LocomotionSystem&, const StateMachineConfig&)` | `state_controller.h`         | ✅                                                           |
| `init()`                             | `StateController::initialize(const BodyPoseConfiguration&)`     | `state_controller.h`         | ✅                                                           |
| `initParameters()`                   | —                                                               | —                            | **Removed** (no ROS param server). Factory pattern replaces. |
| `initGaitParameters()`               | `GaitConfigFactory::create*Config()`                            | `gait_config_factory.h`      | ✅ (redesigned).                                             |
| `initAutoPoseParameters()`           | `BodyPoseConfigFactory`                                         | `body_pose_config_factory.h` | ✅ (redesigned).                                             |
| `loop()`                             | `StateController::update(double)`                               | `state_controller.h`         | ✅                                                           |
| `transitionRobotState()`             | `StateController::handleRobotStateTransition()` (private)       | `state_controller.cpp`       | ✅                                                           |
| `runningState()`                     | Split: `updateVelocityControl()` + `updatePoseControl()`        | `state_controller.cpp`       | ✅                                                           |
| `adjustParameter()`                  | —                                                               | —                            | **Gap**: No dynamic parameter adjustment.                    |
| `changeGait()`                       | `StateController::changeGait(GaitType)`                         | `state_controller.h`         | ✅                                                           |
| `legStateToggle()`                   | `requestLegToggle(int)` + `handleLegStateTransitions()`         | `state_controller.h`         | ✅ (split).                                                  |
| `executePlan()`                      | —                                                               | —                            | **Gap**: No external planner (by design).                    |
| `publishDesiredJointState()`         | `LocomotionSystem::publishJointAnglesToServos()`                | `locomotion_system.cpp`      | ✅ (via IServoInterface).                                    |
| `publishLegState()`                  | —                                                               | —                            | **Removed** (no ROS).                                        |
| `publishVelocity()`                  | —                                                               | —                            | **Removed** (no ROS).                                        |
| `publishPose()`                      | —                                                               | —                            | **Removed** (no ROS).                                        |
| `publishWalkspace()`                 | —                                                               | —                            | **Removed** (no ROS).                                        |
| `publishRotationPoseError()`         | —                                                               | —                            | **Removed** (no ROS).                                        |
| `publishFrameTransforms()`           | —                                                               | —                            | **Removed** (no ROS / TF2).                                  |
| `generateExternalTargetTransforms()` | —                                                               | —                            | **Removed** (no TF2).                                        |
| `RVIZDebugging()`                    | —                                                               | —                            | **Removed** (no RViz).                                       |

#### Callback → Direct API Mapping

| OpenSHC Callback                                                            | HexaMotion Direct API                               | Status                                 |
| :-------------------------------------------------------------------------- | :-------------------------------------------------- | :------------------------------------- |
| `systemStateCallback()`                                                     | `requestSystemState(SystemState)`                   | ✅                                     |
| `robotStateCallback()`                                                      | `requestRobotState(RobotState)`                     | ✅                                     |
| `bodyVelocityInputCallback()`                                               | `setDesiredVelocity(Vector2d, double)`              | ✅                                     |
| `bodyPoseInputCallback()`                                                   | `setDesiredPose(Vector3d, Vector3d)`                | ✅                                     |
| `posingModeCallback()`                                                      | `setPosingMode(PosingMode)`                         | ✅                                     |
| `poseResetCallback()`                                                       | `setPoseResetMode(PoseResetMode)`                   | ✅                                     |
| `gaitSelectionCallback()`                                                   | `changeGait(GaitType)`                              | ✅                                     |
| `cruiseControlCallback()`                                                   | `setCruiseControlMode(CruiseControlMode, Vector3d)` | ✅                                     |
| `plannerModeCallback()`                                                     | —                                                   | **Gap** (no planner).                  |
| `primaryLegSelectionCallback()`                                             | `requestLegToggle(int)`                             | ✅ (consolidated, any leg by index).   |
| `secondaryLegSelectionCallback()`                                           | `requestLegToggle(int)`                             | ✅ (no primary/secondary distinction). |
| `primaryLegStateCallback()` / `secondaryLegStateCallback()`                 | `requestLegToggle(int)`                             | ✅ (consolidated).                     |
| `primaryTipVelocityInputCallback()` / `secondaryTipVelocityInputCallback()` | `setLegTipVelocity(int, Vector3d)`                  | ✅ (any-leg API).                      |
| `primaryTipPoseInputCallback()` / `secondaryTipPoseInputCallback()`         | `setLegTipPose(int, Point3D)`                       | ✅ (per-leg Cartesian pose input).     |
| `parameterSelectionCallback()` / `parameterAdjustCallback()`                | —                                                   | **Gap**: No dynamic params.            |
| `dynamicParameterCallback()`                                                | —                                                   | **Gap**: No dynamic reconfigure.       |
| `imuCallback()`                                                             | `IIMUInterface` + `LocomotionSystem`                | ✅ (via HAL interface).                |
| `jointStatesCallback()`                                                     | `IServoInterface` + `LocomotionSystem`              | ✅ (via HAL interface).                |
| `tipStatesCallback()`                                                       | `IFSRInterface` + `LocomotionSystem`                | ✅ (via HAL interface).                |
| `targetConfigurationCallback()`                                             | —                                                   | **Gap** (no planner).                  |
| `targetBodyPoseCallback()`                                                  | —                                                   | **Gap** (no planner).                  |
| `targetTipPoseCallback()`                                                   | —                                                   | **Gap** (no planner).                  |

#### StateController Member Variables

| OpenSHC Variable                                      | HexaMotion Equivalent                                    | Status                               |
| :---------------------------------------------------- | :------------------------------------------------------- | :----------------------------------- |
| `model_` (shared_ptr)                                 | `locomotion_system_` (reference)                         | ✅ (different ownership model).      |
| `walker_` / `poser_` / `admittance_`                  | Via `LocomotionSystem` accessors                         | ✅                                   |
| `system_state_` / `new_system_state_`                 | `current_system_state_` / `desired_system_state_`        | ✅                                   |
| `robot_state_` / `new_robot_state_`                   | `current_robot_state_` / `desired_robot_state_`          | ✅                                   |
| `gait_selection_`                                     | Via `WalkController` gait config                         | ✅                                   |
| `posing_mode_`                                        | `current_posing_mode_`                                   | ✅                                   |
| `cruise_control_mode_`                                | `current_cruise_control_mode_`                           | ✅                                   |
| `planner_mode_`                                       | `current_planner_mode_`                                  | ✅ (stubbed).                        |
| `parameter_selection_` / `dynamic_parameter_`         | —                                                        | **Gap**.                             |
| `primary_leg_selection_` / `secondary_leg_selection_` | `toggle_leg_index_`                                      | ✅ (simplified to single index).     |
| `manual_leg_count_`                                   | `manual_leg_count_`                                      | ✅                                   |
| `cruise_control_end_time_`                            | `cruise_end_time_`                                       | ✅                                   |
| `gait_change_flag_`                                   | In `changeGait()` flow                                   | ✅ (integrated).                     |
| `toggle_*_leg_state_`                                 | `toggle_leg_state_pending_`                              | ✅ (one flag, not two).              |
| `parameter_adjust_flag_`                              | —                                                        | **Gap**.                             |
| `joint_positions_initialised_`                        | `is_initialized_`                                        | ✅                                   |
| `transition_state_flag_`                              | `is_transitioning_`                                      | ✅                                   |
| `target_*_acquired_` / `plan_step_`                   | —                                                        | **Gap** (no planner).                |
| `linear_velocity_input_` / `angular_velocity_input_`  | `desired_linear_velocity_` / `desired_angular_velocity_` | ✅                                   |
| `primary/secondary_tip_velocity_input_`               | `leg_tip_velocities_[NUM_LEGS]`                          | ✅ (per-leg array).                  |
| `linear/angular_cruise_velocity_`                     | `cruise_velocity_` (Vector3d)                            | ✅                                   |
| `primary/secondary_pose_input_`                       | —                                                        | **Gap** (no per-leg Cartesian pose). |
| ROS subscribers/publishers                            | —                                                        | **Removed**.                         |
| TF2 buffer/listener/broadcaster                       | —                                                        | **Removed**.                         |
| `dynamic_reconfigure_server_`                         | —                                                        | **Removed**.                         |

**HexaMotion-only StateController additions** (not in OpenSHC):

- `StateMachineConfig` struct (compile-time FSM configuration).
- `ErrorCode` enum with 10 error codes and `handleError()`/`clearError()` methods.
- `StopMode` enum (`STOP_UNIFORM`, `STOP_SOFT`).
- `emergencyStop()` / `reset()` methods.
- `getDiagnosticInfo()` — textual FSM state dump.
- Explicit `executeStartupSequence`/`executeShutdownSequence`/`executePackSequence`/`executeUnpackSequence` private methods.

### 6. WalkController (OpenSHC) vs HexaMotion WalkController

**Status**: High Logic Parity, Architectural Enhancements

#### Method Mapping

| OpenSHC WalkController Method                           | HexaMotion Equivalent                                              | Status                                                              |
| :------------------------------------------------------ | :----------------------------------------------------------------- | :------------------------------------------------------------------ |
| `WalkController(model, params)`                         | `WalkController(RobotModel&, Leg[], const BodyPoseConfiguration&)` | ✅ (different params).                                              |
| `init()`                                                | `init(const Vector3d&, const Vector3d&)`                           | ✅ (takes initial body pose).                                       |
| `generateWalkspace()`                                   | `generateWalkspace()` (delegates to `WorkspaceAnalyzer`)           | ✅                                                                  |
| `generateLimits(StepCycle, 4×LimitMap*)`                | `generateLimits(StepCycle)` → `VelocityLimits`                     | ✅ (output mechanism changed).                                      |
| `generateLimits(4×LimitMap*)` (overload)                | —                                                                  | **Removed**: `VelocityLimits::generateLimits(GaitConfig)` replaces. |
| `generateStepCycle(bool)`                               | `GaitConfiguration::generateStepCycle()`                           | ✅ (moved to config object).                                        |
| `getLimit(Vector2d, double, LimitMap)`                  | `getLimit(Vector2d, double, map<int,double>)`                      | ✅                                                                  |
| `updateWalk(Vector2d, double)`                          | `updateWalk(Point3D, double, Vector3d, Vector3d)`                  | ✅ (takes body pose explicitly).                                    |
| `updateManual(int, Vector3d, int, Vector3d)` (velocity) | `updateManual(int, Vector3d, int, Vector3d)`                       | ✅                                                                  |
| `updateManual(int, Pose, int, Pose)` (pose)             | `updateManual(int, Point3D, int, Point3D)`                         | ✅ (Pose→Point3D: no rotation for 3DOF).                            |
| `updateWalkPlane()`                                     | delegated to `BodyPoseController::updateWalkPlanePose()`           | ✅ (moved).                                                         |
| `estimateGravity()`                                     | `estimateGravity()` (delegates to model/terrain)                   | ✅                                                                  |
| `calculateOdometry(double)`                             | `calculateOdometry(double)`                                        | ✅                                                                  |

#### Member Variables

| OpenSHC Variable                        | HexaMotion Equivalent                          | Status                                                |
| :-------------------------------------- | :--------------------------------------------- | :---------------------------------------------------- |
| `model_` (shared_ptr)                   | `model` (reference)                            | ✅                                                    |
| `params_`                               | Via `model.getParams()`                        | ✅                                                    |
| `walk_state_`                           | `walk_state_`                                  | ✅                                                    |
| `pose_state_` (PosingState enum)        | `pose_state_` (int)                            | ✅ (type simplified).                                 |
| `step_` (StepCycle)                     | Via `current_gait_config_.generateStepCycle()` | ✅ (computed on demand).                              |
| `walkspace_` (LimitMap)                 | `walkspace_` (map)                             | ✅                                                    |
| `walk_plane_` / `walk_plane_normal_`    | Via `BodyPoseController`                       | ✅ (delegated).                                       |
| `regenerate_walkspace_`                 | `regenerate_walkspace_`                        | ✅                                                    |
| `desired_linear_velocity_` (Vector2d)   | `desired_linear_velocity_` (Point3D)           | ✅ (2D→3D).                                           |
| `desired_angular_velocity_`             | `desired_angular_velocity_`                    | ✅                                                    |
| `odometry_ideal_`                       | `odometry_ideal_`                              | ✅                                                    |
| `max_linear_speed_` + 3 other LimitMaps | `VelocityLimits` class                         | ✅ (consolidated).                                    |
| `legs_at_correct_phase_`                | `legs_at_correct_phase_`                       | ✅                                                    |
| `legs_completed_first_step_`            | `legs_completed_first_step_`                   | ✅                                                    |
| `return_to_default_attempted_`          | `return_to_default_attempted_`                 | ✅                                                    |
| `leg_it_` / `joint_it_` / `link_it_`    | —                                              | **Removed** (no iterator pattern; index-based loops). |

**HexaMotion WalkController additions** (not in OpenSHC):

- `VelocityLimits` class with 360-bearing `LimitValues` array (replaces 4 separate `LimitMap` objects).
- `TerrainAdaptation` integration (`updateTerrainAdaptation(IFSR*, IIMU*)`).
- Per-leg `StepCycle` storage (instead of single global `step_` on WalkController).
- `GaitConfiguration` factory system replacing YAML parsing.
- Velocity validation/clamping API (`applyVelocityLimits`, `validateVelocityCommand`).
- Rough terrain mode toggle (`enableRoughTerrainMode`).
- `LegTrajectoryInfo` struct for diagnostics.
- `setGait(GaitType)` / `setGait(GaitConfiguration)` — runtime gait switching.
- `CartesianVelocityController` integration for servo speed mapping.

### 7. LegStepper (OpenSHC) vs HexaMotion LegStepper

**Status**: High Logic Parity, Visibility Changes

| OpenSHC LegStepper Method                    | HexaMotion Equivalent                                | Status                                                                |
| :------------------------------------------- | :--------------------------------------------------- | :-------------------------------------------------------------------- |
| `LegStepper(walker, leg, identity_tip_pose)` | `LegStepper(int, Leg&, RobotModel&, const Point3D&)` | ✅ (different params).                                                |
| `LegStepper(leg_stepper)` (copy)             | `LegStepper(const LegStepper&)`                      | ✅                                                                    |
| `updatePhase()`                              | —                                                    | **Removed**: Phase advanced by `WalkController` loop.                 |
| `iteratePhase()`                             | —                                                    | **Removed**: Same reason.                                             |
| `updateStepState()`                          | `updateStepStateFromPhase()`                         | ✅ (renamed).                                                         |
| `updateStride()`                             | `updateStride()`                                     | ✅ (adds stride freezing).                                            |
| `calculateStanceSpanChange()`                | `calculateStanceSpanChange()` (private)              | ✅ (moved to private).                                                |
| `updateDefaultTipPosition()`                 | `updateDefaultTipPosition()` (private)               | ✅ (moved to private).                                                |
| `updateTipPosition()`                        | `updateTipPosition(double, bool, bool)`              | ✅ (takes terrain params).                                            |
| `updateTipRotation()`                        | —                                                    | **Gap**: Not ported (unnecessary for 3DOF legs with no tip rotation). |
| `generatePrimarySwingControlNodes()`         | `generatePrimarySwingControlNodes()` (private)       | ✅                                                                    |
| `generateSecondarySwingControlNodes(bool)`   | `generateSecondarySwingControlNodes(bool)` (private) | ✅                                                                    |
| `generateStanceControlNodes(double)`         | `generateStanceControlNodes(double)` (private)       | ✅                                                                    |
| `forceNormalTouchdown()`                     | `forceNormalTouchdown()` (private)                   | ✅                                                                    |

**Bezier curve implementation**: Both use identical 5-node quartic Bezier curves for primary swing, secondary swing, and stance trajectories with C0/C1/C2 continuity. HexaMotion preserves the exact node computation from OpenSHC.

| OpenSHC LegStepper Variable                                                                 | HexaMotion Equivalent                                               | Status                                                      |
| :------------------------------------------------------------------------------------------ | :------------------------------------------------------------------ | :---------------------------------------------------------- |
| `walker_` (shared_ptr back-pointer)                                                         | —                                                                   | **Removed** (WalkController pushes state).                  |
| `leg_` (shared_ptr)                                                                         | `leg_` (reference)                                                  | ✅                                                          |
| `at_correct_phase_`                                                                         | `at_correct_phase_`                                                 | ✅                                                          |
| `completed_first_step_`                                                                     | `completed_first_step_`                                             | ✅                                                          |
| `touchdown_detection_`                                                                      | `touchdown_detection_`                                              | ✅                                                          |
| `phase_` / `phase_offset_`                                                                  | `phase_` / via `leg_.getPhaseOffset()`                              | ✅                                                          |
| `step_progress_`                                                                            | `step_progress_`                                                    | ✅                                                          |
| `swing_progress_` / `stance_progress_`                                                      | —                                                                   | **Removed**: Derived from `step_progress_` + `step_state_`. |
| `step_state_`                                                                               | `step_state_`                                                       | ✅                                                          |
| `swing_1_nodes_[5]` / `swing_2_nodes_[5]` / `stance_nodes_[5]`                              | Same (Point3D)                                                      | ✅                                                          |
| `walk_plane_`                                                                               | —                                                                   | **Removed**: Managed externally.                            |
| `walk_plane_normal_`                                                                        | `walk_plane_normal_`                                                | ✅                                                          |
| `stride_vector_` / `swing_clearance_`                                                       | Same                                                                | ✅                                                          |
| `swing_delta_t_` / `stance_delta_t_`                                                        | Same                                                                | ✅                                                          |
| `identity_tip_pose_` (Pose)                                                                 | `identity_tip_pose_` (Point3D)                                      | ✅ (3DOF: position only).                                   |
| `default_tip_pose_` / `current_tip_pose_` / `origin_tip_pose_` / `target_tip_pose_`         | Same (Point3D)                                                      | ✅                                                          |
| `external_target_` / `external_default_` (ExternalTarget)                                   | `external_target_` / `external_default_` (LegStepperExternalTarget) | ✅ (simplified: no `transform_`).                           |
| `current_tip_velocity_`                                                                     | `current_tip_velocity_`                                             | ✅                                                          |
| `swing_origin_tip_position_` / `swing_origin_tip_velocity_` / `stance_origin_tip_position_` | Same                                                                | ✅                                                          |

**HexaMotion LegStepper additions**:

- Stride freezing system (`frozen_stride_vector_*`, `stride_frozen_`, `target_frozen_`).
- Workspace validation chain (`validateTargetTipPose`, `constrainToWorkspace`, `calculateSafeTarget`).
- Per-leg `StepCycle` storage.
- Anti-drift diagnostics (TESTING_ENABLED).
- Phase-end snap for precise touchdown alignment.
- `VelocityLimits` integration.

### 8. BodyPoseController (OpenSHC PoseController) vs HexaMotion BodyPoseController

**Status**: Full Logic Parity — Split into Sub-classes

| OpenSHC PoseController Method        | HexaMotion Equivalent                                               | Status                                    |
| :----------------------------------- | :------------------------------------------------------------------ | :---------------------------------------- |
| `PoseController(model, params)`      | `BodyPoseController(RobotModel&, const BodyPoseConfiguration&)`     | ✅                                        |
| `init()`                             | `initializeLegPosers()` + `initializeDefaultPose()`                 | ✅ (split).                               |
| `setAutoPoseParams()`                | `setAutoPoseConfig()`                                               | ✅                                        |
| `updateStance()`                     | `applyAutoPoseToDesiredTips()`                                      | ✅ (renamed; comment references OpenSHC). |
| `executeSequence(SequenceSelection)` | `executeSequence(const std::string&, Leg[])`                        | ✅                                        |
| `directStartup()`                    | `executeStartupSequence()`                                          | ✅                                        |
| `stepToNewStance()`                  | `stepToNewStance()`                                                 | ✅                                        |
| `poseForLegManipulation()`           | `poseForLegManipulation()`                                          | ✅                                        |
| `packLegs(time)`                     | `packLegs(Leg[], time)`                                             | ✅                                        |
| `unpackLegs(time)`                   | `unpackLegs(Leg[], time)`                                           | ✅                                        |
| `transitionConfiguration(time)`      | `LegPoser::transitionConfiguration(time)`                           | ✅ (delegated).                           |
| `transitionStance(time)`             | Partial (via `stepToNewStance`)                                     | **Partial**.                              |
| `updateCurrentPose(RobotState)`      | `updateCurrentPose(double, Leg[])`                                  | ✅                                        |
| `updateManualPose()`                 | `ManualBodyPoseController::processInput()` + `setManualPoseInput()` | ✅ (delegated to sub-class).              |
| `updateIKErrorPose()`                | `updateIKErrorPose()`                                               | ✅                                        |
| `updateTipAlignPose()`               | `updateTipAlignPose()`                                              | ✅                                        |
| `updateWalkPlanePose()`              | `updateWalkPlanePose()`                                             | ✅                                        |
| `updateAutoPose()`                   | `updateAutoPose()`                                                  | ✅                                        |
| `updateIMUPose()`                    | `updateIMUPosePID()`                                                | ✅                                        |
| `updateInclinationPose()`            | Inline in `updateCurrentPose()` (when `inclination_pose_enabled_`)  | ✅ (inlined, not a separate method).      |
| `estimateGravity()`                  | `TerrainAdaptation::estimateGravity()`                              | ✅ (moved).                               |
| `calculateDefaultPose()`             | `calculateDefaultPose()`                                            | ✅                                        |
| `resetAllPosing()`                   | `resetAllPosing()`                                                  | ✅                                        |

#### Pose Composition Variables

| OpenSHC Sub-Pose                               | HexaMotion Equivalent                                | Status                                                                                                         |
| :--------------------------------------------- | :--------------------------------------------------- | :------------------------------------------------------------------------------------------------------------- |
| `manual_pose_`                                 | `manual_pose_`                                       | ✅                                                                                                             |
| `auto_pose_`                                   | `global_auto_pose_` + per-leg `LegPoser::auto_pose_` | ✅ (split).                                                                                                    |
| `imu_pose_`                                    | `imu_pose_`                                          | ✅                                                                                                             |
| `inclination_pose_`                            | `inclination_pose_`                                  | ✅                                                                                                             |
| `admittance_pose_`                             | —                                                    | **Redesigned**: Admittance applied per-leg via `LegPoser::admittance_delta_`, not as body-level pose.          |
| `default_pose_`                                | `default_pose_`                                      | ✅                                                                                                             |
| `ik_error_pose_`                               | `ik_error_pose_`                                     | ✅                                                                                                             |
| `tip_align_pose_` / `origin_tip_align_pose_`   | `tip_align_pose_` / `origin_tip_align_pose_`         | ✅                                                                                                             |
| `walk_plane_pose_` / `origin_walk_plane_pose_` | `walk_plane_pose_`                                   | **Partial**: `walk_plane_pose_` present; `origin_walk_plane_pose_` absent (Bézier interpolation used instead). |
| `rotation_absement_error_`                     | `rotation_absement_error_`                           | ✅                                                                                                             |
| `rotation_position_error_`                     | `rotation_position_error_`                           | ✅                                                                                                             |
| `rotation_velocity_error_`                     | `rotation_velocity_error_`                           | ✅                                                                                                             |

**OpenSHC `AutoPoser` class** → Replaced by `BodyPoseController` auto-pose subsystem:

- `AutoPoserContainer` with multiple `AutoPoser` objects → Single `AutoPoseConfiguration` with amplitude vectors.
- `AutoPoser::updatePose(phase)` → `LegPoser::updateAutoPose(phase, AutoPoseConfiguration, BodyPoseConfiguration)`.
- OpenSHC supports multiple independent auto-pose cycles with separate start/end phases.
- HexaMotion supports a single auto-pose cycle (sufficient for hexapod; configurable via `AutoPoseConfiguration`).

**HexaMotion BodyPoseController additions** (not in OpenSHC):

- Smooth body pose trajectory system (`setBodyPoseSmooth*`, `initializeTrajectoryFromCurrent`, `updateTrajectoryStep`).
- Initial standing pose transition (`beginInitialStandingPoseTransition`, `stepInitialStandingPoseTransition`).
- Walk plane Bézier curve interpolation.
- `ManualBodyPoseController` sub-class with presets and extended modes.
- `IMUAutoPose` sub-class with multi-mode operation (level, inclination, adaptive, custom).
- Progress reporting APIs (`getStartupProgressPercent()`).

### 9. LegPoser (OpenSHC) vs HexaMotion LegPoser

**Status**: Full Parity

| OpenSHC LegPoser Method                                                                             | HexaMotion Equivalent                                                                                   | Status                                  |
| :-------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------ | :-------------------------------------- |
| `LegPoser(poser, leg)`                                                                              | `LegPoser(int, Leg&, RobotModel&)`                                                                      | ✅                                      |
| `LegPoser(copy)`                                                                                    | `LegPoser(const LegPoser*)`                                                                             | ✅                                      |
| `getCurrentTipPose()` / `setCurrentTipPose()`                                                       | Same                                                                                                    | ✅                                      |
| `getTargetTipPose()` / `setTargetTipPose()`                                                         | Same                                                                                                    | ✅                                      |
| `getExternalTarget()` / `setExternalTarget()`                                                       | Same                                                                                                    | ✅                                      |
| `getAutoPose()` / `setAutoPose()`                                                                   | Same                                                                                                    | ✅                                      |
| `getPoseNegationPhaseStart/End()`                                                                   | Private members (no public getter)                                                                      | ✅ (logic present, visibility changed). |
| `getLegCompletedStep()` / `setLegCompletedStep()`                                                   | Same                                                                                                    | ✅                                      |
| `getTransitionPose()` / `hasTransitionPose()` / `addTransitionPose()` / `resetTransitionSequence()` | Same                                                                                                    | ✅                                      |
| `resetStepToPosition()`                                                                             | `resetStepToPosition()`                                                                                 | ✅                                      |
| `transitionConfiguration(time)`                                                                     | `transitionConfiguration(time)`                                                                         | ✅                                      |
| `stepToPosition(target, pose, height, time, apply_delta)`                                           | `stepToPosition(Pose, Pose, double, double, bool)` + simplified `stepToPosition(Point3D, height, time)` | ✅                                      |
| `updateAutoPose(phase)`                                                                             | `updateAutoPose(phase, AutoPoseConfig, BodyPoseConfig)`                                                 | ✅ (takes config params explicitly).    |

**HexaMotion LegPoser additions**:

- `setAdmittanceDelta()` / `getAdmittanceDelta()` with deadband validation.
- `setDesiredConfiguration()` — explicit target for `transitionConfiguration()`.
- `getCurrentStepProgress()` — progress reporting.

### 10. AdmittanceController (OpenSHC) vs HexaMotion AdmittanceController

**Status**: Full Parity, Extended

| OpenSHC Method                        | HexaMotion Equivalent                                                          | Status                |
| :------------------------------------ | :----------------------------------------------------------------------------- | :-------------------- |
| `AdmittanceController(model, params)` | `AdmittanceController(RobotModel&, IIMU*, IFSR*, ComputeConfig)`               | ✅                    |
| `updateAdmittance()`                  | `applyForceAndIntegrate(int, Point3D)` + `updateAllLegs(Point3D[], Point3D[])` | ✅ (per-leg + batch). |
| `updateStiffness(leg, scale)`         | `setLegAdmittance(int, mass, damping, stiffness)` + per-leg `stiffness_scale`  | ✅                    |
| `updateStiffness(walker)`             | `updateStiffness(StepPhase[], Point3D[], double)`                              | ✅                    |

**Implementation details**: Both use mass-spring-damper ODE with force input. OpenSHC uses boost::odeint RK4; HexaMotion provides configurable integration (`EULER_METHOD`, `RUNGE_KUTTA_2`, `RUNGE_KUTTA_4`). Both compute `virtual_damping = damping_ratio * 2 * sqrt(mass * stiffness)`. Both support dynamic stiffness scaling based on swing/load states.

**HexaMotion additions**:

- Multi-precision ODE integration selection.
- `LegAdmittanceState` struct per leg (consolidates OpenSHC's scattered per-leg variables).
- `WorkspaceAnalyzer` integration for stiffness calculations.
- `orientationError()`, `maintainOrientation()`, `checkStability()` helper methods.

### 11. DebugVisualiser (OpenSHC) vs HexaMotion

**Status**: Removed (by design)

OpenSHC `DebugVisualiser` provides RViz markers for: robot model, walk plane, tip trajectories, terrain estimate, bezier curves, default/target tip positions, walkspace, workspace, stride, tip force, joint torques, gravity. All 14 RViz publishers are **removed** in HexaMotion.

HexaMotion substitutes: `CoxaTelemetry` struct in `LocomotionSystem` (compile-time enabled via `TESTING_ENABLED`), `CollisionDiagnostics` class (static utility for workspace overlap analysis), per-leg `DebugState` in `LegStepper` (compile-time via `COXA_STRIDE_TESTING`).

### 12. HexaMotion-Specific Modules (no OpenSHC equivalent)

| Module                                        | Description                                                                                                                                                                                                              |
| :-------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `LocomotionSystem`                            | Top-level orchestrator replacing ROS node. Owns legs, coordinates controllers, manages HAL interfaces (IMU/FSR/Servo), error handling, sensor batch updates.                                                             |
| `CartesianVelocityController`                 | Maps Cartesian body velocity commands → per-joint servo speeds. Includes `VelocityScaling`, `GaitSpeedModifiers`, adaptive workspace-based scaling.                                                                      |
| `CollisionDiagnostics`                        | Static utility for analyzing workspace overlap between adjacent legs and recommending hexagon radius.                                                                                                                    |
| `TerrainAdaptation`                           | Consolidates terrain logic (step plane detection, touchdown events, gravity estimation, walk plane estimation, proactive/reactive adaptation) that in OpenSHC was scattered across `Leg`, `Model`, and `PoseController`. |
| `IMUAutoPose`                                 | Dedicated IMU-based auto-posing with multiple modes (level, inclination, adaptive, custom), separate from the phase-based auto-pose system.                                                                              |
| `ManualBodyPoseController`                    | Dedicated class for manual body pose with extended modes (translation, rotation, individual leg, body height, combined, custom), presets, and quaternion support.                                                        |
| `AnalyticRobotModel`                          | Supplementary kinematics class providing purely analytic FK/Jacobian calculations (no iterative solver).                                                                                                                 |
| `WorkspaceAnalyzer`                           | Dedicated workspace management (generation, validation, walkspace computation, scaling) consolidating OpenSHC's scattered workspace logic from `Model` and `Leg`.                                                        |
| `VelocityLimits`                              | 360-bearing velocity/acceleration limit system replacing OpenSHC's 4 separate `LimitMap` objects with a unified class including overshoot compensation and safety margins.                                               |
| `GaitConfigFactory` / `BodyPoseConfigFactory` | Factory pattern for gait and body pose configuration, replacing YAML parameter loading.                                                                                                                                  |

---

## Summary of Functional Gaps

### True Gaps (functionality absent from HexaMotion)

| #   | Feature                                   | OpenSHC Implementation                                                                                                                                                                     | HexaMotion Status   | Classification                                                                                                                                                          |
| :-- | :---------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------------ | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | **External Planner**                      | `executePlan()`, `targetConfigurationCallback`, `targetBodyPoseCallback`, `targetTipPoseCallback`, `PlannerMode` enum                                                                      | **Not implemented** | By design (MCU target).                                                                                                                                                 |
| 2   | **Dynamic Parameter Adjustment**          | `AdjustableParameter` struct, `ParameterSelection` enum, `adjustParameter()`, `parameterSelectionCallback`, `parameterAdjustCallback`, `dynamicParameterCallback`, ROS dynamic_reconfigure | **Not implemented** | By design (static config).                                                                                                                                              |
| 3   | **Tip Torque Vectors**                    | `tip_torque_calculated_`, `tip_torque_measured_` (3D torque vectors)                                                                                                                       | **Not implemented** | Torque vectors not tracked; scalar FSR only.                                                                                                                            |
| 4   | **AMBLE_GAIT**                            | `GaitDesignation::AMBLE_GAIT` with YAML-configured phase offsets                                                                                                                           | **Not implemented** | Can be added to `GaitConfigFactory`.                                                                                                                                    |
| 5   | **TF Transform for External Targets**     | `ExternalTarget::transform_` (TF2 frame-to-frame transform lookup)                                                                                                                         | **Not implemented** | By design (no TF2 on MCU).                                                                                                                                              |
| 6   | **Joint/Link/Tip Object Model**           | Per-joint/link/tip classes with per-link DH objects and per-tip transforms                                                                                                                 | **Not implemented** | Design decision: flattened to `JointAngles` + `Point3D` (per-joint telemetry exists, object model absent).                                                              |
| 7   | **updateTipRotation()**                   | Tip rotation during swing (orthogonal to walk plane for sensor alignment)                                                                                                                  | **Not implemented** | Not needed for 3DOF legs without tip rotation joints.                                                                                                                   |
| 8   | **numberToString\<T\>()**                 | String conversion utility                                                                                                                                                                  | **Not implemented** | Minor (Arduino `String()` used).                                                                                                                                        |
| 9   | **Model copy constructor**                | `Model(shared_ptr<Model>)` for workspace generation (search model)                                                                                                                         | **Not implemented** | Not needed (workspace generation uses centralized `WorkspaceAnalyzer`).                                                                                                 |
| 10  | **getLegByIDName()**                      | String-based leg lookup                                                                                                                                                                    | **Not implemented** | Minor (index-based access only).                                                                                                                                        |
| 11  | **Auto-navigation mode**                  | `auto_navigation_mode` input / syropod_auto_navigation integration                                                                                                                         | **Not implemented** | By design (no ROS navigation stack on MCU).                                                                                                                             |
| 12  | **Bezier through-control-point variants** | `quadraticBezierCurveThroughControlPoint`, `cubicBezierCurveThroughControlPoint`, `quarticBezierCurveThroughControlPoint`                                                                  | **Not implemented** | Used in some OpenSHC pose transitions for exact curve-through-waypoint interpolation.                                                                                   |
| 13  | **stringFormat\<Args\>()**                | Variadic `snprintf`-based string format utility                                                                                                                                            | **Not implemented** | Minor (Arduino environment uses `String()` and `sprintf` directly).                                                                                                     |
| 14  | **Startup acquisition timeout**           | `ACQUISTION_TIME` (10 s) in `main.cpp` — waits for initial joint state callback before starting control loop                                                                               | **Not implemented** | Architectural difference: HexaMotion uses direct HAL polling (`IServoInterface`) rather than ROS topic callbacks; no equivalent readiness gate before locomotion start. |
| 15  | **transitionStance() as distinct method** | `PoseController::transitionStance(time)` — external-target-driven stance transitions with gravity-aligned tips, reset of external targets                                                  | **Partial**         | `stepToNewStance()` exists (different function); `transitionStance` logic not fully replicated.                                                                         |
| 16  | **origin_walk_plane_pose\_**              | Origin pose for interpolating walk plane pose transitions                                                                                                                                  | **Not implemented** | HexaMotion uses 5-point Bézier control nodes for walk plane pose interpolation instead (`walk_plane_position_nodes_[5]`, `walk_plane_rotation_nodes_[5]`).              |

### Corrections to Previous Report

| Previous Claim                                                  | Corrected Status                                                                                                                                                                                                                      |
| :-------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| "`updateStance()` is missing"                                   | **Present** as `applyAutoPoseToDesiredTips()` (comment cites OpenSHC equivalence).                                                                                                                                                    |
| "`updateInclinationPose()` is not present as a discrete method" | **Present** inline in `updateCurrentPose()` when `inclination_pose_enabled_` is true.                                                                                                                                                 |
| "`legsBearingLoad()` has no 1:1 equivalent"                     | **Present** as public API `BodyPoseController::legsBearingLoad()` and wrapper `LocomotionSystem::legsBearingLoad()`.                                                                                                                  |
| "AutoPoser system not replicated"                               | **Present** split across `IMUAutoPose` (IMU-based) and `BodyPoseController` auto-pose subsystem (phase-based with `AutoPoseConfiguration`).                                                                                           |
| "Workspace methods not present as per-leg methods"              | **Present** centralized in `WorkspaceAnalyzer` (all 4 OpenSHC equivalents: `generateWorkspace`, `getWorkplane`, `makeReachable`, workspace polyhedron).                                                                               |
| "`updateManualPose()` equivalent is not present"                | **Present** via `ManualBodyPoseController::processInput()` → `BodyPoseController::setManualPoseInput()`.                                                                                                                              |
| "Full OpenSHC pose composition pipeline is not replicated 1:1"  | **Replicated** in `updateCurrentPose()`: walk_plane → manual → inclination → IMU → tip_align → ik_error → default (auto pose applied per-leg). Only `admittance_pose*` is architectural different (per-leg delta vs body-level pose). |
| "`Pose::Undefined()` / `Pose::isValid()` missing"               | **Present** in `Pose` (NaN sentinel + finite checks).                                                                                                                                                                                 |
| "Force/torque estimation missing"                               | **Force estimation present** (`calculateTipForce()`); **torque vectors still absent**.                                                                                                                                                |
| "Per-leg cartesian pose input missing"                          | **Present** via `StateController::setLegTipPose()` and consumed in `LocomotionSystem::update()` manual path.                                                                                                                          |
| "Per-joint state tracking absent"                               | **Present** as per-joint velocity/effort fields in `Leg`; joint/link/tip object model still absent.                                                                                                                                   |
| "`UNDEFINED_POSITION`/`UNDEFINED_ROTATION` use zeros"           | **Incorrect**: `Pose::Undefined()` uses NaN sentinel for both position and rotation.                                                                                                                                                  |

### Minor Logic Deviations

| Feature                | OpenSHC Behavior                                                            | HexaMotion Behavior                                                                                                             | Impact                                                                 |
| :--------------------- | :-------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------ | :--------------------------------------------------------------------- |
| **Auto-Pose Appl.**    | Adds `auto_pose` to Body, subtracts from Legs ($Body+Auto$, $Leg-Auto$).    | Does not add to Body, subtracts from Legs ($Leg-Auto$).                                                                         | **None**. Net relative motion is identical ($Body - Leg + Auto$).      |
| **Default Pose Usage** | Adds `default_pose_` (zero-moment offset) to active body pose in main loop. | Calculates `default_pose_` but uses it primarily for manipulation states; not added to `body_pose_current_` in standard update. | **Low**. Zero-moment balancing may be inactive in standard walking.    |
| **Admittance Appl.**   | `admittance_pose_` applied as a body pose offset.                           | `admittance_delta_` applied per-leg to the target tip position.                                                                 | **Low**. Architectural change for efficiency; functional outcome same. |

### Items Confirmed as Non-Gaps (by design)

| Feature                                    | Reason                                                                    |
| :----------------------------------------- | :------------------------------------------------------------------------ |
| ROS publishers/subscribers (26+ callbacks) | Replaced by direct API (`setDesiredVelocity`, `requestRobotState`, etc.). |
| RViz debug visualization                   | Replaced by `CoxaTelemetry` and `CollisionDiagnostics`.                   |
| TF2 frame transforms                       | Not available on MCU targets.                                             |
| YAML configuration files                   | Replaced by `Parameters` struct + factory patterns.                       |
| 8-leg support                              | Hexapod-only (6 legs, `NUM_LEGS=6`).                                      |
| AMBLE_GAIT                                 | Not supported with current morphology/constraints.                        |
| `dynamic_reconfigure` server               | No ROS dependency.                                                        |
| Primary/Secondary leg distinction          | Simplified to per-leg API by index.                                       |

### Documentation Notes (Parity Clarifications)

- Admittance delta is applied per leg in the leg-step pipeline (see [src/leg_poser.cpp](src/leg_poser.cpp)), which mirrors OpenSHC's leg-level `admittance_delta_` application (see [OpenSHC/src/model.cpp](OpenSHC/src/model.cpp)).
- `AMBLE_GAIT` remains unsupported with the current morphology constraints; parity is documented as out of scope (see [AGENTS.md](AGENTS.md)).
- `updateTipRotation()` omission is intentional for 3DOF legs with no tip rotation joints (see [src/leg_stepper.cpp](src/leg_stepper.cpp)).
- Joint/Link/Tip flat model rationale: MCU memory constraints and 3DOF-only kinematics (see [src/leg.h](src/leg.h) and [src/robot_model.h](src/robot_model.h)).
- Per-leg cartesian pose input is supported via `StateController::setLegTipPose()` and consumed in `LocomotionSystem::update()` manual path.

---

## Todo List for 1:1 Logic Parity

### Priority 1: True Functional Gaps

- [x] **Pose::isValid() / Pose::Undefined()**: Implemented in `Pose` (see `robot_model.h`).
- [x] **Force/Torque Vector Estimation**: `Leg::calculateTipForce()` implemented with Jacobian-transpose; effort data sourced via `IServoInterface`. Torque vector remains untracked in 3DOF model.
- [x] **Per-Leg Cartesian Pose Input**: `StateController::setLegTipPose(int, Point3D)` + `LocomotionSystem` manual update path.
- [x] **Admittance Pose Equivalence**: Documented (per-leg admittance delta applied after pose inverse; matches OpenSHC leg-level application ordering).

### Priority 2: Optional Enhancements

- [x] **legsBearingLoad() as Public API**: Exposed via `LocomotionSystem::legsBearingLoad()`.
- [x] **Expose Current Body Pose**: Added `BodyPoseController::getCurrentBodyPose()` and `LocomotionSystem::getCurrentBodyPose()`.
- [x] **Per-Joint State Tracking**: `Leg` stores desired/current joint velocity and effort, populated from `IServoInterface` when available.
- [x] **Stub Methods for Planner**: `StateController::executePlan()` returns "not supported" and `PlannerMode` exists.
- [x] **AdjustableParameter Light**: `LocomotionSystem::setParameter()` supports `step_frequency`, `swing_height`, `stance_span_modifier`.

### Priority 3: Documentation-Only

- [x] Document `admittance_pose_` → per-leg `admittance_delta_` architectural difference.
- [x] Document `AMBLE_GAIT` omission rationale (unsupported with current morphology; see AGENTS.md).
- [x] Document `updateTipRotation()` omission (3DOF: no tip rotation joints).
- [x] Document `Joint/Link/Tip` flat model rationale (MCU memory constraints, 3DOF-only).

### Validation Tasks

- [ ] Run `BezierValidationTest` against OpenSHC reference curves.
- [ ] Verify `generateLimits` produces identical bearing velocity maps for standard gait parameters.
- [ ] Compare `updateWalk()` state machine transitions step-by-step with OpenSHC.
- [ ] Compare `updateCurrentPose()` pose composition order with OpenSHC (walk_plane → manual → inclination → IMU/auto → tip_align → ik_error → default).
- [ ] Compare `AdmittanceController` ODE output with OpenSHC boost::odeint RK4 output for identical inputs.

### Priority 4: New Gaps Found in Validation Pass (2026-02-10)

- [x] **Bezier through-control-point functions**: Implemented `quadraticBezierCurveThroughControlPoint`, `cubicBezierCurveThroughControlPoint`, `quarticBezierCurveThroughControlPoint` in `math_utils.h`. These adjust control nodes so the curve interpolates _through_ the specified point rather than merely being pulled toward it. Degenerate-case guards (`fabs(denom) < 1e-12`) replace OpenSHC's `ROS_WARN` fallback.
- [x] **transitionStance() full logic**: Implemented `BodyPoseController::transitionStance(Leg[], double)` in `body_pose_controller.cpp`. Iterates all legs, composes target from `ExternalTarget.transform + ExternalTarget.pose`, calls `LegPoser::stepToPosition()` with body pose, applies IK, tracks minimum progress, and resets external targets on completion. Gravity-aligned tips omitted (not applicable to 3DOF MCU target).
- [x] **Startup acquisition timeout**: Added `LocomotionSystem::attemptJointAcquisition()` which polls `IServoInterface::getJointAngle()` for all joints up to `ACQUISITION_TIMEOUT_S` (10 s), equivalent to OpenSHC's `ACQUISTION_TIME` spin loop. Called during `initialize()` with fallback to default positions on timeout. Added `jointPositionsInitialised()` public accessor.
- [x] **origin*walk_plane_pose***: Added `origin_walk_plane_pose_` member to `BodyPoseController`. Bézier 5-node transitions now start from `origin_walk_plane_pose_` (not mid-transition `walk_plane_pose_`), matching OpenSHC's `origin.interpolate(c, new)` semantics. Updated on transition completion and direct assignment; included in `resetAllPosing()`.

---

## Audit Summary (2026-02-10 Validation Pass)

### Methodology

All symbols cataloged from OpenSHC (16 source files: 9 headers + 7 implementation files) and HexaMotion (45 source files) were compared exhaustively. Specific claims in the GAP report were verified against actual source code.

### OpenSHC Symbol Counts

- 14 enums, 6 structs, 14 classes
- ~290 public methods, ~200 member variables
- ~30 free functions, 22 constants/defines, 11 typedefs

### HexaMotion Symbol Counts

- ~20 enums, ~50 structs, 24 classes + 3 abstract interfaces
- ~500+ public methods, ~200+ private members
- ~50 free functions, ~100 constants

### Audit Results

| Category                  | Count | Details                                                                                                                                                                                    |
| :------------------------ | :---- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Existing gaps confirmed   | 11    | All 11 True Gaps from the original report validated against source.                                                                                                                        |
| New gaps discovered       | 5     | Gaps #12–#16 added (Bezier through-point, stringFormat, acquisition timeout, transitionStance, origin*walk_plane_pose*).                                                                   |
| Corrections applied       | 12    | All corrections from the original report validated; no new false corrections found.                                                                                                        |
| Report inaccuracies fixed | 4     | `GaitDesignation` ordinal values noted; `UNASSIGNED_VALUE` description refined; `origin_walk_plane_pose_` downgraded from ✅ to Partial; Bezier function table added.                      |
| Constants not in report   | 3     | `JOINT_LIMIT_COST_WEIGHT` → `IK_JOINT_LIMIT_COST_WEIGHT` ✅; `HALF_BODY_DEPTH` → `HALF_BODY_DEPTH_MM` ✅; `TRANSITION_STEP_THRESHOLD` ✅; `IMU_POSING_DEADBAND` ✅. All present, not gaps. |
| TODO DONE items validated | 10    | All items from `OpenSHC_GAP_TODO_DONE.md` confirmed implemented in source.                                                                                                                 |

### TODO DONE Cross-Reference

All items listed in [OpenSHC_GAP_TODO_DONE.md](OpenSHC_GAP_TODO_DONE.md) were verified against source code:

| TODO DONE Item                     | Source Verification                                                                           | Status      |
| :--------------------------------- | :-------------------------------------------------------------------------------------------- | :---------- |
| Pose validation/sentinel           | `Pose::isValid()` and `Pose::Undefined()` in `robot_model.h` use NaN sentinels                | ✅ Verified |
| Force estimation (Jacobian)        | `Leg::calculateTipForce()` in `leg.cpp`; `tip_force_calculated_` member                       | ✅ Verified |
| Per-leg cartesian pose input       | `StateController::setLegTipPose(int, Point3D)` in `state_controller.h`                        | ✅ Verified |
| Admittance delta equivalence       | `LegPoser::admittance_delta_` in `leg_poser.h`; deadband + NaN sanitization                   | ✅ Verified |
| legsBearingLoad public API         | `LocomotionSystem::legsBearingLoad()` in `locomotion_system.h`                                | ✅ Verified |
| Current body pose getter           | `BodyPoseController::getCurrentBodyPose()` + `LocomotionSystem::getCurrentBodyPose()`         | ✅ Verified |
| Per-joint velocity/effort tracking | `Leg::getJointVelocity()`, `setJointVelocity()` in `leg.h`                                    | ✅ Verified |
| Planner stubs                      | `PlannerMode` enum in `state_controller.h`; `executePlan()` returns unsupported               | ✅ Verified |
| Parameter adjustment light         | `LocomotionSystem::setParameter(string, double)` in `locomotion_system.cpp`                   | ✅ Verified |
| Documentation of omissions         | AMBLE*GAIT, updateTipRotation, Joint/Link/Tip, admittance_pose* all documented in this report | ✅ Verified |
