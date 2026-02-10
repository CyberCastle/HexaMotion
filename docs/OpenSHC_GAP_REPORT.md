# OpenSHC Parity Gap Report (HexaMotion)

> **Last updated**: 2026-02-10 — Full method-level audit against OpenSHC source.

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

| OpenSHC Enum         | OpenSHC Values                                                                                             | HexaMotion Equivalent                                 | HexaMotion Values                                                                                                                          | Status                                                                                                                                                               |
| :------------------- | :--------------------------------------------------------------------------------------------------------- | :---------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SystemState`        | `SUSPENDED`, `OPERATIONAL`                                                                                 | Merged into `SystemState` in `hexamotion_constants.h` | `SYSTEM_UNKNOWN`, `SYSTEM_PACKED`, `SYSTEM_READY`, `SYSTEM_RUNNING`                                                                        | **Redesigned**: OpenSHC's two-state toggle replaced by four-state lifecycle. SUSPENDED/OPERATIONAL semantics handled by `system_enabled` flag in `LocomotionSystem`. |
| `RobotState`         | `PACKED`, `READY`, `RUNNING`, `UNKNOWN=-1`, `OFF=-2`                                                       | `RobotState` in `state_controller.h`                  | `ROBOT_PACKED`, `ROBOT_READY`, `ROBOT_RUNNING`, `ROBOT_UNKNOWN=-1`, `ROBOT_OFF=-2`                                                         | ✅ 1:1 (renamed)                                                                                                                                                     |
| `GaitDesignation`    | `WAVE_GAIT`, `AMBLE_GAIT`, `RIPPLE_GAIT`, `TRIPOD_GAIT`                                                    | `GaitType` in `gait_types.h`                          | `WAVE_GAIT`, `RIPPLE_GAIT`, `TRIPOD_GAIT`, `NO_GAIT`, `METACHRONAL_GAIT`, `ADAPTIVE_GAIT`                                                  | `AMBLE_GAIT` not supported due to current morphology. `METACHRONAL_GAIT` and `ADAPTIVE_GAIT` remain HexaMotion extensions.                                           |
| `PosingMode`         | `NO_POSING`, `X_Y_POSING`, `PITCH_ROLL_POSING`, `Z_YAW_POSING`, `EXTERNAL_POSING`                          | `PosingMode` in `state_controller.h`                  | `POSING_NONE`, `POSING_X_Y`, `POSING_PITCH_ROLL`, `POSING_Z_YAW`, `POSING_EXTERNAL`                                                        | ✅ 1:1 (renamed)                                                                                                                                                     |
| `CruiseControlMode`  | `CRUISE_CONTROL_OFF`, `CRUISE_CONTROL_ON`, `CRUISE_CONTROL_EXTERNAL=-1`                                    | `CruiseControlMode` in `state_controller.h`           | Same names                                                                                                                                 | ✅ 1:1                                                                                                                                                               |
| `PlannerMode`        | `PLANNER_MODE_OFF`, `PLANNER_MODE_ON`                                                                      | `PlannerMode` in `state_controller.h`                 | `PLANNER_MODE_OFF`, `PLANNER_MODE_ON`                                                                                                      | ✅ Stubbed (returns not supported on MCU).                                                                                                                           |
| `LegState`           | `WALKING`, `MANUAL`, `WALKING_TO_MANUAL=-1`, `MANUAL_TO_WALKING=-2`                                        | `LegState` in `gait_types.h`                          | `LEG_WALKING`, `LEG_MANUAL`, `LEG_WALKING_TO_MANUAL=-1`, `LEG_MANUAL_TO_WALKING=-2`                                                        | ✅ 1:1 (renamed)                                                                                                                                                     |
| `WalkState`          | `STARTING`, `MOVING`, `STOPPING`, `STOPPED`                                                                | `WalkState` in `gait_types.h`                         | `WALK_STARTING`, `WALK_MOVING`, `WALK_STOPPING`, `WALK_STOPPED`                                                                            | ✅ 1:1 (renamed)                                                                                                                                                     |
| `StepState`          | `SWING`, `STANCE`, `FORCE_STANCE`, `FORCE_STOP`                                                            | `StepState` in `gait_types.h`                         | `STEP_SWING`, `STEP_STANCE`, `STEP_FORCE_STANCE`, `STEP_FORCE_STOP`                                                                        | ✅ 1:1 (renamed)                                                                                                                                                     |
| `PosingState`        | `POSING`, `STOP_POSING`, `POSING_COMPLETE`                                                                 | `PosingState` in `state_controller.h`                 | `POSE_POSING`, `POSE_STOP_POSING`, `POSE_POSING_COMPLETE`                                                                                  | ✅ 1:1 (renamed)                                                                                                                                                     |
| `PoseResetMode`      | `NO_RESET`, `Z_AND_YAW_RESET`, `X_AND_Y_RESET`, `PITCH_AND_ROLL_RESET`, `ALL_RESET`, `IMMEDIATE_ALL_RESET` | `PoseResetMode` in `state_controller.h`               | `POSE_RESET_NONE`, `POSE_RESET_Z_AND_YAW`, `POSE_RESET_X_AND_Y`, `POSE_RESET_PITCH_AND_ROLL`, `POSE_RESET_ALL`, `POSE_RESET_IMMEDIATE_ALL` | ✅ 1:1 (renamed)                                                                                                                                                     |
| `LegDesignation`     | `LEG_0`–`LEG_7`, `LEG_UNDESIGNATED=-1`                                                                     | `LegDesignation` in `hexamotion_constants.h`          | `LEG_0`–`LEG_5`, `LEG_UNDESIGNATED=-1`                                                                                                     | ✅ Reduced to 6 (by design).                                                                                                                                         |
| `ParameterSelection` | 10 values (step_freq, swing_height, etc.)                                                                  | —                                                     | —                                                                                                                                          | **Gap**: No dynamic parameter adjustment system.                                                                                                                     |
| `SequenceSelection`  | `START_UP`, `SHUT_DOWN`                                                                                    | `SequenceType` in `state_controller.h`                | `SEQUENCE_START_UP`, `SEQUENCE_SHUT_DOWN`, `SEQUENCE_PACK`, `SEQUENCE_UNPACK`                                                              | ✅ Extended (pack/unpack added).                                                                                                                                     |

#### Structs — Detailed Mapping

| OpenSHC                                              | HexaMotion                                                   | Status                                        |
| :--------------------------------------------------- | :----------------------------------------------------------- | :-------------------------------------------- |
| `Parameter<T>` (ROS param server binding)            | —                                                            | **Removed** (by design: no ROS).              |
| `AdjustableParameter` (current/min/max/step + ROS)   | —                                                            | **Gap**: No runtime parameter tuning system.  |
| `Parameters` struct (~70 fields, all `Parameter<T>`) | `Parameters` struct in `robot_model.h` (flat numeric fields) | ✅ Equivalent data, different storage format. |

#### Constants — Detailed Mapping

| OpenSHC Constant                   | HexaMotion Equivalent                      | Location                                                 |
| :--------------------------------- | :----------------------------------------- | :------------------------------------------------------- |
| `UNASSIGNED_VALUE`                 | —                                          | **Gap**: No sentinel value; uses validity flags instead. |
| `UNDEFINED_POSITION`               | —                                          | **Gap**: Uses `Point3D(0,0,0)` or explicit checks.       |
| `UNDEFINED_ROTATION`               | —                                          | **Gap**: Uses `Quaterniond::Identity()`.                 |
| `GRAVITY_ACCELERATION`             | Computed from IMU or hardcoded per context | Distributed.                                             |
| `PROGRESS_COMPLETE` (100)          | `PROGRESS_COMPLETE` (100)                  | `state_controller.h` ✅                                  |
| `THROTTLE_PERIOD` (5)              | `THROTTLE_PERIOD` (1.0f)                   | `state_controller.h` — value changed.                    |
| `IK_TOLERANCE` (0.005)             | `IK_TOLERANCE`                             | `hexamotion_constants.h` ✅                              |
| `DLS_COEFFICIENT` (0.02)           | `IK_DLS_COEFFICIENT`                       | `hexamotion_constants.h` ✅                              |
| `BEARING_STEP` (45)                | `BEARING_STEP`                             | `hexamotion_constants.h` ✅                              |
| `MAX_POSITION_DELTA` (0.002)       | `MAX_POSITION_DELTA`                       | `hexamotion_constants.h` ✅                              |
| `MAX_WORKSPACE_RADIUS` (1.0)       | `MAX_WORKSPACE_RADIUS`                     | `hexamotion_constants.h` ✅                              |
| `WORKSPACE_LAYERS` (10)            | `WORKSPACE_LAYERS`                         | `hexamotion_constants.h` ✅                              |
| `JOINT_TOLERANCE` (0.01)           | `JOINT_TOLERANCE` (0.1f)                   | `state_controller.h` — value relaxed.                    |
| `TIP_TOLERANCE` (0.01)             | `TIP_TOLERANCE`                            | `hexamotion_constants.h` ✅                              |
| `SAFETY_FACTOR` (0.15)             | `SAFETY_FACTOR`                            | `hexamotion_constants.h` ✅                              |
| `PACK_TIME` (2.0)                  | `PACK_TIME` (2.0f)                         | `state_controller.h` ✅                                  |
| `MAX_MANUAL_LEGS` (2)              | `MAX_MANUAL_LEGS` (2)                      | `state_controller.h` ✅                                  |
| `HORIZONTAL_TRANSITION_TIME` (1.0) | `HORIZONTAL_TRANSITION_TIME`               | `hexamotion_constants.h` ✅                              |
| `VERTICAL_TRANSITION_TIME` (3.0)   | `VERTICAL_TRANSITION_TIME`                 | `hexamotion_constants.h` ✅                              |
| `STABILITY_THRESHOLD` (100)        | `DEFAULT_STABILITY_THRESHOLD`              | `hexamotion_constants.h` ✅                              |
| `ADMITTANCE_DEADBAND` (0.0)        | Inline in `AdmittanceController`           | ✅                                                       |

### 2. standard_includes.h (OpenSHC) vs HexaMotion utility/constants

**Status**: Refactored — Logic Preserved

| OpenSHC Function                | HexaMotion Equivalent       | Location       | Status                                    |
| :------------------------------ | :-------------------------- | :------------- | :---------------------------------------- |
| `degreesToRadians()`            | `degreesToRadians()`        | `math_utils.h` | ✅                                        |
| `radiansToDegrees()`            | `radiansToDegrees()`        | `math_utils.h` | ✅                                        |
| `mod<T>()`                      | `mod<T>()`                  | `math_utils.h` | ✅                                        |
| `sqr<T>()`                      | `sqr<T>()`                  | `math_utils.h` | ✅                                        |
| `sign<T>()`                     | `sign<T>()`                 | `math_utils.h` | ✅                                        |
| `roundToInt()`                  | `roundToInt()`              | `math_utils.h` | ✅                                        |
| `roundToEvenInt()`              | `roundToEvenInt()`          | `math_utils.h` | ✅                                        |
| `clamped<T>(value, min, max)`   | `clamped()`                 | `math_utils.h` | ✅                                        |
| `clamped<T>(vector, magnitude)` | `clamped()`                 | `math_utils.h` | ✅                                        |
| `setPrecision(double, int)`     | `setPrecision()`            | `math_utils.h` | ✅                                        |
| `setPrecision(Vector3d, int)`   | `setPrecision()`            | `math_utils.h` | ✅                                        |
| `smoothStep()`                  | `smoothStep()`              | `math_utils.h` | ✅                                        |
| `getProjection()`               | `getProjection()`           | `math_utils.h` | ✅                                        |
| `getRejection()`                | `getRejection()`            | `math_utils.h` | ✅                                        |
| `interpolate<T>()`              | `interpolate<T>()`          | `math_utils.h` | ✅                                        |
| `correctRotation()`             | `correctRotation()`         | `math_utils.h` | ✅                                        |
| `eulerAnglesToQuaternion()`     | `eulerAnglesToQuaternion()` | `math_utils.h` | ✅                                        |
| `quaternionToEulerAngles()`     | `quaternionToEulerAngles()` | `math_utils.h` | ✅                                        |
| `numberToString<T>()`           | —                           | —              | **Gap** (minor utility, `String()` used). |

### 3. Pose class (OpenSHC pose.h) vs HexaMotion Pose

**Status**: Functional Equivalent

| OpenSHC Pose Method                   | HexaMotion Equivalent         | Status                                             |
| :------------------------------------ | :---------------------------- | :------------------------------------------------- |
| `Pose(Vector3d, Quaterniond)`         | `Pose(Vector3d, Quaterniond)` | ✅                                                 |
| `Pose(geometry_msgs::Pose)`           | —                             | **Removed** (no ROS).                              |
| `Pose(geometry_msgs::Transform)`      | —                             | **Removed** (no ROS).                              |
| `isValid()`                           | —                             | **Gap**: No validity check.                        |
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
| `Undefined()`                         | —                             | **Gap**: No undefined sentinel.                    |

### 4. Model / Leg / Joint / Link / Tip

**Status**: Architectural Divergence — Core Logic Preserved

#### Model → RobotModel

| OpenSHC Model Method                    | HexaMotion Equivalent                                          | Location                           | Status                                           |
| :-------------------------------------- | :------------------------------------------------------------- | :--------------------------------- | :----------------------------------------------- |
| `Model(params, debug_visualiser)`       | `RobotModel(const Parameters&)`                                | `robot_model.h`                    | ✅ (no debug param)                              |
| `Model(model)` (copy ctor)              | —                                                              | —                                  | **Gap** (not needed: stateless + external legs). |
| `getLegContainer()`                     | — (array in `LocomotionSystem`)                                | —                                  | **Redesigned**: Legs external.                   |
| `getLegCount()`                         | `NUM_LEGS` constant (6)                                        | `hexamotion_constants.h`           | ✅ (hardcoded).                                  |
| `getCurrentPose()` / `setCurrentPose()` | `BodyPoseController::body_pose_current_`                       | `body_pose_controller.h`           | ✅ (moved).                                      |
| `getDefaultPose()` / `setDefaultPose()` | `BodyPoseController::walk_plane_pose_`                         | `body_pose_controller.h`           | ✅ (moved).                                      |
| `getTimeDelta()`                        | `getTimeDelta()`                                               | `robot_model.h`                    | ✅                                               |
| `generate()`                            | — (distributed initialization)                                 | —                                  | **Redesigned**.                                  |
| `initLegs()`                            | `LocomotionSystem::initialize()` + per-leg `Leg::initialize()` | `locomotion_system.cpp`, `leg.cpp` | ✅                                               |
| `legsBearingLoad()`                     | Lambda in `BodyPoseController::executeSequenceInternal()`      | `body_pose_controller.cpp`         | ✅ (logic present, not a public API).            |
| `getLegByIDNumber()`                    | `LocomotionSystem::getLeg(int)`                                | `locomotion_system.h`              | ✅                                               |
| `getLegByIDName()`                      | —                                                              | —                                  | **Gap** (minor: index-only access).              |
| `getImuData()` / `setImuData()`         | `BodyPoseController::setIMUData()` / `IMUAutoPose`             | `body_pose_controller.h`           | ✅ (moved).                                      |
| `updateDefaultConfiguration()`          | —                                                              | —                                  | **Gap** (no dynamic reconfiguration).            |
| `generateWorkspaces()`                  | `WorkspaceAnalyzer::generateWorkspace()`                       | `workspace_analyzer.h`             | ✅                                               |
| `updateModel()`                         | `LocomotionSystem::applyInverseKinematicsToAllLegs()`          | `locomotion_system.cpp`            | ✅                                               |
| `estimateGravity()`                     | `TerrainAdaptation::estimateGravity()`                         | `terrain_adaptation.h`             | ✅ (moved).                                      |

#### Leg

| OpenSHC Leg Feature                                               | HexaMotion Equivalent                                              | Status                                                                                                               |
| :---------------------------------------------------------------- | :----------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------- |
| `Joint`/`Link`/`Tip` sub-objects                                  | —                                                                  | **Gap**: Flattened into `JointAngles` + `Point3D`. Per-joint desired/current position/velocity/effort fields absent. |
| `joint_container_` / `link_container_` / `tip_`                   | —                                                                  | **Gap**: No sub-object containers.                                                                                   |
| `workspace_` (per-leg)                                            | `WorkspaceAnalyzer` (centralized)                                  | ✅ (moved).                                                                                                          |
| `group_` (stepping coordination)                                  | `BodyPoseController::tripod_leg_groups`                            | ✅ (moved).                                                                                                          |
| `leg_stepper_` / `leg_poser_`                                     | Separate classes, owned by `WalkController` / `BodyPoseController` | ✅ (moved).                                                                                                          |
| `leg_state_`                                                      | `leg_state_`                                                       | ✅                                                                                                                   |
| `admittance_delta_`                                               | `LegPoser::admittance_delta_`                                      | ✅ (moved).                                                                                                          |
| `virtual_mass_` / `virtual_stiffness_` / `virtual_damping_ratio_` | `AdmittanceController::LegAdmittanceState`                         | ✅ (moved to centralized controller).                                                                                |
| `admittance_state_` (ODE state vector)                            | `AdmittanceController::leg_dynamics_state_[]`                      | ✅ (moved).                                                                                                          |
| `tip_force_calculated_` / `tip_torque_calculated_`                | —                                                                  | **Gap**: No Jacobian-transpose force estimation from joint effort.                                                   |
| `tip_force_measured_` / `tip_torque_measured_`                    | `Leg::contact_force_` (scalar)                                     | **Partial**: Scalar FSR only, not 3D force/torque vectors.                                                           |
| `desired_tip_velocity_` / `current_tip_velocity_`                 | — in `Leg` (in `LegStepper::current_tip_velocity_`)                | **Partial**: Present in `LegStepper`, not in `Leg` directly.                                                         |
| `step_plane_pose_`                                                | `TerrainAdaptation::step_planes_[]`                                | ✅ (moved).                                                                                                          |
| `desired_tip_pose_` / `current_tip_pose_`                         | `desired_tip_position_` / `tip_position_` (Point3D)                | ✅ (simplified: 3DOF needs position only, not full Pose).                                                            |
| `generate()` / `init()`                                           | `Leg::initialize(const Pose&)`                                     | ✅                                                                                                                   |
| `generateWorkspace()`                                             | `WorkspaceAnalyzer::generateWalkspaceForLeg()`                     | ✅ (moved).                                                                                                          |
| `getWorkplane()`                                                  | `WorkspaceAnalyzer::getWorkplane()`                                | ✅ (moved).                                                                                                          |
| `makeReachable()`                                                 | `RobotModel::makeReachable()`                                      | ✅ (moved).                                                                                                          |
| `setDesiredTipPose(apply_delta)`                                  | `Leg::setDesiredTipPosition()` + `LegPoser::admittance_delta_`     | ✅ (split).                                                                                                          |
| `setDesiredTipVelocity()`                                         | — on Leg                                                           | **Gap** on Leg (tracked in `LegStepper`).                                                                            |
| `calculateTipForce()`                                             | —                                                                  | **Gap**: No Jacobian-transpose force estimation.                                                                     |
| `touchdownDetection()`                                            | `TerrainAdaptation::detectTouchdownEvents()`                       | ✅ (moved).                                                                                                          |
| `solveIK()`                                                       | `RobotModel::solveIK()` / `solveDeltaIK()`                         | ✅ (centralized).                                                                                                    |
| `updateJointPositions()`                                          | `Leg::setJointAngles()` + `updateTipPosition()`                    | ✅                                                                                                                   |
| `applyIK()`                                                       | `Leg::applyIK()` / `applyAdvancedIK()`                             | ✅                                                                                                                   |
| `applyFK()`                                                       | `Leg::updateTipPosition()`                                         | ✅                                                                                                                   |
| `generateDesiredJointStateMsg()`                                  | —                                                                  | **Removed** (no ROS messages).                                                                                       |
| ROS publishers                                                    | —                                                                  | **Removed** (no ROS).                                                                                                |

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

| OpenSHC Callback                                                            | HexaMotion Direct API                               | Status                                    |
| :-------------------------------------------------------------------------- | :-------------------------------------------------- | :---------------------------------------- |
| `systemStateCallback()`                                                     | `requestSystemState(SystemState)`                   | ✅                                        |
| `robotStateCallback()`                                                      | `requestRobotState(RobotState)`                     | ✅                                        |
| `bodyVelocityInputCallback()`                                               | `setDesiredVelocity(Vector2d, double)`              | ✅                                        |
| `bodyPoseInputCallback()`                                                   | `setDesiredPose(Vector3d, Vector3d)`                | ✅                                        |
| `posingModeCallback()`                                                      | `setPosingMode(PosingMode)`                         | ✅                                        |
| `poseResetCallback()`                                                       | `setPoseResetMode(PoseResetMode)`                   | ✅                                        |
| `gaitSelectionCallback()`                                                   | `changeGait(GaitType)`                              | ✅                                        |
| `cruiseControlCallback()`                                                   | `setCruiseControlMode(CruiseControlMode, Vector3d)` | ✅                                        |
| `plannerModeCallback()`                                                     | —                                                   | **Gap** (no planner).                     |
| `primaryLegSelectionCallback()`                                             | `requestLegToggle(int)`                             | ✅ (consolidated, any leg by index).      |
| `secondaryLegSelectionCallback()`                                           | `requestLegToggle(int)`                             | ✅ (no primary/secondary distinction).    |
| `primaryLegStateCallback()` / `secondaryLegStateCallback()`                 | `requestLegToggle(int)`                             | ✅ (consolidated).                        |
| `primaryTipVelocityInputCallback()` / `secondaryTipVelocityInputCallback()` | `setLegTipVelocity(int, Vector3d)`                  | ✅ (any-leg API).                         |
| `primaryTipPoseInputCallback()` / `secondaryTipPoseInputCallback()`         | —                                                   | **Gap**: No per-leg Cartesian pose input. |
| `parameterSelectionCallback()` / `parameterAdjustCallback()`                | —                                                   | **Gap**: No dynamic params.               |
| `dynamicParameterCallback()`                                                | —                                                   | **Gap**: No dynamic reconfigure.          |
| `imuCallback()`                                                             | `IIMUInterface` + `LocomotionSystem`                | ✅ (via HAL interface).                   |
| `jointStatesCallback()`                                                     | `IServoInterface` + `LocomotionSystem`              | ✅ (via HAL interface).                   |
| `tipStatesCallback()`                                                       | `IFSRInterface` + `LocomotionSystem`                | ✅ (via HAL interface).                   |
| `targetConfigurationCallback()`                                             | —                                                   | **Gap** (no planner).                     |
| `targetBodyPoseCallback()`                                                  | —                                                   | **Gap** (no planner).                     |
| `targetTipPoseCallback()`                                                   | —                                                   | **Gap** (no planner).                     |

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

| OpenSHC Sub-Pose                               | HexaMotion Equivalent                                | Status                                                                                                |
| :--------------------------------------------- | :--------------------------------------------------- | :---------------------------------------------------------------------------------------------------- |
| `manual_pose_`                                 | `manual_pose_`                                       | ✅                                                                                                    |
| `auto_pose_`                                   | `global_auto_pose_` + per-leg `LegPoser::auto_pose_` | ✅ (split).                                                                                           |
| `imu_pose_`                                    | `imu_pose_`                                          | ✅                                                                                                    |
| `inclination_pose_`                            | `inclination_pose_`                                  | ✅                                                                                                    |
| `admittance_pose_`                             | —                                                    | **Redesigned**: Admittance applied per-leg via `LegPoser::admittance_delta_`, not as body-level pose. |
| `default_pose_`                                | `default_pose_`                                      | ✅                                                                                                    |
| `ik_error_pose_`                               | `ik_error_pose_`                                     | ✅                                                                                                    |
| `tip_align_pose_` / `origin_tip_align_pose_`   | `tip_align_pose_` / `origin_tip_align_pose_`         | ✅                                                                                                    |
| `walk_plane_pose_` / `origin_walk_plane_pose_` | `walk_plane_pose_`                                   | ✅                                                                                                    |
| `rotation_absement_error_`                     | `rotation_absement_error_`                           | ✅                                                                                                    |
| `rotation_position_error_`                     | `rotation_position_error_`                           | ✅                                                                                                    |
| `rotation_velocity_error_`                     | `rotation_velocity_error_`                           | ✅                                                                                                    |

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

| #   | Feature                                 | OpenSHC Implementation                                                                                                                                                                                                                                                | HexaMotion Status   | Classification                                                                                               |
| :-- | :-------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------------ | :----------------------------------------------------------------------------------------------------------- |
| 1   | **External Planner**                    | `executePlan()`, `targetConfigurationCallback`, `targetBodyPoseCallback`, `targetTipPoseCallback`, `PlannerMode` enum                                                                                                                                                 | **Not implemented** | By design (MCU target).                                                                                      |
| 2   | **Dynamic Parameter Adjustment**        | `AdjustableParameter` struct, `ParameterSelection` enum, `adjustParameter()`, `parameterSelectionCallback`, `parameterAdjustCallback`, `dynamicParameterCallback`, ROS dynamic_reconfigure                                                                            | **Not implemented** | By design (static config).                                                                                   |
| 3   | **Pose::isValid() / Pose::Undefined()** | `Pose::isValid()` checks NaN, `Pose::Undefined()` returns sentinel                                                                                                                                                                                                    | **Not implemented** | Can be added for robustness.                                                                                 |
| 4   | **AMBLE_GAIT**                          | `GaitDesignation::AMBLE_GAIT` with YAML-configured phase offsets                                                                                                                                                                                                      | **Not implemented** | Can be added to `GaitConfigFactory`.                                                                         |
| 5   | **Force/Torque Vector Estimation**      | `Leg::calculateTipForce()` (Jacobian-transpose from joint effort), `tip_force_calculated_`, `tip_torque_calculated_`, per-joint `current_effort_`                                                                                                                     | **Not implemented** | Requires joint torque sensing hardware.                                                                      |
| 6   | **Per-Leg Cartesian Pose Input**        | `primaryTipPoseInputCallback`, `secondaryTipPoseInputCallback` → `Pose` input for manual leg state                                                                                                                                                                    | **Not implemented** | Can be added to `StateController`.                                                                           |
| 7   | **TF Transform for External Targets**   | `ExternalTarget::transform_` (TF2 frame-to-frame transform lookup)                                                                                                                                                                                                    | **Not implemented** | By design (no TF2 on MCU).                                                                                   |
| 8   | **Joint/Link/Tip Object Model**         | Per-joint `desired_position_`, `desired_velocity_`, `desired_effort_`, `current_position_`, `current_velocity_`, `current_effort_`, `min/max_position_`, `packed_positions_[]`, `offset_`, `max_angular_speed_`; per-link DH parameters as objects; per-tip transform | **Not implemented** | Design decision: flattened to `JointAngles` + `Point3D`. Per-joint state tracking beyond position is absent. |
| 9   | **updateTipRotation()**                 | Tip rotation during swing (orthogonal to walk plane for sensor alignment)                                                                                                                                                                                             | **Not implemented** | Not needed for 3DOF legs without tip rotation joints.                                                        |
| 10  | **numberToString\<T\>()**               | String conversion utility                                                                                                                                                                                                                                             | **Not implemented** | Minor (Arduino `String()` used).                                                                             |
| 11  | **Model copy constructor**              | `Model(shared_ptr<Model>)` for workspace generation (search model)                                                                                                                                                                                                    | **Not implemented** | Not needed (workspace generation uses centralized `WorkspaceAnalyzer`).                                      |
| 12  | **getLegByIDName()**                    | String-based leg lookup                                                                                                                                                                                                                                               | **Not implemented** | Minor (index-based access only).                                                                             |

### Corrections to Previous Report

| Previous Claim                                                  | Corrected Status                                                                                                                                                                                                                        |
| :-------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| "`updateStance()` is missing"                                   | **Present** as `applyAutoPoseToDesiredTips()` (comment cites OpenSHC equivalence).                                                                                                                                                      |
| "`updateInclinationPose()` is not present as a discrete method" | **Present** inline in `updateCurrentPose()` when `inclination_pose_enabled_` is true.                                                                                                                                                   |
| "`legsBearingLoad()` has no 1:1 equivalent"                     | **Present** as lambda in `BodyPoseController::executeSequenceInternal()`. Not a public API, but logic is 1:1.                                                                                                                           |
| "AutoPoser system not replicated"                               | **Present** split across `IMUAutoPose` (IMU-based) and `BodyPoseController` auto-pose subsystem (phase-based with `AutoPoseConfiguration`).                                                                                             |
| "Workspace methods not present as per-leg methods"              | **Present** centralized in `WorkspaceAnalyzer` (all 4 OpenSHC equivalents: `generateWorkspace`, `getWorkplane`, `makeReachable`, workspace polyhedron).                                                                                 |
| "`updateManualPose()` equivalent is not present"                | **Present** via `ManualBodyPoseController::processInput()` → `BodyPoseController::setManualPoseInput()`.                                                                                                                                |
| "Full OpenSHC pose composition pipeline is not replicated 1:1"  | **Replicated** in `updateCurrentPose()`: walk*plane → manual → inclination → IMU → auto → tip_align → ik_error → default. All sub-poses present. Only `admittance_pose*` is architectural different (per-leg delta vs body-level pose). |

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

---

## Todo List for 1:1 Logic Parity

### Priority 1: True Functional Gaps

- [ ] **Pose::isValid() / Pose::Undefined()**: Add `isValid()` (NaN/sentinel check) and `Undefined()` (sentinel factory) to `Pose` struct in `robot_model.h`. Low effort, improves defensive coding.
- [ ] **AMBLE_GAIT**: Add `AMBLE_GAIT` to `GaitType` enum and implement `GaitConfigFactory::createAmbleGaitConfig()` with appropriate phase offsets. Reference: OpenSHC amble gait config YAML.
- [ ] **Force/Torque Vector Estimation**: Add `calculateTipForce()` to `Leg` class using Jacobian-transpose method from OpenSHC (`J^T * tau = F`). Requires per-joint effort data from `IServoInterface`. Add `tip_force_calculated_` (Vector3d) member. Lower priority but needed for accurate admittance control.
- [ ] **Per-Leg Cartesian Pose Input**: Add `setLegTipPose(int, Point3D)` to `StateController` to support direct-positioning leg manipulation mode (OpenSHC's pose-based `updateManual` overload).
- [ ] **Admittance Pose Equivalence**: Verify that per-leg `LegPoser::admittance_delta_` application produces numerically equivalent results to OpenSHC's body-level `admittance_pose_` in the pose composition pipeline. Document any difference.

### Priority 2: Optional Enhancements

- [ ] **legsBearingLoad() as Public API**: Extract the lambda in `BodyPoseController::executeSequenceInternal()` to a public method on `LocomotionSystem` or `RobotModel` for external callers.
- [ ] **Expose Current Body Pose**: Add a public getter for `BodyPoseController::body_pose_current_` (or a `LocomotionSystem` passthrough) so callers can query the current composed body pose similar to OpenSHC's `Model::getCurrentPose()`.
- [ ] **Per-Joint State Tracking**: Add optional per-joint `desired_velocity_`, `current_velocity_`, `current_effort_` fields to `Leg` (populated from `IServoInterface` if available) to enable future force estimation.
- [ ] **Stub Methods for Planner**: Add no-op `executePlan()` / `PlannerMode` stubs in `StateController` returning "not supported" if strict API surface parity is required.
- [ ] **AdjustableParameter Light**: Implement a lightweight `setParameter(name, value)` API on `LocomotionSystem` (serial command interface) allowing runtime adjustment of `step_frequency`, `swing_height`, `stance_span_modifier` with min/max/step validation. Not full OpenSHC parity but preserves the concept.

### Priority 3: Documentation-Only

- [ ] Document `admittance_pose_` → per-leg `admittance_delta_` architectural difference and verify numerical equivalence.
- [ ] Document `AMBLE_GAIT` omission rationale or implementation plan.
- [ ] Document `updateTipRotation()` omission (3DOF: no tip rotation joints).
- [ ] Document `Joint/Link/Tip` flat model rationale (MCU memory constraints, 3DOF-only).

### Validation Tasks

- [ ] Run `BezierValidationTest` against OpenSHC reference curves.
- [ ] Verify `generateLimits` produces identical bearing velocity maps for standard gait parameters.
- [ ] Compare `updateWalk()` state machine transitions step-by-step with OpenSHC.
- [ ] Compare `updateCurrentPose()` pose composition order with OpenSHC (walk_plane → manual → inclination → IMU/auto → tip_align → ik_error → default).
- [ ] Compare `AdmittanceController` ODE output with OpenSHC boost::odeint RK4 output for identical inputs.
