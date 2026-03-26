# OpenSHC ↔ HexaMotion Parity Gap Report

**Date:** 2026-02-16 (updated)
**Scope:** Exhaustive comparison of enums, classes, and methods between OpenSHC and HexaMotion
**Methodology:** Enum-first analysis (structure, semantics, usage), then class mapping, then method-level delta
**Reference:** AGENTS.md architectural constraints preserved throughout

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Enum Parity Analysis](#2-enum-parity-analysis)
3. [Class / Structure Parity Analysis](#3-class--structure-parity-analysis)
4. [Method Parity Analysis](#4-method-parity-analysis)
5. [Architectural Differences (By Design)](#5-architectural-differences-by-design)
6. [Recommendations & Prioritization](#6-recommendations--prioritization)

---

## 1. Executive Summary

HexaMotion achieves approximately **~99%** functional parity with OpenSHC's locomotion logic. The core walking pipeline — gait planning, Bezier trajectory generation, admittance control, body pose composition, IK, and joint output — is fully ported. Since the prior analysis, all previously-identified significant gaps have been closed:

- **`Pose` class** is now a fully dedicated module (`pose.h`/`pose.cpp`) with all OpenSHC-equivalent non-ROS operations.
- **`updateDefaultConfiguration()`** is now implemented in both `Leg` and `LocomotionSystem`, called after startup and standing pose transitions — matching OpenSHC's `Model::updateDefaultConfiguration()` → `Leg::updateDefaultConfiguration()` flow.
- **Per-joint offsets and max angular speed** are now implemented via `Parameters::joint_angle_offset_deg[NUM_LEGS][DOF_PER_LEG]` and `Parameters::joint_max_angular_speed_deg_s[NUM_LEGS][DOF_PER_LEG]`, applied through `LocomotionSystem::applyJointOutputCalibration()` and `limitJointAngularSpeedCommand()`.
- **Previous-iteration joint state tracking** is now implemented via `LocomotionSystem::DesiredJointCommandState` with `prev_desired_position_rad`, `prev_desired_velocity_rad_s`, and `prev_desired_effort`.
- **Utility function gaps resolved:** `projectVector()`/`rejectVector()`, `clampedVector()`/`clampedVector2d()`, `setPrecision()`/`setPrecisionVec()`, `correctRotation()`.
- **Parameter gaps resolved:** `overlapping_walkspaces`, `packed_pose_joints[]`/`unpacked_pose_joints[]`, `load_stiffness_scaler`/`swing_stiffness_scaler`, `use_joint_effort`.

Remaining gaps are limited to:

- `velocity_input_mode` parameter ("real" vs "throttle") — intentionally excluded per AGENTS.md.
- `ignore_IK_warnings` parameter — intentionally excluded per AGENTS.md.
- `sqr()` utility — trivial helper (`x*x`).

---

## 2. Enum Parity Analysis

### 2.1. SystemState

| OpenSHC                  | HexaMotion               | Status                        |
| ------------------------ | ------------------------ | ----------------------------- |
| `SUSPENDED` (0)          | `SUSPENDED` (0)          | ✅ Match                      |
| `OPERATIONAL` (1)        | `OPERATIONAL` (1)        | ✅ Match                      |
| `SYSTEM_STATE_COUNT` (2) | `SYSTEM_STATE_COUNT` (2) | ✅ Match                      |
| _(none)_                 | `SYSTEM_UNKNOWN = -1`    | ➕ HexaMotion addition (safe) |

**Usage:** Both use `SystemState` to gate the main control loop. OpenSHC defaults to `SUSPENDED`; HexaMotion adds `SYSTEM_UNKNOWN` as a defensive initialization sentinel. No functional gap.

---

### 2.2. RobotState

| OpenSHC                 | HexaMotion              | Status     |
| ----------------------- | ----------------------- | ---------- |
| `PACKED` (0)            | `ROBOT_PACKED` (0)      | ✅ Renamed |
| `READY` (1)             | `ROBOT_READY` (1)       | ✅ Renamed |
| `RUNNING` (2)           | `ROBOT_RUNNING` (2)     | ✅ Renamed |
| `ROBOT_STATE_COUNT` (3) | `ROBOT_STATE_COUNT` (3) | ✅ Match   |
| `UNKNOWN = -1`          | `ROBOT_UNKNOWN = -1`    | ✅ Renamed |
| `OFF = -2`              | `ROBOT_OFF = -2`        | ✅ Renamed |

**Usage:** Both use `RobotState` for the same hierarchical state machine (UNKNOWN→PACKED↔READY↔RUNNING). The prefix `ROBOT_` prevents collision with generic names. No functional gap.

---

### 2.3. GaitDesignation / GaitType

| OpenSHC (`GaitDesignation`)  | HexaMotion (`GaitType`) | Status                                  |
| ---------------------------- | ----------------------- | --------------------------------------- |
| `WAVE_GAIT` (0)              | `WAVE_GAIT` (0)         | ✅ Ordinal aligned                      |
| `AMBLE_GAIT` (1)             | _(absent)_              | ✅ Intentionally excluded per AGENTS.md |
| `RIPPLE_GAIT` (2)            | `RIPPLE_GAIT` (2)       | ✅ Ordinal aligned                      |
| `TRIPOD_GAIT` (3)            | `TRIPOD_GAIT` (3)       | ✅ Ordinal aligned                      |
| `GAIT_DESIGNATION_COUNT` (4) | `GAIT_TYPE_COUNT` (6)   | ✅ Count sentinel present               |
| `GAIT_UNDESIGNATED = -1`     | `NO_GAIT = -1`          | ✅ Semantic aligned                     |
| _(none)_                     | `METACHRONAL_GAIT` (4)  | ➕ HexaMotion extension                 |
| _(none)_                     | `ADAPTIVE_GAIT` (5)     | ➕ HexaMotion extension                 |

**Analysis:**

1. **Ordinals aligned:** Shared OpenSHC gaits preserve canonical values (`WAVE=0`, `RIPPLE=2`, `TRIPOD=3`). `AMBLE_GAIT=1` is intentionally skipped.
2. **AMBLE_GAIT excluded:** Per AGENTS.md — _"AMBLE_GAIT is not supported with current morphology/constraints."_

No functional gap.

---

### 2.4. PosingMode

| OpenSHC                 | HexaMotion              | Status     |
| ----------------------- | ----------------------- | ---------- |
| `NO_POSING` (0)         | `POSING_NONE` (0)       | ✅ Renamed |
| `X_Y_POSING` (1)        | `POSING_X_Y` (1)        | ✅ Renamed |
| `PITCH_ROLL_POSING` (2) | `POSING_PITCH_ROLL` (2) | ✅ Renamed |
| `Z_YAW_POSING` (3)      | `POSING_Z_YAW` (3)      | ✅ Renamed |
| `EXTERNAL_POSING` (4)   | `POSING_EXTERNAL` (4)   | ✅ Renamed |
| `POSING_MODE_COUNT` (5) | `POSING_MODE_COUNT` (5) | ✅ Match   |

No functional gap.

---

### 2.5. CruiseControlMode

| OpenSHC                         | HexaMotion | Status         |
| ------------------------------- | ---------- | -------------- |
| `CRUISE_CONTROL_OFF` (0)        | _(absent)_ | ✅ Intentional |
| `CRUISE_CONTROL_ON` (1)         | _(absent)_ | ✅ Intentional |
| `CRUISE_CONTROL_MODE_COUNT` (2) | _(absent)_ | ✅ Intentional |
| `CRUISE_CONTROL_EXTERNAL = -1`  | _(absent)_ | ✅ Intentional |

Per AGENTS.md: _"Cruise control is intentionally not ported."_

---

### 2.6. PlannerMode

| OpenSHC                  | HexaMotion | Status         |
| ------------------------ | ---------- | -------------- |
| `PLANNER_MODE_OFF` (0)   | _(absent)_ | ✅ Intentional |
| `PLANNER_MODE_ON` (1)    | _(absent)_ | ✅ Intentional |
| `PLANNER_MODE_COUNT` (2) | _(absent)_ | ✅ Intentional |

Per AGENTS.md: _"Planner mode is intentionally not ported."_

---

### 2.7. LegState

| OpenSHC                  | HexaMotion                   | Status     |
| ------------------------ | ---------------------------- | ---------- |
| `WALKING` (0)            | `LEG_WALKING` (0)            | ✅ Renamed |
| `MANUAL` (1)             | `LEG_MANUAL` (1)             | ✅ Renamed |
| `LEG_STATE_COUNT` (2)    | `LEG_STATE_COUNT` (2)        | ✅ Match   |
| `WALKING_TO_MANUAL = -1` | `LEG_WALKING_TO_MANUAL = -1` | ✅ Renamed |
| `MANUAL_TO_WALKING = -2` | `LEG_MANUAL_TO_WALKING = -2` | ✅ Renamed |

No functional gap.

---

### 2.8. WalkState

| OpenSHC                | HexaMotion             | Status     |
| ---------------------- | ---------------------- | ---------- |
| `STARTING` (0)         | `WALK_STARTING` (0)    | ✅ Renamed |
| `MOVING` (1)           | `WALK_MOVING` (1)      | ✅ Renamed |
| `STOPPING` (2)         | `WALK_STOPPING` (2)    | ✅ Renamed |
| `STOPPED` (3)          | `WALK_STOPPED` (3)     | ✅ Renamed |
| `WALK_STATE_COUNT` (4) | `WALK_STATE_COUNT` (4) | ✅ Match   |

No functional gap.

---

### 2.9. StepState

| OpenSHC                | HexaMotion              | Status     |
| ---------------------- | ----------------------- | ---------- |
| `SWING` (0)            | `STEP_SWING` (0)        | ✅ Renamed |
| `STANCE` (1)           | `STEP_STANCE` (1)       | ✅ Renamed |
| `FORCE_STANCE` (2)     | `STEP_FORCE_STANCE` (2) | ✅ Renamed |
| `FORCE_STOP` (3)       | `STEP_FORCE_STOP` (3)   | ✅ Renamed |
| `STEP_STATE_COUNT` (4) | `STEP_STATE_COUNT` (4)  | ✅ Match   |

**Note:** HexaMotion additionally defines `StepPhase` (`STANCE_PHASE=0`, `SWING_PHASE=1`, `LIFT_PHASE=2`, `TOUCHDOWN_PHASE=3`) for the `Leg` class gait phase tracking — a separate concept from `StepState` (walk controller state machine). No functional gap.

---

### 2.10. PosingState

| OpenSHC                  | HexaMotion               | Status   |
| ------------------------ | ------------------------ | -------- |
| `POSING` (0)             | `POSING` (0)             | ✅ Match |
| `STOP_POSING` (1)        | `STOP_POSING` (1)        | ✅ Match |
| `POSING_COMPLETE` (2)    | `POSING_COMPLETE` (2)    | ✅ Match |
| `POSING_STATE_COUNT` (3) | `POSING_STATE_COUNT` (3) | ✅ Match |

No functional gap.

---

### 2.11. PoseResetMode

| OpenSHC                     | HexaMotion                  | Status   |
| --------------------------- | --------------------------- | -------- |
| `NO_RESET` (0)              | `NO_RESET` (0)              | ✅ Match |
| `Z_AND_YAW_RESET` (1)       | `Z_AND_YAW_RESET` (1)       | ✅ Match |
| `X_AND_Y_RESET` (2)         | `X_AND_Y_RESET` (2)         | ✅ Match |
| `PITCH_AND_ROLL_RESET` (3)  | `PITCH_AND_ROLL_RESET` (3)  | ✅ Match |
| `ALL_RESET` (4)             | `ALL_RESET` (4)             | ✅ Match |
| `IMMEDIATE_ALL_RESET` (5)   | `IMMEDIATE_ALL_RESET` (5)   | ✅ Match |
| `POSE_RESET_MODE_COUNT` (6) | `POSE_RESET_MODE_COUNT` (6) | ✅ Match |

No functional gap.

---

### 2.12. SequenceSelection

| OpenSHC                        | HexaMotion           | Status     |
| ------------------------------ | -------------------- | ---------- |
| `START_UP` (0)                 | `START_UP` (0)       | ✅ Match   |
| `SHUT_DOWN` (1)                | `SHUT_DOWN` (1)      | ✅ Match   |
| `SEQUENCE_SELECTION_COUNT` (2) | `SEQUENCE_COUNT` (2) | ✅ Renamed |

No functional gap.

---

### 2.13. LegDesignation

| OpenSHC                     | HexaMotion                  | Status          |
| --------------------------- | --------------------------- | --------------- |
| `LEG_0` through `LEG_7`     | `LEG_0` through `LEG_5`     | ✅ 6-leg subset |
| `LEG_DESIGNATION_COUNT` (8) | `LEG_DESIGNATION_COUNT` (6) | ✅ Adapted      |
| `LEG_UNDESIGNATED = -1`     | `LEG_UNDESIGNATED = -1`     | ✅ Match        |

OpenSHC supports up to 8 legs; HexaMotion is fixed to 6 per AGENTS.md. No functional gap.

---

### 2.14. ParameterSelection

| OpenSHC                                       | HexaMotion | Status         |
| --------------------------------------------- | ---------- | -------------- |
| `NO_PARAMETER_SELECTION` (0)                  | _(absent)_ | ✅ Intentional |
| `STEP_FREQUENCY` (1) through `FORCE_GAIN` (9) | _(absent)_ | ✅ Intentional |
| `PARAMETER_SELECTION_COUNT` (10)              | _(absent)_ | ✅ Intentional |

Per AGENTS.md: _"`ParameterSelection` is intentionally not implemented in HexaMotion."_

---

### 2.15. HexaMotion-Only Enums

| Enum             | Location                        | Purpose                       |
| ---------------- | ------------------------------- | ----------------------------- |
| `PrecisionLevel` | `precision_config.h`            | MCU resource management       |
| `StepPhase`      | `robot_model.h`                 | Leg gait phase tracking       |
| `IMUMode`        | `robot_model.h`                 | Hardware-specific IMU modes   |
| `BodyPoseMode`   | `manual_body_pose_controller.h` | Manual body pose input modes  |
| `AutoPoseMode`   | `imu_auto_pose.h`               | IMU-based auto-pose selection |
| `StopMode`       | `locomotion_system.h`           | Gait stop behavior            |
| `ErrorCode`      | `locomotion_system.h`           | Error categorization          |

All are valid HexaMotion extensions for MCU-specific needs.

---

### 2.16. Enum Summary Matrix

| Enum                     | OpenSHC | HexaMotion           | Parity                |
| ------------------------ | ------- | -------------------- | --------------------- |
| SystemState              | ✅      | ✅ (+sentinel)       | Full                  |
| RobotState               | ✅      | ✅ (ROBOT\_ prefix)  | Full                  |
| GaitDesignation/GaitType | ✅      | ✅ + extensions      | Full (AMBLE excluded) |
| PosingMode               | ✅      | ✅ (POSING\_ prefix) | Full                  |
| CruiseControlMode        | ✅      | ❌ Intentional       | N/A per AGENTS.md     |
| PlannerMode              | ✅      | ❌ Intentional       | N/A per AGENTS.md     |
| LegState                 | ✅      | ✅ (LEG\_ prefix)    | Full                  |
| WalkState                | ✅      | ✅ (WALK\_ prefix)   | Full                  |
| StepState                | ✅      | ✅ (STEP\_ prefix)   | Full                  |
| PosingState              | ✅      | ✅                   | Full                  |
| PoseResetMode            | ✅      | ✅                   | Full                  |
| SequenceSelection        | ✅      | ✅                   | Full                  |
| LegDesignation           | ✅ (8)  | ✅ (6)               | Full (6-leg)          |
| ParameterSelection       | ✅      | ❌ Intentional       | N/A per AGENTS.md     |

**Result:** All 14 OpenSHC enums are either fully matched (11) or intentionally excluded (3). Additional 7 HexaMotion-only enums extend functionality for embedded/hardware needs.

---

## 3. Class / Structure Parity Analysis

### 3.1. Core Model Classes

| OpenSHC Class  | HexaMotion Class           | Status                | Notes                                                           |
| -------------- | -------------------------- | --------------------- | --------------------------------------------------------------- |
| `Model`        | `RobotModel`               | ✅ Ported             | Renamed; DH chain, FK/IK, workspace generation                  |
| `Leg`          | `Leg`                      | ✅ Ported             | Value type; stores RobotModel ref                               |
| `Joint`        | _(inlined in Leg)_         | ✅ Ported differently | `JointAngles` struct + per-joint data in `Leg` and `Parameters` |
| `Link`         | _(inlined in RobotModel)_  | ✅ Ported differently | DH params stored in `RobotModel::dh_transforms` arrays          |
| `Tip`          | _(inlined in Leg)_         | ✅ Ported differently | Tip position stored directly in `Leg::tip_position_`            |
| `ImuData`      | `IMUData`                  | ✅ Ported + extended  | Extra: `IMUAbsoluteData`, `IMUMode`, `has_absolute_capability`  |
| `Pose` (class) | `Pose` (struct, dedicated) | ✅ Ported (no ROS)    | `pose.h`/`pose.cpp`, full non-ROS API                           |

**OpenSHC Joint-level features — current coverage:**

| Feature                                    | OpenSHC          | HexaMotion                                             | Status            |
| ------------------------------------------ | ---------------- | ------------------------------------------------------ | ----------------- |
| `packed_positions_` (per joint)            | Per-joint vector | `packed_pose_joints[NUM_LEGS]` in Parameters           | ✅ Ported         |
| `unpacked_position_` (per joint)           | Per-joint        | `unpacked_pose_joints[NUM_LEGS]` in Parameters         | ✅ Ported         |
| `max_angular_speed_` (per joint)           | Per-joint        | `joint_max_angular_speed_deg_s[NUM_LEGS][DOF_PER_LEG]` | ✅ **Now ported** |
| `offset_` (per-joint output offset)        | Per-joint        | `joint_angle_offset_deg[NUM_LEGS][DOF_PER_LEG]`        | ✅ **Now ported** |
| `prev_desired_position_/velocity_/effort_` | Per-joint        | `DesiredJointCommandState` in LocomotionSystem         | ✅ **Now ported** |
| `default_position_` / `default_velocity_`  | Per-joint        | At leg level via `default_angles_`                     | ✅ Simplified     |
| `desired_effort_` / `current_effort_`      | Per-joint        | `JointAngles current_joint_effort_` in Leg             | ✅ Ported         |

No remaining gaps.

---

### 3.2. Pose Class

| OpenSHC `Pose` Method                      | HexaMotion `Pose`                                                      | Status        |
| ------------------------------------------ | ---------------------------------------------------------------------- | ------------- |
| `Pose(Vector3d, Quaterniond)`              | `Pose(Point3D/Vector3d, Quaterniond)`                                  | ✅            |
| `Pose(euler angles)`                       | `Pose(Point3D, Vector3d euler_angles_deg)`                             | ✅ Extra      |
| `Pose(geometry_msgs)` (2 ctors)            | _(absent)_                                                             | ✅ No ROS     |
| `isValid()` / `isUndefined()`              | `isValid()` / `isUndefined()`                                          | ✅            |
| `positionVector()`                         | `positionVector()` → `Eigen::Vector3d`                                 | ✅            |
| `toPoseMessage()` / `toTransformMessage()` | _(absent)_                                                             | ✅ No ROS     |
| `operator==` / `operator!=`                | `operator==` / `operator!=`                                            | ✅            |
| `operator~` (inverse)                      | `operator~`                                                            | ✅            |
| `inverse()`                                | `inverse()`                                                            | ✅            |
| `transform(Matrix4d)`                      | `transform(Matrix4d)`                                                  | ✅            |
| `transform(Pose)`                          | _(via addPose)_                                                        | ✅ Equivalent |
| `transformVector(Vector3d)`                | `transformVector(Point3D)` / `transformVector(Vector3d)`               | ✅            |
| `inverseTransformVector(Vector3d)`         | `inverseTransformVector(Point3D)` / `inverseTransformVector(Vector3d)` | ✅            |
| `addPose()` / `removePose()`               | `addPose()` / `removePose()`                                           | ✅            |
| `interpolate()`                            | `interpolate()`                                                        | ✅            |
| `Identity()` / `Undefined()`               | `Identity()` / `Undefined()`                                           | ✅            |

Complete parity for all non-ROS `Pose` operations.

---

### 3.3. Controller Classes

| OpenSHC Controller     | HexaMotion Equivalent    | Status           | Notes                                          |
| ---------------------- | ------------------------ | ---------------- | ---------------------------------------------- |
| `StateController`      | `StateController`        | ✅ Ported        | Same state machine; ROS callbacks → direct API |
| `WalkController`       | `WalkController`         | ✅ Ported        | Walk state machine + stride calculation        |
| `PoseController`       | `BodyPoseController`     | ✅ Ported        | Renamed; pose composition pipeline             |
| `AdmittanceController` | `AdmittanceController`   | ✅ Ported        | RK4 ODE solver equivalent                      |
| _(ROS node main)_      | `LocomotionSystem`       | ✅ New wrapper   | Replaces ROS graph with direct API             |
| _(ROS callbacks)_      | `StateControllerContext` | ✅ New interface | Decouples state controller from system         |

---

### 3.4. Walk Controller Sub-Classes

| OpenSHC Class    | HexaMotion Class           | Status                      | Notes                                                      |
| ---------------- | -------------------------- | --------------------------- | ---------------------------------------------------------- |
| `LegStepper`     | `LegStepper`               | ✅ Ported                   | Bezier trajectory, stride computation                      |
| `StepCycle`      | `StepCycle`                | ✅ Ported                   | Identical struct fields                                    |
| `ExternalTarget` | `LegStepperExternalTarget` | ✅ Intentional (simplified) | Per AGENTS.md: simplified; full ROS/TF workflow not ported |
| _(none)_         | `TerrainAdaptation`        | ➕ New class                | Extracted from OpenSHC inline code                         |
| _(none)_         | `VelocityLimits`           | ➕ New class                | Extracted from `WalkController::generateLimits()`          |

**`ExternalTarget` analysis:**
Per AGENTS.md: _"`ExternalTarget` is intentionally not ported to HexaMotion."_ HexaMotion provides three simplified internal structures:

- `LegStepperExternalTarget` in `leg_stepper.h` — position, swing_clearance, defined
- `TerrainAdaptation::ExternalTarget` in `terrain_adaptation.h` — position, swing_clearance, defined
- `LegPoser::ExternalTarget` in `leg_poser.h` — pose, swing_clearance, transform, defined

These provide internal scaffolding without TF/ROS transport. No functional gap for the intended architecture.

---

### 3.5. Pose Controller Sub-Classes

| OpenSHC Class                | HexaMotion Class           | Status       | Notes                               |
| ---------------------------- | -------------------------- | ------------ | ----------------------------------- |
| `LegPoser`                   | `LegPoser`                 | ✅ Ported    | Bezier stepping, auto pose negation |
| `AutoPoser`                  | `AutoPoser`                | ✅ Ported    | Phase-based cyclic pose generation  |
| _(inline in PoseController)_ | `IMUAutoPose`              | ➕ Extracted | IMU PID posing as standalone class  |
| _(inline in PoseController)_ | `ManualBodyPoseController` | ➕ Extracted | Manual posing as standalone class   |

---

### 3.6. Data Structures

| OpenSHC Structure         | HexaMotion Equivalent              | Status         | Notes                                     |
| ------------------------- | ---------------------------------- | -------------- | ----------------------------------------- |
| `Parameter<T>`            | _(absent)_                         | ✅ No ROS      | ROS parameter server wrapper              |
| `AdjustableParameter`     | _(absent)_                         | ✅ Intentional | ParameterSelection excluded per AGENTS.md |
| `Parameters` (full)       | `Parameters` (comprehensive)       | ✅ Ported      | See field comparison below                |
| `LimitMap` (typedef)      | `VelocityLimits::LimitMap`         | ✅ Equivalent  |                                           |
| `LegContainer` (typedef)  | `Leg[]` arrays                     | ✅ Equivalent  |                                           |
| `JointContainer`          | `JointAngles` struct               | ✅ Simplified  |                                           |
| `Workspace` / `Workplane` | `Workspace` / `Workplane` typedefs | ✅ Equivalent  |                                           |

---

### 3.7. Parameters Structure Comparison

| OpenSHC `Parameters` Field                                                                                   | HexaMotion `Parameters` Field                                                                  | Status                             |
| ------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------- | ---------------------------------- |
| `time_delta`                                                                                                 | `time_delta`                                                                                   | ✅                                 |
| `imu_posing`                                                                                                 | `imu_posing` + `BodyPoseConfiguration`                                                         | ✅                                 |
| `auto_posing`                                                                                                | `auto_posing` + `BodyPoseConfiguration`                                                        | ✅                                 |
| `manual_posing`                                                                                              | `manual_posing` + `BodyPoseConfiguration`                                                      | ✅                                 |
| `inclination_posing`                                                                                         | `inclination_posing` + `BodyPoseConfiguration`                                                 | ✅                                 |
| `gravity_aligned_tips`                                                                                       | `gravity_aligned_tips` + `BodyPoseConfiguration`                                               | ✅                                 |
| `rough_terrain_mode`                                                                                         | _(in TerrainAdaptation)_                                                                       | ✅ Moved                           |
| `admittance_control`                                                                                         | `Parameters::AdmittanceConfig`                                                                 | ✅ Nested                          |
| `syropod_type`                                                                                               | _(absent)_                                                                                     | ✅ Single robot type               |
| `leg_id[]` / `joint_id[]` / `link_id[]`                                                                      | _(hardcoded)_                                                                                  | ✅ Fixed 6-leg/3DOF                |
| `leg_DOF`                                                                                                    | `DOF_PER_LEG` (3)                                                                              | ✅ Constant                        |
| `clamp_joint_positions`                                                                                      | `IKConfig::clamp_joints`                                                                       | ✅                                 |
| `clamp_joint_velocities`                                                                                     | _(always true)_                                                                                | ✅                                 |
| `ignore_IK_warnings`                                                                                         | _(absent)_                                                                                     | ✅ Intentional per AGENTS.md       |
| `joint_parameters[8][6]`                                                                                     | `coxa/femur/tibia_angle_limits[]` + `joint_angle_offset_deg` + `joint_max_angular_speed_deg_s` | ✅ **Now fully ported**            |
| `link_parameters[8][7]`                                                                                      | `coxa/femur/tibia_length`, `hexagon_radius`                                                    | ✅ Simplified                      |
| `packed_joint_positions_[8][]`                                                                               | `packed_pose_joints[NUM_LEGS]`                                                                 | ✅                                 |
| `unpacked_joint_positions_[8]`                                                                               | `unpacked_pose_joints[NUM_LEGS]`                                                               | ✅                                 |
| `gait_type`                                                                                                  | `gait_type` + `GaitConfiguration`                                                              | ✅                                 |
| `body_clearance`                                                                                             | _(in BodyPoseConfiguration)_                                                                   | ✅ Moved                           |
| `step_frequency` (adjustable)                                                                                | `step_frequency` + `GaitConfiguration`                                                         | ✅ Static (no AdjustableParameter) |
| `swing_height` / `swing_width` / `step_depth` / `stance_span_modifier` (adjustable)                          | _(in GaitConfiguration / TerrainAdaptation)_                                                   | ✅ Static                          |
| `velocity_input_mode`                                                                                        | _(absent)_                                                                                     | ✅ Intentional per AGENTS.md       |
| `body_velocity_scaler`                                                                                       | _(in CartesianVelocityController)_                                                             | ✅ Moved                           |
| `force_cruise_velocity` / `angular_cruise_velocity` / `cruise_control_time_limit` / `linear_cruise_velocity` | _(absent)_                                                                                     | ✅ Per AGENTS.md (cruise external) |
| `overlapping_walkspaces`                                                                                     | `overlapping_walkspaces`                                                                       | ✅                                 |
| `force_normal_touchdown`                                                                                     | _(in TerrainAdaptation)_                                                                       | ✅ Moved                           |
| `touchdown_threshold` / `liftoff_threshold`                                                                  | `fsr_touchdown_threshold` / `fsr_liftoff_threshold`                                            | ✅                                 |
| `leg_stance_positions[8]`                                                                                    | _(in BodyPoseConfiguration)_                                                                   | ✅ Moved                           |
| `auto_pose_type`                                                                                             | _(in AutoPoseConfiguration)_                                                                   | ✅ Moved                           |
| `start_up_sequence`                                                                                          | _(in BodyPoseConfiguration)_                                                                   | ✅ Moved                           |
| `time_to_start`                                                                                              | `time_to_start` + `BodyPoseConfiguration`                                                      | ✅                                 |
| `rotation_pid_gains`                                                                                         | `BodyCompConfig::imu_pid_kp/ki/kd`                                                             | ✅                                 |
| `max_translation` / `max_rotation`                                                                           | _(in BodyPoseConfiguration)_                                                                   | ✅ Moved                           |
| `max_translation_velocity` / `max_rotation_velocity`                                                         | Both in Parameters and BodyPoseConfiguration                                                   | ✅                                 |
| `leg_manipulation_mode`                                                                                      | `ManualLegConfig::joint_control` (bool)                                                        | ✅ Simplified                      |
| `dynamic_stiffness` / `use_joint_effort` / `integrator_step_time`                                            | `AdmittanceConfig::*`                                                                          | ✅                                 |
| `virtual_mass/stiffness/damping_ratio/force_gain` (adjustable)                                               | `AdmittanceConfig::*`                                                                          | ✅ Static                          |
| `load_stiffness_scaler` / `swing_stiffness_scaler`                                                           | `AdmittanceConfig::*`                                                                          | ✅                                 |
| All gait phase params                                                                                        | _(in GaitPhaseConfig)_                                                                         | ✅ Moved                           |
| All auto-pose parameters                                                                                     | _(in AutoPoseConfiguration)_                                                                   | ✅ Moved                           |
| All debug parameters                                                                                         | `#ifdef TESTING_ENABLED` conditionals                                                          | ✅ Conditional                     |

No remaining parameter gaps.

---

### 3.8. HexaMotion-Only Classes (No OpenSHC equivalent)

| HexaMotion Class                   | Purpose                                      |
| ---------------------------------- | -------------------------------------------- |
| `LocomotionSystem`                 | ROS-less facade replacing external ROS graph |
| `StateControllerContext`           | Abstract interface for decoupling            |
| `CartesianVelocityController`      | Per-joint servo speed from body velocity     |
| `ManualBodyPoseController`         | Standalone manual pose with presets          |
| `IMUAutoPose`                      | Standalone IMU pose compensation             |
| `AnalyticRobotModel`               | Cross-validation model (testing)             |
| `WorkspaceAnalyzer`                | Extracted workspace analysis                 |
| `VelocityLimits`                   | Extracted velocity limit computation         |
| `TerrainAdaptation`                | Extracted terrain adaptation                 |
| `GaitConfigFactory`                | Factory for gait configurations              |
| `BodyPoseConfigFactory`            | Factory for body pose configurations         |
| `ComputeConfig` / `PrecisionLevel` | MCU resource management                      |

---

### 3.9. OpenSHC-Only Classes (Not ported)

| OpenSHC Class            | Reason for absence                                                |
| ------------------------ | ----------------------------------------------------------------- |
| `DebugVisualiser`        | No ROS/RVIZ (per AGENTS.md)                                       |
| `Link` (separate class)  | Consolidated into RobotModel DH arrays                            |
| `Joint` (separate class) | Consolidated into Leg + JointAngles + Parameters per-joint arrays |
| `Tip` (separate class)   | Consolidated into Leg tip*position*                               |
| `Parameter<T>` template  | No ROS parameter server                                           |
| `AdjustableParameter`    | ParameterSelection intentionally excluded per AGENTS.md           |

---

### 3.10. Hardware Interface Classes (HexaMotion-only)

| Interface         | Methods | Purpose                    |
| ----------------- | ------- | -------------------------- |
| `IIMUInterface`   | 12      | IMU hardware abstraction   |
| `IFSRInterface`   | 5       | FSR sensor abstraction     |
| `IServoInterface` | 13      | Servo hardware abstraction |

---

## 4. Method Parity Analysis

### 4.1. StateController Methods

| OpenSHC Method                       | HexaMotion Method                                    | Status                                |
| ------------------------------------ | ---------------------------------------------------- | ------------------------------------- |
| `StateController()`                  | `StateController()`                                  | ✅ (no ROS, takes Context)            |
| `~StateController()`                 | `~StateController()`                                 | ✅                                    |
| `init()`                             | `initialize()`                                       | ✅                                    |
| `loop()`                             | `update()`                                           | ✅                                    |
| `transitionRobotState()`             | `handleRobotStateTransition()`                       | ✅                                    |
| `runningState()`                     | `update()` → inline + `updateWalkState()`            | ✅ Split                              |
| `adjustParameter()`                  | _(absent)_                                           | ✅ Per AGENTS.md (ParameterSelection) |
| `changeGait()`                       | `changeGait()`                                       | ✅                                    |
| `legStateToggle()`                   | `requestLegToggle()` + `handleLegStateTransitions()` | ✅                                    |
| `executePlan()`                      | _(absent)_                                           | ✅ Per AGENTS.md (planner external)   |
| `publishDesiredJointState()`         | _(in LocomotionSystem)_                              | ✅ Different path                     |
| All `publish*()` methods (~15)       | _(absent)_                                           | ✅ No ROS publishers                  |
| All `*Callback()` methods (~25)      | Direct API methods                                   | ✅ ROS callbacks → direct API         |
| `cruiseControlCallback()`            | _(absent)_                                           | ✅ Per AGENTS.md                      |
| `plannerModeCallback()`              | _(absent)_                                           | ✅ Per AGENTS.md                      |
| `parameterSelectionCallback()`       | _(absent)_                                           | ✅ Per AGENTS.md                      |
| `parameterAdjustCallback()`          | _(absent)_                                           | ✅ Per AGENTS.md                      |
| `dynamicParameterCallback()`         | _(absent)_                                           | ✅ No dynamic reconfigure             |
| `generateExternalTargetTransforms()` | _(absent)_                                           | ✅ No TF; ExternalTarget excluded     |
| `initParameters()`                   | _(in LocomotionSystem)_                              | ✅ No ROS param server                |
| `initGaitParameters()`               | `GaitConfigFactory`                                  | ✅ Refactored                         |
| `initAutoPoseParameters()`           | `BodyPoseConfigFactory`                              | ✅ Refactored                         |

No functional gaps.

---

### 4.2. WalkController Methods

| OpenSHC Method                   | HexaMotion Equivalent                       | Status                                           |
| -------------------------------- | ------------------------------------------- | ------------------------------------------------ |
| `WalkController()`               | `WalkController()`                          | ✅                                               |
| `WalkController(copy)`           | _(absent — not needed)_                     | ✅ WorkspaceAnalyzer replaces model-copy pattern |
| `init()`                         | `init()`                                    | ✅                                               |
| `generateWalkspace()`            | `generateWalkspace()`                       | ✅                                               |
| `generateLimits(leg, step, ...)` | `VelocityLimits::generateLimits(step, ...)` | ✅ Extracted                                     |
| `generateStepCycle()`            | `GaitConfiguration::generateStepCycle()`    | ✅ Moved to struct                               |
| `getLimit()`                     | `VelocityLimits::getLimit()`                | ✅ Extracted                                     |
| `updateWalk()`                   | `updateWalk()`                              | ✅                                               |
| `updateManual()` (velocity)      | `updateManual()` (velocity)                 | ✅                                               |
| `updateManual()` (pose)          | `updateManual()` (position)                 | ✅                                               |
| `updateWalkPlane()`              | `BodyPoseController::updateWalkPlanePose()` | ✅ Moved                                         |
| `calculateOdometry()`            | `calculateOdometry()`                       | ✅                                               |
| `estimateGravity()`              | `estimateGravity()`                         | ✅                                               |

No functional gaps.

---

### 4.3. LegStepper Methods

| OpenSHC Method                         | HexaMotion Equivalent                                  | Status                                             |
| -------------------------------------- | ------------------------------------------------------ | -------------------------------------------------- |
| `LegStepper()`                         | `LegStepper()`                                         | ✅                                                 |
| `LegStepper(copy)`                     | `LegStepper(const LegStepper&) = delete`               | ✅ Architecturally unnecessary (WorkspaceAnalyzer) |
| `updatePhase()`                        | _(inline in iteratePhase)_                             | ✅ Merged                                          |
| `iteratePhase()`                       | `iteratePhase()`                                       | ✅                                                 |
| `updateStepState()`                    | `updateStepStateFromPhase()`                           | ✅                                                 |
| `updateStride()`                       | `updateStride()`                                       | ✅                                                 |
| `calculateStanceSpanChange()`          | `calculateStanceSpanChange()`                          | ✅ (private)                                       |
| `updateDefaultTipPosition()`           | `updateDefaultTipPosition()`                           | ✅ (private)                                       |
| `updateTipPosition()`                  | `updateTipPosition()` + `updateTipPositionIterative()` | ✅                                                 |
| `updateTipRotation()`                  | _(absent)_                                             | ✅ Intentionally excluded per AGENTS.md            |
| `generatePrimarySwingControlNodes()`   | `generatePrimarySwingControlNodes()`                   | ✅                                                 |
| `generateSecondarySwingControlNodes()` | `generateSecondarySwingControlNodes()`                 | ✅                                                 |
| `generateStanceControlNodes()`         | `generateStanceControlNodes()`                         | ✅                                                 |
| `forceNormalTouchdown()`               | `forceNormalTouchdown()`                               | ✅                                                 |

**`updateTipRotation()` — intentionally excluded:**
Per AGENTS.md: _"`updateTipRotation` (OpenSHC LegStepper tip-rotation path) is intentionally not ported. With 3DOF legs, tip orientation is not controllable as an independent task variable during gait."_

Related exclusions: swing-phase tip rotation blending, gravity-aligned tip orientation as a rotational objective, rotation-constrained tip IK objectives.

No functional gaps.

---

### 4.4. PoseController / BodyPoseController Methods

| OpenSHC Method              | HexaMotion Equivalent                           | Status         |
| --------------------------- | ----------------------------------------------- | -------------- |
| `PoseController()`          | `BodyPoseController()`                          | ✅             |
| `PoseController(copy)`      | _(absent — not needed)_                         | ✅             |
| `init()`                    | `initializeLegPosers()` + `setAutoPoseParams()` | ✅ Split       |
| `setAutoPoseParams()`       | `setAutoPoseParams()`                           | ✅             |
| `updateStance()`            | `updateStance()`                                | ✅             |
| `executeSequence()`         | `executeSequence()`                             | ✅             |
| `directStartup()`           | `directStartup()`                               | ✅             |
| `stepToNewStance()`         | `stepToNewStance()`                             | ✅             |
| `poseForLegManipulation()`  | `poseForLegManipulation()`                      | ✅             |
| `packLegs()`                | `packLegs()`                                    | ✅             |
| `unpackLegs()`              | `unpackLegs()`                                  | ✅             |
| `transitionConfiguration()` | `transitionConfiguration()`                     | ✅             |
| `transitionStance()`        | `transitionStance()`                            | ✅             |
| `updateCurrentPose()`       | `updateCurrentPose()`                           | ✅             |
| `updateManualPose()`        | `updateManualPose()`                            | ✅             |
| `updateIKErrorPose()`       | `updateIKErrorPose()`                           | ✅             |
| `updateTipAlignPose()`      | `updateTipAlignPose()`                          | ✅             |
| `updateWalkPlanePose()`     | `updateWalkPlanePose()`                         | ✅             |
| `updateAutoPose()`          | `updateAutoPose()` (two overloads)              | ✅             |
| `updateIMUPose()`           | `updateIMUPosePID()`                            | ✅             |
| `updateInclinationPose()`   | `updateInclinationPose()`                       | ✅             |
| `calculateDefaultPose()`    | `calculateDefaultPose()`                        | ✅             |
| `estimateGravity()`         | _(in TerrainAdaptation / IMUAutoPose)_          | ✅ Distributed |

No functional gaps.

---

### 4.5. LegPoser Methods

| OpenSHC Method              | HexaMotion Equivalent                         | Status |
| --------------------------- | --------------------------------------------- | ------ |
| `LegPoser()`                | `LegPoser()`                                  | ✅     |
| `LegPoser(copy)`            | `LegPoser(const LegPoser*)`                   | ✅     |
| `transitionConfiguration()` | `transitionConfiguration()`                   | ✅     |
| `stepToPosition()`          | `stepToPosition()` (Pose + Point3D overloads) | ✅     |
| `updateAutoPose()`          | `updateAutoPose()`                            | ✅     |
| `resetStepToPosition()`     | `resetStepToPosition()`                       | ✅     |
| `resetTransitionSequence()` | `resetTransitionSequence()`                   | ✅     |
| `addTransitionPose()`       | `addTransitionPose()`                         | ✅     |
| `hasTransitionPose()`       | `hasTransitionPose()`                         | ✅     |
| `getTransitionPose()`       | `getTransitionPose()`                         | ✅     |

No functional gaps.

---

### 4.6. AutoPoser Methods

| OpenSHC Method          | HexaMotion Equivalent                                                                    | Status             |
| ----------------------- | ---------------------------------------------------------------------------------------- | ------------------ |
| `AutoPoser()`           | `AutoPoser()`                                                                            | ✅                 |
| `updatePose(int phase)` | `updatePose(phase, phase_length, normaliser, posing_state, pose_frequency, gravity_dir)` | ✅ Extended params |
| `resetChecks()`         | `resetChecks()`                                                                          | ✅                 |
| `isPosing()`            | `isPosing()`                                                                             | ✅                 |
| All amplitude setters   | All amplitude setters + `setGravityAmplitude()`                                          | ✅                 |

No functional gaps.

---

### 4.7. Model / RobotModel Methods

| OpenSHC Method                  | HexaMotion Equivalent                                                                  | Status                                |
| ------------------------------- | -------------------------------------------------------------------------------------- | ------------------------------------- |
| `Model()`                       | `RobotModel()`                                                                         | ✅                                    |
| `Model(copy)`                   | _(absent)_                                                                             | ✅ WorkspaceAnalyzer replaces pattern |
| `generate()`                    | `initializeDH()`                                                                       | ✅                                    |
| `initLegs()`                    | _(in Leg::initialize)_                                                                 | ✅ Distributed                        |
| `legsBearingLoad()`             | `BodyPoseController::legsBearingLoad()`                                                | ✅ Moved                              |
| `getLegByIDNumber/Name()`       | _(array access)_                                                                       | ✅                                    |
| `getImuData()` / `setImuData()` | _(in LocomotionSystem)_                                                                | ✅                                    |
| `updateDefaultConfiguration()`  | `LocomotionSystem::updateDefaultConfiguration()` → `Leg::updateDefaultConfiguration()` | ✅ **Now ported**                     |
| `generateWorkspaces()`          | `WorkspaceAnalyzer::generateWorkspace()`                                               | ✅ Extracted                          |
| `updateModel()`                 | _(in LocomotionSystem::applyInverseKinematicsToAllLegs)_                               | ✅                                    |
| `estimateGravity()`             | `TerrainAdaptation` / `IMUAutoPose`                                                    | ✅ Distributed                        |

No remaining gaps.

---

### 4.8. Leg Methods

| OpenSHC `Leg` Method             | HexaMotion Equivalent                                  | Status                |
| -------------------------------- | ------------------------------------------------------ | --------------------- |
| `Leg()`                          | `Leg()`                                                | ✅                    |
| `Leg(copy)`                      | _(default copy)_                                       | ✅ Value type         |
| `generate()`                     | _(in constructor)_                                     | ✅                    |
| `init()`                         | `initialize()`                                         | ✅                    |
| `generateWorkspace()`            | `WorkspaceAnalyzer`                                    | ✅ Extracted          |
| `getWorkplane()`                 | `WorkspaceAnalyzer::getWorkplane()`                    | ✅                    |
| `makeReachable()`                | `RobotModel::makeReachable()`                          | ✅ Moved              |
| `updateDefaultConfiguration()`   | `Leg::updateDefaultConfiguration()`                    | ✅ **Now ported**     |
| `generateDesiredJointStateMsg()` | _(absent — no ROS)_                                    | ✅                    |
| `getJointByIDNumber/Name()`      | _(direct access)_                                      | ✅                    |
| `getLinkByIDNumber/Name()`       | _(absent)_                                             | ✅                    |
| `setDesiredTipPose(Pose, bool)`  | `Leg::setDesiredTipPosition(Point3D)`                  | ✅ 3DOF — no rotation |
| `setDesiredTipVelocity()`        | `Leg::setDesiredJointVelocity()` via Jacobian          | ✅                    |
| `calculateTipForce()`            | `Leg::calculateTipForce()`                             | ✅                    |
| `touchdownDetection()`           | `TerrainAdaptation::detectTouchdownEvents()`           | ✅ Moved              |
| `solveIK()`                      | `RobotModel::solveDeltaIK()` / `RobotModel::solveIK()` | ✅ Moved              |
| `updateJointPositions()`         | _(in RobotModel::applyAdvancedIK)_                     | ✅                    |
| `applyIK()`                      | `Leg::applyIK()` / `Leg::applyAdvancedIK()`            | ✅                    |
| `applyFK()`                      | `Leg::updateTipPosition()` (via RobotModel FK)         | ✅                    |

No remaining gaps.

---

### 4.9. AdmittanceController Methods

| OpenSHC Method                    | HexaMotion Equivalent                     | Status |
| --------------------------------- | ----------------------------------------- | ------ |
| `AdmittanceController()`          | `AdmittanceController()`                  | ✅     |
| `updateAdmittance()`              | `updateAdmittance(Leg[], IFSRInterface*)` | ✅     |
| `updateStiffness(Leg, double)`    | `updateStiffness(Leg[], int, double)`     | ✅     |
| `updateStiffness(WalkController)` | `updateStiffness(Leg[], WalkController*)` | ✅     |

No functional gaps.

---

### 4.10. Utility Functions (standard_includes.h → math_utils.h)

| OpenSHC Function                                          | HexaMotion Equivalent                                                   | Status       |
| --------------------------------------------------------- | ----------------------------------------------------------------------- | ------------ |
| `degreesToRadians()` / `radiansToDegrees()`               | `math_utils::degreesToRadians/radiansToDegrees()`                       | ✅           |
| `mod()`                                                   | `math_utils::mod()`                                                     | ✅           |
| `sqr()`                                                   | _(absent)_                                                              | ⚠️ Trivial   |
| `sign()`                                                  | `math_utils::sign()`                                                    | ✅           |
| `roundToInt()` / `roundToEvenInt()`                       | `math_utils::roundToInt/roundToEvenInt()`                               | ✅           |
| `clamped(T, T, T)`                                        | `math_utils::clamped()` + `math_utils::clamp()`                         | ✅           |
| `clamped(Vector, magnitude)`                              | `math_utils::clampedVector()`                                           | ✅           |
| `clamped(Vector, Vector limits)`                          | `math_utils::clampedVector2d()`                                         | ✅           |
| `setPrecision(double)` / `setPrecision(Vector3d)`         | `math_utils::setPrecision()` / `setPrecisionVec()`                      | ✅           |
| `smoothStep()`                                            | `math_utils::smoothStep()`                                              | ✅           |
| `getProjection()` / `getRejection()`                      | `math_utils::projectVector()` / `rejectVector()`                        | ✅           |
| `interpolate()` (template)                                | `math_utils::interpolate()` (template)                                  | ✅           |
| `correctRotation()`                                       | `math_utils::correctRotation()`                                         | ✅           |
| `eulerAnglesToQuaternion()` / `quaternionToEulerAngles()` | `math_utils::eulerAnglesToQuaterniond()` / `quaterniondToEulerAngles()` | ✅           |
| `quadraticBezier()`                                       | `math_utils::quadraticBezier()`                                         | ✅           |
| `quadraticBezierCurveThroughControlPoint()`               | `math_utils::quadraticBezierCurveThroughControlPoint()`                 | ✅           |
| `cubicBezier()` / `cubicBezierDot()`                      | `math_utils::cubicBezier()` / `cubicBezierDot()`                        | ✅           |
| `cubicBezierCurveThroughControlPoint()`                   | `math_utils::cubicBezierCurveThroughControlPoint()`                     | ✅           |
| `quarticBezier()` / `quarticBezierDot()`                  | `math_utils::quarticBezier()` / `quarticBezierDot()`                    | ✅           |
| `quarticBezierCurveThroughControlPoint()`                 | `math_utils::quarticBezierCurveThroughControlPoint()`                   | ✅           |
| `createDHMatrix()`                                        | `math_utils::dhTransform()` + `dhTransformY()`                          | ✅ Extended  |
| `numberToString()` / `stringFormat()`                     | _(absent)_                                                              | ✅ ROS debug |

**HexaMotion additional math utilities:**

- `normalizeAngle()`, `rotatePoint()`, `distance3D/2D()`, `magnitude()`, `distance()`
- `pointToLineDistance()`, `crossProduct()`
- `rotationMatrixX/Y/Z()`
- Quaternion utilities (multiply, inverse, slerp, various conversions)
- `solveLeastSquaresPlane()` — least-squares plane fitting
- `StateVector<T>`, `DerivativeFunction<T>` — integrator framework
- `rungeKutta4()`, `rungeKutta2()`, `forwardEuler()` — numerical integrators

**Only remaining absent utility:** `sqr()` — trivial square helper (`x*x`). Functional impact: none.

---

## 5. Architectural Differences (By Design)

These differences are intentional per AGENTS.md and do NOT represent gaps:

| Aspect                  | OpenSHC                       | HexaMotion                          | Justification                              |
| ----------------------- | ----------------------------- | ----------------------------------- | ------------------------------------------ |
| ROS transport           | ROS subscribers/publishers    | Direct API methods                  | AGENTS.md: no ROS                          |
| TF transforms           | tf2_ros                       | _(absent)_                          | AGENTS.md: no ROS                          |
| RVIZ visualization      | DebugVisualiser               | _(absent)_                          | AGENTS.md: no ROS                          |
| YAML configuration      | ROS param server              | `Parameters` struct                 | AGENTS.md: no YAML                         |
| Dynamic reconfigure     | `dynamic_reconfigure::Server` | _(absent)_                          | AGENTS.md: no dynamic config               |
| Leg count               | 1–8 configurable              | 6 fixed                             | AGENTS.md: 6 legs only                     |
| DOF per leg             | 1–6 configurable              | 3 fixed                             | AGENTS.md: 3DOF only                       |
| AMBLE_GAIT              | Supported                     | Excluded                            | AGENTS.md: not supported                   |
| Planner mode            | Internal                      | External API                        | AGENTS.md: external planner                |
| Cruise control          | Internal                      | External API                        | AGENTS.md: external cruise                 |
| ParameterSelection      | `AdjustableParameter` + enum  | `LocomotionSystem` APIs             | AGENTS.md: intentionally excluded          |
| `updateTipRotation`     | Tip rotation during swing     | Excluded                            | AGENTS.md: 3DOF cannot control tip orient. |
| Tip rotation blending   | Swing-phase tip blending      | Excluded                            | AGENTS.md: out of scope for 3DOF           |
| Gravity-aligned tip rot | Rotational objective          | Excluded                            | AGENTS.md: out of scope for 3DOF           |
| `ExternalTarget` (full) | ROS/TF external targets       | Simplified + LocomotionSystem API   | AGENTS.md: intentionally not ported        |
| `velocity_input_mode`   | "real" vs "throttle"          | External software                   | AGENTS.md: intentionally not implemented   |
| `ignore_IK_warnings`    | Debug flag                    | Not ported                          | AGENTS.md: intentionally not ported        |
| Class organization      | Monolithic model.h            | Split into focused files            | AGENTS.md: more readable                   |
| Memory model            | `shared_ptr` everywhere       | Value types + references            | MCU-appropriate                            |
| Workspace generation    | Full model copy               | `WorkspaceAnalyzer` over live model | AGENTS.md: MCU-efficient                   |

---

## 6. Recommendations & Prioritization

### 🟢 Low Priority — Remaining Items (Polish only)

| #   | Gap             | Impact                                                           |
| --- | --------------- | ---------------------------------------------------------------- |
| L1  | `sqr()` utility | Trivial helper; inline `x*x` used instead. No functional impact. |

### ✅ Completed (since prior analysis)

| #   | Completed item                                         | Where implemented                                                                                      |
| --- | ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------ |
| C1  | `Pose` full module                                     | `src/pose.h`, `src/pose.cpp`                                                                           |
| C2  | `Pose::transform(Matrix4d)`                            | `src/pose.h`                                                                                           |
| C3  | `Pose::transformVector()` / `inverseTransformVector()` | `src/pose.h`                                                                                           |
| C4  | `Pose::isValid()` / `isUndefined()` / `Undefined()`    | `src/pose.h`                                                                                           |
| C5  | `Pose::operator~` / `inverse()`                        | `src/pose.h`                                                                                           |
| C6  | `GaitType` count + `NO_GAIT = -1`                      | `src/gait_types.h`                                                                                     |
| C7  | `projectVector()` / `rejectVector()`                   | `src/math_utils.h`                                                                                     |
| C8  | `clampedVector()` / `clampedVector2d()`                | `src/math_utils.h`                                                                                     |
| C9  | `setPrecision()` / `setPrecisionVec()`                 | `src/math_utils.h`                                                                                     |
| C10 | `correctRotation()`                                    | `src/math_utils.h`                                                                                     |
| C11 | `overlapping_walkspaces` parameter                     | `src/robot_model.h` (Parameters)                                                                       |
| C12 | `packed_pose_joints[]` / `unpacked_pose_joints[]`      | `src/robot_model.h` (Parameters)                                                                       |
| C13 | `load_stiffness_scaler` / `swing_stiffness_scaler`     | `src/robot_model.h` (AdmittanceConfig)                                                                 |
| C14 | `use_joint_effort` flag                                | `src/robot_model.h` (AdmittanceConfig)                                                                 |
| C15 | `leg_manipulation_mode`                                | `src/robot_model.h` (ManualLegConfig)                                                                  |
| C16 | Pose-first integration in walk/stepper paths           | `src/leg_stepper.*`, `src/walk_controller.cpp`                                                         |
| C17 | `updateDefaultConfiguration()`                         | `src/leg.cpp`, `src/locomotion_system.cpp`                                                             |
| C18 | Per-joint `offset_` (output offset)                    | `Parameters::joint_angle_offset_deg[NUM_LEGS][DOF_PER_LEG]` + `applyJointOutputCalibration()`          |
| C19 | Per-joint `max_angular_speed_`                         | `Parameters::joint_max_angular_speed_deg_s[NUM_LEGS][DOF_PER_LEG]` + `limitJointAngularSpeedCommand()` |
| C20 | Previous-iteration tracking (`prev_desired_*`)         | `LocomotionSystem::DesiredJointCommandState`                                                           |

### ❌ Intentionally Excluded (per AGENTS.md)

| Feature                                      | AGENTS.md Justification                                               |
| -------------------------------------------- | --------------------------------------------------------------------- |
| `updateTipRotation()`                        | 3DOF legs cannot control tip orientation as independent task variable |
| Swing-phase tip rotation blending            | Out of scope for 3DOF                                                 |
| Gravity-aligned tip orientation (rot.)       | Out of scope for 3DOF                                                 |
| Rotation-constrained tip IK                  | Out of scope for 3DOF                                                 |
| `ExternalTarget` (full ROS/TF)               | Equivalent behavior through LocomotionSystem API                      |
| `ParameterSelection` + `AdjustableParameter` | Runtime parameter adjustment through LocomotionSystem APIs instead    |
| `CruiseControlMode`                          | External software component responsibility                            |
| `PlannerMode`                                | External software component responsibility                            |
| `velocity_input_mode`                        | External software should pre-scale commands                           |
| `ignore_IK_warnings`                         | Would interfere with HexaMotion's control/diagnostic flow             |
| `AMBLE_GAIT`                                 | Not supported with current morphology/constraints                     |
| `DebugVisualiser`                            | No ROS/RVIZ                                                           |
| `Parameter<T>` template                      | No ROS parameter server                                               |
| Dynamic configuration                        | Out of scope                                                          |

---

### Summary Statistics

| Category                      | Count              | Notes                                                                                   |
| ----------------------------- | ------------------ | --------------------------------------------------------------------------------------- |
| Enums fully matched           | 11/14              | 3 intentionally excluded (planner/cruise/param selection); 7 HexaMotion-only extensions |
| Classes fully ported          | 14/14 (functional) | All functional classes present; ROS-only classes intentionally absent                   |
| Controller methods matched    | ~100%              | All core methods present                                                                |
| Utility functions matched     | ~99%               | Only absent: `sqr()` (trivial)                                                          |
| Pose class methods matched    | 100% (non-ROS)     | All non-ROS methods present                                                             |
| LegStepper methods matched    | ~99%               | `updateTipRotation` intentionally excluded per 3DOF constraint                          |
| Parameters matched            | ~100%              | All locomotion-relevant params ported; ROS/cruise/planner params excluded by design     |
| **Overall functional parity** | **~99%**           | Single trivial gap: `sqr()` utility. All functional locomotion logic ported.            |
