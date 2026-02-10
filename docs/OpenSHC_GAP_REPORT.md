# OpenSHC Parity Gap Report (HexaMotion)

## Context (from AGENTS.md)

HexaMotion is a 1:1 port of OpenSHC without ROS support. The goal is to run OpenSHC's locomotion logic on MCU targets such as the STM32H7 (Arduino Giga R1) for a hexapod robot with a hexagonal body, six legs spaced 60 degrees apart, and three joints per leg. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling. The interfaces `IIMUInterface`, `IFSRInterface` and `IServoInterface` must be implemented to connect the IMU, FSR sensors and smart servos.

Key differences from OpenSHC:

- Supports only 3DOF per leg.
- Supports only six legs.
- `LocomotionSystem` orchestrates the control classes, analogous to the OpenSHC top-level orchestration.
- No YAML configuration files; everything is configured through the `Parameters` structure.
- OpenSHC logic is split into specific classes so the code is more readable and maintainable; the current HexaMotion organization follows this.
- Includes tests to verify hexapod kinematics and dynamics logic.
- Certain configurations are handled via factory patterns.
- No dynamic configuration support.

Additional constraints and reference data from AGENTS.md:

- Code style: C++11, four-space indentation, opening brace on same line, Doxygen for public APIs.
- Implementation lives in src/ and include/. No Arduino examples. OpenSHC is the reference baseline.
- Testing: run tests in tests/ (setup Eigen via tests/setup.sh, then make).
- Physical robot characteristics: hexagon radius 200 mm, coxa 50 mm, femur 101 mm, tibia 208 mm, robot height 208 mm, default standing height 150 mm.
- Default height: default_height_offset controls Z when joint angles are 0.0; if 0.0, falls back to -tibia_length. Explicitly set to -208.0 for the physical robot.
- Leg base orientation: BASE_THETA_OFFSETS in hexamotion_constants.h (AR +30, BR +90, CR +150, CL -150, BL -90, AL -30).
- Units: internal kinematics use mm, mm/s, mm/s^2; convert to SI only at integration boundaries.
- Stance/workspace radii derived from standing pose, not sum of all link lengths.
- Symmetry requirement: standing pose and reach are symmetric across opposite leg pairs only.
- Test parameters: see AGENTS.md for exact Parameters values and limits.

## Scope and Method

Compared OpenSHC headers in OpenSHC/include/syropod_highlevel_controller and sources in OpenSHC/src against HexaMotion sources in src. Focus is on public APIs, major state variables, and functional behavior.

## Gap Checklist by Module

### parameters_and_states.h (OpenSHC) vs HexaMotion enums/config

Missing or partial parity:

- OpenSHC `SystemState` includes `SUSPENDED` and `OPERATIONAL` plus a separate system/robot state split; HexaMotion only defines `SYSTEM_PACKED/READY/RUNNING` in [src/hexamotion_constants.h](src/hexamotion_constants.h) and does not model `SUSPENDED/OPERATIONAL` semantics.
- OpenSHC enums not represented or only partially represented:
    - `PlannerMode` (OpenSHC) not present in HexaMotion [src/state_controller.h](src/state_controller.h).
    - `ParameterSelection` and `AdjustableParameter` map (OpenSHC) are missing; there is no adjustable parameter registry or selection workflow in [src/robot_model.h](src/robot_model.h).
    - `SequenceSelection` (OpenSHC) has no 1:1 equivalent; HexaMotion uses `SequenceType` but lacks OpenSHC sequence indexing behavior and progress semantics.
- OpenSHC `Parameters` includes the `Parameter<T>` wrapper, dynamic adjustment map, and config-backed maps for joints/links/leg IDs and stance positions; HexaMotion `Parameters` in [src/robot_model.h](src/robot_model.h) is static and lacks runtime-adjustable parameter infrastructure.
- OpenSHC `GaitDesignation`/`LegState`/`WalkState` enums do not align 1:1 with HexaMotion's `GaitType`, `LegState`, and `WalkState` naming and count (OpenSHC includes negative transition states and enum counts; HexaMotion omits transition states and uses different names).
- OpenSHC constants/macros from standard_includes.h (e.g., `UNASSIGNED_VALUE`, `UNDEFINED_POSITION`, `UNDEFINED_ROTATION`, `GRAVITY_ACCELERATION`, `THROTTLE_PERIOD`) are not mirrored in HexaMotion or have different values/units (e.g., `THROTTLE_PERIOD` differs and gravity is not a shared global).

### standard_includes.h (OpenSHC) vs HexaMotion platform layer

Missing parity (utility helpers):

- OpenSHC math helpers embedded in standard_includes.h (e.g., `clamped`, `smoothStep`, `mod`, `roundToEvenInt`, bezier helpers) are not 1:1 exposed in HexaMotion headers; some are in math_utils but not API-compatible.

### Pose class (OpenSHC pose.h) vs HexaMotion Pose

Missing or partial parity:

- OpenSHC `Pose::isValid()` and `Pose::Undefined()` are missing in HexaMotion.

### Model / Leg / Joint / Link / Tip

Missing or partial parity:

- OpenSHC’s `Model` owns `Leg`, `Joint`, `Link`, and `Tip` objects with per-joint state, publishers, and identity/current transforms (OpenSHC model.h). HexaMotion has `RobotModel` + `Leg` only and does not model `Joint`, `Link`, or `Tip` objects with per-joint state, desired state, or effort tracking.
- OpenSHC `Leg` stores and publishes tip force/torque measured and calculated, `admittance_state_`, and per-leg virtual mass/stiffness/damping ratio (OpenSHC model.h). HexaMotion `Leg` lacks tip force/torque vectors, admittance state, virtual stiffness/mass/damping, and per-leg step plane pose.
- OpenSHC joint state fields (`current_position_`, `desired_position_`, `effort_`, packed/unpacked positions) are not present in HexaMotion. There is no equivalent of joint state publishers in HexaMotion.
- OpenSHC `Model::generateWorkspaces()` performs per-leg workspace sampling and stores per-leg workspaces in each leg. HexaMotion has `WorkspaceAnalyzer` but does not expose a direct 1:1 `Model::generateWorkspaces()` pipeline nor per-leg workspace storage in `Leg` equivalent to OpenSHC’s `Workspace` map.
- OpenSHC `Model::estimateGravity()` exists in HexaMotion but is split across `TerrainAdaptation`/`IMUAutoPose` and does not live in `RobotModel` as a single cohesive method.
- OpenSHC `Model::initLegs(bool)` and `updateDefaultConfiguration()` have no direct equivalents in HexaMotion `RobotModel` (initialization paths are fixed in LocomotionSystem and BodyPoseController).

### StateController (OpenSHC state_controller.h/.cpp)

Missing or partial parity:

- OpenSHC `executePlan`, `targetConfigurationCallback`, `targetBodyPoseCallback`, `targetTipPoseCallback` are missing in HexaMotion.
- OpenSHC `system_state_` and `robot_state_` transitions rely on a richer start/stop flow and joint state acquisition; HexaMotion uses a simplified `StateController` without an equivalent system suspend/resume flow.
- OpenSHC `parameterSelectionCallback` / `parameterAdjustCallback` and the `AdjustableParameter` slew logic are not present; HexaMotion has no per-parameter incremental adjustment state or dynamic clamp.

### WalkController (OpenSHC walk_controller.h/.cpp)

Missing or partial parity:

- OpenSHC walk plane estimation is internal to `WalkController::updateWalkPlane()`. HexaMotion moves walk plane responsibility to `BodyPoseController` and does not provide a direct equivalent to OpenSHC’s `updateWalkPlane()` computation and storage in `WalkController` itself.
- OpenSHC `generateLimits()` takes a `StepCycle` and produces four bearing-based limit maps. HexaMotion uses `VelocityLimits` with similar intent but a different data model (no direct exposure of OpenSHC’s `LimitMap` structures or setter injection like `setLinearSpeedLimitMap()` in OpenSHC).
- OpenSHC `getWalkPlane()`/`getWalkPlaneNormal()` reflect internal estimates; HexaMotion returns values via BodyPoseController and does not persist a local walk plane in `WalkController`.

### LegStepper (OpenSHC walk_controller.h) vs HexaMotion LegStepper

Missing or partial parity:

- OpenSHC `LegStepper` explicitly exposes `getWalkPlane()` and `getWalkPlaneNormal()` derived from `WalkController`; HexaMotion stores the normal but does not keep a full walk plane equation like OpenSHC.
- OpenSHC `LegStepper` stores `Pose` for target/default/current tip; HexaMotion stores `Point3D` without full rotational pose tracking (no tip orientation handling).

### PoseController (OpenSHC pose_controller.h/.cpp) vs BodyPoseController

Missing or partial parity:

- OpenSHC `AutoPoser` class system (multiple independent auto pose curves driven by auto_pose.yaml) is not present in HexaMotion. HexaMotion uses a simplified `AutoPoseConfiguration` in [src/body_pose_config.h](src/body_pose_config.h) and a single-phase aggregation approach.
- OpenSHC `updateManualPose()` and `PoseResetMode` logic are not fully modeled in HexaMotion `BodyPoseController::updateCurrentPose()`; reset modes and explicit manual pose input handling are simplified or externalized.
- OpenSHC `updateInclinationPose()` and `updateIMUPose()` are partially present: HexaMotion has IMU pose PID and inclination correction, but the exact OpenSHC layering (manual -> imu -> inclination -> walk plane -> auto -> tip align -> ik error) and reset behavior is not fully replicated.
- OpenSHC `transitionConfiguration()` on full robot configuration and `transitionStance()` are not exposed at the same orchestration level. HexaMotion has LegPoser-level `transitionConfiguration()` but not a full controller-level equivalent of `PoseController::transitionConfiguration()` and `PoseController::transitionStance()` flows.
- OpenSHC `pose_phase_` and `pose_phase_length_` with multiple AutoPoser instances is only partially mirrored (HexaMotion stores phase data in AutoPoseConfiguration but not the AutoPoser container or per-poser life cycle).

### AdmittanceController (OpenSHC admittance_controller.h/.cpp)

Missing or partial parity:

- OpenSHC uses tip force estimates from joint effort or sensor data (`use_joint_effort`) with a configurable `force_gain` and Runge-Kutta integration via `integrator_step_time`. HexaMotion does not compute tip forces from joint effort and does not expose a `force_gain` or integrator step time equivalent.
- OpenSHC deadband (`ADMITTANCE_DEADBAND`) and per-axis delta clamp logic are not replicated in HexaMotion’s `AdmittanceController` API.
- OpenSHC dynamic stiffness uses step height ratios derived from `LegStepper` swing progression (z-diff from default). HexaMotion computes stiffness using workspace bounds and does not replicate OpenSHC’s step-reference logic.
- OpenSHC adjacency-based stiffness scaling (swing leg vs two adjacent legs) is only partially implemented; HexaMotion has `updateAdjacentLegStiffness` but does not map OpenSHC leg adjacency topology (explicit neighbor identification per leg index) in the same way.

## Detailed Method and Logic Gaps (Method-Level Analysis)

The following sections detail specific missing methods, member variables, and logic found by directly comparing `OpenSHC` headers/sources with `HexaMotion` equivalents.

### RobotModel (vs OpenSHC `Model`)

**Missing Methods:**

- `generateWorkspaces(void)`: OpenSHC generates per-leg workspaces suitable for the current configuration. HexaMotion relies on `WorkspaceAnalyzer` but lacks the central orchestration method in `RobotModel` to regenerate and store these for lookups.
- `legsBearingLoad(void)`: OpenSHC estimates if legs are supporting weight by comparing tip positions to the body plane (`HALF_BODY_DEPTH` threshold). This is missing in HexaMotion `RobotModel`, impacting "direct step" checks in pose control.
- `updateDefaultConfiguration(void)`: OpenSHC allows resetting the "default" joint configuration based on current positions. This is missing in HexaMotion.
- `initLegs(bool use_default_joint_positions)`: OpenSHC allows conditional initialization. HexaMotion `initialize()` is rigid.

**Logic Differences:**

- **IMU Handling**: OpenSHC `Model` owns `ImuData` and `estimateGravity()`. HexaMotion moves gravity estimation to `TerrainAdaptation` and `WalkController`, spreading the responsibility.
- **Leg Ownership**: OpenSHC `Model` owns `Joint`/`Link`/`Tip` hierarchies. HexaMotion `RobotModel` owns `Leg`s which have `Leg3D` references but lack the full separate object hierarchy for links (simplified 3DOF kinematics).
- **Workspace Caching**: OpenSHC caches per-leg `Workspace` in `Leg`; HexaMotion computes workspace in `WorkspaceAnalyzer` without persisting it per-leg.

### StateController

**Missing Methods:**

- `adjustParameter(enum, value)`: OpenSHC implements smooth "slew rate" adjustment of runtime parameters (e.g. changing step height over time). HexaMotion parameters are static or instantly changed; the smooth adjustment logic is missing.
- `executePlan(Plan)`: Logic to step through a high-level plan (sequence of targets) and publishing feedback is missing.

**Missing Constants/Members:**

- `SUSPENDED` state: OpenSHC logic handles a `SUSPENDED` system state where loops run but outputs are zeroed. HexaMotion has `SYSTEM_READDY/RUNNING` but no equivalent "hot standby" suspended logic.
- `target_configuration_` / `target_body_pose_`: Buffers for external planner targets are missing.
- `planner_mode_` state and planner callback plumbing are missing.

### WalkController

**Missing/Changed Logic:**

- **Walk Plane Estimation** (`updateWalkPlane`): OpenSHC calculates the `walk_plane_` internally using a least-squares fit of _default_ tip positions. HexaMotion delegates this to `BodyPoseController`, losing the independent "neutral plane" estimate relative to the legs themselves.
- **Velocity Input Modes**: OpenSHC has a "throttle" mode (scales linear speed by `1 - abs(angular_speed)`). HexaMotion only implements "real" mode (direct velocity clamping).
- **Limit Generation**: OpenSHC's `generateLimits(StepCycle, ...)` uses a specific lookahead based on `time_to_max_stride` per leg phase. HexaMotion `VelocityLimits` uses a simpler bounding box approach without the same temporal lookahead logic for "stance overshoot".
- **Gravity Alignment**: OpenSHC explicitly computes `identity_tip_pose_` by rotating stance positions by gravity during init. HexaMotion relies on `BodyPoseConfig` to provide these, potentially missing dynamic startup gravity compensation.

### AdmittanceController

**Missing Logic:**

- **Force Estimation**: OpenSHC can estimate tip forces from joint effort (`use_joint_effort`). HexaMotion relies exclusively on FSR/Sensors (interfaces) and has no internal effort-to-force projection logic.
- **Deadband**: `ADMITTANCE_DEADBAND` logic is missing in HexaMotion.
- **Adjacent Leg Stiffness**: OpenSHC `updateStiffness(walker)` increases stiffness on _adjacent_ legs (neighbors) when a leg swings. HexaMotion's `load_stiffness_scaler` applies to "loaded" legs but lacks the specific topological "neighbor" identification logic (e.g. increasing stiffness specifically on legs neighboring the lifting one).
- **Integration**: OpenSHC uses `boost::numeric::odeint`. HexaMotion uses custom `rungeKutta4`/`euler` methods. While functional, the exact solver behavior and parameters (like fixed step consistency vs boost's adaptive capabilities) differ.

### PoseController (vs OpenSHC `PoseController`)

**Missing functionality:**

- **AutoPoser**: The entire system of "Auto Pose Curves" (spline-based automatic body posing based on cycle time) is missing.
- **Reset Modes**: OpenSHC supports multiple reset modes (`RESET_SIT`, `RESET_NEUTRAL`, `RESET_DEFAULT`). HexaMotion `BodyPoseController` has simpler reset logic.
- **Direct Step Check**: Relies on `legsBearingLoad()` which is missing. The logic to decide whether to "step" or "slide" into a pose is absent.
- **Pose conversion and IMU pose pipeline**: OpenSHC converts between pose representations and runs a strict ordering of manual, IMU, inclination, walk-plane, auto, tip-align, and IK error poses. HexaMotion does not preserve the full ordering or reset semantics.

## Notes on New HexaMotion Components (not in OpenSHC)

HexaMotion adds or restructures functionality not present in OpenSHC:

- `WorkspaceAnalyzer` combines workspace, walkspace, and collision validation into a unified module (OpenSHC spreads this across multiple classes and debug utilities).
- `VelocityLimits`, `TerrainAdaptation`, `CartesianVelocityController`, `IMUAutoPose`, and `ManualBodyPoseController` provide MCU-appropriate replacements for OpenSHC behaviors, but they do not match OpenSHC API signatures or message-driven workflows.

## Validation Summary Against Current HexaMotion Headers

The gaps listed above were validated against:

- OpenSHC headers in OpenSHC/include/syropod_highlevel_controller.
- HexaMotion headers in src/ (robot_model.h, state_controller.h, walk_controller.h, body_pose_controller.h, leg_stepper.h, admittance_controller.h).

Confirmed as still missing or not 1:1:

- OpenSHC parameter/adjustable parameter infrastructure.
- OpenSHC `Model` hierarchy (Joint/Link/Tip), per-joint state fields, and per-leg workspace caches.
- `Pose::isValid()`/`Pose::Undefined()`.
- `PlannerMode`, `ParameterSelection`, and OpenSHC's parameter adjust workflow and slew logic.
- Walk plane estimation owned by WalkController.
- AutoPoser multi-curve system and full reset-mode handling in pose control.

Partially present but not identical:

- Walk/pose/admittance control logic exists, but ordering, parameterization, and internal state tracking differ from OpenSHC (notably walk plane storage, dynamic stiffness adjacency, and velocity input mixing).
- State machine and enums use different names and omit OpenSHC's negative transition states.

If strict, line-by-line parity across OpenSHC cpp local variables is required, a dedicated code diff pass (OpenSHC/src vs src) is still pending.

## CPP-Level Parity Findings (OpenSHC/src vs src)

This section captures additional gaps found by comparing OpenSHC implementations to HexaMotion implementations in the cpp files (method bodies, internal variables, and sequencing logic).

### StateController runtime flow

- OpenSHC `StateController::loop()` sequences `poser_->updateCurrentPose()`, `walker_->setPoseState()`, `generateExternalTargetTransforms()`, then `admittance_` updates before running state transitions in [OpenSHC/src/state_controller.cpp](OpenSHC/src/state_controller.cpp). HexaMotion updates are spread across LocomotionSystem and StateController without an equivalent ordered pipeline (pose -> walk -> admittance) in one place.
- OpenSHC transition logic explicitly calls `model_->updateDefaultConfiguration()` and `model_->generateWorkspaces()` when entering RUNNING or after direct startup, and regenerates `walker_->generateWalkspace()` in [OpenSHC/src/state_controller.cpp](OpenSHC/src/state_controller.cpp). HexaMotion does not call any `RobotModel`-level workspace regeneration or default-configuration update at these transition points.
- OpenSHC cruise control uses `cruise_control_end_time_` with `params_.cruise_control_time_limit` and throttles log output; HexaMotion has time limits but does not mirror OpenSHC’s exact throttle/logging control flow.
- OpenSHC `executePlan()` is a full planner step flow with target acquisition flags; HexaMotion has no planner step execution loop or equivalent plan-step state in [src/state_controller.cpp](src/state_controller.cpp).

### WalkController and LegStepper internal behavior

- OpenSHC walkspace generation uses per-leg `Workspace` projections and `getWorkplane()` interpolation inside `WalkController::generateWalkspace()` in [OpenSHC/src/walk_controller.cpp](OpenSHC/src/walk_controller.cpp). HexaMotion approximates this in `WorkspaceAnalyzer::generateWorkspace()` in [src/workspace_analyzer.cpp](src/workspace_analyzer.cpp), but does not store per-leg `Workspace` in `Leg` or use it in `WalkController` at runtime.
- OpenSHC `generateLimits()` computes `max_stance_extension`, `time_to_max_stride`, and stance overshoot using leg phase offsets in [OpenSHC/src/walk_controller.cpp](OpenSHC/src/walk_controller.cpp). HexaMotion uses a different overshoot and limit derivation in [src/velocity_limits.cpp](src/velocity_limits.cpp) that does not replicate OpenSHC’s phase-offset lookahead or overshoot per-leg calculation.
- OpenSHC `updateWalk()` supports `velocity_input_mode` = real/throttle and applies `body_velocity_scaler` before clamping. HexaMotion does not implement the OpenSHC throttle-mixing mode in [src/walk_controller.cpp](src/walk_controller.cpp).
- OpenSHC `updateWalkPlane()` uses least-squares fitting of default tip positions (per-walk update), while HexaMotion computes walk plane in `BodyPoseController::updateWalkPlanePose()` and terrain adaptation, not as a standalone walk-controller-owned estimate.

### PoseController sequencing and state

- OpenSHC `PoseController::executeSequence()` stores and reuses per-leg transition pose sequences, with safety factor decay and `transition_step_count_` optimization logic in [OpenSHC/src/pose_controller.cpp](OpenSHC/src/pose_controller.cpp). HexaMotion’s `BodyPoseController::executeSequence()` is simplified and does not reproduce the multi-step transition pose caching and safety-factor progression.
- OpenSHC `updateStance()` removes global auto pose then applies per-leg auto pose and transforms tip pose (position + rotation). HexaMotion stores `Point3D` only and omits tip orientation posing in [src/body_pose_controller.cpp](src/body_pose_controller.cpp).
- OpenSHC `updateManualPose()` applies reset modes and velocity limits to the manual pose at each cycle; HexaMotion’s manual pose is set externally and does not mirror the internal reset-state logic or transitions in [src/body_pose_controller.cpp](src/body_pose_controller.cpp).
- OpenSHC `updateTipAlignPose()` and `updateIKErrorPose()` operate on `Pose` rotations and use OpenSHC conventions for alignment. HexaMotion versions are simplified and do not replicate orientation handling or OpenSHC’s smoothing/decay semantics.

### AdmittanceController differences

- OpenSHC integrates a mass-spring-damper ODE per leg with `integrator_step_time` and Runge-Kutta 4, clamps delta per axis, and applies a deadband in [OpenSHC/src/admittance_controller.cpp](OpenSHC/src/admittance_controller.cpp). HexaMotion uses custom integration with workspace-based stiffness scaling and does not implement the same deadband and per-axis clamp behavior in [src/admittance_controller.cpp](src/admittance_controller.cpp).
- OpenSHC computes adjacency-based stiffness for the swinging leg and its immediate neighbors using explicit leg index topology. HexaMotion uses a similar idea but computes stiffness scale via workspace bounds and does not replicate OpenSHC’s z-diff step reference from the leg stepper.

### Model/Leg internals

- OpenSHC `Leg` tracks `current_position_`, `desired_position_`, `current_velocity_`, and `effort_` per joint, and updates these in `Leg::init()` and IK updates in [OpenSHC/src/model.cpp](OpenSHC/src/model.cpp). HexaMotion `Leg` stores only joint angles and tip positions without per-joint state or effort tracking in [src/leg.cpp](src/leg.cpp).
- OpenSHC uses `Model::updateModel()` to apply posed tip positions plus admittance deltas and then IK in a single pass. HexaMotion spreads this across `LocomotionSystem`, `LegStepper`, and `BodyPoseController`, without a single Model-level update loop.

### Debug and visualization

- OpenSHC provides a debug visualiser for robot model, walk plane, trajectories, workspaces, and forces in [OpenSHC/src/debug_visualiser.cpp](OpenSHC/src/debug_visualiser.cpp). HexaMotion has no equivalent debug visualizer implementation.

## Detailed Implementation Roadmap (1:1 Parity)

Based on the logical gaps identified above, the following tasks must be completed to achieve strict functional parity with OpenSHC:

### 1. RobotModel & Core Logic

- [ ] **Implement `legsBearingLoad()`**: Port the geometric check (tip z-pos vs body plane) to enable direct-step logic in `BodyPoseController`.
- [ ] **Restore `generateWorkspaces()`**: Centralize workspace generation in `RobotModel` to populate per-leg workspace caches, ensuring `StateController` can trigger regeneration.
- [ ] **Dynamic Initialization**: Implement `updateDefaultConfiguration()` and `initLegs(bool)` to allow resetting safe-start configurations at runtime.

### 2. State & Parameter Management

- [ ] **Slew Rate Logic**: Implement `adjustParameter()` in `StateController` with the specific "step-by-step" float adjustment logic for runtime smoothing.
- [ ] **Suspended State**: Add logic to handle a `SUSPENDED` system state where the standard update loop continues but actuator outputs are explicitly zeroed/locked.

### 3. Locomotion Logic (WalkController)

- [ ] **Internal Walk Plane Estimation**: Move the least-squares plane fit logic back into `WalkController::updateWalkPlane()` so it maintains its own independent estimate of the neutral plane derived from default tips.
- [ ] **Stance Overshoot Lookahead**: Update `generateLimits` or `VelocityLimits` to calculate `max_stance_extension` using the specific temporal lookahead (phase offsets) logic from OpenSHC.
- [ ] **Throttle Mode**: Add the "Throttle" velocity input mixing mode (`linear * (1 - abs(angular))`) to `updateWalk`.
- [ ] **Gravity Alignment**: Restore the specific `identity_tip_pose` calculation that rotates stance positions by gravity estimate during initialization.

### 4. Body Posing (PoseController)

- [ ] **Port AutoPoser**: Re-implement the `AutoPoser` class and its curve-based logic (splines) to drive body pose based on cycle time, replacing the current static implementation.
- [ ] **Reset Modes**: Implement the specific state machine logic for `RESET_SIT`, `RESET_NEUTRAL`, and `RESET_DEFAULT`.

### 5. Admittance Control

- [ ] **Force Estimation**: Implement the logic to estimate tip forces from joint efforts (even if using dummy effort values initially) and apply `force_gain`.
- [ ] **Deadband**: Apply `ADMITTANCE_DEADBAND` to force inputs before integration.
- [ ] **Neighbor Stiffness**: Implement the topology-aware stiffness adjustment that targets _adjacent_ legs specifically during swing phases.
