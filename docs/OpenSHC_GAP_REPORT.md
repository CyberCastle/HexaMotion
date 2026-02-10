# OpenSHC Parity Gap Report (HexaMotion)

## Context (from AGENTS.md)

HexaMotion is a 1:1 port of OpenSHC without ROS support. The goal is to run OpenSHC's locomotion logic on MCU targets such as the STM32H7 (Arduino Giga R1) for a hexapod robot with a hexagonal body, six legs spaced 60 degrees apart, and three joints per leg. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling. The interfaces `IIMUInterface`, `IFSRInterface` and `IServoInterface` must be implemented to connect the IMU, FSR sensors and smart servos.

Key differences from OpenSHC:

- Supports only 3DOF per leg.
- Supports only six legs.
- `LocomotionSystem` orchestrates the control classes, analogous to how a ROS script orchestrates publishers and subscriptions in OpenSHC.
- No YAML configuration files; everything is configured through the `Parameters` structure.
- OpenSHC logic is split into specific classes so the code is more readable and maintainable; the current HexaMotion organization follows this.
- Includes tests to verify hexapod kinematics and dynamics logic.
- Certain configurations are handled via factory patterns.
- No dynamic configuration support.

## Scope and Method

Compared OpenSHC headers in OpenSHC/include/syropod_highlevel_controller and sources in OpenSHC/src against HexaMotion sources in src. Focus is on public APIs, major state variables, and functional behavior. ROS-specific plumbing is called out as missing by design but still listed as parity gaps because OpenSHC depends on it.

## Gap Checklist by Module

### parameters_and_states.h (OpenSHC) vs HexaMotion enums/config

Missing or partial parity:

- OpenSHC `SystemState` includes `SUSPENDED` and `OPERATIONAL` plus a separate system/robot state split; HexaMotion only defines `SYSTEM_PACKED/READY/RUNNING` in [src/hexamotion_constants.h](src/hexamotion_constants.h) and does not model `SUSPENDED/OPERATIONAL` semantics.
- OpenSHC enums not represented or only partially represented:
    - `PlannerMode` (OpenSHC) not present in HexaMotion [src/state_controller.h](src/state_controller.h).
    - `ParameterSelection` and `AdjustableParameter` map (OpenSHC) are missing; there is no adjustable parameter registry or selection workflow in [src/robot_model.h](src/robot_model.h).
    - `SequenceSelection` (OpenSHC) has no 1:1 equivalent; HexaMotion uses `SequenceType` but lacks OpenSHC sequence indexing behavior and progress semantics.
- OpenSHC `Parameters` includes the ROS-backed `Parameter<T>` wrapper, dynamic adjustment map, and config-backed maps for joints/links/leg IDs and stance positions; HexaMotion `Parameters` in [src/robot_model.h](src/robot_model.h) is static and lacks runtime-adjustable parameter infrastructure.

### standard_includes.h (OpenSHC) vs HexaMotion platform layer

Missing parity (ROS platform features):

- No ROS publishers/subscribers, message types, TF broadcasters, or dynamic reconfigure server equivalents in HexaMotion. OpenSHC’s logging, TF framing, and topic interfaces have no MCU-level replacement layer.

### Pose class (OpenSHC pose.h) vs HexaMotion Pose

Missing or partial parity:

- OpenSHC `Pose` has geometry message conversions (`toPoseMessage`, `toTransformMessage`) and constructors from ROS types; HexaMotion `Pose` in [src/robot_model.h](src/robot_model.h) lacks these (expected without ROS, but a parity gap).
- OpenSHC `Pose::isValid()` and `Pose::Undefined()` are missing in HexaMotion.

### Model / Leg / Joint / Link / Tip

Missing or partial parity:

- OpenSHC’s `Model` owns `Leg`, `Joint`, `Link`, and `Tip` objects with per-joint state, publishers, and identity/current transforms (OpenSHC model.h). HexaMotion has `RobotModel` + `Leg` only and does not model `Joint`, `Link`, or `Tip` objects with per-joint state, desired state, or effort tracking.
- OpenSHC `Leg` stores and publishes tip force/torque measured and calculated, `admittance_state_`, and per-leg virtual mass/stiffness/damping ratio (OpenSHC model.h). HexaMotion `Leg` lacks tip force/torque vectors, admittance state, virtual stiffness/mass/damping, and per-leg step plane pose.
- OpenSHC joint state fields (`current_position_`, `desired_position_`, `effort_`, packed/unpacked positions) are not present in HexaMotion. There is no equivalent of joint state publishers in HexaMotion.
- OpenSHC `Model::generateWorkspaces()` performs per-leg workspace sampling and stores per-leg workspaces in each leg. HexaMotion has `WorkspaceAnalyzer` but does not expose a direct 1:1 `Model::generateWorkspaces()` pipeline nor per-leg workspace storage in `Leg` equivalent to OpenSHC’s `Workspace` map.

### StateController (OpenSHC state_controller.h/.cpp)

Missing or partial parity:

- ROS callback layer is absent: `systemStateCallback`, `robotStateCallback`, `bodyVelocityInputCallback`, `bodyPoseInputCallback`, gait selection, parameter adjust, planner interfaces, and sensor callbacks do not exist in HexaMotion because there is no ROS layer.
- Dynamic reconfigure support (`dynamic_reconfigure::Server` and `dynamicParameterCallback`) has no HexaMotion equivalent.
- Planner integration: OpenSHC `executePlan`, `targetConfigurationCallback`, `targetBodyPoseCallback`, `targetTipPoseCallback` are missing in HexaMotion.
- Publishing and debugging outputs (`publishVelocity`, `publishPose`, `publishWalkspace`, `publishRotationPoseError`, `publishFrameTransforms`, RVIZ helpers) have no HexaMotion equivalent.
- OpenSHC `system_state_` and `robot_state_` transitions rely on ROS start/stop flows and joint state acquisition; HexaMotion uses a simplified `StateController` without an equivalent system suspend/resume flow.

### WalkController (OpenSHC walk_controller.h/.cpp)

Missing or partial parity:

- OpenSHC external target transform support (frame IDs + transform cache and temporal transforms) is not implemented in HexaMotion. HexaMotion supports `ExternalTarget` in `TerrainAdaptation`, but does not transform targets across frames as OpenSHC does.
- OpenSHC walk plane estimation is internal to `WalkController::updateWalkPlane()`. HexaMotion moves walk plane responsibility to `BodyPoseController` and does not provide a direct equivalent to OpenSHC’s `updateWalkPlane()` computation and storage in `WalkController` itself.
- OpenSHC `generateLimits()` takes a `StepCycle` and produces four bearing-based limit maps. HexaMotion uses `VelocityLimits` with similar intent but a different data model (no direct exposure of OpenSHC’s `LimitMap` structures or setter injection like `setLinearSpeedLimitMap()` in OpenSHC).

### LegStepper (OpenSHC walk_controller.h) vs HexaMotion LegStepper

Missing or partial parity:

- OpenSHC `LegStepper` holds `ExternalTarget` with transform logic based on ROS time; HexaMotion `LegStepperExternalTarget` lacks any transform update mechanism.
- OpenSHC `LegStepper` explicitly exposes `getWalkPlane()` and `getWalkPlaneNormal()` derived from `WalkController`; HexaMotion stores the normal but does not keep a full walk plane equation like OpenSHC.

### PoseController (OpenSHC pose_controller.h/.cpp) vs BodyPoseController

Missing or partial parity:

- OpenSHC `AutoPoser` class system (multiple independent auto pose curves driven by auto_pose.yaml) is not present in HexaMotion. HexaMotion uses a simplified `AutoPoseConfiguration` in [src/body_pose_config.h](src/body_pose_config.h) and a single-phase aggregation approach.
- OpenSHC `updateManualPose()` and `PoseResetMode` logic are not fully modeled in HexaMotion `BodyPoseController::updateCurrentPose()`; reset modes and explicit manual pose input handling are simplified or externalized.
- OpenSHC `updateInclinationPose()` and `updateIMUPose()` are partially present: HexaMotion has IMU pose PID and inclination correction, but the exact OpenSHC layering (manual -> imu -> inclination -> walk plane -> auto -> tip align -> ik error) and reset behavior is not fully replicated.
- OpenSHC `transitionConfiguration()` on full robot configuration and `transitionStance()` are not exposed at the same orchestration level. HexaMotion has LegPoser-level `transitionConfiguration()` but not a full controller-level equivalent of `PoseController::transitionConfiguration()` and `PoseController::transitionStance()` flows.

### AdmittanceController (OpenSHC admittance_controller.h/.cpp)

Missing or partial parity:

- OpenSHC uses tip force estimates from joint effort or sensor data (`use_joint_effort`) with a configurable `force_gain` and Runge-Kutta integration via `integrator_step_time`. HexaMotion does not compute tip forces from joint effort and does not expose a `force_gain` or integrator step time equivalent.
- OpenSHC deadband (`ADMITTANCE_DEADBAND`) and per-axis delta clamp logic are not replicated in HexaMotion’s `AdmittanceController` API.
- OpenSHC dynamic stiffness uses step height ratios derived from `LegStepper` swing progression (z-diff from default). HexaMotion computes stiffness using workspace bounds and does not replicate OpenSHC’s step-reference logic.

### DebugVisualiser (OpenSHC debug_visualiser.h/.cpp)

Missing parity:

- No HexaMotion equivalent to RVIZ marker generation, walk plane visualization, workspace visualization, stride/torque plotting, or gravity/terrain markers.

### ROS Integration (OpenSHC main.cpp)

Missing parity:

- No equivalent to OpenSHC’s ROS main loop setup (`ros::init`, topic subscriptions, `ros::Rate`, or `ros::spinOnce`). HexaMotion depends on MCU loop integration via `LocomotionSystem::update()` instead.

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

### StateController

**Missing Methods:**

- `adjustParameter(enum, value)`: OpenSHC implements smooth "slew rate" adjustment of runtime parameters (e.g. changing step height over time). HexaMotion parameters are static or instantly changed; the smooth adjustment logic is missing.
- `executePlan(Plan)`: Logic to step through a high-level plan (sequence of targets) and publishing feedback is missing.
- `publish*`: All ROS publishing methods (`publishVelocity`, `publishPose`, etc.) are missing. While expected, the _metrics calculation_ logic inside them (e.g. `rotation_pose_error`) is also gone.

**Missing Constants/Members:**

- `SUSPENDED` state: OpenSHC logic handles a `SUSPENDED` system state where loops run but outputs are zeroed. HexaMotion has `SYSTEM_READDY/RUNNING` but no equivalent "hot standby" suspended logic.
- `target_configuration_` / `target_body_pose_`: Buffers for external planner targets are missing.

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

## Notes on New HexaMotion Components (not in OpenSHC)

HexaMotion adds or restructures functionality not present in OpenSHC:

- `WorkspaceAnalyzer` combines workspace, walkspace, and collision validation into a unified module (OpenSHC spreads this across multiple classes and ROS debug tools).
- `VelocityLimits`, `TerrainAdaptation`, `CartesianVelocityController`, `IMUAutoPose`, and `ManualBodyPoseController` provide MCU-appropriate replacements for ROS-centric behaviors, but they do not match OpenSHC API signatures or message-driven workflows.

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
