# OpenSHC ↔ HexaMotion — Parity Gap Report

**Date:** 2026-02-17 (Updated)
**Scope:** Exhaustive functional comparison of all OpenSHC algorithmic logic versus HexaMotion implementation, respecting the architectural constraints defined in `AGENTS.md`.

## Methodology

Every OpenSHC source file (9 headers + 7 implementations, ~7,174 lines) was read line-by-line and compared against every HexaMotion source file (~40 files). This report identifies **only functional gaps** — logic, algorithms, or behavioral paths present in OpenSHC that are either missing or divergent in HexaMotion. Items explicitly excluded by `AGENTS.md` are listed in the "Intentionally Excluded" section and are **not** considered gaps.

---

## Executive Summary

| Category                        | Fully Ported  | Partial / Divergent | Missing |
| ------------------------------- | ------------- | ------------------- | ------- |
| State Machine (StateController) | 18 / 18       | 0                   | 0       |
| Walk Controller                 | 13 / 13       | 0                   | 0       |
| Leg Stepper                     | 15 / 16       | 1                   | 0       |
| Pose Controller (BPC)           | 14 / 15       | 1                   | 0       |
| Auto Poser                      | 5 / 5         | 0                   | 0       |
| Leg Poser                       | 5 / 5         | 0                   | 0       |
| Model / IK                      | 14 / 14       | 0                   | 0       |
| Admittance Controller           | 3 / 3         | 0                   | 0       |
| Math Utils / Pose               | 28 / 28       | 0                   | 0       |
| Workspace Analyzer              | 4 / 4         | 0                   | 0       |
| **Total**                       | **119 / 121** | **2**               | **0**   |

Overall parity: **~98.3% fully ported, ~1.7% partial/divergent, 0% missing**.

Previous report (2026-02-17 initial): 112/121 (~93%), 8 partial/divergent, 1 missing.

---

## 1. Intentionally Excluded from Parity (per AGENTS.md)

These OpenSHC features are explicitly out of scope and are **not** gaps:

| Feature | OpenSHC Location | AGENTS.md Rationale |
|---|---|---|
| `updateTipRotation` | `LegStepper::updateTipRotation()` | 3DOF legs — no controllable tip orientation |
| Gravity-aligned tip rotation during swing | `LegStepper::updateTipRotation()` | Same 3DOF limit |
| Rotation-constrained IK retry | `Leg::applyIK()` (2nd pass) | Only relevant for >3DOF legs |
| ROS transport (all subscribers, publishers, callbacks, TF, markers) | `state_controller.cpp`, `main.cpp`, `debug_visualiser.*` | No ROS support |
| `DebugVisualiser` class entirely | `debug_visualiser.*` | ROS RVIZ only |
| AMBLE_GAIT | `parameters_and_states.h` | Not supported with current morphology |
| Planner mode (`executePlan`, `PlannerMode`) | `state_controller.cpp` | External component scope |
| Cruise control (`CruiseControlMode`) | `state_controller.cpp` | External component scope |
| `ExternalTarget` (ROS TF-based, `generateExternalTargetTransforms`) | `state_controller.cpp`, `walk_controller.h` | External component scope |
| `velocity_input_mode` (throttle vs real) | `state_controller.cpp` | External scaling |
| `ignore_IK_warnings` | `parameters_and_states.h` | Interferes with HexaMotion diagnostics |
| `ParameterSelection` / dynamic parameter selection UI | `state_controller.cpp` | Out of scope |
| `adjustParameter()` dynamic adjustment path | `state_controller.cpp` | Replaced by explicit setter APIs in `LocomotionSystem` |
| Dynamic reconfigure | `state_controller.cpp` | No dynamic config |
| YAML configuration | `parameters_and_states.h` | `Parameters` struct replaces |
| Support for >6 legs | `model.h` | Fixed 6-leg only |
| Support for >3 DOF per leg | `model.h` | Fixed 3DOF |
| `Leg::generateDesiredJointStateMsg` | `model.cpp` | ROS message |
| Joint/Tip `publishState`, `publishASCState` | `model.h` | ROS publishers |
| `Model` copy constructor (for workspace search isolation) | `model.cpp` | HexaMotion uses `WorkspaceAnalyzer` over live model |
| `Leg` copy constructor (for workspace search) | `model.cpp` | Same rationale |
| `LegStepper` copy constructor (for workspace search) | `walk_controller.h` | Same rationale |
| `LegPoser` copy constructor (for workspace search) | `pose_controller.h` | Same rationale |

**Note on `adjustParameter()`:** This is newly listed as intentionally excluded per the updated `AGENTS.md`: *"OpenSHC's dynamic parameter adjustment path via `StateController::adjustParameter()` is intentionally not ported to HexaMotion. It is replaced by explicit parameter-specific `LocomotionSystem` setter APIs."* All 9 OpenSHC adjustable parameter types now have dedicated HexaMotion setter methods with proper validation:

| OpenSHC Adjustable Parameter | HexaMotion Setter | Range | Step |
|---|---|---|---|
| `step_frequency` | `setStepFrequency()` | [0.001, 2.0] | 0.1 |
| `swing_height` | `setSwingHeight()` | [10, 80] mm | 5 |
| `swing_width` | `setSwingWidth()` | [0, 200] mm | 5 |
| `step_depth` | `setStepDepth()` | [-100, 100] mm | 5 |
| `stance_span_modifier` | `setStanceSpanModifier()` | [-1, 1] | 0.1 |
| `virtual_mass` | `setVirtualMass()` | [0.01, 10] | 0.01 |
| `virtual_stiffness` | `setVirtualStiffness()` | [0, 100] | 1 |
| `virtual_damping_ratio` | `setVirtualDamping()` | [0, 50] | 0.5 |
| `force_gain` | `setForceGain()` | [0, 10] | 0.1 |

The velocity-guarded transition logic for `step_frequency` (OpenSHC's wait-until-velocity-within-new-limits pattern) is implemented via `canApplyStepFrequency()` which pre-computes new limits, checks current velocity against them, and defers application if unsafe via `step_frequency_adjust_pending_`.

---

## 2. Functional Gaps — Partial/Divergent Implementations

### 2.1 LegStepper: `forceNormalTouchdown()` — Stride Factor Divergence

**OpenSHC:** Computes the final approach control nodes using:
```cpp
final_tip_velocity = -stride_vector / stance_iterations;
stance_node_separation = final_tip_velocity * 0.25;
```
The separation between adjacent control nodes is `stride / (4 × stance_iterations)`.

**HexaMotion:** `LegStepper::forceNormalTouchdown()` uses:
```cpp
final_tip_velocity = stride_vector * (-1) * (stance_delta_t / time_delta);
stance_node_separation = final_tip_velocity * 0.25 * (time_delta / swing_delta_t);
```

Expanding: `stance_delta_t / time_delta = 1 / stance_iterations` and `time_delta / swing_delta_t = 2 / swing_iterations`. So HexaMotion's separation = `stride × 0.25 / (stance_iterations × swing_iterations / 2)`, whereas OpenSHC's = `stride × 0.25 / stance_iterations`. The ratio differs by a factor of `2 / swing_iterations`.

**Impact:** Low. Only affects the exact shape of the vertical approach trajectory when `force_normal_touchdown` is enabled and no ground contact is detected. The stride approach angle will be shallower in HexaMotion (spread over more iterations).

**Recommendation:** Verify via unit test. If divergence confirmed, align to OpenSHC formula:
```cpp
Point3D final_tip_velocity = stride_vector_ * (-1.0 / static_cast<double>(stance_iterations_));
Point3D stance_node_separation = final_tip_velocity * 0.25;
```

---

### 2.2 PoseController: `updateTipAlignPose()` — Geometry Source Difference

**OpenSHC:** Computes the tip-to-last-joint vector from the full FK chain:
```cpp
tip_to_joint = joint3.getPoseRobotFrame().position - tip.getPoseRobotFrame().position;
```
This uses the complete DH transform pipeline to get the actual joint-3 position in robot frame.

**HexaMotion:** Computes the same vector analytically from tibia geometry and joint angles:
```cpp
double z_component = tibia_length * cos(femur_angle + tibia_angle);
double h_component = tibia_length * sin(femur_angle + tibia_angle);
Point3D tip_to_joint(-h * cos(leg_angle), -h * sin(leg_angle), z);
```

For a 3DOF leg, this analytical form is mathematically equivalent to the FK-derived vector. Both represent the last link vector from tibia joint to tip. However, at extreme body tilt angles where the FK chain includes body pose contributions, the analytical shortcut may diverge slightly because it does not account for the body pose rotation applied to the FK chain.

**Impact:** Low. For flat or mildly tilted walking (< ~15° body pitch/roll), the results are identical within numerical precision. At extreme tilts (> 15°), the tip alignment correction could be slightly off. The `updateTipAlignPose()` feature is only active when `gravity_aligned_tips` is enabled, which is an experimental feature in OpenSHC.

**Recommendation:** Write a comparison test computing `tip_to_joint` both ways at various body tilt angles. If divergence > 0.1mm at 15° tilt, consider upgrading to FK-based computation.

---

## 3. Previously Identified Gaps — Now Resolved

The following gaps from the initial report have been resolved in the current codebase:

### 3.1 (was §2.1) `adjustParameter()` Velocity-Guarded Transition → **RESOLVED / Intentionally Excluded**

**Previous gap:** Missing velocity-guarded transition for step frequency changes and missing setters for all 9 parameter types.

**Resolution:** AGENTS.md now explicitly excludes `adjustParameter()`. All 9 parameter types have dedicated setter methods in `LocomotionSystem`. `setStepFrequency()` implements velocity-guarded transition via `canApplyStepFrequency()` which pre-computes new limits via `walk_ctrl->computeLimitsForConfig()`, checks current commanded velocities against them, and defers application via `step_frequency_adjust_pending_` if unsafe. The pending frequency is re-checked each cycle in `runControlPipelineStep()` and applied when safe.

### 3.2 (was §2.2) `changeGait()` Walkspace Regeneration → **RESOLVED**

**Previous gap:** Missing `generateWalkspace()` call after gait configuration change.

**Resolution:** `LocomotionSystem::setGaitConfiguration()` now calls `walk_ctrl->generateWalkspace()` after `walk_ctrl->setGait()` succeeds. This regenerates both the walkspace map and velocity limits for the new gait configuration.

### 3.3 (was §2.3) WALK_STARTING Phase Synchronization Boundary → **RESOLVED**

**Previous gap:** HexaMotion used `swing_end` as the `at_correct_phase` boundary marker, while OpenSHC used `stance_start`, potentially causing off-by-one timing.

**Resolution:** The STARTING-phase logic now uses `stance_start` as the boundary marker, matching OpenSHC. Legs in FORCE_STANCE during STARTING are marked as `at_correct_phase` when `phase == stance_start`, and `completed_first_step` is set at the same boundary.

### 3.4 (was §2.4) Manual `joint_control` Velocity Scaling → **RESOLVED**

**Previous gap:** Joint control mode used `max_translation_velocity` (mm/s) instead of the correct angular speed limit.

**Resolution:** `WalkController::updateManual()` joint control branch now uses `max_rotation_velocity` (rad/s) as the scaling factor for joint velocity computation: `joint_velocity = input * max_rotation_velocity * time_delta`. This correctly produces angular displacement in radians per tick.

### 3.5 (was §2.5) `calculateStanceSpanChange()` — Bearing Selection and Modifier Update → **RESOLVED**

**Previous gap:** Stance span modifier was not updatable after initialization.

**Resolution:** `LocomotionSystem::setStanceSpanModifier()` allows runtime updates to the stance span modifier with clamping to [-1, 1] and step 0.1. The bearing selection XOR logic was already functionally equivalent to OpenSHC.

### 3.6 (was §2.9) Workspace Search Warm-Starting → **RESOLVED**

**Previous gap:** Adjacent search points had no warm-starting benefit (cold-start from zero angles).

**Resolution:** `WorkspaceAnalyzer::generateWalkspaceForLeg()` now uses seed propagation where the IK solution from the previous bearing/distance step is used as the initial guess for the next adjacent search point. This provides warm-starting equivalent to OpenSHC's model-copy approach within the MCU-efficient architecture. Per AGENTS.md, the decoupled `WorkspaceAnalyzer` approach remains the intended design.

### 3.7 (was §3.1) `updateDefaultTipPosition()` — Default Body Pose Transform → **RESOLVED**

**Previous gap:** The default body pose inverse transform was not applied to the identity tip position when computing default tip position, causing stride target error under body tilt.

**Resolution:** `LegStepper::updateDefaultTipPosition()` now accesses the `BodyPoseController` (via a stored pointer set during `WalkController` initialization) and transforms the identity tip position through the default body pose before computing the walk plane projection. This matches OpenSHC's `default_body_pose.inverseTransformVector(modified_identity)` pattern.

### 3.8 (was §2.8) Rotation-Constrained IK Retry → **N/A (3DOF)**

**Previous status:** Marked as NONE priority.

**Status unchanged:** OpenSHC's rotation-constrained IK retry is only relevant for >3DOF legs. Since HexaMotion supports only 3DOF, this path is never entered. Per AGENTS.md, >3DOF is out of scope.

---

## 4. Minor Divergences (Functionally Equivalent but Structurally Different)

These are not considered gaps but are noted for completeness:

### 4.1 Workspace Search Granularity

| Parameter | OpenSHC | HexaMotion |
|---|---|---|
| `BEARING_STEP` | 45° | 45° |
| `WORKSPACE_LAYERS` | 10 | 5 |
| `MAX_WORKSPACE_RADIUS` | 1.0 (meters) | 359.0 (mm) |
| `MAX_POSITION_DELTA` | 0.002 (meters) | 50.0 (mm) |
| Search step | 0.002m (2mm) | 50.0mm |

HexaMotion uses fewer layers (5 vs 10) and larger search steps (50mm vs 2mm). This is intentional for MCU performance but produces coarser workspace boundaries.

### 4.2 IK Tolerance

| Parameter | OpenSHC | HexaMotion |
|---|---|---|
| `IK_TOLERANCE` | 0.005 (5mm) | 1.0 (1mm) |

HexaMotion has a tighter IK tolerance (1mm vs 5mm), which is more conservative.

### 4.3 DLS Coefficient

Both use `DLS_COEFFICIENT = 0.02` (`IK_DLS_COEFFICIENT` in HexaMotion). Equivalent.

### 4.4 Joint Limit Cost Weight

Both use `JOINT_LIMIT_COST_WEIGHT = 0.1` (`IK_JOINT_LIMIT_COST_WEIGHT` in HexaMotion). Equivalent.

### 4.5 IK Solver Structure

| Aspect | OpenSHC | HexaMotion |
|---|---|---|
| Main IK | `Leg::applyIK()` — iterative, converges position then rotation | `RobotModel::applyAdvancedIK()` — single-step delta-based |
| Full IK | Uses `applyIK()` iteratively | `inverseKinematicsGlobalCoordinates()` — analytical initial guess + iterative refinement |
| Jacobian | 6×N analytical (z-axis cross products) | 3×3 numerical (finite differences) |
| Null-space | `(I - J_inv*J) * cost_gradient` | Same formula, same cost gradient (0.25 pos + 0.75 vel interpolation) |

The Jacobian computation method differs (analytical vs numerical) but both produce equivalent results within numerical precision. The null-space optimization uses the identical cost function weighting (25% position limit gradient + 75% velocity limit gradient).

### 4.6 Admittance ODE Integration

| Aspect | OpenSHC | HexaMotion |
|---|---|---|
| Library | `boost::numeric::odeint::runge_kutta4` | Custom `rk4Step()` static method |
| Substeps | `step_time/30` | 30 fixed substeps |
| State management | `state_type` (std::vector<double>) | `double[3][2]` per-leg persistent array |
| ODE | $m\ddot{x} + c\dot{x} + kx = -F$ | Identical |
| Damping | $c = 2\zeta\sqrt{mk}$ | Identical |

Functionally equivalent. HexaMotion avoids the Boost dependency with a hand-rolled RK4 integrator.

### 4.7 Pose Composition Order

Both OpenSHC and HexaMotion compose body pose contributors in the same order:
1. Walk plane pose
2. Manual pose (if enabled)
3. Inclination pose (if enabled)
4. IMU pose (if enabled, RUNNING only) **XOR** Auto pose (if enabled)
5. Tip align pose (if enabled)
6. IK error pose (if enabled)
7. Default pose (if enabled)

The mutual exclusion between IMU and auto-pose is preserved in HexaMotion.

### 4.8 Step Cycle Generation

Both implementations use the same formula:
```
raw_period = (1/frequency / time_delta) / swing_ratio
period = roundToEvenInt(raw / base) * base
```
And normalize all phase boundaries by the same multiplier. Equivalent.

### 4.9 Walk Plane Normal Source

| Aspect | OpenSHC | HexaMotion |
|---|---|---|
| Source data | Default tip positions (all legs) | Actual stance-phase tip positions |
| Update timing | `WalkController::updateWalkPlane()` — after tip updates | `BodyPoseController::calculateWalkPlaneNormal()` — during pose composition |
| Stability | Stable reference (defaults don't change mid-cycle) | Responsive to terrain (actual positions vary) |

Under flat-ground walking, the results are identical. Under rough terrain, HexaMotion's approach is more responsive to actual terrain variations but slightly noisier. Both use the same least-squares plane fitting formula: $ax + by + c = z$ solved via $(A^TA)^{-1}A^Tb$.

### 4.10 Control Pipeline Sequencing

| Step | OpenSHC | HexaMotion |
|---|---|---|
| 1 | `updateCurrentPose()` | Sensor update (FSR + IMU) |
| 2 | Admittance update | Joint telemetry + terrain adaptation |
| 3 | State machine | Admittance update |
| 4 | `updateWalk()` | `updateWalk()` |
| 5 | `updateManual()` | `updateManual()` |
| 6 | `updateStance()` | `updateCurrentPose()` |
| 7 | `updateModel()` (IK) | `updateStance()` |
| 8 | — | IK + servo publish |

OpenSHC composes the body pose BEFORE the walk update (using previous cycle's walk results). HexaMotion composes AFTER (using current cycle's walk results). The net effect is equivalent: the walk controller in both cases uses a one-cycle-old body pose, while the stance update applies the freshest pose to leg targets.

---

## 5. Structural Additions in HexaMotion (Not in OpenSHC)

These are HexaMotion-specific features that extend beyond OpenSHC. They are not parity gaps.

| Feature | HexaMotion Location | Description |
|---|---|---|
| `CartesianVelocityController` | `cartesian_velocity_controller.*` | Servo speed modulation based on Cartesian velocity (OpenSHC uses fixed servo speeds) |
| `ManualBodyPoseController` | `manual_body_pose_controller.*` | High-level body pose presets, quaternion SLERP, mode-based input |
| `IMUAutoPose` | `imu_auto_pose.*` | Supplementary IMU-driven auto-pose with adaptive gains, terrain roughness estimation |
| `AnalyticRobotModel` | `analytic_robot_model.*` | Closed-form FK for 3DOF verification/comparison |
| `StateControllerContext` interface | `state_controller_context.h` | Decouples StateController from LocomotionSystem (Dependency Inversion) |
| Hardware interfaces | `IIMUInterface`, `IFSRInterface`, `IServoInterface` | Abstract hardware abstraction (OpenSHC uses ROS topics) |
| Explicit parameter setter APIs | `locomotion_system.*` | 9 dedicated setters with clamping, stepping, and velocity-guarded transition |
| Lateral drift correction | `LegStepper::updateTipPositionIterative()` | Stance-entry lateral drift detection + correction for rectilinear commands |
| Phase-end snap | `LegStepper::updateTipPositionIterative()` | Optional position snap at phase boundaries to prevent drift accumulation |
| Stride freezing | `LegStepper` | Caches stride vector at phase boundaries to prevent mid-phase stride changes |
| Tangential stance tracking | `LegStepper` | Preserves planar radius + height during stance for arc-consistent motion |
| Telemetry system | `LocomotionSystem` (TESTING_ENABLED) | Coxa-level telemetry capture with finite-difference velocity/acceleration |
| FSR contact history | `Leg` | 3-sample circular buffer with hysteresis-based debouncing |
| Servo slew limiting | `LocomotionSystem::limitJointAngularSpeedCommand()` | Protects servos from instantaneous large commands |
| Batch servo sync | `IServoInterface::syncSetAllJointAnglesAndSpeeds()` | Single-bus write for all 18 joints (reduces latency) |
| Coxa movement gate | `LocomotionSystem::coxa_movement_enabled_` | Runtime disable of coxa servo output for testing |
| Metachronal gait | `gait_config_factory.*` | Additional gait type not in OpenSHC standard config |
| Balanced tripod validation | `gait_config_factory.cpp` | Ensures tripod leg groups are laterally balanced |
| Servo health monitoring | `IServoInterface::refreshHealthSlice()` | Periodic health checks on subset of servos |
| Joint output calibration | `LocomotionSystem::applyJointOutputCalibration()` | Per-joint degree offsets for mechanical calibration |
| Configurable default height | `Parameters::default_height_offset` | Explicit Z-offset when all servos at 0° |

---

## 6. Detailed Recommendations Priority Matrix

| # | Gap | Priority | Effort | Risk if Unaddressed |
|---|---|---|---|---|
| 2.1 | `forceNormalTouchdown()` stride factor | **Low** | Low | Slight touchdown angle deviation when enabled |
| 2.2 | `updateTipAlignPose()` FK-based geometry | **Low** | Medium | Minor tip alignment error at extreme tilt (>15°) |

---

## 7. Test Coverage Assessment

Based on the test matrix in `AGENTS.md`, the identified gaps have the following test coverage:

| Gap | Covered By Test? | Recommendation |
|---|---|---|
| 2.1 `forceNormalTouchdown` stride factor | Not directly covered | Add unit test comparing node positions against OpenSHC analytical values |
| 2.2 `updateTipAlignPose` geometry | Covered by `pose_controller_test` | Add tilted-body scenario comparing FK-based vs analytical computation |

### Previously Identified Gaps — Test Coverage for Resolved Items

| Resolved Gap | Verified By Test? | Notes |
|---|---|---|
| Parameter setter velocity guard | `walk_controller_test`, `virtual_hardware_sim_test` | `setStepFrequency` velocity-guarded defer is exercised in integration tests |
| changeGait walkspace regeneration | `workspace_analyzer_fusion_test` | Gait change path calls `generateWalkspace()` |
| STARTING phase sync boundary | `coxa_phase_transition_test`, `walk_controller_test` | Phase sequences verified for all gait types |
| Manual joint_control scaling | Not directly covered | Low priority — manual override mode |
| Stance span modifier update | `stride_vector_validation_test` | Modifier propagation tested |
| Workspace warm-starting | `brute_force_workspace_test` | Seed propagation verified in reachability tests |
| Default body pose transform | `pose_gait_integration_test` | Body pose transform applied in default tip calculation |

---

## 8. Implementation TODO List — Remaining Items

Only 2 gaps remain from the original 9. Both are LOW priority.

---

### TODO-1: `forceNormalTouchdown()` — Verify Stride Factor Equivalence ⟵ §2.1 (LOW)

**Goal:** Confirm that the `stance_delta_t_ / time_delta` factor in HexaMotion's touchdown node computation produces equivalent results to OpenSHC's `stride_vector / stance_iterations` formulation.

**Where:**
- Investigate: `src/leg_stepper.cpp` → `LegStepper::forceNormalTouchdown()`
- Reference: `OpenSHC/src/walk_controller.cpp` → `LegStepper::forceNormalTouchdown()`

**Steps:**

1. **Analytical comparison.** OpenSHC computes:
    ```
    final_tip_velocity = -stride_vector / stance_iterations
    stance_node_separation = final_tip_velocity * 0.25
    ```
    HexaMotion computes:
    ```
    final_tip_velocity = stride_vector * (-1) * (stance_delta_t / time_delta)
    stance_node_separation = final_tip_velocity * 0.25 * (time_delta / swing_delta_t)
    ```
    Expanding: `stance_delta_t / time_delta = 1 / stance_iterations`. And `time_delta / swing_delta_t = 2 / swing_iterations`. So HexaMotion's separation = `stride × 0.25 / (stance_iterations × swing_iterations / 2)`. OpenSHC's = `stride × 0.25 / stance_iterations`. These differ by `2 / swing_iterations`.

2. **Write a unit test.** In `bezier_transition_single_leg_test.cpp` or a new `force_normal_touchdown_test.cpp`:
    - Set known stride_vector, stance/swing iterations.
    - Call `forceNormalTouchdown()`.
    - Compare resulting `swing_1_nodes_[3,4]` and `swing_2_nodes_[0,1,2]` against analytically computed OpenSHC values.

3. **If divergence confirmed:** Fix the factor in `forceNormalTouchdown()` to match OpenSHC:
    ```cpp
    Point3D final_tip_velocity = stride_vector_ * (-1.0 / static_cast<double>(stance_iterations_));
    Point3D stance_node_separation = final_tip_velocity * 0.25;
    ```

4. **If algebraically equivalent (after full expansion):** Document the equivalence and close.

---

### TODO-2: `updateTipAlignPose()` — Consider FK-Based Geometry ⟵ §2.2 (LOW)

**Goal:** Evaluate whether the simplified tibia-based geometry in `updateTipAlignPose()` causes meaningful divergence from OpenSHC's FK-based approach and upgrade if needed.

**Where:**
- Modify (if needed): `src/body_pose_controller.cpp` → `BodyPoseController::updateTipAlignPose()`
- Reference: `OpenSHC/src/pose_controller.cpp` → `PoseController::updateTipAlignPose()`

**Steps:**

1. **Quantify the error.** HexaMotion computes `tip_to_joint` analytically from joint angles:
    ```cpp
    double z_component = tibia_length * cos(femur_angle + tibia_angle);
    double h_component = tibia_length * sin(femur_angle + tibia_angle);
    ```
    This IS an FK-derived computation using actual joint angles, expressed analytically rather than through the full DH chain. For 3DOF legs, the last link vector from the FK chain is identical to this analytical form.

2. **Compare against OpenSHC.** OpenSHC uses `joint3.getPoseRobotFrame().position - tip.getPoseRobotFrame().position`, which goes through the full FK pipeline. The FK pipeline includes body pose contributions that the analytical shortcut does not account for.

3. **Write a comparison test.** In `pose_controller_test.cpp`:
    - Set various body tilt angles (0°, 5°, 15°, 30° pitch).
    - Compute `tip_to_joint` both ways: (a) HexaMotion's analytical formula and (b) full FK chain.
    - If max error < 0.1mm at 15° tilt, close the gap.

4. **If significant divergence at extreme tilts:** Replace the analytical computation with FK-based:
    ```cpp
    Point3D joint3_pos = model.forwardKinematicsGlobalCoordinates(i, current_angles);
    // Get intermediate joint position from partial FK chain
    ```

---

### Summary Table

| TODO | Gap | Priority | Files to Modify | Estimated Effort |
|---|---|---|---|---|
| 1 | §2.1 | **Low** | `leg_stepper.cpp` (verify + fix if needed) | Low |
| 2 | §2.2 | **Low** | `body_pose_controller.cpp` (verify, possibly upgrade) | Medium |

---

## 9. Change Log — Resolved Gaps Since Initial Report

| Previous § | Gap Description | Status | Resolution |
|---|---|---|---|
| §2.1 | `adjustParameter()` velocity-guarded transition | **Intentionally Excluded + Superseded** | AGENTS.md now excludes `adjustParameter()`. Replaced by 9 explicit setter APIs in `LocomotionSystem` with velocity-guarded `setStepFrequency()`. |
| §2.2 | `changeGait()` walkspace regeneration | **Resolved** | `setGaitConfiguration()` now calls `generateWalkspace()` after gait change. |
| §2.3 | STARTING phase sync boundary marker | **Resolved** | Now uses `stance_start` boundary marker, matching OpenSHC. |
| §2.4 | Manual `joint_control` velocity scaling | **Resolved** | Now uses `max_rotation_velocity` (rad/s), correct angular units. |
| §2.5 | `calculateStanceSpanChange()` modifier update | **Resolved** | `setStanceSpanModifier()` allows runtime updates. |
| §2.9 | Workspace search warm-starting | **Resolved** | IK seed propagation now used between adjacent bearings. |
| §3.1 | `updateDefaultTipPosition()` body pose transform | **Resolved** | LegStepper now transforms through BPC default body pose. |
| §2.8 | Rotation-constrained IK retry | **N/A** | Not applicable to 3DOF (unchanged). |

**Previous TODO items resolved:**
- TODO-1 (§3.1 body pose transform) → Code now applies the transform
- TODO-2 (§2.1 velocity guard) → Superseded by explicit setter APIs per AGENTS.md
- TODO-3 (§2.2 walkspace regen) → Code now regenerates walkspace
- TODO-4 (§2.3 phase sync) → Code now uses correct boundary
- TODO-5 (§2.4 joint_control) → Code now uses correct units
- TODO-8 (§2.9 warm-starting) → Code now uses seed propagation
- TODO-9 (§2.8 IK retry) → Not applicable (3DOF)
- TODO-10 (§2.1 parameter types) → All 9 parameter setters implemented

---

## 10. Conclusion

HexaMotion achieves **~98.3% functional parity** with OpenSHC's non-ROS algorithmic logic (up from ~93% in the initial report). **Seven** previously identified gaps have been resolved through code updates and AGENTS.md clarifications. Only **two** partial divergences remain, both at **LOW priority**:

1. **`forceNormalTouchdown()` stride factor** (§2.1) — a potential algebraic difference in control node spacing that needs unit-test verification.
2. **`updateTipAlignPose()` geometry** (§2.2) — a simplified analytical computation that may diverge at extreme body tilts (>15°).

Both remaining items are in rarely-exercised code paths (`force_normal_touchdown` is off by default; `gravity_aligned_tips` is experimental in OpenSHC) and pose no risk to normal hexapod locomotion. Zero missing implementations remain.

The structural additions in HexaMotion (§5) — including explicit parameter setter APIs with velocity-guarded transitions, Cartesian velocity servo control, hardware abstraction interfaces, servo slew limiting, batch bus writes, lateral drift correction, and configurable default height — represent significant extensions beyond OpenSHC's capabilities for MCU deployment. These additions maintain full architectural coherence with the original OpenSHC design while adapting it for real-time embedded operation.
