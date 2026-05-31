# TODO Report — OpenSHC ↔ HexaMotion 1:1 Parity Reconciliation

**Date:** 2026-05-31
**Scope:** Action plan derived from the line-by-line verification in
[ANALISIS_PORT_1A1_OPENSHC.md](ANALISIS_PORT_1A1_OPENSHC.md).
**Goal:** Make HexaMotion a _near-complete 1:1 port of OpenSHC without ROS_, either by aligning the
code to OpenSHC or by explicitly declaring the divergence in `AGENTS.md`/`README.md` when the
HexaMotion variant is provably the better choice for an MCU target.

> **Convention reminder:** all code, comments and documentation text must be written in **English**.
> No dead, legacy, backward-compatible or deprecated code may remain after each task.

---

## 0. Decision matrix (summary)

| Item     | Divergence                                     | Decision                                   | Rationale (short)                                                            |
| -------- | ---------------------------------------------- | ------------------------------------------ | ---------------------------------------------------------------------------- |
| 2.1      | Added stability in `LegStepper`                | **Master flag (off by default)**           | Required by user; pure OpenSHC path must be selectable and default.          |
| 2.2      | Iterative timing + stance forced-even + min-10 | **ALIGN to 1:1**                           | Equally cheap on MCU; OpenSHC rounding is simpler and restores parity.       |
| 2.3      | Manual leg toggle 1-at-a-time                  | **ALIGN to 1:1**                           | Pure feature gap, negligible MCU cost; restore primary+secondary.            |
| 2.4      | Walk plane from stance legs                    | **ALIGN to 1:1**                           | Same 3×3 pseudo-inverse cost; OpenSHC (all default poses) is more stable.    |
| 2.5      | IK internal loop + 5° clamp + analytic seed    | **DOCUMENT discrepancy**                   | Internal loop is _more effective_ at 50 Hz MCU loop; analytic seed cuts CPU. |
| 2.6      | `rough_terrain_mode` partial                   | **PORT full logic** to `TerrainAdaptation` | Declared gap; keep HexaMotion architecture.                                  |
| 4.bis-D1 | `swing_2_nodes_[4].z` snap                     | **Master flag (2.1)**                      | It is part of the anti-drift extension; gate with the same flag.             |
| 4.bis-D2 | `forceNormalTouchdown` simplified factor       | **ALIGN to 1:1**                           | Not an optimization — a wrong scale factor; fix to match OpenSHC.            |

---

## 1. Master parity flag disabling `LegStepper` stability (item 2.1)

**Objective:** Add a single global flag `strict_openshc_parity` that, when `true`, fully disables every
HexaMotion-only stability extension inside `LegStepper`. It must be **`false` by default** (HexaMotion
keeps its extensions on by default), and it must sit **above** the existing fine-grained flags in the
hierarchy: when `strict_openshc_parity == true`, the existing flags (`preserve_swing_end_pose`,
`enable_phase_end_snap`, `enable_workspace_constrain`, `drift_*`) are **ignored/forced off**.

### 1.1 Add the flag to `Parameters`

File: [src/robot_model.h](src/robot_model.h#L192) (next to `preserve_swing_end_pose`).

```cpp
// --- Master OpenSHC parity switch ---
// When true, HexaMotion reproduces OpenSHC's LegStepper trajectory pipeline verbatim and disables
// ALL HexaMotion-only stability extensions (stride freezing, hybrid anti-drift, lateral residual
// cleanup, phase-end snap, in-gait workspace constraining and swing-end Z snapping).
// This switch has the HIGHEST precedence: when set, the fine-grained flags below
// (preserve_swing_end_pose, enable_phase_end_snap, enable_workspace_constrain, drift_*) are ignored.
// Default: false (HexaMotion stability extensions remain active).
bool strict_openshc_parity = false;
```

### 1.2 Resolve the effective flags once per update

File: [src/leg_stepper.cpp](src/leg_stepper.cpp#L402), at the top of `updateTipPositionIterative`.
Introduce a single resolution point so the rest of the method reads local booleans, not raw params.

```cpp
const Parameters &params = params_;

// Master parity switch overrides every fine-grained extension flag.
const bool parity        = params.strict_openshc_parity;
const bool use_drift      = !parity && !params.preserve_swing_end_pose; // hybrid anti-drift
const bool use_lateral    = !parity;                                    // lateral residual cleanup
const bool use_phase_snap = !parity && params.enable_phase_end_snap;
const bool use_ws_clamp   = !parity && params.enable_workspace_constrain;
const bool use_stride_freeze = !parity;                                 // stride freezing
```

### 1.3 Gate every 2.1 block with the resolved booleans

Touch points (replace each raw `params_.*`/`if (!stride_frozen_)` guard accordingly):

- **Stride freezing** — [src/leg_stepper.cpp](src/leg_stepper.cpp#L132) `updateStride()`: wrap the
  freeze branch with `if (use_stride_freeze)`. When `parity`, always recompute and use `stride_vector_`.
    ```cpp
    if (use_stride_freeze) {
        if (!stride_frozen_) { /* freeze ... */ }
    }
    // generateStanceControlNodes(): pick live stride under parity
    Point3D stride_vector_to_use = (use_stride_freeze && stride_frozen_)
                                       ? frozen_stride_vector_total_ : stride_vector_;
    ```
- **Target freezing / safe target** — [src/leg_stepper.cpp](src/leg_stepper.cpp#L491):
    ```cpp
    if (!use_stride_freeze) {                 // parity: never freeze target, raw mid-stride target
        target_tip_pose_ = raw_target;
    } else if (!target_frozen_) {
        target_tip_pose_ = use_ws_clamp ? calculateSafeTarget(raw_target) : raw_target;
        frozen_target_tip_pose_ = target_tip_pose_;
        target_frozen_ = true;
    } else {
        target_tip_pose_ = frozen_target_tip_pose_;
    }
    ```
- **Swing workspace clamp** — [src/leg_stepper.cpp](src/leg_stepper.cpp#L568): replace
  `if (params_.enable_workspace_constrain)` with `if (use_ws_clamp)` (the `makeReachable` call).
- **Lateral residual cleanup + hybrid anti-drift** — [src/leg_stepper.cpp](src/leg_stepper.cpp#L588-L690):
  guard the whole stance-entry correction block:
    ```cpp
    if (use_lateral) { /* lateral residual cleanup ... */ }
    if (use_drift)   { /* hybrid hard/soft reset ... */ }
    else if (parity) { stance_origin_tip_position_ = current_tip_pose_; } // verbatim OpenSHC
    ```
- **Swing-end / stance Z lock (4.bis-D1)** — [src/leg_stepper.cpp](src/leg_stepper.cpp#L687) and
  [src/leg_stepper.cpp](src/leg_stepper.cpp#L388) (`generateSecondarySwingControlNodes`
  `swing_2_nodes_[4].z = default_tip_pose_.z;`): gate with `if (!parity)`.
- **Phase-end snap** — [src/leg_stepper.cpp](src/leg_stepper.cpp#L702): replace
  `if (params.enable_phase_end_snap && target_frozen_)` with `if (use_phase_snap && target_frozen_)`.

### 1.4 Expose the switch through the public API

File: [src/locomotion_system.h](src/locomotion_system.h) / `.cpp` — add a setter consistent with the
other parameter-specific setters declared in `AGENTS.md`:

```cpp
/** Enable/disable strict OpenSHC parity mode (disables all LegStepper stability extensions). */
void setStrictOpenSHCParity(bool enabled);
```

---

## 2. Per-item reconciliation (2.2 – 2.5, 4.bis)

### 2.2 — Iterative timing + stance rounding → **ALIGN to 1:1**

**Analysis.** OpenSHC and HexaMotion both compute explicit `swing_iterations`/`stance_iterations`
(see [walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L1035)); the iterative representation is
_already_ what OpenSHC does, so it is not a divergence. The only real divergence is the rounding
policy: HexaMotion forces **even** counts for _both_ swing and stance and clamps a **minimum of 10**
to both, whereas OpenSHC rounds **only swing** to even and applies **no minimum** to stance. On an
STM32H7 both forms cost a single integer division; there is **no efficiency advantage** to the
HexaMotion rounding. Therefore align to OpenSHC and keep only a divide-by-zero guard.

**Code.** File: [src/leg_stepper.cpp](src/leg_stepper.cpp#L200) `calculateSwingTiming`.

```cpp
// Swing: even rounding (OpenSHC roundToEvenInt). KEEP.
swing_iterations_ = int((double(step_cycle_.swing_period_) / step_cycle_.period_) /
                        (step_cycle_.frequency_ * time_delta));
if (swing_iterations_ % 2 != 0) swing_iterations_++;
// REMOVE: minimum-10 clamp for swing (not in OpenSHC).
swing_delta_t_ = 1.0 / (swing_iterations_ / 2.0);

// Stance: OpenSHC uses a plain truncating int, no even-rounding, no minimum.
stance_iterations_ = int((double(step_cycle_.stance_period_) / step_cycle_.period_) /
                         (step_cycle_.frequency_ * time_delta));
// REMOVE: forced-even and minimum-10 blocks (src/leg_stepper.cpp#L233-L242).
if (stance_iterations_ < 1) stance_iterations_ = 1; // divide-by-zero guard only
stance_delta_t_ = 1.0 / static_cast<double>(stance_iterations_);
```

> Because the swing minimum-10 was justified "for Bezier curve development", keep that guarantee
> **only** in HexaMotion-extension mode if needed; under `strict_openshc_parity` it is gone. Simplest:
> remove it entirely (OpenSHC has no such guard and runs fine). Delete the dead branches outright.

### 2.3 — Manual leg toggle (1-at-a-time → 2 simultaneous) → **ALIGN to 1:1**

**Analysis.** OpenSHC's `legStateToggle`
([state_controller.cpp](OpenSHC/src/state_controller.cpp#L542)) tracks two independent pending
transitions (`toggle_primary_leg_state_`, `toggle_secondary_leg_state_`), letting an operator move two
legs to/from `MANUAL` concurrently (bounded by `MAX_MANUAL_LEGS`). HexaMotion keeps a single
`toggle_leg_index_`/`toggle_leg_state_pending_`. This is a pure capability reduction, not a perf
trade-off — the extra state is two ints/bools and negligible MCU cost. Align.

**Code.** File: [src/state_controller.h](src/state_controller.h) — replace the single pending request
with a primary/secondary pair:

```cpp
int  primary_leg_selection_   = -1;
int  secondary_leg_selection_ = -1;
bool toggle_primary_pending_   = false;
bool toggle_secondary_pending_ = false;
```

File: [src/state_controller.cpp](src/state_controller.cpp#L826) `handleLegStateTransitions` — process
both pending slots in one pass, mirroring OpenSHC's primary/secondary branch selection. Provide two
public setters on `LocomotionSystem`:

```cpp
/** Request a state toggle for the primary selected leg (WALKING<->MANUAL). */
void togglePrimaryLegState(int leg_index);
/** Request a state toggle for the secondary selected leg (WALKING<->MANUAL). */
void toggleSecondaryLegState(int leg_index);
```

> Remove the old `toggle_leg_index_`/`toggle_leg_state_pending_` members and all their references —
> no compatibility shim.

### 2.4 — Walk plane estimation (stance legs → all default poses) → **ALIGN to 1:1**

**Analysis.** OpenSHC fits the walk plane by least squares `(AᵀA)⁻¹Aᵀ` over the **default tip poses of
all legs** ([walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L748)). HexaMotion fits it over the
**legs currently in stance** inside `BodyPoseController`. Both are the same 3×3 solve (trivial on MCU),
but the stance-only set changes membership each cycle, causing plane jitter and a moving reference for
`updateStride`. OpenSHC's all-default-poses fit is steadier and equally cheap → align.

**Code.** File: [src/body_pose_controller.cpp](src/body_pose_controller.cpp#L615)
`calculateWalkPlaneNormal`/`calculateWalkPlaneHeight` — build `A`/`B` from
`leg_steppers_[i].getDefaultTipPose()` for **all** `NUM_LEGS`, not only stance legs:

```cpp
// OpenSHC parity: use the DEFAULT tip pose of every leg (not just stance legs).
for (int i = 0; i < NUM_LEGS; ++i) {
    const Point3D d = leg_steppers_[i].getDefaultTipPose();
    A.row(i) << d.x, d.y, 1.0;
    B(i) = d.z;
}
Eigen::Vector3d plane = (A.transpose() * A).inverse() * A.transpose() * B;
walk_plane_normal_ = Eigen::Vector3d(-plane[0], -plane[1], 1.0).normalized();
```

> The terrain-reactive (stance-based) variant, if still desired for rough terrain, belongs in
> `TerrainAdaptation` (see §3), not in the nominal walk-plane path. Delete the stance-only fit from
> `BodyPoseController`.

### 2.5 — IK strategy (internal loop + 5° clamp + analytic seed) → **DOCUMENT discrepancy**

**Analysis (kept, with justification).**

- **Convergence strategy.** OpenSHC runs **one DLS step per control cycle**
  ([model.cpp](OpenSHC/src/model.cpp#L861)) and relies on its high-rate ROS loop to converge over many
  cycles. HexaMotion's control loop runs at ~50 Hz on the MCU; a single DLS step per cycle would leave
  a visible multi-cycle tracking lag whenever the target jumps. HexaMotion's **bounded internal loop**
  (`IK_DEFAULT_MAX_ITERATIONS = 30`, early-exit at `IK_TOLERANCE = 1 mm`) converges within the cycle,
  which is **more effective** at low control rates. Because the per-cycle trajectory delta is small,
  the loop typically exits in 1–3 iterations, so the worst-case 30 is rarely reached.
- **Analytic seed.** The law-of-cosines/atan2 seed
  ([robot_model.cpp](src/robot_model.cpp#L280)) places the starting guess near the solution, which
  **reduces** the number of DLS iterations — a net **CPU saving** on the MCU versus seeding from the
  current pose.
- **5° per-iteration clamp.** Bounds joint excursions for numerical safety; identical DLS core
  otherwise (`dls=0.02`, `joint_limit_weight=0.1`, Fahimi `0.25/0.75`).

**Conclusion:** the HexaMotion IK is the better fit for the MCU target; **do not align**. Instead,
declare the discrepancy in `AGENTS.md` and `README.md` (see §5) and expose the budget so users can tune
CPU vs accuracy:

```cpp
// robot_model.h (params.ik): document and expose
int    max_iterations = IK_DEFAULT_MAX_ITERATIONS; // bounded internal convergence loop
double tolerance_mm   = IK_TOLERANCE;              // early-exit threshold
// NOTE: damping_lambda is UNUSED by solveIK (dls_coefficient is fixed at IK_DLS_COEFFICIENT).
```

> **Dead-code cleanup (mandatory):** `params.ik.damping_lambda` is never read by `solveIK`. Either wire
> it in as the actual DLS coefficient or **remove it**. Per the no-dead-code rule, remove it and any
> references unless it is deliberately wired in.

### 4.bis-D2 — `forceNormalTouchdown` node separation → **ALIGN to 1:1 (fix)**

**Analysis.** This is not an optimization but a **wrong scale factor**. OpenSHC
([walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L1313)):

```cpp
final_tip_velocity   = -stride_vector_ * (stance_delta_t_ / timeDelta);
stance_node_seperation = 0.25 * final_tip_velocity * (timeDelta / swing_delta_t_);
```

HexaMotion ([src/leg_stepper.cpp](src/leg_stepper.cpp#L344)) drops the `(timeDelta / swing_delta_t_)`
factor and uses `(-1/stance_iterations_)`, diverging by ≈ `swing_iterations/2`. Fix to match OpenSHC
exactly:

```cpp
Point3D final_tip_velocity   = stride_vector_ * (-(stance_delta_t_ / time_delta));
Point3D stance_node_separation = final_tip_velocity * 0.25 * (time_delta / swing_delta_t_);
```

> 4.bis-D1 (`swing_2_nodes_[4].z` snap) is handled by the master flag in §1.3, not here.

---

## 3. Port full `rough_terrain_mode` logic into `TerrainAdaptation` (item 2.6)

**Objective:** Reproduce OpenSHC's `LegStepper::updateTipPosition` rough-terrain behaviour
([walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L1018)) while keeping HexaMotion's architecture
(logic lives in `TerrainAdaptation`, consumed by `LegStepper`). Missing pieces vs OpenSHC:

1. **Proactive step-surface targeting** — project the tip target onto the detected step plane along
   the walk-plane normal (`getProjection(difference, walk_plane_normal_)`).
2. **Reactive probing** — when no step plane is known but touchdown detection is on, lower the target
   by `step_depth` along Z (already partially present, must be driven by `TerrainAdaptation`).
3. **`getStepPlanePose()` equivalent** — `TerrainAdaptation` must expose a per-leg step-plane pose used
   both by the swing target shift and by `ground_contact` gating of the secondary swing nodes.
4. **Default-tip-position refresh at phase entry** — `updateDefaultTipPosition()` already called when
   `rough_terrain_mode`; keep, but feed it from `TerrainAdaptation`'s plane estimate.

**Code — `TerrainAdaptation` public surface.** File: [src/terrain_adaptation.h](src/terrain_adaptation.h#L120):

```cpp
/** @brief Per-leg detected step-plane pose (OpenSHC Leg::getStepPlanePose equivalent).
 *  @param leg_index Leg index [0..NUM_LEGS).
 *  @return Step plane as {position, normal, valid}. valid==false means "undefined". */
const StepPlane &getStepPlane(int leg_index) const;

/** @brief Compute the rough-terrain swing target shift for a leg.
 *  Mirrors OpenSHC's proactive (project onto step plane) and reactive (probe by step_depth) logic.
 *  @param leg_index    Leg index.
 *  @param current_tip  Current tip position.
 *  @param default_target Nominal mid-stride target (default_tip + 0.5*stride).
 *  @param walk_plane_normal Walk-plane normal used for projection.
 *  @return Adjusted target tip position. */
Point3D computeRoughTerrainTarget(int leg_index, const Point3D &current_tip,
                                  const Point3D &default_target,
                                  const Point3D &walk_plane_normal) const;
```

**Code — `LegStepper` consumes it.** File: [src/leg_stepper.cpp](src/leg_stepper.cpp#L512), replacing
the inline rough-terrain block with a delegated call:

```cpp
if (rough_terrain_mode && terrain_adaptation_ != nullptr) {
    target_tip_pose_ = terrain_adaptation_->computeRoughTerrainTarget(
        leg_index_, current_tip_pose_, target_tip_pose_, walk_plane_normal_);
    const TerrainAdaptation::StepPlane &sp = terrain_adaptation_->getStepPlane(leg_index_);
    ground_contact = sp.valid; // gate secondary swing nodes like OpenSHC
}
```

**Code — `computeRoughTerrainTarget` body (in [src/terrain_adaptation.cpp](src/terrain_adaptation.cpp)):**

```cpp
Point3D TerrainAdaptation::computeRoughTerrainTarget(int leg_index, const Point3D &current_tip,
                                                     const Point3D &default_target,
                                                     const Point3D &walk_plane_normal) const {
    const StepPlane &sp = step_planes_[leg_index];
    if (sp.valid) { // PROACTIVE: shift onto detected step surface
        Point3D step_plane_position = sp.position - current_tip;
        Point3D target_tip_position = current_tip + step_plane_position;
        Point3D difference = target_tip_position - default_target;
        return default_target + math_utils::projectVector(difference, walk_plane_normal);
    }
    if (touchdown_detection_[leg_index]) { // REACTIVE: probe downward by step depth
        Point3D probed = default_target;
        probed.z -= step_depth_;
        return probed;
    }
    return default_target; // no terrain data: nominal target
}
```

> Remove the duplicated inline rough-terrain branch in `LegStepper` once delegated. Ensure
> `LegStepper` holds a `TerrainAdaptation*` injected by `WalkController`/`LocomotionSystem`.

---

## 4. Doxygen documentation backlog (missing pieces)

Add `/** ... */` Doxygen to the following public/semi-public pieces (currently undocumented or
under-documented). All comments in English.

- [src/leg_stepper.cpp](src/leg_stepper.cpp#L402) `LegStepper::updateTipPositionIterative` — full
  description: parameters, swing/stance branches, parity-flag behaviour.
- [src/leg_stepper.cpp](src/leg_stepper.cpp#L200) `calculateSwingTiming` — document OpenSHC formula and
  the (post-alignment) rounding policy.
- [src/leg_stepper.cpp](src/leg_stepper.cpp#L132) `updateStride` — document stride-freezing semantics
  and parity behaviour.
- [src/leg_stepper.cpp](src/leg_stepper.cpp#L344) `forceNormalTouchdown` — document the (fixed) node
  separation formula and its OpenSHC equivalence.
- New `strict_openshc_parity` field — [src/robot_model.h](src/robot_model.h#L192) (covered by §1.1).
- New `LegStepper` helpers referenced but undocumented: `computeLateralComponent`,
  `isRectilinearCommand`, `calculateStanceStrideScaler`, `calculateSafeTarget`,
  `validateAndFixControlNodes`, `computeLateralComponent` — [src/leg_stepper.h](src/leg_stepper.h).
- `TerrainAdaptation::getStepPlane` / `computeRoughTerrainTarget` — [src/terrain_adaptation.h](src/terrain_adaptation.h)
  (covered by §3).
- `BodyPoseController::calculateWalkPlaneNormal`/`calculateWalkPlaneHeight` —
  [src/body_pose_controller.h](src/body_pose_controller.h) — document the OpenSHC-aligned all-legs fit.
- New `StateController` primary/secondary toggle members and setters —
  [src/state_controller.h](src/state_controller.h) (covered by §2.3).
- `LocomotionSystem` new setters: `setStrictOpenSHCParity`, `togglePrimaryLegState`,
  `toggleSecondaryLegState` — [src/locomotion_system.h](src/locomotion_system.h).

---

## 5. Update `AGENTS.md` and `README.md` (sincere the differences)

### 5.1 `AGENTS.md`

- Add `strict_openshc_parity` to the _Key differences_ list as the master parity switch (default off).
- Add an entry declaring the **IK strategy discrepancy** (item 2.5): "HexaMotion runs a bounded
  internal DLS convergence loop (≤30 iters, 1 mm early-exit) with an analytic seed and a 5°/iteration
  clamp, instead of OpenSHC's single DLS step per control cycle. This is intentional: it converges
  within one ~50 Hz MCU cycle and the analytic seed lowers average iteration count."
- Remove from the _Key differences_ list any item now realigned (2.2 timing, 2.3 toggle, 2.4 walk
  plane, 4.bis-D2) — they are no longer divergences once §2 lands.
- Add a **No-dead-code rule** under _Code Style_ (see §7).

### 5.2 `README.md`

- Adjust the headline claim to match reality: keep _"near-complete 1:1 port of OpenSHC without ROS"_,
  and add a short "Parity & deliberate divergences" subsection summarising: master parity flag,
  IK-strategy note, and the rough-terrain logic now living in `TerrainAdaptation`.

```markdown
### Parity & deliberate divergences

HexaMotion targets a near-complete 1:1 reproduction of OpenSHC. A master switch
`Parameters::strict_openshc_parity` (default **false**) disables every HexaMotion-only stability
extension in `LegStepper` to obtain OpenSHC-verbatim trajectories. One intentional divergence is
retained for MCU effectiveness: inverse kinematics uses a bounded internal DLS loop with an analytic
seed instead of OpenSHC's single step per cycle (see `AGENTS.md`).
```

---

## 6. Documentation under `docs/`

Most files in `docs/` are stale (per the analysis note). Required edits:

- Update [docs/OpenSHC_GAP_REPORT.md](docs/OpenSHC_GAP_REPORT.md) and the parity gap reports to mark
  items 2.2/2.3/2.4/4.bis-D2 as **resolved (aligned)** and 2.1/2.5/2.6 as **addressed (flag/doc/port)**.
- Update [docs/WALKSPACE_WORKSPACE_INTEGRATION_GUIDE.md](docs/WALKSPACE_WORKSPACE_INTEGRATION_GUIDE.md)
  to reflect the OpenSHC-aligned walk-plane fit (all default poses).
- Add a short note in any IK-related doc about the bounded internal loop + analytic seed.
- Do **not** expand stale docs beyond these reconciliation notes; flag clearly outdated sections.

---

## 7. `AGENTS.md` — add the No-dead-code rule

Under **Code Style**, add:

```markdown
- No dead, legacy, backward-compatible, deprecated or unused code may be committed. Remove unused
  parameters, fields, branches and helpers instead of guarding or commenting them out. Every symbol
  must have at least one live use.
```

Immediate enforcement targets from this report:

- Remove `params.ik.damping_lambda` (unused by `solveIK`) — §2.5.
- Remove the stance forced-even / minimum-10 branches — §2.2.
- Remove the old single-slot toggle members after §2.3.
- Remove the stance-only walk-plane fit after §2.4.
- Remove the inline `LegStepper` rough-terrain branch after §3.

---

## 8. Tests — new coverage + consolidation/dedup

### 8.1 New tests (functionality not covered)

| New test file                            | Validates                                                                                                                       | Pieces                                                          |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| `strict_parity_legstepper_test.cpp`      | With `strict_openshc_parity=true`, no stride-freeze/anti-drift/snap/ws-clamp/Z-snap occurs; trajectory matches OpenSHC formula. | `leg_stepper`, `walk_controller`, `robot_model`                 |
| `timing_rounding_parity_test.cpp`        | Swing even-rounded, stance plain int (no forced-even, no min-10) after §2.2.                                                    | `leg_stepper`, `gait_config_factory`                            |
| `manual_leg_toggle_dual_test.cpp`        | Two legs transition WALKING↔MANUAL concurrently up to `MAX_MANUAL_LEGS`.                                                        | `state_controller`, `body_pose_controller`, `locomotion_system` |
| `walk_plane_all_legs_test.cpp`           | Walk-plane fit uses all default poses; matches OpenSHC pseudo-inverse result.                                                   | `body_pose_controller`, `robot_model`                           |
| `force_normal_touchdown_parity_test.cpp` | `forceNormalTouchdown` node separation equals OpenSHC formula (§4.bis-D2).                                                      | `leg_stepper`, `robot_model`                                    |
| `rough_terrain_adaptation_test.cpp`      | Proactive step-plane projection + reactive step-depth probing via `TerrainAdaptation`.                                          | `terrain_adaptation`, `leg_stepper`, `walk_controller`          |
| `ik_internal_loop_test.cpp`              | Bounded loop converges < `IK_TOLERANCE`; analytic seed reduces iteration count; 5° clamp.                                       | `robot_model`                                                   |

Example skeleton (`strict_parity_legstepper_test.cpp`):

```cpp
// Verify that strict_openshc_parity disables all HexaMotion stability extensions.
Parameters p = makeTestParams();           // shared helper from test_pose_helpers.h
p.strict_openshc_parity = true;
// drift/snap/ws flags must be IGNORED under parity:
p.preserve_swing_end_pose = false;          // would normally enable hybrid anti-drift
p.enable_phase_end_snap   = true;
p.enable_workspace_constrain = true;
RobotModel model(p);
LegStepper stepper(/* ... */);
// Drive a full swing+stance cycle and assert the tip path equals the pure-Bezier OpenSHC path
// (no stride freezing, no stance reset, no phase-end snap).
assertTrajectoryMatchesOpenSHC(stepper /* ... */);
```

### 8.2 Consolidation / dedup of existing tests

Group equivalent/related tests into single files and delete redundant ones:

| Consolidated file              | Absorbs (delete originals)                                                                                                                                                                                                   |
| ------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ik_test.cpp`                  | `simple_ik_test.cpp`, `simple_advanced_ik_test.cpp`, `ik_tracking_diagnostic_test.cpp` (+ new `ik_internal_loop_test.cpp`)                                                                                                   |
| `dh_kinematics_test.cpp`       | `simple_dh_test.cpp`, `dh_vs_analytic_test.cpp`, `complete_physical_offset_test.cpp`, `joint_output_calibration_test.cpp`                                                                                                    |
| `bezier_test.cpp`              | `bezier_validation_test.cpp`, `bezier_curve_deterministic_test.cpp`, `bezier_transition_single_leg_test.cpp`, `bezier_transition_all_legs_test.cpp`                                                                          |
| `coxa_stride_test.cpp`         | `coxa_phase_transition_test.cpp`, `coxa_stride_decomposition_test.cpp`, `coxa_tripod_symmetry_analytic_test.cpp`, `swing_coxa_orientation_test.cpp`, `stride_vector_validation_test.cpp`, `stride_deviation_limits_test.cpp` |
| `trajectory_test.cpp`          | `trajectory_tip_position_test.cpp`, `trajectory_all_legs_test.cpp`, `hexapod_trajectory_analysis_test.cpp`                                                                                                                   |
| `tripod_walk_test.cpp`         | `tripod_linearity_test.cpp`, `tripod_walk_visualization_test.cpp`                                                                                                                                                            |
| `runtime_api_test.cpp`         | `runtime_parameter_setter_test.cpp`, `convenience_motion_api_test.cpp`, `publication_accessor_test.cpp`                                                                                                                      |
| `numeric_integration_test.cpp` | `runge_kutta_validation_test.cpp`, `quaternion_functions_test.cpp` (merge into `math_utils_test.cpp` if trivial)                                                                                                             |

**Rules for consolidation:**

- Keep one `main()` per consolidated file aggregating the sub-cases; preserve every distinct
  assertion (no loss of coverage).
- Delete the absorbed source files and update [tests/Makefile](tests/Makefile),
  [tests/run_all_tests.sh](tests/run_all_tests.sh) and the **Test coverage matrix** in `AGENTS.md`.
- Do not keep empty stubs or commented-out old tests (no-dead-code rule).

---

## 9. Execution order (suggested)

1. §7 add no-dead-code rule to `AGENTS.md` (governs everything below).
2. §1 master parity flag (unblocks parity-mode tests).
3. §2.2 → §2.3 → §2.4 → §4.bis-D2 alignments (each removes a divergence + its dead code).
4. §2.5 documentation + `damping_lambda` removal.
5. §3 rough-terrain port into `TerrainAdaptation`.
6. §4 Doxygen pass.
7. §5/§6 `AGENTS.md`/`README.md`/`docs/` reconciliation.
8. §8 tests: add new, consolidate, dedup; run focused suites then `bash run_all_tests.sh`.

```

```
