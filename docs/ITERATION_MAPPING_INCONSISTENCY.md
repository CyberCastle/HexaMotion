# Swing/Stance Iteration Mapping — 1:1 Inconsistency with OpenSHC

## Summary

The `LegStepper::updateTipPositionIterative()` implementation in HexaMotion is not 1:1 equivalent to OpenSHC's `LegStepper::updateTipPosition()`. Two structural divergences compensate each other in production, but they break calls that do not exactly replicate the `WalkController` flow.

---

## 1. Call-Order Divergence

### OpenSHC (`walk_controller.cpp`, lines 637–639)

```cpp
leg_stepper->updateTipPosition();   // uses current phase_
leg_stepper->updateTipRotation();
leg_stepper->iteratePhase();        // phase_ = (phase_ + 1) % period_
```

**`phase_` is used first and incremented afterward.** The first swing iteration receives `phase_ = swing_start_`, and the first stance iteration receives `phase_ = stance_start_`.

### HexaMotion (`walk_controller.cpp`, lines 562–621)

```cpp
int current_phase = leg_stepper->getPhase();
current_phase = (current_phase + 1) % step_cycle.period_;
leg_stepper->setPhase(current_phase);           // phase_ already incremented
leg_stepper->updateStepStateFromPhase();         // determines SWING/STANCE
// ...
leg_stepper->updateTipPosition(time_delta, ...); // uses incremented phase_
```

**`phase_` is incremented first and used afterward.** Each `phase_` value entering `updateTipPositionIterative` is shifted by +1 compared to OpenSHC.

---

## 2. Phase-Cycle Layout Divergence

### OpenSHC — Wrapping Layout (wrapping stance)

The cycle is organized as `[stance₁ | SWING | stance₂]`, where stance wraps around the cycle boundary:

| Field           | Value (tripod 50/50, period=52) |
| --------------- | ------------------------------- |
| `stance_end_`   | 13                              |
| `swing_start_`  | 13                              |
| `swing_end_`    | 39                              |
| `stance_start_` | 39                              |

Stance covers phases `[39..51] ∪ [0..12]` (wrapping). For this reason, OpenSHC uses `mod()` with an offset to re-index stance iteration.

### HexaMotion — Contiguous Layout

The cycle is generated in `GaitConfiguration::generateStepCycle()` as `[STANCE | SWING]`:

| Field           | Value (tripod 50/50, period=52) |
| --------------- | ------------------------------- |
| `stance_start_` | 0                               |
| `stance_end_`   | 26                              |
| `swing_start_`  | 26                              |
| `swing_end_`    | 52                              |

Stance covers phases `[0..25]` and swing covers `[26..51]`. There is no wraparound.

---

## 3. Combined Effect on Iteration Formulas

### SWING

**OpenSHC** (`phase_` not incremented yet):

```cpp
int iteration = phase_ - step.swing_start_ + 1;
// phase_=13 → 13-13+1 = 1     (first iteration)
// phase_=38 → 38-13+1 = 26    (last iteration)
// Range: [1..26]  ✓
```

**HexaMotion** (`phase_` already incremented, `iteration = phase_`):

```cpp
int swing_iteration = iteration % swing_iterations_;
if (swing_iteration == 0) swing_iteration = swing_iterations_;
// iteration=26 → 26%26=0 → corrected to 26   (last instead of first)
// iteration=27 → 27%26=1                      (second)
// iteration=51 → 51%26=25                     (second-to-last)
// Range: [26, 1, 2, ..., 25]  ❌
```

The first swing iteration produces `swing_iteration=26` (`time_input=1.0`), while it should be 1 (`time_input=swing_delta_t_`).

**Why this is not visible in production:** `initializeSwingPeriod()` resets `swing_origin` to current position, and control nodes are regenerated each iteration from that same origin. `quarticBezierDot` at `t=1.0` with origin ≈ current position yields `delta_pos ≈ 0`. This becomes a tolerated “phantom step.”

### STANCE

**OpenSHC** (`phase_` not incremented yet, wrapping layout):

```cpp
int iteration = mod(phase_ + (step.period_ - modified_stance_start), step.period_) + 1;
// modified_stance_start=39, period_=52
// phase_=39 → mod(39+13, 52)+1 = 0+1 = 1       (first)
// phase_=12 → mod(12+13, 52)+1 = 25+1 = 26     (last)
// Range: [1..26]  ✓
```

**HexaMotion** (`phase_` already incremented, contiguous layout):

```cpp
int stance_iteration = (iteration % stance_iterations_) + 1;
// iteration=phase_=0 → (0%26)+1 = 1       (first)
// iteration=phase_=25 → (25%26)+1 = 26    (last)
// Range: [1..26]  ✓
```

In production, the +1 shift lands at `phase_=0` at stance entry (`(51+1)%52=0`), and `(0%26)+1=1`. **Stance mapping works in production due to compensation between pre-increment and contiguous layout from zero.**

---

## 4. Equivalence Summary Table

| Aspect                   | OpenSHC                                 | HexaMotion                                | 1:1? |
| ------------------------ | --------------------------------------- | ----------------------------------------- | ---- |
| Phase/update order       | `update → phase++`                      | `phase++ → update`                        | ❌   |
| Stance layout            | Wrapping `[start..P) ∪ [0..end)`        | Contiguous `[0..N)`                       | ❌   |
| Swing iteration formula  | `phase_ - swing_start_ + 1` → [1..N]    | `phase_ % N`, fix `0→N` → [N, 1..N-1]     | ❌   |
| Stance iteration formula | `mod(phase_ + offset, P) + 1` → [1..N]  | `(phase_ % N) + 1` → [1..N]               | ✅\* |
| Swing `time_input`       | `swing_delta_t_ * iter`, `iter∈[1..N]`  | First tick: `t=1.0` (“phantom step”)      | ❌   |
| Stance `time_input`      | `iter * stance_delta_t_`, `iter∈[1..N]` | `iter * stance_delta_t_`, `iter∈[1..N]`   | ✅   |
| Swing origin init        | `if (iteration == 1)`                   | fallback via `iteration < last_iteration` | ~✅  |
| Stance origin init       | `if (iteration == 1)`                   | `if (stance_iteration == 1)`              | ✅\* |

**(\*)** Works in production due to implicit compensation. It fails if called with iteration values that do not follow production `phase_` convention (for example, tests with 1-based iteration).

---

## 5. Observable Impact

### In Production

- **Swing**: the phantom step in the first iteration yields `delta_pos ≈ 0`, which is tolerated. The path evolves correctly from the second iteration.
- **Stance**: behaves correctly due to compensation.
- **No reported errors** because `WalkController` always calls with the exact production `phase_` convention.

### In Tests

Any test that passes 1-based iterations (for example, `trajectory_tip_position_test`) observes:

- Stance: `iteration=swing_iterations+1` causes shifted `stance_iteration`; origin init runs on the **last** iteration instead of the first, producing a measurable discontinuity.
- Swing: `iteration=1` produces `swing_iteration=1` correctly by coincidence (`1%26=1`).

---

## 6. Corrective Actions

### Action 1: Align call order with OpenSHC

Modify `WalkController::updateWalk()` so `updateTipPosition` is called **before** phase increment, identical to OpenSHC.

**File:** `src/walk_controller.cpp`

**Change:** Restructure the loop sequence to:

```cpp
// BEFORE (current)
current_phase = (current_phase + 1) % step_cycle.period_;
leg_stepper->setPhase(current_phase);
leg_stepper->updateStepStateFromPhase();
// ... STARTING/STOPPING state ...
leg_stepper->updateTipPosition(time_delta, ...);

// AFTER (proposed - OpenSHC equivalent)
leg_stepper->updateTipPosition(time_delta, ...);
current_phase = (current_phase + 1) % step_cycle.period_;
leg_stepper->setPhase(current_phase);
leg_stepper->updateStepStateFromPhase();
// ... STARTING/STOPPING state ...
```

**Risk:** High. `updateStepStateFromPhase()` determines SWING/STANCE before `updateTipPosition`, and that result controls which branch the stepper executes. If moved after, `step_state_` may be stale during trajectory generation.

**Mitigation:** Separate state determination from phase iteration. One option: call `updateStepStateFromPhase()` at tick start (without increment), then `updateTipPosition`, then iterate phase (`phase++` + updateStepState):

```cpp
leg_stepper->updateStepStateFromPhase();         // state from current phase_
leg_stepper->updateTipPosition(time_delta, ...); // uses current phase_
// ... STARTING/STOPPING state ...
current_phase = (current_phase + 1) % step_cycle.period_;
leg_stepper->setPhase(current_phase);
leg_stepper->updateStepStateFromPhase();         // prepare state for next tick
```

### Action 2: Rewrite swing iteration formula as in OpenSHC

Use explicit `phase_ - swing_start_ + 1` instead of generic modulo.

**File:** `src/leg_stepper.cpp`, `STEP_SWING` branch in `updateTipPositionIterative`

**Change:**

```cpp
// BEFORE (current)
int swing_iteration = iteration % swing_iterations_;
if (swing_iteration == 0)
    swing_iteration = swing_iterations_;

// AFTER (proposed — OpenSHC equivalent)
int swing_iteration = iteration - step_cycle_.swing_start_ + 1;
// Safety clamp
if (swing_iteration < 1) swing_iteration = 1;
if (swing_iteration > swing_iterations_) swing_iteration = swing_iterations_;
```

**Precondition:** Requires Action 1 so `iteration = phase_` is not pre-incremented before entry.

### Action 3: Rewrite stance iteration formula as in OpenSHC

Use `mod` + `modified_stance_start` formula to support any layout robustly.

**File:** `src/leg_stepper.cpp`, `STEP_STANCE` branch in `updateTipPositionIterative`

**Change:**

```cpp
// BEFORE (current)
int stance_iteration = (iteration % std::max(1, stance_iterations_)) + 1;

// AFTER (proposed — OpenSHC equivalent)
int modified_stance_start = (step_state_ == STEP_SWING || completed_first_step_)
    ? step_cycle_.stance_start_
    : static_cast<int>(phase_offset_ * step_cycle_.period_);
int stance_iteration = math_utils::mod(iteration + (step_cycle_.period_ - modified_stance_start),
                                       step_cycle_.period_) + 1;
```

**Precondition:** Requires Action 1, and `math_utils::mod` must implement always-positive modulo: `(a % b + b) % b`.

### Action 4: Add `math_utils::mod` (always-positive modulo)

**File:** `src/math_utils.h`

```cpp
/** Always-positive modulo, matching OpenSHC's mod() template. */
inline int mod(int a, int b) { return ((a % b) + b) % b; }
```

### Action 5: Fix `trajectory_tip_position_test.cpp`

After Actions 1–4, the test should call `updateTipPositionIterative` with production-equivalent `phase_` values, without ad-hoc conventions.

As an intermediate measure **without production changes**, the test can simulate the exact production flow:

```cpp
// Instead of:
for (int iteration = 1; iteration <= swing_iterations; iteration++) {
    stepper.updateTipPositionIterative(iteration, ...);
}

// Use production flow:
for (int phase = step_cycle.swing_start_; phase < step_cycle.swing_end_; phase++) {
    stepper.setPhase(phase);
    stepper.updateStepStateFromPhase();
    stepper.updateTipPosition(time_delta, false, false);
}
```

---

## 7. Recommended Implementation Order

| Priority | Action                              | Dependencies | Impact                      |
| -------- | ----------------------------------- | ------------ | --------------------------- |
| 1        | Action 4 — `math_utils::mod`        | None         | Low (additive)              |
| 2        | Action 1 — reorder phase/update     | Action 4     | **High** (main flow change) |
| 3        | Action 2 — swing iteration formula  | Action 1     | Medium                      |
| 4        | Action 3 — stance iteration formula | Actions 1, 4 | Medium                      |
| 5        | Action 5 — update tests             | Actions 1–4  | Low                         |

### Conservative Alternative (tests only, no production changes)

If changing the main flow is too risky right now:

1. Document that `updateTipPositionIterative` expects production-convention `phase_` values (pre-incremented, 0-based).
2. Ensure tests replicate the exact `WalkController` flow: increment `phase_`, then `updateStepState`, then `updateTipPosition`.
3. Accept the swing “phantom step” as known behavior.

---

## 8. Verification

After applying the corrections, the following criteria should hold:

1. **Swing**: first swing tick `time_input` should be `swing_delta_t_` (≈0.077), not 1.0.
2. **Stance**: `stance_iteration == 1` exactly on the first stance iteration (not the last).
3. **Test**: in `trajectory_tip_position_test`, stance validation should report coxa as the dominant joint: `abs(total_coxa_change) > abs(total_tibia_change)`.
4. **Regression**: all existing tests (`make` inside `tests/`) should still pass.
