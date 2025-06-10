# Documentation: Multiple Initial Configurations in Inverse Kinematics

## Implementation Summary

A configurable system has been added to control the use of multiple initial configurations in the DLS (Damped Least Squares) inverse kinematics algorithm.

## For Loop Motivation

The `for` loop in the `inverseKinematics` method (lines 106-233) **IS NOT present in the original CSIRO syropod_highlevel_controller implementation**. It was added as an enhancement to address:

### Problems It Solves:

-   **Improved convergence** for targets near singularities
-   **Better handling** of targets at workspace boundaries
-   **Multiple solutions** - finds the best among several options
-   **Robustness** against poor initial configurations

### How It Works:

The algorithm tries up to **5 different initial configurations**:

1. `JointAngles(coxa_start, -45.0f, 60.0f)` - Typical walking pose
2. `JointAngles(coxa_start, 30.0f, -60.0f)` - High pose
3. `JointAngles(coxa_start, -30.0f, 45.0f)` - Intermediate pose
4. `JointAngles(coxa_start, 0.0f, 0.0f)` - Straight pose
5. `JointAngles(coxa_start, -60.0f, 80.0f)` - Extended pose

## Flag Control

### Configuration:

```cpp
// In Parameters struct
struct IKConfig {
    // ...existing parameters...
    bool use_multiple_starts = true; ///< Enable multiple starting configurations
} ik;
```

### Usage:

```cpp
// Enable multiple configurations (default)
params.ik.use_multiple_starts = true;

// Disable for original CSIRO behavior
params.ik.use_multiple_starts = false;
```

## Performance Impact

### With `use_multiple_starts = true`:

-   **Time**: ~44,471 microseconds
-   **Accuracy**: Potentially better for difficult cases
-   **Behavior**: Tests 5 initial configurations

### With `use_multiple_starts = false`:

-   **Time**: ~11,622 microseconds (4x faster)
-   **Accuracy**: Same as original CSIRO implementation
-   **Behavior**: Single initial configuration

## Validation

### Implemented Tests:

1. **`ik_multiple_starts_test`** - Validates that the flag correctly controls behavior
2. **`tripod_gait_ik_config_test`** - Verifies that both configurations maintain tripod gait corrections

### Results:

-   ✅ **Flag works correctly** - Controls the number of initial configurations
-   ✅ **Tripod gait maintains corrections** - All 6 legs move actively in both modes
-   ✅ **CSIRO compatibility** - Original behavior preserved when disabled

## Usage Recommendations

### Use `use_multiple_starts = true` when:

-   Targets are near workspace boundaries
-   Maximum precision is required
-   Computation time is not critical
-   There are problematic targets that don't converge

### Use `use_multiple_starts = false` when:

-   Maximum performance is required (4x faster)
-   Exact CSIRO fidelity is desired
-   Targets are simple and within central workspace
-   Running on resource-limited hardware

## Conclusion

The implementation provides:

1. **Flexibility** - Configurable according to needs
2. **Compatibility** - Maintains original behavior when disabled
3. **Improvement** - Potential improved convergence when enabled
4. **Validation** - All tripod gait corrections remain intact

The `params.ik.use_multiple_starts` flag allows choosing between the original CSIRO behavior (fast, single configuration) and the improved behavior (more robust, multiple configurations).
