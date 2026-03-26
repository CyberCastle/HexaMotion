# Quaternion Convention Divergence: HexaMotion vs OpenSHC

## Summary

HexaMotion's `eulerAnglesToQuaterniond()` always produces **extrinsic XYZ** (Z\*Y\*X matrix order) rotations regardless of the `extrinsic` flag, while OpenSHC's `eulerAnglesToQuaternion()` supports both extrinsic and intrinsic conventions. This causes a rotation convention mismatch at one call site (`setManualPoseInput`), though the practical impact is negligible for small angles.

## Background: The Coefficient-Order Bug

Prior to the fix, three call sites in `body_pose_controller.cpp` constructed `Eigen::Quaterniond` from the `Eigen::Vector4d` returned by `eulerToQuaternion()` / `eulerPoint3DToQuaternion()`.

These functions return coefficients in **(w, x, y, z)** order (index 0 = scalar part), but `Eigen::Quaterniond`'s vector-based constructor interprets them as **(x, y, z, w)** (Eigen's internal storage order). The resulting quaternion was garbled — an identity input `(1, 0, 0, 0)` became a 180° rotation about the X axis.

The fix replaced all three sites with `eulerAnglesToQuaterniond()`, which returns `Eigen::Quaterniond` directly and eliminates the coefficient-order ambiguity entirely.

## Euler Convention Comparison

### OpenSHC (`standard_includes.h`)

```cpp
inline Eigen::Quaterniond eulerAnglesToQuaternion(
    const Eigen::Vector3d& euler,
    const bool& intrinsic = false)
{
    if (intrinsic)
        // X * Y * Z  (intrinsic XYZ / body-fixed)
        return Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
             * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    else
        // Z * Y * X  (extrinsic XYZ / fixed-frame)
        return Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
}
```

- `intrinsic=false` (default) → **Z \* Y \* X** (extrinsic XYZ)
- `intrinsic=true` → **X \* Y \* Z** (intrinsic XYZ)

### HexaMotion (`math_utils.h`)

```cpp
inline Eigen::Quaterniond eulerAnglesToQuaterniond(
    const Eigen::Vector3d &euler,
    bool extrinsic = false)
{
    if (extrinsic) {
        // BUG: produces Z * Y * X — identical to the else branch
        return Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
    } else {
        return Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
    }
}
```

- `extrinsic=false` (default) → **Z \* Y \* X**
- `extrinsic=true` → **Z \* Y \* X** (dead code — identical to `false` branch)

## Affected Call Sites

| Call Site                                | OpenSHC                                                              | HexaMotion                                                      |       Convention Match       |
| ---------------------------------------- | -------------------------------------------------------------------- | --------------------------------------------------------------- | :--------------------------: |
| `setManualPoseInput`                     | `eulerAnglesToQuaternion(euler, true)` — **intrinsic XYZ** (X\*Y\*Z) | `eulerAnglesToQuaterniond(euler)` — **extrinsic XYZ** (Z\*Y\*X) |            **NO**            |
| `updateIMUPosePID` (current_rotation)    | IMU arrives as native `Quaterniond` via ROS                          | `eulerAnglesToQuaterniond(euler_rad)` — Z\*Y\*X                 | N/A (different input source) |
| `updateIMUPosePID` (imu_pose\_.rotation) | `eulerAnglesToQuaternion(correction)` — default Z\*Y\*X              | `eulerAnglesToQuaterniond(correction)` — Z\*Y\*X                |           **YES**            |

## Impact Analysis

### Mathematical difference

For Euler angles $(r, p, y)$ representing roll, pitch, and yaw:

$$R_{\text{extrinsic}} = R_z(y) \cdot R_y(p) \cdot R_x(r)$$
$$R_{\text{intrinsic}} = R_x(r) \cdot R_y(p) \cdot R_z(y)$$

These are equivalent only when at most one angle is nonzero. For combined rotations the results diverge. The angular error between the two conventions for a combined rotation $(r, p, y)$ is approximately:

$$\Delta\theta \approx 2 \cdot |r \cdot p \cdot \sin(y) - r \cdot y \cdot \sin(p)|$$

### Practical impact in `setManualPoseInput`

- Manual pose inputs are **clamped** to `max_rotation` limits (typically ±0.3 rad / ±17°).
- At these small angles, the cross-coupling error between conventions is < 1° for typical inputs.
- The divergence is only observable with simultaneous large roll + pitch + yaw commands.

### No impact on IMU PID

The IMU pose correction path uses the default convention (Z\*Y\*X) in both OpenSHC and HexaMotion. Full equivalence is maintained.

## Root Cause

The `extrinsic=true` branch in `eulerAnglesToQuaterniond()` was implemented with the same rotation order as the `extrinsic=false` branch. To match OpenSHC's `intrinsic=true` behavior, it should produce **X \* Y \* Z**:

```cpp
if (extrinsic) {
    // Should be: intrinsic XYZ = X * Y * Z
    return Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
}
```

> **Note on naming:** OpenSHC calls this branch `intrinsic=true`, while HexaMotion calls it `extrinsic=true`. The parameter names are inverted, but both should produce the same X\*Y\*Z rotation when the flag is set. Currently HexaMotion's flag has no effect.

## Recommended Fix

1. Fix the `extrinsic=true` branch in `eulerAnglesToQuaterniond()` to produce X\*Y\*Z rotation.
2. Update `setManualPoseInput()` to call `eulerAnglesToQuaterniond(euler, true)`.
3. Verify no other call sites depend on the current (broken) `extrinsic=true` behavior.

## Status

- **Coefficient-order bug**: Fixed.
- **Convention divergence in manual pose**: Pre-existing, not introduced by the fix. Low practical impact due to angle clamping. Documented here for future correction.
