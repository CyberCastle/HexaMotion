# Smooth Movement Configuration Guide

This document explains how to configure and use the new smooth trajectory interpolation feature in HexaMotion, which enables OpenSHC-style smooth movements that start from current servo positions.

## Overview

The smooth trajectory feature provides:

1. **Current servo position starting point**: Trajectories begin from actual servo positions rather than theoretical positions
2. **Smooth interpolation**: Gradual transition between poses instead of immediate jumps
3. **Configurable parameters**: Adjustable interpolation speed and precision
4. **Backward compatibility**: Original immediate movement behavior is still available
5. **OpenSHC equivalence**: Similar behavior to OpenSHC's pose interpolation system

## Configuration

### Parameters Structure

The new `SmoothTrajectoryConfig` in the `Parameters` struct provides complete control:

```cpp
struct SmoothTrajectoryConfig {
    bool use_current_servo_positions = true;  // Enable current position starting point
    bool enable_pose_interpolation = true;    // Enable smooth interpolation
    float interpolation_speed = 0.1f;         // Speed factor (0.01-1.0)
    float position_tolerance_mm = 1.0f;       // Position tolerance for completion
    uint8_t max_interpolation_steps = 20;     // Maximum interpolation steps
    bool use_quaternion_slerp = true;         // Use spherical interpolation for orientations
} smooth_trajectory;
```

### Default Behavior

**New Default (Recommended)**: Smooth trajectory interpolation is **enabled by default** and automatically used when calling standard pose methods:

```cpp
// This now uses smooth trajectory by default
locomotion_system.setBodyPose(position, orientation);
```

### Runtime Configuration

Configure smooth movement behavior at runtime:

```cpp
// Enable smooth movement with custom parameters
locomotion_system.configureSmoothMovement(
    true,    // Enable smooth movement
    0.15f,   // Interpolation speed (0.01-1.0)
    25       // Maximum steps
);

// Check if trajectory is in progress
if (locomotion_system.isSmoothMovementInProgress()) {
    Serial.println("Smooth movement active");
}

// Reset trajectory if needed
locomotion_system.resetSmoothMovement();
```

## Usage Methods

### 1. Default Smooth Movement

```cpp
// Uses smooth trajectory automatically (recommended)
Eigen::Vector3f position(0, 0, 120.0f);
Eigen::Vector3f orientation(0, 10.0f, 0);
locomotion_system.setBodyPose(position, orientation);
```

### 2. Explicit Smooth Movement

```cpp
// Explicitly request smooth trajectory
locomotion_system.setBodyPoseSmooth(position, orientation);
```

### 3. Immediate Movement (Legacy)

```cpp
// Bypass smooth trajectory for immediate response
locomotion_system.setBodyPoseImmediate(position, orientation);
```

## OpenSHC Equivalence

This implementation provides equivalent functionality to OpenSHC's pose interpolation:

| OpenSHC Feature                       | HexaMotion Equivalent                           |
| ------------------------------------- | ----------------------------------------------- |
| `stepToPosition()` with Bezier curves | `setBodyPoseSmooth()` with linear interpolation |
| Current tip pose as origin            | Current servo positions as starting point       |
| Bezier curve interpolation            | Linear/SLERP interpolation                      |
| Time-based progression                | Step-based progression with speed control       |
| Pose transition sequences             | Configurable interpolation parameters           |

## Parameter Guidelines

### Interpolation Speed

-   **0.01-0.05**: Very slow, ultra-smooth movement
-   **0.05-0.15**: Smooth movement (recommended range)
-   **0.15-0.30**: Moderate speed
-   **0.30-1.00**: Fast movement, less smooth

### Maximum Steps

-   **5-10**: Fast but potentially jerky
-   **15-25**: Balanced (recommended range)
-   **25-50**: Very smooth but slower response

### Position Tolerance

-   **0.5-2.0mm**: Precise positioning
-   **2.0-5.0mm**: Normal use (recommended)
-   **5.0+mm**: Fast completion, less precise

## Example Usage Patterns

### Basic Setup

```cpp
void setup() {
    Parameters params;
    // ... configure robot dimensions ...

    // Configure smooth trajectory
    params.smooth_trajectory.use_current_servo_positions = true;
    params.smooth_trajectory.interpolation_speed = 0.15f;
    params.smooth_trajectory.max_interpolation_steps = 20;

    locomotion_system.setParams(params);
    locomotion_system.initialize(&imu, &fsr, &servos);
}
```

### Dynamic Movement

```cpp
void demonstrateSmoothMovement() {
    // Configure for ultra-smooth movement
    locomotion_system.configureSmoothMovement(true, 0.08f, 30);

    // Perform smooth pose changes
    locomotion_system.setBodyPose(Eigen::Vector3f(0, 0, 120), Eigen::Vector3f(0, 0, 0));

    // Wait for completion
    while (locomotion_system.isSmoothMovementInProgress()) {
        locomotion_system.update();
        delay(20);
    }

    // Next pose change will start from current servo positions
    locomotion_system.setBodyPose(Eigen::Vector3f(0, 0, 80), Eigen::Vector3f(0, 15, 0));
}
```

### Performance Optimization

```cpp
void fastResponse() {
    // Configure for responsive movement
    locomotion_system.configureSmoothMovement(true, 0.25f, 15);
}

void preciseMovement() {
    // Configure for maximum smoothness
    locomotion_system.configureSmoothMovement(true, 0.05f, 40);
}
```

## Integration with Existing Code

### Backward Compatibility

Existing code continues to work with improved behavior:

```cpp
// This code automatically gets smooth movement
bool result = locomotion_system.setBodyPose(new_position, new_orientation);
```

### Disabling Smooth Movement

To use original behavior:

```cpp
// Disable smooth trajectory globally
locomotion_system.configureSmoothMovement(false);

// Or use immediate methods explicitly
locomotion_system.setBodyPoseImmediate(position, orientation);
```

## Benefits

1. **Smoother robot movement**: Eliminates sudden jerky motions
2. **Improved servo life**: Reduces stress on servo mechanisms
3. **Better stability**: Gradual transitions maintain balance
4. **OpenSHC compatibility**: Similar behavior to reference implementation
5. **Enhanced user experience**: More natural and professional robot movement
6. **Configurable precision**: Balance between speed and smoothness

## Troubleshooting

### Movement Too Slow

-   Increase `interpolation_speed` (0.2-0.5)
-   Reduce `max_interpolation_steps` (10-15)

### Movement Too Jerky

-   Decrease `interpolation_speed` (0.05-0.1)
-   Increase `max_interpolation_steps` (25-40)

### Trajectory Not Starting

-   Verify `use_current_servo_positions = true`
-   Check that servo interface returns valid positions
-   Ensure `enable_pose_interpolation = true`

### Unexpected Behavior

-   Call `resetSmoothMovement()` to clear trajectory state
-   Use `setBodyPoseImmediate()` for troubleshooting

This feature represents a significant improvement in robot movement quality and brings HexaMotion's behavior closer to professional robotics platforms like OpenSHC.
