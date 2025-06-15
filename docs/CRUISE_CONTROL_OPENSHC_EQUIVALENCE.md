# Cruise Control Implementation - OpenSHC Equivalence

## Overview

The cruise control functionality in HexaMotion has been enhanced to be equivalent to the OpenSHC (Open Source Hexapod Controller) implementation. This document describes the improvements made and how they align with OpenSHC's behavior.

## Key Improvements

### 1. Combined Velocity Control

**Before:**

-   Only used `cruise_velocity_.x()` for forward movement
-   Ignored Y-axis and angular velocity components
-   Used separate method calls for different movement types

**After:**

-   Uses all three velocity components (x, y, angular_z)
-   Employs `planGaitSequence(vx, vy, omega)` for combined movement
-   Equivalent to OpenSHC's `walker_->updateWalk(linear_velocity_input_, angular_velocity_input_)`

### 2. Automatic Velocity Capture

**OpenSHC Behavior:**

```cpp
// Save current velocity input as cruise input
linear_cruise_velocity_ = linear_velocity_input_;
angular_cruise_velocity_ = angular_velocity_input_;
```

**HexaMotion Implementation:**

```cpp
// Save current velocity input as cruise velocity (equivalent to OpenSHC behavior)
cruise_velocity_.x() = desired_linear_velocity_.x();
cruise_velocity_.y() = desired_linear_velocity_.y();
cruise_velocity_.z() = desired_angular_velocity_;
```

### 3. Time Limit Support

**New Configuration Parameter:**

```cpp
struct StateMachineConfig {
    // ...
    float cruise_control_time_limit = 0.0f; ///< Time limit for cruise control (0 = unlimited)
    // ...
};
```

**Automatic Expiration:**

-   Monitors cruise control duration
-   Automatically disables when time limit is reached
-   Equivalent to OpenSHC's `cruise_control_time_limit` parameter

### 4. Enhanced Status Monitoring

**New Methods Added:**

-   `getCruiseVelocity()` - Get current cruise velocity
-   `getCruiseStartTime()` - Get start time
-   `getCruiseRemainingTime()` - Get remaining time before expiration
-   `isCruiseControlActive()` - Check if cruise control is active and valid

## OpenSHC Equivalence Mapping

| OpenSHC Feature                 | HexaMotion Implementation                            |
| ------------------------------- | ---------------------------------------------------- |
| `linear_cruise_velocity_[0,1]`  | `cruise_velocity_.x(), cruise_velocity_.y()`         |
| `angular_cruise_velocity_`      | `cruise_velocity_.z()`                               |
| `cruise_control_time_limit`     | `config_.cruise_control_time_limit`                  |
| `cruise_control_end_time_`      | `cruise_end_time_`                                   |
| `walker_->updateWalk(lin, ang)` | `locomotion_system_.planGaitSequence(vx, vy, omega)` |

## Usage Examples

### Basic Cruise Control

```cpp
// Set specific cruise velocity
Eigen::Vector3f velocity(100.0f, 50.0f, 15.0f); // x, y, angular_z
state_controller.setCruiseControlMode(CRUISE_CONTROL_ON, velocity);
```

### Auto-Capture Current Velocity

```cpp
// Set current movement as cruise velocity
state_controller.setDesiredVelocity(Eigen::Vector2f(80.0f, -30.0f), 10.0f);
state_controller.setCruiseControlMode(CRUISE_CONTROL_ON); // Uses current velocity
```

### Time-Limited Cruise Control

```cpp
StateMachineConfig config;
config.cruise_control_time_limit = 30.0f; // 30 seconds
StateController controller(locomotion_system, config);
```

### Monitor Status

```cpp
if (controller.isCruiseControlActive()) {
    float remaining = controller.getCruiseRemainingTime();
    Serial.print("Time remaining: "); Serial.println(remaining);
}
```

## Implementation Details

### Velocity Control Logic

The enhanced `updateVelocityControl()` method now follows this logic:

1. **Check Time Limit:** Verify if cruise control has expired
2. **Select Velocity Source:** Use cruise velocity or direct input
3. **Apply Combined Movement:** Use `planGaitSequence()` for simultaneous x, y, and angular movement
4. **Error Handling:** Log failures and set error states

### Thread Safety

The implementation maintains the same thread safety characteristics as the original code, with atomic operations for state changes and proper error handling.

### Performance Impact

-   **Minimal Overhead:** Time checks only performed when cruise control is active
-   **Improved Efficiency:** Single `planGaitSequence()` call instead of multiple separate movement commands
-   **Memory Efficient:** Reuses existing data structures with minimal additions

## Compatibility

This implementation maintains backward compatibility with existing code while adding OpenSHC equivalent functionality. All existing method signatures remain unchanged, with new functionality added through optional parameters and new methods.

## Testing

The enhanced cruise control functionality can be tested using:

-   `examples/cruise_control_example.ino` - Comprehensive example
-   Unit tests in `tests/` directory
-   Integration with existing state machine tests

## Future Enhancements

Potential future improvements to further align with OpenSHC:

1. External cruise control mode support
2. Dynamic parameter adjustment during cruise
3. Integration with path planning systems
4. Velocity ramping and smoothing
