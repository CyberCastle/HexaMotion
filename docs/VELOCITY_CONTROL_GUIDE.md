# HexaMotion Velocity Control Guide

This document describes different ways to adjust locomotion speed in HexaMotion, based on the current architecture and code behavior.

## Executive Summary

HexaMotion provides multiple methods to control and tune hexapod motion speed:

1. **Direct Cartesian velocity commands**
2. **Gait parameters (step length and frequency)**
3. **Default servo speed configuration**
4. **Dynamic velocity limits**
5. **Cruise velocity mode**

## 1. Cartesian Velocity Commands

### Main Method: `setDesiredVelocity`

```cpp
// Using StateController
Eigen::Vector2d linear_vel(30.0, 10.0);  // x=30mm/s, y=10mm/s
double angular_vel = 15.0;                // 15°/s rotation
state_controller.setDesiredVelocity(linear_vel, angular_vel);
```

### Direct Methods in `LocomotionSystem`

```cpp
// Forward movement
bool walkForward(double velocity);                    // velocity in mm/s

// Backward movement
bool walkBackward(double velocity);

// Side movement
bool walkSideways(double velocity, bool right_direction = true);

// In-place turn
bool turnInPlace(double angular_velocity);          // angular_velocity in °/s

// Combined control (most flexible method)
bool planGaitSequence(double vx, double vy, double omega);
```

### Basic Usage Example

```cpp
void setup() {
    // System initialization...
    state_controller.requestRobotState(ROBOT_RUNNING);
}

void loop() {
    if (state_controller.isReadyForOperation()) {
        // Direct movement
        locomotion_system.walkForward(25.0f);     // 25 mm/s forward
        delay(2000);
        locomotion_system.turnInPlace(30.0f);     // 30°/s turn
        delay(1000);
    // API update: stopMovement() replaced by stopWalking()
    locomotion_system.stopWalking();
    }

    state_controller.update(0.02);
    locomotion_system.update();
}
```

## 2. Gait Parameters

### Step Length Configuration

NOTE (consistency update): After stride-planning unification, `step_length` is based on the standing horizontal reach (a conservative horizontal reach with vertical tibia and femur projection), not on full geometric extension (`coxa + femur + tibia`). This removes stride overestimation and aligns velocity limits with real reachable trajectories. The `gait_*_length_factor` multipliers are still applied over this horizontal reach.

`step_length` is computed dynamically based on several factors:

```cpp
// In LocomotionSystem::updateStepParameters()
switch (current_gait) {
case TRIPOD_GAIT:
    // Longer steps for speed
    step_length = leg_reach * params.gait_factors.tripod_length_factor;
    break;
case WAVE_GAIT:
    // Shorter steps for stability
    step_length = leg_reach * params.gait_factors.wave_length_factor;
    break;
// ...
}
```

### Factors Affecting `step_length`

```cpp
float LocomotionSystem::getStepLength() const {
    float base_step_length = step_length;
    float leg_reach = calculateLegReach();
    float max_safe_step = leg_reach * params.gait_factors.max_length_factor;

    // Stability adjustment
    float stability_factor = 1.0f;
    if (stability_index < 0.5f) {
        stability_factor = 0.7f + 0.3f * stability_index;
    }

    // Terrain adjustment (using IMU)
    float terrain_factor = 1.0f;
    if (terrain_complexity > threshold) {
        terrain_factor *= adjustment_factor;
    }

    return base_step_length * stability_factor * terrain_factor;
}
```

### Frequency Configuration

Gait frequency can be adjusted with parameters:

```cpp
// In OpenSHC (dynamic parameters)
params_.step_frequency.current_value = new_frequency;  // Hz

// Equivalent in HexaMotion
gait_config.frequency = 1.5f;                         // 1.5 Hz
walk_controller.updateVelocityLimits(
    gait_config.frequency,
    gait_config.stance_ratio,
    gait_config.time_to_max_stride
);
```

## 3. Default Servo Speed

### Global Configuration

```cpp
// In HexaModel.h - configuration parameters
struct ServoConfig {
    float default_servo_speed = 100.0f;  // Default speed
    float max_servo_speed = 200.0f;      // Maximum speed
    // ...
};

// Use the unified interface
servo_interface->setJointAngleAndSpeed(angle, speed);
```

### Relationship with Cartesian Velocity

Servo speed is related to Cartesian velocity through the **Jacobian matrix**:

```
joint_velocities = J^(-1) * cartesian_velocities
```

In HexaMotion, this relation is handled implicitly through:

- Inverse-kinematics target position calculations
- Gait timing and command-frequency adjustments
- Dynamically computed velocity limits

## 4. Dynamic Velocity Limits

### `VelocityLimits` System

HexaMotion implements a dynamic velocity limit system:

```cpp
class VelocityLimits {
public:
    struct LimitValues {
        float linear_x;      // Linear speed in X (mm/s)
        float linear_y;      // Linear speed in Y (mm/s)
        float angular_z;     // Angular speed in Z (°/s)
    };

    struct WorkspaceConfig {
        float leg_length;           // Total leg length
        float body_radius;          // Robot body radius
        float safety_margin;        // Safety margin (0-1)
        float min_ground_clearance; // Minimum ground clearance
    };
};
```

### Limit Calculation

```cpp
// In VelocityLimits::calculateMaxLinearSpeed()
float calculateMaxLinearSpeed(float walkspace_radius,
                              float on_ground_ratio,
                              float frequency) const {
    if (on_ground_ratio <= 0.0f || frequency <= 0.0f) {
        return 0.0f;
    }

    float cycle_time = on_ground_ratio / frequency;
    float max_speed = (walkspace_radius * 2.0f) / cycle_time;

    return std::min(max_speed, 5.0f); // Safety cap: 5 m/s
}
```

### Limit Application

```cpp
// In WalkController
VelocityLimits::LimitValues WalkController::applyVelocityLimits(
    float vx, float vy, float omega) const {

    auto limits = getVelocityLimits();

    VelocityLimits::LimitValues result;
    result.linear_x = std::clamp(vx, -limits.linear_x, limits.linear_x);
    result.linear_y = std::clamp(vy, -limits.linear_y, limits.linear_y);
    result.angular_z = std::clamp(omega, -limits.angular_z, limits.angular_z);

    return result;
}
```

## 5. Cruise Velocity Mode

### Configuration

```cpp
// In StateController
state_controller.setCruiseControlMode(CRUISE_CONTROL_ON, Eigen::Vector3d(20.0, 5.0, 10.0));
state_controller.setCruiseControlMode(CRUISE_CONTROL_OFF);
```

### Usage in Velocity Control

```cpp
void StateController::updateVelocityControl() {
    float linear_x, linear_y, angular_z;

    if (cruise_mode_enabled_) {
        // Use predefined cruise velocity
        linear_x = cruise_velocity_.x();
        linear_y = cruise_velocity_.y();
        angular_z = cruise_velocity_.z();
    } else {
        // Use direct velocity input
        linear_x = desired_linear_velocity_.x();
        linear_y = desired_linear_velocity_.y();
        angular_z = desired_angular_velocity_;
    }

    // Apply velocity control
    if (abs(linear_x) > 0.01f || abs(linear_y) > 0.01f || abs(angular_z) > 0.01f) {
        locomotion_system_.planGaitSequence(linear_x, linear_y, angular_z);
    }
}
```

## 6. Velocity Control Chain

### Data Flow

```
User/Application
       ↓
StateController::setDesiredVelocity()
       ↓
StateController::updateVelocityControl()
       ↓
LocomotionSystem::planGaitSequence()
       ↓
WalkController::planGaitSequence()
       ↓
WalkController::applyVelocityLimits()
       ↓
LocomotionSystem::update()
       ↓
IServoInterface::setJointAngleAndSpeed()
```

### Key Components

1. **VelocityLimits**: Computes dynamic limits from geometry and gait
2. **WalkController**: Applies limits and validates velocity commands
3. **LocomotionSystem**: Executes gait patterns and coordinates movement
4. **StateController**: Manages states and provides a high-level interface

## 7. Practical Examples

### Example 1: Basic Velocity Control

```cpp
void basicVelocityControl() {
    // Initial setup
    state_controller.requestRobotState(ROBOT_RUNNING);

    while (!state_controller.isReadyForOperation()) {
        state_controller.update(0.01);
        delay(10);
    }

    // Controlled movement
    Eigen::Vector2d linear(25.0, 0.0);  // 25 mm/s forward
    double angular = 0.0;

    state_controller.setDesiredVelocity(linear, angular);

    // Execute for 3 seconds
    for (int i = 0; i < 300; i++) {
        state_controller.update(0.01);
        locomotion_system.update();
        delay(10);
    }

    // Stop
    state_controller.setDesiredVelocity(Eigen::Vector2d(0.0, 0.0), 0.0);
}
```

### Example 2: Adaptive Velocity Tuning

```cpp
void adaptiveVelocityControl() {
    float base_speed = 30.0f;  // mm/s

    while (true) {
        // Read sensor data
        if (imu_interface && imu_interface->isConnected()) {
            IMUData imu_data = imu_interface->readIMU();

            // Adjust speed according to terrain tilt
            float tilt = sqrt(imu_data.roll * imu_data.roll +
                             imu_data.pitch * imu_data.pitch);

            float speed_factor = 1.0f;
            if (tilt > 10.0f) {  // Reduce speed on inclined terrain
                speed_factor = std::max(0.5f, 1.0f - (tilt - 10.0f) / 20.0f);
            }

            float adjusted_speed = base_speed * speed_factor;

            Eigen::Vector2d linear(adjusted_speed, 0.0);
            state_controller.setDesiredVelocity(linear, 0.0f);
        }

        state_controller.update(0.05);
        locomotion_system.update();
        delay(50);
    }
}
```

### Example 3: Cruise Control

```cpp
void cruiseControlDemo() {
    // Configure cruise velocity
    Eigen::Vector3d cruise_vel(20.0, 5.0, 10.0);  // x, y, angular
    state_controller.setCruiseControlMode(CRUISE_CONTROL_ON, cruise_vel);

    // Robot maintains this velocity automatically
    while (cruiseActive) {
        state_controller.update(0.02);
        locomotion_system.update();
        delay(20);
    }

    // Disable cruise mode
    state_controller.setCruiseControlMode(CRUISE_CONTROL_OFF);
}
```

## 8. Advanced Configuration

### Gait Parameter Tuning

```cpp
// Configure gait factors for different speed targets
struct GaitFactors {
    float tripod_length_factor = 0.8f;   // High speed
    float wave_length_factor = 0.6f;     // High stability
    float ripple_length_factor = 0.7f;   // Balanced

    float max_length_factor = 0.9f;      // Safety limit
};

// Adjust based on conditions
if (high_speed_mode) {
    params.gait_factors.tripod_length_factor = 0.9f;
    locomotion_system.setGaitType(TRIPOD_GAIT);
} else if (stability_required) {
    params.gait_factors.wave_length_factor = 0.5f;
    locomotion_system.setGaitType(WAVE_GAIT);
}
```

### Velocity Limit Setup

```cpp
// Configure workspace and limits
VelocityLimits::WorkspaceConfig workspace;
workspace.leg_length = 150.0f;           // mm
workspace.body_radius = 100.0f;          // mm
workspace.safety_margin = 0.8f;          // 80% of workspace
workspace.min_ground_clearance = 20.0f;  // mm

// Configure gait for limit calculations
GaitConfiguration gait;
gait.step_frequency = 1.5f;              // Hz
gait.time_to_max_stride = 2.0f;          // s
// gait.phase_config is configured automatically from gait type

// Apply configuration
walk_controller.updateVelocityLimits(gait);
walk_controller.setVelocitySafetyMargin(workspace.safety_margin);
```

## 9. Troubleshooting

### Problem: Robot does not reach desired speed

**Possible causes:**

1. Velocity commands exceed computed limits
2. Restrictive gait parameters
3. Terrain/conditions reduce dynamic limits

**Solutions:**

```cpp
// Check current limits
auto limits = walk_controller.getVelocityLimits();
Serial.println("Max linear X: " + String(limits.linear_x));
Serial.println("Max linear Y: " + String(limits.linear_y));
Serial.println("Max angular Z: " + String(limits.angular_z));

// Tune parameters if needed
walk_controller.setVelocitySafetyMargin(0.9f);  // Increase usable margin
walk_controller.updateVelocityLimits(2.0f, 0.5f, 1.5f);  // Higher frequency
```

### Problem: Unstable motion at high speed

**Solutions:**

```cpp
// Switch to a more stable gait
locomotion_system.setGaitType(WAVE_GAIT);

// Reduce step length
params.gait_factors.wave_length_factor = 0.5f;

// Increase frequency while preserving speed target
walk_controller.updateVelocityLimits(2.5f, 0.7f, 1.0f);
```

## 10. Comparison with OpenSHC

| Aspect              | HexaMotion                               | OpenSHC                                        |
| ------------------- | ---------------------------------------- | ---------------------------------------------- |
| **Velocity Limits** | Dynamically computed by `VelocityLimits` | Computed by `WalkController::generateLimits()` |
| **Parameters**      | Configurable structures                  | Runtime-tunable parameters                     |
| **Interface**       | `StateController::setDesiredVelocity()`  | `WalkController::updateWalk()`                 |
| **Gait**            | Enum with predefined types               | Flexible timing parameters                     |
| **Servo Speed**     | Unified `setJointAngleAndSpeed()`        | Separate speed parameters                      |

## Conclusion

HexaMotion provides a complete and flexible velocity-control system that enables:

1. **Direct control** through Cartesian velocity commands
2. **Advanced configuration** via gait and workspace parameters
3. **Safety** through dynamic limits and command validation
4. **Adaptability** with condition-based adjustments
5. **Flexibility** with multiple control approaches for different applications

The system is designed to be simple for basic use and powerful for advanced scenarios that require fine locomotion tuning.
