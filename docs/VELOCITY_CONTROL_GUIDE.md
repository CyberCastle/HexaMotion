# HexaMotion Velocity Control Guide

This document describes how to control and tune locomotion speed in HexaMotion, based on the current architecture and code behavior.

> **Scope note:** HexaMotion core operates in REAL velocity mode only (physical units: mm/s and rad/s). It does **not** implement an internal "throttle"/normalized input mode, a `body_velocity_scaler`, or cruise control — per `AGENTS.md` these are the responsibility of external software, which must pre-scale or maintain commands before calling the library.

## Methods at a Glance

1. Direct Cartesian velocity commands
2. Gait parameters (step length and frequency)
3. Servo speed control (`CartesianVelocityController`)
4. Dynamic velocity limits

## 1. Cartesian Velocity Commands

### Via `StateController`

```cpp
Eigen::Vector2d linear_vel(30.0, 10.0);  // x = 30 mm/s, y = 10 mm/s
double angular_vel = 15.0;                // 15°/s rotation
state_controller.setDesiredVelocity(linear_vel, angular_vel);
```

### Direct Methods in `LocomotionSystem`

```cpp
bool walkForward(double velocity);                       // mm/s
bool walkBackward(double velocity);                      // mm/s
bool walkSideways(double velocity, bool right = true);   // mm/s
bool turnInPlace(double angular_velocity);               // °/s
bool planGaitSequence(double vx, double vy, double omega); // most flexible
bool stopWalking();
```

### Basic Usage

```cpp
void loop() {
    if (state_controller.isReadyForOperation()) {
        locomotion_system.walkForward(25.0f);  // 25 mm/s forward
        locomotion_system.turnInPlace(30.0f);  // 30°/s turn
        locomotion_system.stopWalking();
    }
    state_controller.update(0.02);
    locomotion_system.update();
}
```

## 2. Gait Parameters

### Step Length

`step_length` is based on the standing horizontal reach (conservative reach with vertical tibia and femur projection), **not** on full geometric extension (`coxa + femur + tibia`). This removes stride overestimation and aligns velocity limits with reachable trajectories. The `gait_*_length_factor` multipliers are applied over this horizontal reach.

```cpp
// LocomotionSystem::updateStepParameters()
switch (current_gait) {
case TRIPOD_GAIT:
    step_length = leg_reach * params.gait_factors.tripod_length_factor; // longer steps for speed
    break;
case WAVE_GAIT:
    step_length = leg_reach * params.gait_factors.wave_length_factor;   // shorter steps for stability
    break;
// ...
}
```

`getStepLength()` additionally scales the base step length by stability and terrain factors derived from the stability index and IMU-based terrain complexity.

### Frequency

```cpp
gait_config.frequency = 1.5f; // Hz
walk_controller.updateVelocityLimits(gait_config);
```

## 3. Servo Speed Control (`CartesianVelocityController`)

The `CartesianVelocityController` dynamically adjusts servo speeds based on commanded Cartesian velocities, equivalent to OpenSHC's velocity-driven joint speed behavior (REAL velocity mode).

### Speed Calculation

```
final_speed = base_speed × linear_scale × angular_scale × gait_factor × leg_compensation
```

- **Linear scale** grows with linear velocity magnitude.
- **Angular scale** increases servo speed for outer legs during rotation.
- **Gait factor** is a per-gait multiplier (e.g., tripod 1.2x, wave 0.8x, ripple 0.9x).
- **Per-leg compensation** accounts for distance from body center and motion direction.
- **Per-joint scaling**: coxa 0.9x, femur 1.0x, tibia 1.1x.

### Setup and Configuration

```cpp
CartesianVelocityController* velocity_ctrl = locomotion.getVelocityController();
locomotion.setVelocityControlEnabled(true); // false → constant default speeds

CartesianVelocityController::VelocityScaling scaling;
scaling.linear_velocity_scale = 2.0f;
scaling.angular_velocity_scale = 1.5f;
scaling.minimum_speed_ratio = 0.2f;  // 20% of base speed
scaling.maximum_speed_ratio = 1.8f;  // 180% of base speed
locomotion.setVelocityScaling(scaling);

CartesianVelocityController::GaitSpeedModifiers modifiers;
modifiers.tripod_speed_factor = 1.3f;
modifiers.wave_speed_factor = 0.7f;
modifiers.ripple_speed_factor = 0.9f;
locomotion.setGaitSpeedModifiers(modifiers);
```

### Commanding Motion (speeds scale automatically)

```cpp
locomotion.planGaitSequence(100.0, 0.0, 0.0); // 100 mm/s forward
locomotion.planGaitSequence(0.0, 0.0, 0.5);   // 0.5 rad/s rotation (outer legs faster)
locomotion.planGaitSequence(80.0, 0.0, 0.3);  // combined, balanced
```

### Reading Current Speeds

```cpp
float coxa_speed  = locomotion.getCurrentServoSpeed(0, 0);
float femur_speed = locomotion.getCurrentServoSpeed(0, 1);
float tibia_speed = locomotion.getCurrentServoSpeed(0, 2);
bool enabled = velocity_ctrl->isVelocityControlEnabled();
float mag = velocity_ctrl->getCurrentVelocityMagnitude();
```

### Tuning

- **Faster response:** increase `linear_velocity_scale`, `angular_velocity_scale`, `maximum_speed_ratio`, and gait factors.
- **Smoother motion:** decrease the scales, increase `minimum_speed_ratio`, lower gait factors.
- **Power efficiency:** use moderate scaling factors and terrain-tuned gait factors.

## 4. Dynamic Velocity Limits

HexaMotion computes velocity/acceleration limits dynamically from leg geometry and gait timing via `VelocityLimits` (equivalent to OpenSHC's `WalkController::generateLimits()`).

```cpp
struct LimitValues { float linear_x; float linear_y; float angular_z; };

float calculateMaxLinearSpeed(float walkspace_radius, float on_ground_ratio, float frequency) const {
    if (on_ground_ratio <= 0.0f || frequency <= 0.0f) return 0.0f;
    float cycle_time = on_ground_ratio / frequency;
    return (walkspace_radius * 2.0f) / cycle_time;
}
```

Limits are applied by clamping each commanded component in `WalkController` before planning the gait sequence.

```cpp
// Configure workspace + gait, then refresh limits
walk_controller.updateVelocityLimits(gait);
walk_controller.setVelocitySafetyMargin(0.8f);

auto limits = walk_controller.getVelocityLimits(); // inspect linear_x/linear_y/angular_z
```

## 5. Velocity Control Chain

```
User/Application
       ↓
StateController::setDesiredVelocity()
       ↓
StateController::updateVelocityControl()
       ↓
LocomotionSystem::planGaitSequence()
       ↓
WalkController::planGaitSequence()  →  applyVelocityLimits()
       ↓
LocomotionSystem::update()
       ↓
IServoInterface::setJointAngleAndSpeed()
```

## 6. Examples

### Basic Velocity Control

```cpp
state_controller.requestRobotState(ROBOT_RUNNING);
while (!state_controller.isReadyForOperation()) { state_controller.update(0.01); }

state_controller.setDesiredVelocity(Eigen::Vector2d(25.0, 0.0), 0.0); // 25 mm/s forward
for (int i = 0; i < 300; i++) { state_controller.update(0.01); locomotion_system.update(); }

state_controller.setDesiredVelocity(Eigen::Vector2d(0.0, 0.0), 0.0);  // stop
```

### Adaptive Velocity Tuning (terrain tilt)

```cpp
IMUData imu_data = imu_interface->readIMU();
float tilt = sqrt(imu_data.roll * imu_data.roll + imu_data.pitch * imu_data.pitch);
float speed_factor = (tilt > 10.0f) ? std::max(0.5f, 1.0f - (tilt - 10.0f) / 20.0f) : 1.0f;
state_controller.setDesiredVelocity(Eigen::Vector2d(base_speed * speed_factor, 0.0), 0.0f);
```

## 7. Troubleshooting

**Robot does not reach desired speed** — commands may exceed the computed limits or gait parameters are too restrictive:

```cpp
auto limits = walk_controller.getVelocityLimits();
walk_controller.setVelocitySafetyMargin(0.9f);
walk_controller.updateVelocityLimits(gait); // with higher frequency
```

**Unstable motion at high speed** — switch to a more stable gait and/or reduce step length:

```cpp
locomotion_system.setGaitType(WAVE_GAIT);
params.gait_factors.wave_length_factor = 0.5f;
```

## 8. Comparison with OpenSHC

| Aspect          | HexaMotion                               | OpenSHC                                        |
| --------------- | ---------------------------------------- | ---------------------------------------------- |
| Velocity Limits | Dynamically computed by `VelocityLimits` | Computed by `WalkController::generateLimits()` |
| Parameters      | Configurable structures                  | Runtime-tunable parameters                     |
| Interface       | `StateController::setDesiredVelocity()`  | `WalkController::updateWalk()`                 |
| Gait            | Enum with predefined types               | Flexible timing parameters                     |
| Servo Speed     | Unified `setJointAngleAndSpeed()`        | Separate speed parameters                      |
| Input mode      | REAL only (external throttle/cruise)     | REAL + throttle + cruise control               |
