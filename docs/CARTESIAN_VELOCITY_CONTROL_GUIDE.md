# Cartesian Velocity Control System

## Overview

The Cartesian Velocity Control system in HexaMotion provides equivalent functionality to OpenSHC's velocity control mechanism. It dynamically adjusts servo speeds based on commanded Cartesian velocities, enabling responsive and natural robot motion.

## Key Features

### 1. Velocity-to-Speed Mapping

-   **Linear Velocity Scaling**: Higher linear velocities result in faster servo speeds
-   **Angular Velocity Compensation**: Rotation commands increase servo speeds for outer legs
-   **Combined Motion Handling**: Simultaneous linear and angular motion is properly balanced

### 2. Gait-Specific Adjustments

-   **Tripod Gait**: 1.2x speed multiplier for fast locomotion
-   **Wave Gait**: 0.8x speed multiplier for stable, careful movement
-   **Ripple Gait**: 0.9x speed multiplier for balanced motion
-   **Metachronal Gait**: 1.0x speed multiplier for smooth operation
-   **Adaptive Gait**: 1.1x speed multiplier for terrain adaptation

### 3. Per-Leg Compensation

Each leg's servo speeds are individually adjusted based on:

-   Distance from robot center (outer legs move faster during rotation)
-   Direction of motion relative to leg position
-   Velocity magnitude requirements for that specific leg

### 4. Joint-Specific Scaling

-   **Coxa Joint**: 0.9x scaling (lower speed requirements)
-   **Femur Joint**: 1.0x scaling (baseline speed)
-   **Tibia Joint**: 1.1x scaling (fine positioning, higher speeds)

## Usage

### Basic Setup

```cpp
#include "locomotion_system.h"
#include "cartesian_velocity_controller.h"

// Initialize locomotion system
LocomotionSystem locomotion(parameters);
locomotion.initialize(&imu, &fsr, &servo);

// Get velocity controller reference
CartesianVelocityController* velocity_ctrl = locomotion.getVelocityController();
```

### Enable/Disable Velocity Control

```cpp
// Enable velocity-based servo speed control
locomotion.setVelocityControlEnabled(true);

// Disable (use constant default speeds)
locomotion.setVelocityControlEnabled(false);
```

### Configure Velocity Scaling

```cpp
CartesianVelocityController::VelocityScaling scaling;
scaling.linear_velocity_scale = 2.0f;    // Linear velocity impact
scaling.angular_velocity_scale = 1.5f;   // Angular velocity impact
scaling.minimum_speed_ratio = 0.2f;      // Minimum 20% of base speed
scaling.maximum_speed_ratio = 1.8f;      // Maximum 180% of base speed

locomotion.setVelocityScaling(scaling);
```

### Configure Gait Speed Modifiers

```cpp
CartesianVelocityController::GaitSpeedModifiers modifiers;
modifiers.tripod_speed_factor = 1.3f;    // Faster tripod gait
modifiers.wave_speed_factor = 0.7f;      // Slower wave gait
modifiers.ripple_speed_factor = 0.9f;    // Standard ripple gait

locomotion.setGaitSpeedModifiers(modifiers);
```

### Command Motion with Automatic Speed Control

```cpp
// Forward motion - servo speeds automatically scale with velocity (values now in mm/s)
locomotion.planGaitSequence(100.0f, 0.0f, 0.0f);  // 100 mm/s forward

// Rotation - outer legs automatically get higher speeds (angular remains in rad/s)
locomotion.planGaitSequence(0.0f, 0.0f, 0.5f);  // 0.5 rad/s rotation

// Combined motion - speeds balanced for both linear and angular
locomotion.planGaitSequence(80.0f, 0.0f, 0.3f); // 80 mm/s + rotation
```

### Get Current Servo Speeds

```cpp
// Get speed for specific joint
float coxa_speed = locomotion.getCurrentServoSpeed(0, 0);    // Leg 0, Coxa
float femur_speed = locomotion.getCurrentServoSpeed(0, 1);   // Leg 0, Femur
float tibia_speed = locomotion.getCurrentServoSpeed(0, 2);   // Leg 0, Tibia

// Get all speeds for a leg
const auto& leg_speeds = velocity_ctrl->getLegServoSpeeds(0);
float coxa_effective = leg_speeds.coxa.getEffectiveSpeed();
```

## OpenSHC Equivalence

This system provides equivalent functionality to OpenSHC's velocity control in REAL velocity mode only. HexaMotion core
no implementa internamente el modo "throttle"; cualquier joystick o entrada normalizada debe convertirse fuera de la
biblioteca a velocidades físicas (mm/s y rad/s) antes de llamar `planGaitSequence`.

### 1. Body Velocity Scaling

Un efecto similar a `body_velocity_scaler` puede lograrse pre-escalando las velocidades o ajustando los factores de servo.

### 2. Joint Velocity Limits

Equivalentes mediante escalado dinámico de velocidades de servos.

### 3. Gait-Dependent Speed Control

Factores específicos por gait replican el ajuste de OpenSHC.

## Technical Implementation

### Speed Calculation Formula

For each servo, the final speed is calculated as:

```
final_speed = base_speed × linear_scale × angular_scale × gait_factor × leg_compensation
```

Where:

-   `base_speed`: Robot's default servo speed parameter
-   `linear_scale`: Scaling based on linear velocity magnitude
-   `angular_scale`: Scaling based on angular velocity magnitude
-   `gait_factor`: Gait-specific speed multiplier
-   `leg_compensation`: Per-leg adjustment based on position and motion

### Velocity-to-Speed Mapping

Linear velocity scaling:

```
velocity_ratio = velocity_magnitude / max_velocity
linear_scale = min_ratio + (max_ratio - min_ratio) × velocity_ratio
```

Angular velocity scaling:

```
angular_ratio = angular_velocity / max_angular_velocity
angular_scale = 1.0 + angular_ratio × angular_scale_factor
```

## Configuration Parameters

### Default Values

-   **Linear Velocity Scale**: 2.0 (2x impact of linear velocity)
-   **Angular Velocity Scale**: 1.5 (1.5x impact of angular velocity)
-   **Minimum Speed Ratio**: 0.2 (20% minimum speed)
-   **Maximum Speed Ratio**: 1.8 (180% maximum speed)
-   **Tripod Speed Factor**: 1.2 (20% faster)
-   **Wave Speed Factor**: 0.8 (20% slower)

### Tuning Recommendations

**For Faster Response:**

-   Increase `linear_velocity_scale` and `angular_velocity_scale`
-   Increase `maximum_speed_ratio`
-   Use higher gait speed factors

**For Smoother Motion:**

-   Decrease `linear_velocity_scale` and `angular_velocity_scale`
-   Increase `minimum_speed_ratio`
-   Use lower gait speed factors

**For Power Efficiency:**

-   Use moderate scaling factors
-   Set appropriate minimum speeds
-   Tune gait factors based on terrain

## Example Applications

### 1. Speed-Responsive Locomotion

Robot automatically adjusts movement speed based on commanded velocity:

```cpp
locomotion.planGaitSequence(50.0f, 0.0f, 0.0f);   // Slow, careful movement (mm/s)
locomotion.planGaitSequence(150.0f, 0.0f, 0.0f);  // Fast, responsive movement (mm/s)
```

### 2. Adaptive Turning

Outer legs automatically move faster during rotation:

```cpp
locomotion.planGaitSequence(0.0f, 0.0f, 1.0f);   // Fast rotation, servo speeds adjusted per leg
```

### 3. Gait-Optimized Movement

Different gaits automatically use appropriate servo speeds:

```cpp
locomotion.setGaitType(TRIPOD_GAIT);              // Fast gait, higher servo speeds
locomotion.setGaitType(WAVE_GAIT);                // Stable gait, lower servo speeds
```

### 4. Terrain-Adaptive Speeds

Motion speed automatically adjusts to terrain requirements through gait selection and velocity commands.

## Debugging and Monitoring

### Check Velocity Control Status

```cpp
bool is_enabled = velocity_ctrl->isVelocityControlEnabled();
float velocity_magnitude = velocity_ctrl->getCurrentVelocityMagnitude();
```

### Monitor Servo Speeds

```cpp
for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int joint = 0; joint < DOF_PER_LEG; joint++) {
        float speed = locomotion.getCurrentServoSpeed(leg, joint);
        Serial.printf("Leg %d Joint %d: %.2f\n", leg, joint, speed);
    }
}
```

This velocity control system ensures that servo speeds appropriately reflect the commanded Cartesian motion, providing natural and responsive robot movement equivalent to OpenSHC's sophisticated velocity control capabilities.
