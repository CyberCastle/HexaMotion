# HexaMotion

Library for controlling locomotion, gait, and pose of a hexapod robot.

## Overview

HexaMotion provides kinematic and gait planning utilities to drive a six legged robot. It is designed for use with the **Arduino Giga R1** and relies on the [ArduinoEigen](https://github.com/arduino-libraries/ArduinoEigen) package for matrix math.
The robot body forms a hexagon, so each coxa joint is mounted at a 60° interval around the center.

## Features

-   **Smooth Trajectory Interpolation**: OpenSHC-style movement using current servo positions as starting points for natural, smooth robot motion.
-   Inverse kinematics using DH parameters and Jacobians.
-   Pose and orientation control via IMU feedback.
-   Multiple gait planner with tripod, wave, ripple and metachronal options.
-   FSR input for contact detection.
-   Smart servo interface for precise joint control.
-   Error reporting and self tests.

## 🚀 Smooth Movement Feature

HexaMotion includes **smooth trajectory interpolation** that uses current servo positions as starting points for pose changes (OpenSHC-style). This provides:

-   **Natural movement**: Smooth transitions instead of sudden position jumps
-   **Current position awareness**: Trajectories start from actual servo positions
-   **Configurable smoothness**: Adjustable interpolation speed and precision
-   **Backward compatibility**: Existing code works with improved behavior

### Quick Start with Smooth Movement

```cpp
// Configure smooth trajectory parameters
params.smooth_trajectory.use_current_servo_positions = true;  // Enable feature
params.smooth_trajectory.interpolation_speed = 0.15f;        // Smooth speed
params.smooth_trajectory.max_interpolation_steps = 20;       // Precision

// Standard pose changes use smooth trajectories automatically when enabled in params
locomotion_system.setBodyPose(new_position, new_orientation);
```

See [Smooth Movement Guide](docs/SMOOTH_MOVEMENT_GUIDE.md) for complete documentation.

## Prerequisites

-   Arduino IDE with board support for **Arduino Giga R1**.
-   Install the **ArduinoEigen** library using the Library Manager or by copying it into your `libraries` folder.

## Including the library

Place this repository inside your Arduino `libraries` directory. In your sketch include the unified header:

```cpp
#include <HexaMotion.h>
```

## Basic usage

Create hardware interface classes that implement `IIMUInterface`, `IFSRInterface` and `IServoInterface`. Then pass instances of these classes to `LocomotionSystem` together with your robot parameters.

**Note**: HexaMotion supports advanced IMUs like the BNO055 with enhanced terrain adaptation and locomotion capabilities. The system automatically detects and leverages absolute positioning data while maintaining backward compatibility with basic IMUs.

```cpp
#include <Arduino.h>
#include <HexaMotion.h>

class MyIMU : public IIMUInterface {
    // Implement basic methods and optionally support absolute positioning
    // For BNO055-style IMUs, implement hasAbsolutePositioning() to return true
    /* ... */
};
class MyFSR : public IFSRInterface {
    // Implement all required methods including the new update() method
    // that uses AdvancedAnalog DMA for simultaneous FSR readings
    bool update() override { /* Update internal ADC registers */ }
    /* ... other methods ... */
};
class MyServo : public IServoInterface { /* ... */ };

Parameters params;            // fill your robot parameters
params.coxa_angle_limits[0] = -90;
params.coxa_angle_limits[1] = 90;
params.femur_angle_limits[0] = -90;
params.femur_angle_limits[1] = 90;
params.tibia_angle_limits[0] = -90;
params.tibia_angle_limits[1] = 90;
LocomotionSystem robot(params);
MyIMU imu;
MyFSR fsr;
MyServo servos;

void setup() {
    // Optional: configure smoothing before init
    params.smooth_trajectory.use_current_servo_positions = true;
    params.smooth_trajectory.enable_pose_interpolation = true;
    params.smooth_trajectory.interpolation_speed = 0.15f; // 0.01 - 1.0

    // Create a body pose configuration (factory pattern)
    BodyPoseConfiguration pose_cfg = BodyPoseConfigFactory::create("default", params);

    // Initialize hardware + controllers
    robot.initialize(&imu, &fsr, &servos, pose_cfg);

    // Move to calibrated standing pose
    robot.setStandingPose();

    // Select gait (TRIPOD_GAIT, WAVE_GAIT, RIPPLE_GAIT, METACHRONAL_GAIT)
    robot.setGaitType(TRIPOD_GAIT);

    // Start continuous forward motion (mm/s), or use turnInPlace / walkSideways
    robot.walkForward(30.0);
    // robot.turnInPlace(0.3);      // alternative rotation (rad/s)
}

void loop() {
    // Run control loop (~50 Hz). Consider using a timing guard for real hardware.
    robot.update();
}
```

## 🔧 Servo Interface Changes

HexaMotion now requires **simultaneous control of servo position and speed** for better motor control and smoother movement:

### **New Combined Interface:**

```cpp
class MyServo : public IServoInterface {
public:
    bool initialize() override { return true; }

    // NEW: Primary method - sets angle and speed together
    bool setJointAngleAndSpeed(int leg_index, int joint_index, float angle, float speed) override {
        // Your servo control code here
        // angle: target position in degrees
        // speed: movement speed (0.1-3.0, where 1.0 is normal speed)
        return sendServoCommand(leg_index, joint_index, angle, speed);
    }

    float getJointAngle(int leg_index, int joint_index) override {
        return readServoPosition(leg_index, joint_index);
    }

    bool isJointMoving(int leg_index, int joint_index) override { return false; }
    bool enableTorque(int leg_index, int joint_index, bool enable) override { return true; }
};
```

### **Key Changes:**

-   **`setJointAngleAndSpeed()`** is now the **only** method for servo control
-   **`setJointAngle()`** and **`setJointSpeed()`** have been **removed**
-   Position and speed **must be set together** - no separate control
-   Default servo speed can be configured in `Parameters::default_servo_speed`

### **Migration Guide:**

```cpp
// OLD - separate calls (no longer available)
servo->setJointAngle(leg, joint, 45.0f);
servo->setJointSpeed(leg, joint, 1.5f);

// NEW - combined call (required)
servo->setJointAngleAndSpeed(leg, joint, 45.0f, 1.5f);
```

Make sure the joint limit arrays (`coxa_angle_limits`, `femur_angle_limits` and
`tibia_angle_limits`) are populated with valid ranges before creating the
`LocomotionSystem` instance. If these values remain at their defaults the system
will flag `KINEMATICS_ERROR` and skip sending servo commands.

Example mock implementations of these interfaces can be created following the skeletons above (a dedicated `examples/` directory was removed).

Debug logging can be enabled by defining the `ENABLE_LOG` macro before
including the library. When active, certain events such as joint limit
violations will be printed to the serial console.

## Configurable parameters

The `Parameters` structure defines the physical dimensions and control limits of the robot. Key fields include:

-   `hexagon_radius`, `coxa_length`, `femur_length`, `tibia_length`.
-   `robot_height` and `robot_weight`.
-   Joint angle limits for coxa, femur and tibia.
-   IMU and FSR calibration settings.
-   Gait tuning factors and control frequency.
    See `src/locomotion_system.h` for a detailed list.

## Running tests

The test suite depends on the Eigen library. A helper script in the
`tests/` directory installs this dependency automatically on both Linux and
macOS. Ensure you have **Homebrew** installed when running on macOS.
The accompanying `Makefile` will detect the Eigen installation path on
macOS using `brew --prefix eigen`, so no manual configuration is required.
After executing the script, build the tests using `make`:

```bash
cd tests
./setup.sh
make
```

Each test binary can be executed individually once the build completes.

## IMU Integration

Advanced IMUs (e.g. BNO055) automatically enable: absolute orientation usage, improved terrain adaptation, gravity-free linear acceleration, quaternion-based terrain analysis and multi-factor stability estimation. When an IMU does not provide absolute positioning, the system gracefully falls back to the baseline algorithms without code changes required. Implement `hasAbsolutePositioning()` accordingly in your `IIMUInterface` implementation.
