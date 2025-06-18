# HexaMotion

Library for controlling locomotion, gait, and pose of a hexapod robot.

## Overview

HexaMotion provides kinematic and gait planning utilities to drive a six legged robot. It is designed for use with the **Arduino Giga R1** and relies on the [ArduinoEigen](https://github.com/arduino-libraries/ArduinoEigen) package for matrix math.
The robot body forms a hexagon, so each coxa joint is mounted at a 60° interval around the center.

## Features

-   Inverse kinematics using DH parameters and Jacobians.
-   Pose and orientation control via IMU feedback.
-   Multiple gait planner with tripod, wave, ripple and metachronal options.
-   FSR input for contact detection.
-   Smart servo interface for precise joint control.
-   Error reporting and self tests.

## Prerequisites

-   Arduino IDE with board support for **Arduino Giga R1**.
-   Install the **ArduinoEigen** library using the Library Manager or by copying it into your `libraries` folder.

## Including the library

Place this repository inside your Arduino `libraries` directory. In your sketch include the header:

```cpp
#include <locomotion_system.h>
```

## Basic usage

Create hardware interface classes that implement `IIMUInterface`, `IFSRInterface` and `IServoInterface`. Then pass instances of these classes to `LocomotionSystem` together with your robot parameters.

**Note**: HexaMotion now **fully supports advanced IMUs like the BNO055** with enhanced terrain adaptation and locomotion capabilities. The system automatically detects and leverages absolute positioning data while maintaining 100% backward compatibility with basic IMUs. See [Enhanced IMU Implementation Report](ENHANCED_IMU_IMPLEMENTATION_REPORT.md) for complete details.

```cpp
#include <Arduino.h>
#include <locomotion_system.h>

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
  robot.initialize(&imu, &fsr, &servos);
}

void loop() {
  robot.update();
}
```

Make sure the joint limit arrays (`coxa_angle_limits`, `femur_angle_limits` and
`tibia_angle_limits`) are populated with valid ranges before creating the
`LocomotionSystem` instance. If these values remain at their defaults the system
will flag `KINEMATICS_ERROR` and skip sending servo commands.

Simple mock implementations of these interfaces are provided under the
`examples/` directory.

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

## 🚀 Enhanced IMU Integration

HexaMotion now provides **complete support for advanced IMUs** like the BNO055, offering significant improvements in precision and capability:

### **Enhanced Capabilities:**

-   **🎯 Precision Terrain Adaptation**: Uses absolute orientation for accurate gravity estimation
-   **⚡ Dynamic Motion Detection**: Leverages gravity-free linear acceleration data
-   **🧭 Quaternion-Based Analysis**: Advanced terrain complexity assessment
-   **📊 Multi-Factor Stability**: Enhanced stability calculations using all available sensor data
-   **🔄 Intelligent Fallback**: Automatically degrades gracefully to basic algorithms when needed

### **Supported IMU Types:**

-   **Advanced IMUs** (BNO055, etc.): Full enhanced feature set
-   **Basic IMUs** (MPU6050, etc.): Original functionality preserved

### **Key Benefits:**

```cpp
// Enhanced features automatically enabled for advanced IMUs
MyBNO055IMU advanced_imu;  // Implements hasAbsolutePositioning() = true
LocomotionSystem robot(params);
robot.initialize(&advanced_imu, &fsr, &servos);
// System automatically uses enhanced algorithms

// Basic IMUs work exactly as before
MyMPU6050IMU basic_imu;    // hasAbsolutePositioning() = false
robot.initialize(&basic_imu, &fsr, &servos);
// Uses proven original algorithms
```

**📚 Documentation**: See [Enhanced IMU Implementation Report](ENHANCED_IMU_IMPLEMENTATION_REPORT.md) for complete technical details.
