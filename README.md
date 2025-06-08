# HexaMotion
Library for controlling locomotion, gait, and pose of a hexapod robot.

## Overview
HexaMotion provides kinematic and gait planning utilities to drive a six legged robot.  It is designed for use with the **Arduino Giga R1** and relies on the [ArduinoEigen](https://github.com/arduino-libraries/ArduinoEigen) package for matrix math.

## Prerequisites
- Arduino IDE with board support for **Arduino Giga R1**.
- Install the **ArduinoEigen** library using the Library Manager or by copying it into your `libraries` folder.

## Including the library
Place this repository inside your Arduino `libraries` directory.  In your sketch include the header:

```cpp
#include <locomotion_system.h>
```

## Basic usage
Create hardware interface classes that implement `IIMUInterface`, `IFSRInterface` and `IServoInterface`.  Then pass instances of these classes to `LocomotionSystem` together with your robot parameters.

```cpp
#include <Arduino.h>
#include <locomotion_system.h>

class MyIMU : public IIMUInterface { /* ... */ };
class MyFSR : public IFSRInterface { /* ... */ };
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

