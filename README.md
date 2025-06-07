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
#include <hexapod_locomotion_system.h>
```

## Basic usage
Create hardware interface classes that implement `IIMUInterface`, `IFSRInterface` and `IServoInterface`.  Then pass instances of these classes to `HexapodLocomotionSystem` together with your robot parameters.

```cpp
#include <Arduino.h>
#include <hexapod_locomotion_system.h>

class MyIMU : public IIMUInterface { /* ... */ };
class MyFSR : public IFSRInterface { /* ... */ };
class MyServo : public IServoInterface { /* ... */ };

HexapodParameters params;            // fill your robot parameters
HexapodLocomotionSystem robot(params);
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

Simple mock implementations of these interfaces are provided under the
`examples/` directory.

## Running tests
Basic tests for the utility functions can be built using `make` inside the
`tests/` folder:

```bash
cd tests
make && ./utils_test
```

