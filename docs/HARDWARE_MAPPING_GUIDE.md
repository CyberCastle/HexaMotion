# Hardware Mapping Guide for HexaMotion Library

**Comprehensive Guide to Leg, Servo, and FSR Mapping**

**Date:** June 12, 2025
**Version:** 1.0
**Target:** Hardware Integration and Production Implementation

## Overview

This document provides a comprehensive guide for mapping physical hardware components (legs, servos, FSR sensors) to the HexaMotion library's logical structure. Understanding this mapping is crucial for successful hardware integration and proper robot operation.

## Table of Contents

1. [Leg Indexing System](#leg-indexing-system)
2. [Servo Mapping Architecture](#servo-mapping-architecture)
3. [FSR Sensor Integration](#fsr-sensor-integration)
4. [Hardware Interface Implementation](#hardware-interface-implementation)
5. [Complete Example Implementation](#complete-example-implementation)
6. [Pin Assignment Examples](#pin-assignment-examples)
7. [Troubleshooting and Validation](#troubleshooting-and-validation)

## Leg Indexing System

### Hexapod Leg Numbering Convention

The HexaMotion library uses a standardized leg indexing system based on clockwise numbering when viewed from above:

```
     FRONT
        ┌─────┐
   L5 ┌─┘     └─┐ L0  ← Front Right (Index 0)
      │         │
   L4 │    ●    │ L1  ← Middle Right (Index 1)
      │         │
   L3 └─┐     ┌─┘ L2  ← Rear Right (Index 2)
        └─────┘
     REAR
```

**Leg Index Mapping:**

-   **Leg 0**: Front Right (FR) - Starting position, 0° offset
-   **Leg 1**: Middle Right (MR) - 60° clockwise from Leg 0
-   **Leg 2**: Rear Right (RR) - 120° clockwise from Leg 0
-   **Leg 3**: Rear Left (RL) - 180° clockwise from Leg 0
-   **Leg 4**: Middle Left (ML) - 240° clockwise from Leg 0
-   **Leg 5**: Front Left (FL) - 300° clockwise from Leg 0

### Leg Base Position Calculation

Each leg's base position is calculated relative to the robot's center:

```cpp
// From locomotion_system.cpp - leg base position calculation
Point3D LocomotionSystem::getLegBasePosition(int leg_index) {
    float angle_deg = leg_index * 60.0f;  // 60° between legs
    float angle_rad = angle_deg * M_PI / 180.0f;

    float x = params.hexagon_radius * cos(angle_rad);
    float y = params.hexagon_radius * sin(angle_rad);
    float z = 0.0f;  // Base level

    return Point3D(x, y, z);
}
```

## Servo Mapping Architecture

### Joint Indexing per Leg

Each leg has 3 degrees of freedom (DOF) with standardized joint indexing:

```
         Hip/Coxa Joint (Index 0)
              │
         ┌────┴────┐
         │         │
    Knee/Femur     │
    Joint (Index 1)│
         │         │
         └────┬────┘
              │
         Ankle/Tibia Joint (Index 2)
              │
            [FOOT]
```

**Joint Index Convention:**

-   **Joint 0**: Coxa (Hip) - Horizontal rotation around vertical axis
-   **Joint 1**: Femur (Knee) - Vertical rotation, first leg segment
-   **Joint 2**: Tibia (Ankle) - Vertical rotation, second leg segment

### Servo Address Calculation

The library uses a systematic approach to map logical joint indices to physical servo addresses:

```cpp
// Servo addressing formula
int servo_address = leg_index * DOF_PER_LEG + joint_index;

// Examples:
// Leg 0, Joint 0 (Front Right Coxa): servo_address = 0 * 3 + 0 = 0
// Leg 0, Joint 1 (Front Right Femur): servo_address = 0 * 3 + 1 = 1
// Leg 0, Joint 2 (Front Right Tibia): servo_address = 0 * 3 + 2 = 2
// Leg 1, Joint 0 (Middle Right Coxa): servo_address = 1 * 3 + 0 = 3
```

### Complete Servo Mapping Table

| Leg Index | Joint Name | Joint Index | Servo Address | Physical Location  |
| --------- | ---------- | ----------- | ------------- | ------------------ |
| 0         | Coxa       | 0           | 0             | Front Right Hip    |
| 0         | Femur      | 1           | 1             | Front Right Knee   |
| 0         | Tibia      | 2           | 2             | Front Right Ankle  |
| 1         | Coxa       | 0           | 3             | Middle Right Hip   |
| 1         | Femur      | 1           | 4             | Middle Right Knee  |
| 1         | Tibia      | 2           | 5             | Middle Right Ankle |
| 2         | Coxa       | 0           | 6             | Rear Right Hip     |
| 2         | Femur      | 1           | 7             | Rear Right Knee    |
| 2         | Tibia      | 2           | 8             | Rear Right Ankle   |
| 3         | Coxa       | 0           | 9             | Rear Left Hip      |
| 3         | Femur      | 1           | 10            | Rear Left Knee     |
| 3         | Tibia      | 2           | 11            | Rear Left Ankle    |
| 4         | Coxa       | 0           | 12            | Middle Left Hip    |
| 4         | Femur      | 1           | 13            | Middle Left Knee   |
| 4         | Tibia      | 2           | 14            | Middle Left Ankle  |
| 5         | Coxa       | 0           | 15            | Front Left Hip     |
| 5         | Femur      | 1           | 16            | Front Left Knee    |
| 5         | Tibia      | 2           | 17            | Front Left Ankle   |

## FSR Sensor Integration

### FSR Mapping Convention

Each leg has one FSR sensor located at the foot/tip. The FSR indexing follows the same leg indexing system:

```cpp
// FSR index directly corresponds to leg index
int fsr_sensor_id = leg_index;

// Examples:
// Leg 0 (Front Right): FSR 0
// Leg 1 (Middle Right): FSR 1
// Leg 2 (Rear Right): FSR 2
// ... and so on
```

### FSR Data Structure

Each FSR provides three key data points:

```cpp
struct FSRData {
    float pressure;      // Force measurement in Newtons or pressure units
    bool in_contact;     // Boolean contact detection
    float contact_time;  // Duration of continuous contact in seconds
};
```

### Advanced FSR Integration with DMA

The library supports simultaneous FSR reading using AdvancedAnalog with DMA:

```cpp
class ProductionFSR : public IFSRInterface {
private:
    uint16_t adc_values[NUM_LEGS];    // Raw ADC readings
    FSRData processed_data[NUM_LEGS]; // Processed FSR data

public:
    bool update() override {
        // Trigger simultaneous ADC reading on all FSR channels
        // This method is called once per control cycle
        analogRead_DMA_start(fsr_channels, NUM_LEGS);

        // Process raw values
        for (int i = 0; i < NUM_LEGS; i++) {
            processed_data[i] = convertADCToFSRData(adc_values[i], i);
        }
        return true;
    }

    FSRData readFSR(int leg_index) override {
        return processed_data[leg_index];
    }
};
```

## Hardware Interface Implementation

### Complete Servo Interface Example

```cpp
class HexapodServoController : public IServoInterface {
private:
    // Servo pin mapping - adjust for your hardware
    int servo_pins[TOTAL_DOF] = {
        // Leg 0 (Front Right)
        2, 3, 4,     // Coxa, Femur, Tibia
        // Leg 1 (Middle Right)
        5, 6, 7,     // Coxa, Femur, Tibia
        // Leg 2 (Rear Right)
        8, 9, 10,    // Coxa, Femur, Tibia
        // Leg 3 (Rear Left)
        11, 12, 13,  // Coxa, Femur, Tibia
        // Leg 4 (Middle Left)
        14, 15, 16,  // Coxa, Femur, Tibia
        // Leg 5 (Front Left)
        17, 18, 19   // Coxa, Femur, Tibia
    };

    Servo servos[TOTAL_DOF];
    float current_angles[NUM_LEGS][DOF_PER_LEG];

public:
    bool initialize() override {
        for (int i = 0; i < TOTAL_DOF; i++) {
            servos[i].attach(servo_pins[i]);
            servos[i].write(90); // Center position
        }
        return true;
    }

    bool setJointAngleAndSpeed(int leg_index, int joint_index, float angle, float speed) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS) return false;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG) return false;

        int servo_id = leg_index * DOF_PER_LEG + joint_index;

        // Convert from library angles to servo angles
        float servo_angle = convertLibraryToServoAngle(leg_index, joint_index, angle);

        // Apply speed control if servo supports it (e.g., Dynamixel servos)
        // For standard hobby servos, speed parameter may be ignored
        // Example for Dynamixel: setGoalPosition(servo_id, servo_angle); setMovingSpeed(servo_id, speed);

        servos[servo_id].write(servo_angle);
        current_angles[leg_index][joint_index] = angle;

        // Store speed for potential use in advanced servo control
        // current_speeds[leg_index][joint_index] = speed;

        return true;
    }

    float getJointAngle(int leg_index, int joint_index) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS) return 0.0f;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG) return 0.0f;

        return current_angles[leg_index][joint_index];
    }

private:
    float convertLibraryToServoAngle(int leg, int joint, float library_angle) {
        // Convert from library coordinate system to servo coordinate system
        // This may involve offset and inversion depending on servo mounting

        float servo_angle = library_angle + 90.0f; // Offset to 0-180 range

        // Apply per-servo corrections if needed
        if (leg % 2 == 1) { // Right side legs
            if (joint == 1 || joint == 2) {
                servo_angle = 180.0f - servo_angle; // Invert for mirrored mounting
            }
        }

        return constrain(servo_angle, 0, 180);
    }
};
```

### Complete FSR Interface Example

```cpp
class HexapodFSRController : public IFSRInterface {
private:
    // FSR analog pin mapping
    int fsr_pins[NUM_LEGS] = {A0, A1, A2, A3, A4, A5};

    // Calibration data per FSR
    struct FSRCalibration {
        float baseline;
        float threshold;
        float max_pressure;
        float calibration_curve[10][2]; // [ADC, Force] pairs
    } calibration[NUM_LEGS];

    FSRData current_data[NUM_LEGS];
    unsigned long last_update_time;

public:
    bool initialize() override {
        // Initialize analog pins
        for (int i = 0; i < NUM_LEGS; i++) {
            pinMode(fsr_pins[i], INPUT);

            // Initialize calibration with defaults
            calibration[i].baseline = 0.0f;
            calibration[i].threshold = 50.0f;  // ADC units
            calibration[i].max_pressure = 100.0f; // Newtons
        }

        last_update_time = millis();
        return true;
    }

    bool update() override {
        // Read all FSR channels simultaneously
        for (int i = 0; i < NUM_LEGS; i++) {
            int raw_reading = analogRead(fsr_pins[i]);
            current_data[i] = processRawReading(i, raw_reading);
        }

        last_update_time = millis();
        return true;
    }

    FSRData readFSR(int leg_index) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS) {
            return FSRData{0.0f, false, 0.0f};
        }
        return current_data[leg_index];
    }

    bool calibrateFSR(int leg_index) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS) return false;

        // Measure baseline (no load)
        float sum = 0.0f;
        for (int i = 0; i < 100; i++) {
            sum += analogRead(fsr_pins[leg_index]);
            delay(10);
        }

        calibration[leg_index].baseline = sum / 100.0f;
        calibration[leg_index].threshold = calibration[leg_index].baseline + 30.0f;

        return true;
    }

    float getRawReading(int leg_index) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS) return 0.0f;
        return analogRead(fsr_pins[leg_index]);
    }

private:
    FSRData processRawReading(int leg_index, int raw_adc) {
        FSRData data;

        // Convert ADC to pressure using calibration curve
        data.pressure = convertADCToPressure(leg_index, raw_adc);

        // Contact detection with hysteresis
        static bool previous_contact[NUM_LEGS] = {false};
        static unsigned long contact_start_time[NUM_LEGS] = {0};

        bool raw_contact = raw_adc > calibration[leg_index].threshold;

        if (raw_contact && !previous_contact[leg_index]) {
            contact_start_time[leg_index] = millis();
        }

        data.in_contact = raw_contact &&
                         (millis() - contact_start_time[leg_index] > 20); // 20ms debounce

        if (data.in_contact) {
            data.contact_time = (millis() - contact_start_time[leg_index]) / 1000.0f;
        } else {
            data.contact_time = 0.0f;
            contact_start_time[leg_index] = 0;
        }

        previous_contact[leg_index] = data.in_contact;
        return data;
    }

    float convertADCToPressure(int leg_index, int adc_value) {
        // Simple linear conversion - replace with actual calibration curve
        float normalized = (adc_value - calibration[leg_index].baseline) / 1023.0f;
        return normalized * calibration[leg_index].max_pressure;
    }
};
```

## Complete Example Implementation

### Robot Configuration and Initialization

```cpp
#include <Arduino.h>
#include <locomotion_system.h>

// Hardware interface instances
HexapodServoController servo_controller;
HexapodFSRController fsr_controller;
HexapodIMUController imu_controller; // Implementation not shown

// Robot parameters
Parameters robot_params;
LocomotionSystem* locomotion_system;

void setup() {
    Serial.begin(115200);
    Serial.println("HexaMotion Hardware Integration Example");

    // Configure robot parameters
    initializeRobotParameters();

    // Create locomotion system
    locomotion_system = new LocomotionSystem(robot_params);

    // Initialize hardware interfaces
    if (!servo_controller.initialize()) {
        Serial.println("ERROR: Servo controller initialization failed");
        return;
    }

    if (!fsr_controller.initialize()) {
        Serial.println("ERROR: FSR controller initialization failed");
        return;
    }

    if (!imu_controller.initialize()) {
        Serial.println("ERROR: IMU controller initialization failed");
        return;
    }

    // Connect hardware to locomotion system
    locomotion_system->initialize(&imu_controller, &fsr_controller, &servo_controller);

    // Calibrate system
    if (locomotion_system->calibrateSystem()) {
        Serial.println("✓ System calibration successful");
    } else {
        Serial.println("✗ System calibration failed");
        return;
    }

    // Set standing pose
    locomotion_system->setStandingPose();

    Serial.println("Hardware integration complete - robot ready");
}

void initializeRobotParameters() {
    // Physical dimensions (in millimeters)
    robot_params.hexagon_radius = 150.0f;
    robot_params.coxa_length = 50.0f;
    robot_params.femur_length = 100.0f;
    robot_params.tibia_length = 120.0f;

    // Robot characteristics
    robot_params.robot_height = 80.0f;
    robot_params.robot_weight = 2.5f;
    robot_params.center_of_mass = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    // Joint limits (degrees)
    robot_params.coxa_angle_limits[0] = -90.0f;
    robot_params.coxa_angle_limits[1] = 90.0f;
    robot_params.femur_angle_limits[0] = -90.0f;
    robot_params.femur_angle_limits[1] = 90.0f;
    robot_params.tibia_angle_limits[0] = -90.0f;
    robot_params.tibia_angle_limits[1] = 90.0f;

    // Sensor parameters
    robot_params.fsr_threshold = 50.0f;           // Touchdown threshold (ADC units)
    robot_params.fsr_liftoff_threshold = 25.0f;   // Liftoff threshold (ADC units)
    robot_params.fsr_max_pressure = 100.0f;       // Newtons

    // Control parameters
    robot_params.max_velocity = 100.0f;         // mm/s
    robot_params.max_angular_velocity = 45.0f;  // degrees/s
    robot_params.control_frequency = 50.0f;     // Hz
}

void loop() {
    // Update locomotion system
    locomotion_system->update();

    // Print status every second
    static unsigned long last_status = 0;
    if (millis() - last_status > 1000) {
        printSystemStatus();
        last_status = millis();
    }

    delay(20); // 50Hz control loop
}

void printSystemStatus() {
    Serial.println("=== System Status ===");

    // Print leg positions
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D pos = locomotion_system->getCurrentLegPosition(leg);
        FSRData fsr = fsr_controller.readFSR(leg);

        Serial.print("Leg ");
        Serial.print(leg);
        Serial.print(": Pos(");
        Serial.print(pos.x);
        Serial.print(", ");
        Serial.print(pos.y);
        Serial.print(", ");
        Serial.print(pos.z);
        Serial.print(") FSR: ");
        Serial.print(fsr.pressure);
        Serial.print("N Contact: ");
        Serial.println(fsr.in_contact ? "YES" : "NO");
    }

    Serial.println();
}
```

## Pin Assignment Examples

### Arduino Mega 2560 Pin Assignment

```cpp
// Servo pins (PWM capable)
int servo_pins[18] = {
    // Legs 0-2 (Right side)
    2, 3, 4,    // Leg 0: FR Coxa, Femur, Tibia
    5, 6, 7,    // Leg 1: MR Coxa, Femur, Tibia
    8, 9, 10,   // Leg 2: RR Coxa, Femur, Tibia
    // Legs 3-5 (Left side)
    11, 12, 13, // Leg 3: RL Coxa, Femur, Tibia
    22, 23, 24, // Leg 4: ML Coxa, Femur, Tibia
    25, 26, 27  // Leg 5: FL Coxa, Femur, Tibia
};

// FSR analog pins
int fsr_pins[6] = {A0, A1, A2, A3, A4, A5};

// IMU I2C pins (standard)
// SDA: Pin 20
// SCL: Pin 21
```

### Arduino Giga R1 Pin Assignment (Recommended)

```cpp
// Servo pins (more PWM channels available)
int servo_pins[18] = {
    // High-performance PWM pins for precise control
    2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, A0, A1, A2, A3, A4, A5
};

// FSR pins using AdvancedAnalog
int fsr_pins[6] = {A6, A7, A8, A9, A10, A11};

// IMU I2C pins
// SDA: Pin 20
// SCL: Pin 21
```

## Troubleshooting and Validation

### Hardware Validation Checklist

```cpp
void validateHardwareMapping() {
    Serial.println("=== Hardware Validation ===");

    // Test 1: Servo mapping validation
    Serial.println("Testing servo mapping...");
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int joint = 0; joint < DOF_PER_LEG; joint++) {
            Serial.print("Moving Leg ");
            Serial.print(leg);
            Serial.print(" Joint ");
            Serial.print(joint);
            Serial.println(" to 45°");

            servo_controller.setJointAngle(leg, joint, 45.0f);
            delay(1000);

            servo_controller.setJointAngle(leg, joint, 0.0f);
            delay(500);
        }
    }

    // Test 2: FSR validation
    Serial.println("Testing FSR sensors...");
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        FSRData data = fsr_controller.readFSR(leg);
        Serial.print("FSR ");
        Serial.print(leg);
        Serial.print(": ");
        Serial.print(data.pressure);
        Serial.println("N");
    }

    // Test 3: Coordinate system validation
    Serial.println("Testing coordinate system...");
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D base_pos = locomotion_system->getLegBasePosition(leg);
        Serial.print("Leg ");
        Serial.print(leg);
        Serial.print(" base: (");
        Serial.print(base_pos.x);
        Serial.print(", ");
        Serial.print(base_pos.y);
        Serial.println(")");
    }
}
```

### Common Mapping Issues and Solutions

#### Issue 1: Servo Direction Inversion

**Problem**: Leg moves in opposite direction
**Solution**: Implement servo angle conversion

```cpp
float convertLibraryToServoAngle(int leg, int joint, float library_angle) {
    float servo_angle = library_angle + 90.0f; // Center at 90°

    // Invert for left-side legs (legs 3, 4, 5)
    if (leg >= 3) {
        if (joint == 1 || joint == 2) { // Femur and Tibia
            servo_angle = 180.0f - servo_angle;
        }
    }

    return constrain(servo_angle, 0, 180);
}
```

#### Issue 2: FSR Contact Detection Problems

**Problem**: False positives or missed contacts
**Solution**: Implement adaptive thresholding

```cpp
bool detectContact(int leg_index, float raw_pressure) {
    static float baseline[NUM_LEGS] = {0};
    static float noise_level[NUM_LEGS] = {0};

    // Update baseline during non-contact periods
    if (raw_pressure < baseline[leg_index] + 2 * noise_level[leg_index]) {
        baseline[leg_index] = baseline[leg_index] * 0.99f + raw_pressure * 0.01f;
    }

    // Estimate noise
    noise_level[leg_index] = noise_level[leg_index] * 0.95f +
                            abs(raw_pressure - baseline[leg_index]) * 0.05f;

    // Adaptive threshold
    float threshold = baseline[leg_index] + 3 * noise_level[leg_index] + 10.0f;

    return raw_pressure > threshold;
}
```

#### Issue 3: Coordinate System Misalignment

**Problem**: Robot moves in wrong direction
**Solution**: Verify leg base positions and coordinate system

```cpp
void verifyCoordinateSystem() {
    // Test forward movement
    locomotion_system->setDesiredVelocity(Point3D(50, 0, 0), 0); // Move forward

    // Expected behavior: Front legs (0, 5) should step forward
    // If robot moves backward, coordinate system is inverted

    // Test rotation
    locomotion_system->setDesiredVelocity(Point3D(0, 0, 0), 30); // Rotate CCW

    // Expected behavior: Right legs should step backward relative to center
    // If robot rotates CW, angular direction is inverted
}
```

## Best Practices

1. **Always validate hardware mapping** before full operation
2. **Implement safety limits** for servo angles and movements
3. **Use adaptive thresholding** for FSR contact detection
4. **Calibrate sensors** before each operation session
5. **Monitor system health** during operation
6. **Document your specific pin assignments** and hardware modifications

## Conclusion

This guide provides the complete framework for mapping hardware components to the HexaMotion library. The systematic approach ensures reliable operation and makes troubleshooting easier. Always test each component individually before integrating the complete system.

---

**For additional information, see:**

-   `docs/HARDWARE_INTEGRATION_CONSIDERATIONS.md` - Hardware-specific implementation details
-   `docs/FSR_ADVANCED_ANALOG_INTEGRATION.md` - Advanced FSR integration
-   `examples/complete_state_machine_demo.ino` - Complete working example

**Status**: ✅ Complete technical documentation
**Implementation**: Ready for production hardware integration
