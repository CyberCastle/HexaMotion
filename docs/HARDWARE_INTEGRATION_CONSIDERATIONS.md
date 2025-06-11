# Hardware Integration Considerations for HexaMotion

**IMU and FSR Real-World Implementation Guide**

**Date:** June 11, 2025
**Version:** 1.0
**Target:** Production Hardware Integration

## Overview

During the development and testing of OpenSHC equivalent features, several critical issues were discovered in the simulation interfaces (DummyIMU and DummyFSR) that reveal important considerations when integrating real hardware sensors. This document provides guidance for successful real-world implementation.

## 🚨 Critical Issues Discovered

### IMU Integration Issues

#### Issue 1: Gravity Vector Convention Mismatch

**Problem Discovered:**

```cpp
// Original problematic implementation
Point3D gravity_estimate = accel * (-1.0f);  // Wrong for our DummyIMU
```

**Root Cause:**
Different IMU manufacturers use different acceleration vector conventions:

-   **Physics Convention:** IMU reports acceleration opposing gravity (e.g., +9.81 m/s² when stationary)
-   **Direct Convention:** IMU reports gravity vector directly (e.g., -9.81 m/s² pointing down)

**Real-World Impact:**

-   Auto-posing systems may compensate in wrong direction
-   Gravity estimation will be incorrect
-   Robot may become unstable during IMU-based corrections

#### Issue 2: Timing and Update Rate Problems

**Problem Discovered:**

```cpp
// Timing check prevented updates in test environment
if (current_time - last_update_time_ < update_interval_) {
    return; // Skip update - problematic in real systems
}
```

**Real-World Impact:**

-   IMU data may be missed during critical moments
-   Auto-pose corrections may lag behind actual orientation changes
-   System may become unresponsive during high-frequency operations

#### Issue 3: Filter Convergence Time

**Problem Discovered:**
Low-pass filters with small alpha values (0.05-0.1) require many updates to converge:

```cpp
gravity_filter_ = lowPassFilter(gravity_estimate, gravity_filter_, filter_alpha_);
// With alpha=0.1, takes ~10 updates to reach 63% of target value
```

**Real-World Impact:**

-   Slow response to orientation changes
-   Robot may not adapt quickly enough to terrain changes
-   Startup time may be extended for initial calibration

### FSR Integration Issues

#### Issue 4: Contact Detection Sensitivity

**Problem Pattern:**

```cpp
// Simple threshold-based detection (from DummyFSR analysis)
data.in_contact = (data.pressure > threshold);
```

**Real-World Challenges:**

-   Static thresholds don't account for surface variations
-   Noise and vibration can cause false triggers
-   Temperature and humidity affect sensor readings

#### Issue 5: Force Calibration and Scaling

**Problem Identified:**
FSR sensors provide non-linear resistance-to-force relationships that require proper calibration.

## 🔧 Real Hardware Implementation Guidelines

### IMU Implementation Best Practices

#### 1. Determine Your IMU Convention

**Before Implementation:**

```cpp
// Test your specific IMU to determine convention
void calibrateIMUConvention() {
    // Place IMU flat on level surface
    IMUData data = imu.readIMU();

    if (data.accel_z > 8.0f) {
        // Physics convention: reports opposing acceleration
        use_physics_convention = true;
    } else if (data.accel_z < -8.0f) {
        // Direct convention: reports gravity directly
        use_physics_convention = false;
    }
}
```

#### 2. Adaptive Gravity Estimation

**Recommended Implementation:**

```cpp
void updateGravityEstimate(const IMUData& imu_data) {
    Point3D accel(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
    float magnitude = math_utils::magnitude(accel);

    if (magnitude > 0.5f && magnitude < 15.0f) { // Valid range
        Point3D gravity_estimate;

        if (use_physics_convention) {
            // IMU reports acceleration opposing gravity
            gravity_estimate = accel * (-1.0f);
        } else {
            // IMU reports gravity directly
            gravity_estimate = accel;
        }

        // Adaptive filter based on motion state
        float alpha = (robot_moving) ? 0.02f : 0.1f;
        gravity_filter_ = lowPassFilter(gravity_estimate, gravity_filter_, alpha);
    }
}
```

#### 3. Robust Timing Implementation

**Production-Ready Timing:**

```cpp
void update(float dt) {
    // Always process if dt is provided (manual override)
    if (dt > 0.0f) {
        processIMUUpdate(dt);
        return;
    }

    // Normal timing check with fallback
    unsigned long current_time = millis();
    unsigned long elapsed = current_time - last_update_time_;

    // Process if enough time elapsed OR if too much time elapsed (missed updates)
    if (elapsed >= update_interval_ || elapsed > (update_interval_ * 3)) {
        last_update_time_ = current_time;
        processIMUUpdate(elapsed / 1000.0f); // Convert to seconds
    }
}
```

#### 4. IMU Calibration Sequence

**Startup Calibration:**

```cpp
bool calibrateIMU() {
    std::cout << "Starting IMU calibration..." << std::endl;

    // 1. Determine convention
    calibrateIMUConvention();

    // 2. Bias calibration (robot stationary)
    Point3D accel_sum(0, 0, 0);
    Point3D gyro_sum(0, 0, 0);
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        IMUData data = readIMU();
        accel_sum = accel_sum + Point3D(data.accel_x, data.accel_y, data.accel_z);
        gyro_sum = gyro_sum + Point3D(data.gyro_x, data.gyro_y, data.gyro_z);
        delay(10);
    }

    // Calculate bias offsets
    accel_bias_ = accel_sum * (1.0f / samples);
    gyro_bias_ = gyro_sum * (1.0f / samples);

    // 3. Validate calibration
    if (use_physics_convention) {
        // Expect ~9.81 magnitude
        expected_gravity_z = 9.81f;
    } else {
        // Expect ~-9.81 value
        expected_gravity_z = -9.81f;
    }

    return (abs(abs(accel_bias_.z) - 9.81f) < 1.0f);
}
```

### FSR Implementation Best Practices

#### 1. Dynamic Threshold Adaptation

**Adaptive Contact Detection:**

```cpp
class SmartFSR {
private:
    float baseline_pressure;
    float noise_level;
    float adaptive_threshold;

public:
    bool detectContact(float raw_pressure) {
        // Update baseline during non-contact periods
        if (!in_contact && raw_pressure < (baseline_pressure + 2 * noise_level)) {
            baseline_pressure = baseline_pressure * 0.99f + raw_pressure * 0.01f;
        }

        // Update noise estimation
        float pressure_variation = abs(raw_pressure - baseline_pressure);
        noise_level = noise_level * 0.95f + pressure_variation * 0.05f;

        // Adaptive threshold
        adaptive_threshold = baseline_pressure + 3 * noise_level + min_contact_force;

        return (raw_pressure > adaptive_threshold);
    }
};
```

#### 2. Force Calibration

**Non-linear FSR Calibration:**

```cpp
float calibrateFSRForce(float raw_reading) {
    // FSR typically has inverse relationship: R = k / F
    // Voltage divider: V = Vcc * R2 / (R1 + R2) where R1 = FSR

    if (raw_reading < min_valid_reading) return 0.0f;

    // Convert ADC to voltage
    float voltage = (raw_reading / 1023.0f) * supply_voltage;

    // Calculate FSR resistance
    float fsr_resistance = pulldown_resistor * (supply_voltage / voltage - 1.0f);

    // Convert resistance to force (calibrated curve)
    // This requires physical calibration with known weights
    return forceFromResistance(fsr_resistance);
}

float forceFromResistance(float resistance) {
    // Example calibration curve (requires actual measurement)
    // Force = A / (Resistance - B) + C
    const float A = 50000.0f;  // Calibration constants
    const float B = 1000.0f;   // determined by physical
    const float C = 0.0f;      // testing

    if (resistance < B) return 0.0f;
    return A / (resistance - B) + C;
}
```

#### 3. Contact Time and Stability

**Robust Contact Detection:**

```cpp
struct ContactState {
    bool raw_contact;
    bool stable_contact;
    unsigned long contact_start_time;
    unsigned long last_contact_time;
    float contact_confidence;
};

void updateContactState(int leg_index, float pressure) {
    ContactState& state = contact_states[leg_index];
    unsigned long current_time = millis();

    // Raw contact detection
    state.raw_contact = (pressure > adaptive_threshold[leg_index]);

    if (state.raw_contact) {
        if (state.contact_start_time == 0) {
            state.contact_start_time = current_time;
        }
        state.last_contact_time = current_time;

        // Build confidence over time
        unsigned long contact_duration = current_time - state.contact_start_time;
        state.contact_confidence = min(1.0f, contact_duration / 100.0f); // 100ms to full confidence
    } else {
        // Check for recent contact
        if (current_time - state.last_contact_time > 50) { // 50ms timeout
            state.contact_start_time = 0;
            state.contact_confidence = 0.0f;
        }
    }

    // Stable contact requires both detection and confidence
    state.stable_contact = state.raw_contact && (state.contact_confidence > 0.5f);
}
```

## 🏭 Production Hardware Recommendations

### IMU Selection Criteria

**Recommended Specifications:**

-   **Update Rate:** Minimum 100Hz, preferred 200Hz+
-   **Gyro Range:** ±500°/s minimum for hexapod applications
-   **Accelerometer Range:** ±4g minimum (±8g preferred)
-   **Digital Interface:** I2C or SPI (avoid analog outputs)
-   **Built-in Filtering:** Digital filtering capabilities preferred
-   **Temperature Compensation:** Essential for outdoor applications

**Recommended Models:**

-   **MPU-6050/6500:** Budget option, good for indoor use
-   **BNO055:** Built-in sensor fusion, excellent for beginners
-   **ICM-20948:** High precision, good temperature stability
-   **Bosch BMI088:** Professional grade, excellent stability

### FSR Selection and Installation

**FSR Specifications:**

-   **Force Range:** 0-50N minimum per leg (adjust based on robot weight)
-   **Response Time:** <1ms for dynamic applications
-   **Repeatability:** <3% error for consistent measurements
-   **Operating Temperature:** Match your environment requirements

**Installation Guidelines:**

-   **Mounting:** Ensure even pressure distribution across sensor area
-   **Protection:** Waterproof housing for outdoor applications
-   **Wiring:** Use shielded cables to reduce noise
-   **Backup:** Consider redundant sensors for critical applications

## 🧪 Testing and Validation Procedures

### IMU Validation Tests

```cpp
void validateIMUHardware() {
    // 1. Static test (robot stationary)
    validateStaticReadings();

    // 2. Known orientation test
    validateKnownOrientations();

    // 3. Dynamic response test
    validateDynamicResponse();

    // 4. Temperature stability test
    validateTemperatureStability();
}
```

### FSR Validation Tests

```cpp
void validateFSRHardware() {
    // 1. Zero load test
    validateZeroReading();

    // 2. Known weight test
    validateKnownWeights();

    // 3. Dynamic loading test
    validateDynamicLoading();

    // 4. Cross-talk test (ensure leg independence)
    validateLegIndependence();
}
```

## ⚠️ Common Pitfalls and Solutions

### IMU Pitfalls

1. **Magnetic Interference:** Keep away from motors and metal structures
2. **Vibration Sensitivity:** Use shock mounting for high-vibration environments
3. **Drift Over Time:** Implement periodic recalibration
4. **Power Supply Noise:** Use clean, regulated power supplies

### FSR Pitfalls

1. **Hysteresis:** FSR readings may differ for loading vs unloading
2. **Creep:** Long-term loading can shift baseline readings
3. **Edge Effects:** Uneven loading can cause inconsistent readings
4. **Aging:** FSR characteristics may change over time

## 🔮 Advanced Considerations

### Multi-Sensor Fusion

```cpp
// Combine IMU with other sensors for improved reliability
class SensorFusion {
    IMUData imu_data;
    EncoderData encoder_data;
    VisualData camera_data;

public:
    OrientationEstimate getFusedOrientation() {
        // Kalman filter or complementary filter
        // combining multiple sensor inputs
    }
};
```

### Predictive Maintenance

```cpp
// Monitor sensor health over time
class SensorHealth {
    void monitorIMU() {
        // Track noise levels, bias drift, response times
        if (noise_level > threshold || bias_drift > limit) {
            triggerCalibrationAlert();
        }
    }

    void monitorFSR() {
        // Track baseline changes, response consistency
        if (baseline_shift > limit || response_variation > threshold) {
            triggerMaintenanceAlert();
        }
    }
};
```

## 📋 Integration Checklist

### Pre-Implementation

-   [ ] Determine IMU acceleration convention
-   [ ] Characterize FSR force-resistance relationship
-   [ ] Plan sensor mounting and protection
-   [ ] Design calibration procedures
-   [ ] Prepare validation test procedures

### Implementation

-   [ ] Implement adaptive thresholding for FSR
-   [ ] Add robust timing and error handling for IMU
-   [ ] Include temperature compensation if needed
-   [ ] Implement sensor health monitoring
-   [ ] Add diagnostic and debug capabilities

### Validation

-   [ ] Test all sensors under expected operating conditions
-   [ ] Validate sensor fusion algorithms
-   [ ] Test failure modes and error handling
-   [ ] Document calibration procedures
-   [ ] Train operators on maintenance procedures

---

**Document Version:** 1.0
**Last Updated:** June 11, 2025
**Next Review:** September 11, 2025

_This document is based on real issues discovered during simulation testing and should be updated as hardware integration experience is gained._
