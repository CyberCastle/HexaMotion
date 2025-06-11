# Simulation vs Real Hardware Problem Analysis

**Lessons Learned from DummyIMU and DummyFSR**

**Date:** June 11, 2025
**Type:** Technical Analysis
**Purpose:** Document critical problems found in simulation and their impact on real hardware

## Executive Summary

During the development of the OpenSHC equivalent system for HexaMotion, several critical problems were discovered in the simulation interfaces (`DummyIMU` and `DummyFSR`) that reveal fundamental considerations for real hardware implementation. This document analyzes these specific problems and their implications.

## 🔍 Specific Problems Found

### Problem 1: IMU Gravity Vector Convention

#### What happened in DummyIMU:

```cpp
// In test_stubs.h - DummyIMU
IMUData readIMU() override {
    IMUData data{};
    data.roll = test_roll;
    data.pitch = test_pitch;
    data.yaw = test_yaw;
    data.accel_x = 0.0f;
    data.accel_y = 0.0f;
    data.accel_z = -9.81f;  // ← Here's the critical problem
    data.is_valid = true;
    return data;
}
```

#### The problem in the processing code:

```cpp
// In imu_auto_pose.cpp - PROBLEMATIC VERSION
void IMUAutoPose::updateGravityEstimate(const IMUData& imu_data) {
    Point3D accel(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
    float magnitude = math_utils::magnitude(accel);

    if (magnitude > 0.5f) {
        Point3D normalized_accel = accel * (1.0f / magnitude);
        gravity_filter_ = lowPassFilter(normalized_accel * (-9.81f), gravity_filter_, filter_alpha_);
        //                                                 ^^^^^^^^^ PROBLEM: double negation
    }
}
```

#### The incorrect result:

-   `accel_z = -9.81` (from DummyIMU)
-   `normalized_accel.z = -1.0`
-   `normalized_accel * (-9.81f) = (0, 0, 9.81)` ← Gravity pointing upward!

#### How it manifested:

```bash
Debug: accel=(0,0,-9.81) magnitude=9.81 gravity_estimate=(-0,-0,9.81)
Debug: After filter gravity_filter_=(0,0,-7.848)
```

#### Why is this critical for real hardware?

**Different IMUs use different conventions:**

1. **Physics Convention (most commercial IMUs):**

    ```cpp
    // IMU reports acceleration that counteracts gravity
    // Robot stationary on table: accel_z = +9.81 (table pushing upward)
    // Real gravity: (0, 0, -9.81) (pointing downward)
    ```

2. **Direct Convention (some specialized IMUs):**
    ```cpp
    // IMU reports gravity vector directly
    // Robot stationary: accel_z = -9.81 (gravity downward)
    // Real gravity: (0, 0, -9.81) (same value)
    ```

**Impact on Real Hardware:**

-   Auto-pose system will compensate in wrong direction
-   Robot will become more unstable instead of stabilizing
-   Orientation detection will be completely wrong

### Problem 2: Low-Pass Filters and Convergence Time

#### The discovered problem:

```cpp
// In imu_auto_pose.cpp
void IMUAutoPose::initialize() {
    gravity_filter_ = Point3D(0, 0, -9.81f);  // Correct initial value

    switch (config_.precision) {
        case PRECISION_MEDIUM:
            filter_alpha_ = 0.1f; // ← Problem: converges very slowly
            break;
    }
}

// Low-pass filter
Point3D lowPassFilter(Point3D new_value, Point3D old_value, float alpha) {
    return old_value * (1.0f - alpha) + new_value * alpha;
    // With alpha=0.1: new = 0.9 * old + 0.1 * new
}
```

#### Mathematical analysis of the problem:

With `alpha = 0.1`, the filter needs multiple updates to converge:

-   Update 1: `0.9 * (-9.81) + 0.1 * (-9.81) = -9.81` ✓ (works in this case because values are equal)
-   But if there's change: `0.9 * (-9.81) + 0.1 * (9.81) = -7.848` (as we saw in debug)

#### Why is this critical for real hardware?

**Response Time Issues:**

```cpp
// Example: Robot detects 15° tilt
// With alpha=0.05 (high precision), takes ~45 updates for 95% correction
// At 50Hz: 45/50 = 0.9 seconds to respond
// Robot could fall before system responds!
```

**Implemented Solution:**

```cpp
// Adaptive filter based on robot state
float alpha = (robot_moving) ? 0.02f : 0.1f;  // Faster when stationary
```

### Problem 3: Time Verification and `millis()`

#### The problem in simulation:

```cpp
void IMUAutoPose::update(float dt) {
    unsigned long current_time = millis();  // ← In test environment, always returns 0
    if (current_time - last_update_time_ < update_interval_) {
        return;  // ← Always executes, never processes IMU data
    }
}
```

#### Why it occurred:

-   In test environment, `millis()` doesn't advance automatically
-   `current_time - last_update_time_` is always 0
-   System never processes IMU data

#### Why is this critical for real hardware?

**Real-Time Problems:**

1. **Data Loss:** If main processing delays, IMU readings are lost
2. **Time Jitter:** Variations in execution time cause irregular updates
3. **System Overload:** If main loop is slow, time checks fail

**Robust Solution Implemented:**

```cpp
void update(float dt) {
    // Allow manual override for testing
    if (dt > 0.0f) {
        processIMUUpdate(dt);
        return;
    }

    // Robust time verification for real hardware
    unsigned long current_time = millis();
    unsigned long elapsed = current_time - last_update_time_;

    // Process if: 1) enough time OR 2) too much time (recovery)
    if (elapsed >= update_interval_ || elapsed > (update_interval_ * 3)) {
        last_update_time_ = current_time;
        processIMUUpdate(elapsed / 1000.0f);
    }
}
```

## 🔧 DummyFSR Problems and Their Implications

### Problem 4: Simplified Contact Detection

#### Current DummyFSR implementation:

```cpp
// In test_stubs.h
struct DummyFSR : IFSRInterface {
    FSRData readFSR(int leg_index) override {
        FSRData data{};
        data.pressure = 0.0f;        // ← Always zero
        data.in_contact = false;     // ← Never in contact
        data.contact_time = 0.0f;    // ← No history
        return data;
    }
};
```

#### Why is this problematic for real hardware?

**Real FSR Challenges:**

1. **Noise and Vibrations:**

    ```cpp
    // Real FSR reading can vary like this:
    // Time: 0ms   10ms  20ms  30ms  40ms
    // Value:  0.1   0.3   15.2  14.8  15.1  ← Real contact with noise
    ```

2. **Baseline Drift:**

    ```cpp
    // FSR can change its "zero" value with time/temperature
    // Start of day: baseline = 0.1
    // After 2 hours: baseline = 0.3  ← Thermal drift
    ```

3. **Hysteresis:**
    ```cpp
    // Non-linear behavior
    // Applying force: 0 → 5N = reading 100
    // Removing force: 5N → 0 = reading 80  ← Different path
    ```

### Problem 5: Non-Linear Calibration and Scaling

#### What DummyFSR doesn't simulate:

```cpp
// Real FSR has inverse relationship: Resistance = K / Force
// Voltage = Vcc * R_pulldown / (R_FSR + R_pulldown)
// ADC→Force conversion requires complex calibration
```

#### Implementation needed for real hardware:

```cpp
class ProductionFSR {
private:
    float calibration_curve[10][2];  // Calibration points [ADC, Newton]
    float baseline_pressure;
    float noise_threshold;

public:
    float convertADCToForce(int adc_reading) {
        // Spline interpolation of calibration curve
        return interpolateCalibrationCurve(adc_reading);
    }

    bool detectContact(float force) {
        // Adaptive threshold based on history
        return force > (baseline_pressure + 3 * noise_threshold);
    }
};
```

## 📊 Comparison: Simulation vs Real Hardware

| Aspect          | DummyIMU/FSR    | Real Hardware        | Impact of Difference      |
| --------------- | --------------- | -------------------- | ------------------------- |
| **Noise**       | Perfect data    | Noise, drift, jitter | Needs robust filtering    |
| **Timing**      | Instantaneous   | Variable latency     | Needs buffers and timeout |
| **Calibration** | Pre-calibrated  | Requires calibration | Startup procedures        |
| **Failures**    | Never fails     | Can fail             | Detection and recovery    |
| **Temperature** | Stable          | Thermal drift        | Automatic compensation    |
| **Power**       | No restrictions | Energy consumption   | Energy management         |

## 🚨 Warning Signs for Real Hardware

### IMU - Problem Signals:

1. **Auto-pose worsens stability** → Wrong acceleration convention
2. **Slow response to tilt** → Filters too conservative
3. **Erratic readings during movement** → Vibration/noise problems
4. **Gradual drift during operation** → Incorrect bias calibration

### FSR - Problem Signals:

1. **False contacts during walking** → Threshold too sensitive
2. **Undetected contacts** → Threshold too high or damaged sensor
3. **Inconsistent readings per leg** → Assembly/calibration problems
4. **Gradual degradation** → Sensor aging

## 🔧 Implementation Recommendations

### For IMU:

```cpp
// 1. Automatic convention determination
void autoDetectIMUConvention() {
    // Test both conventions and see which gives logical results
}

// 2. Adaptive filters
void adaptiveFiltering(bool robot_moving, float motion_intensity) {
    if (robot_moving) {
        filter_alpha_ = 0.2f;  // Fast response
    } else {
        filter_alpha_ = 0.05f; // Maximum smoothing
    }
}

// 3. Continuous validation
void validateIMUHealth() {
    if (noise_level > MAX_NOISE || bias_drift > MAX_DRIFT) {
        triggerRecalibration();
    }
}
```

### For FSR:

```cpp
// 1. Automatic calibration
void autoCalibrateFSR() {
    // No-load period to establish baseline
    // Apply known weights for calibration curve
}

// 2. Robust detection
bool robustContactDetection(float current_reading) {
    updateAdaptiveThreshold();
    return (current_reading > adaptive_threshold) &&
           (contact_stable_for_ms > MIN_CONTACT_TIME);
}

// 3. Health monitoring
void monitorFSRHealth() {
    if (baseline_drift > MAX_DRIFT || noise_increase > MAX_NOISE) {
        scheduleRecalibration();
    }
}
```

## 📈 Real Hardware Validation Plan

### Phase 1: Individual Sensor Validation

```cpp
void validateSensorIndividually() {
    // IMU: known orientations, dynamic response
    // FSR: known weights, temporal response
}
```

### Phase 2: Integration Validation

```cpp
void validateSensorIntegration() {
    // Coordination between sensors
    // Detection of data conflicts
}
```

### Phase 3: Complete System Validation

```cpp
void validateCompleteSystem() {
    // Operation under real conditions
    // Failure handling and recovery
}
```

## 📋 Pre-Implementation Checklist

### IMU:

-   [ ] Identify specific model acceleration convention
-   [ ] Establish bias calibration procedure
-   [ ] Implement adaptive filters
-   [ ] Add sensor health monitoring
-   [ ] Test vibration resistance
-   [ ] Validate thermal stability

### FSR:

-   [ ] Characterize specific force-resistance curve
-   [ ] Establish baseline calibration procedure
-   [ ] Implement robust contact detection
-   [ ] Add drift compensation
-   [ ] Test consistency between sensors
-   [ ] Validate response to dynamic loads

---

**Conclusion:** The problems found in DummyIMU and DummyFSR are not implementation errors, but revelations of the real complexity we will face with physical hardware. This experience allows us to prepare robust solutions before real implementation.

**Document Generated:** June 11, 2025
**Next Review:** After implementation with real hardware
**Status:** Ready to guide production implementation
