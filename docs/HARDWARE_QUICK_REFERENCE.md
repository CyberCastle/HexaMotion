# Quick Reference: Real Hardware Integration

**IMU and FSR - Critical Points for Developers**

## 🚨 Critical Problems Discovered in Simulation

### IMU - Acceleration Convention

```cpp
// ❌ PROBLEM: Wrong convention causes gravity inversion
// Result: Robot destabilizes instead of stabilizing

// ✅ SOLUTION: Detect convention during calibration
void detectIMUConvention() {
    IMUData data = imu.readIMU(); // Robot flat on surface

    if (data.accel_z > 8.0f) {
        convention = PHYSICS_CONVENTION;  // +9.81 = support force
        gravity = -accel;  // Inversion needed
    } else if (data.accel_z < -8.0f) {
        convention = DIRECT_CONVENTION;   // -9.81 = direct gravity
        gravity = accel;   // No inversion
    }
}
```

### IMU - Filter Convergence Time

```cpp
// ❌ PROBLEM: Slow filters (alpha=0.05) take ~45 updates
// At 50Hz = 0.9 seconds to respond to 15° tilt

// ✅ SOLUTION: Adaptive filters
float alpha = robot_moving ? 0.2f : 0.05f;  // Fast if moving
```

### FSR - Static vs Adaptive Thresholds

```cpp
// ❌ PROBLEM: Fixed threshold doesn't work with environmental variations
bool contact = (pressure > FIXED_THRESHOLD);

// ✅ SOLUTION: Adaptive threshold
bool contact = (pressure > baseline + 3*noise_level + min_force);
```

## 🔧 Real Hardware Reference Code

### IMU Calibration

```cpp
struct IMUCalibration {
    bool physics_convention;
    Point3D accel_bias;
    Point3D gyro_bias;

    bool calibrate() {
        // 1. Detect convention
        detectConvention();

        // 2. Measure bias (100 samples)
        for(int i=0; i<100; i++) {
            IMUData data = readIMU();
            accel_sum += Point3D(data.accel_x, data.accel_y, data.accel_z);
            gyro_sum += Point3D(data.gyro_x, data.gyro_y, data.gyro_z);
        }
        accel_bias = accel_sum * 0.01f;
        gyro_bias = gyro_sum * 0.01f;

        // 3. Validate
        return (abs(abs(accel_bias.z) - 9.81f) < 1.0f);
    }
};
```

### Robust FSR Contact Detection

```cpp
class SmartFSR {
    float baseline = 0.0f;
    float noise_level = 0.1f;
    bool prev_contact = false;
    unsigned long contact_start = 0;

public:
    bool detectContact(float raw_pressure) {
        // Update baseline when no contact
        if (!prev_contact && raw_pressure < baseline + 2*noise_level) {
            baseline = baseline*0.99f + raw_pressure*0.01f;
        }

        // Estimate noise
        noise_level = noise_level*0.95f + abs(raw_pressure - baseline)*0.05f;

        // Adaptive threshold
        float threshold = baseline + 3*noise_level + 0.5f; // +0.5N minimum

        bool raw_contact = (raw_pressure > threshold);

        // Temporal filter (debouncing)
        if (raw_contact && !prev_contact) {
            contact_start = millis();
        }

        bool stable_contact = raw_contact &&
                             (millis() - contact_start > 20); // 20ms stable

        prev_contact = stable_contact;
        return stable_contact;
    }
};
```

### Robust Timing for Real Systems

```cpp
void robustUpdate(float dt_override = 0.0f) {
    if (dt_override > 0.0f) {
        // Override for testing
        processUpdate(dt_override);
        return;
    }

    unsigned long now = millis();
    unsigned long elapsed = now - last_update;

    // Process if: enough time OR too much time (recovery)
    if (elapsed >= update_interval || elapsed > update_interval*3) {
        last_update = now;
        float dt = elapsed / 1000.0f;  // Convert to seconds
        processUpdate(dt);
    }
}
```

## 📋 Pre-Implementation Checklist

### IMU:

-   [ ] ✅ Determine acceleration convention (physics vs direct)
-   [ ] ✅ Implement automatic bias calibration
-   [ ] ✅ Add adaptive filters based on movement state
-   [ ] ✅ Validate thermal and temporal stability
-   [ ] ✅ Implement failure detection and recovery

### FSR:

-   [ ] ✅ Characterize force-resistance curve with known weights
-   [ ] ✅ Implement adaptive thresholds based on history
-   [ ] ✅ Add compensation for temporal/thermal drift
-   [ ] ✅ Implement temporal filtering and debouncing
-   [ ] ✅ Validate independence between sensors (no cross-talk)

### System:

-   [ ] ✅ Implement robust timing with failure recovery
-   [ ] ✅ Add sensor health monitoring
-   [ ] ✅ Create automatic recalibration procedures
-   [ ] ✅ Implement logging for post-failure diagnosis

## ⚠️ Warning Signs in Production

### IMU:

-   **Auto-pose worsens stability** → Wrong convention
-   **Very slow response** → Filters too conservative
-   **Erratic readings** → Noise/vibration problems
-   **Gradual drift** → Incorrect bias calibration

### FSR:

-   **False contacts** → Threshold too sensitive or excessive noise
-   **Missed contacts** → Threshold too high or damaged sensor
-   **Inconsistency between legs** → Assembly problems
-   **Gradual degradation** → Sensor aging

## 🔍 Diagnostic Tools

### IMU Health Monitor:

```cpp
void monitorIMUHealth() {
    float noise = calculateNoiseLevel();
    float bias_drift = calculateBiasDrift();

    if (noise > MAX_NOISE_THRESHOLD) {
        log("IMU: Excessive noise detected");
        triggerRecalibration();
    }

    if (bias_drift > MAX_DRIFT_THRESHOLD) {
        log("IMU: Bias drift detected");
        recalibrateBias();
    }
}
```

### FSR Health Monitor:

```cpp
void monitorFSRHealth(int leg) {
    float baseline_change = calculateBaselineChange(leg);
    float response_consistency = calculateResponseConsistency(leg);

    if (baseline_change > MAX_BASELINE_DRIFT) {
        log("FSR leg " + leg + ": Baseline drift");
        recalibrateFSR(leg);
    }

    if (response_consistency < MIN_CONSISTENCY) {
        log("FSR leg " + leg + ": Inconsistent response");
        flagForMaintenance(leg);
    }
}
```

---

**For more details see:**

-   `docs/HARDWARE_INTEGRATION_CONSIDERATIONS.md` - Complete guide
-   `docs/SIMULACION_VS_HARDWARE_REAL_ANALISIS.md` - Detailed problem analysis
-   `tests/test_stubs.h` - Comments in simulation code

**Date:** June 11, 2025
**Status:** Based on real problems found in simulation
