# IMUs with Absolute Positioning (BNO055) in HexaMotion

## Introduction

HexaMotion includes full support for advanced IMUs such as the **BNO055**, which provide absolute orientation data and built-in sensor fusion. You can choose between sensor-side fusion or library-side algorithms to get the best accuracy for your needs. Absolute positioning data is not limited to the auto-pose system — it is consumed across multiple HexaMotion subsystems.

## Implemented Features

### 1. IMU Operation Modes

Three operation modes are supported:

- **`IMU_MODE_RAW_DATA`**: Raw sensor data processed with library algorithms
- **`IMU_MODE_FUSION`**: The IMU's internal sensor fusion
- **`IMU_MODE_ABSOLUTE_POS`**: Sensor absolute positioning (for example, BNO055)

### 2. Data Structures

#### `IMUAbsoluteData`

```cpp
struct IMUAbsoluteData {
    float absolute_roll, absolute_pitch, absolute_yaw;            // Absolute orientation (deg)
    float linear_accel_x, linear_accel_y, linear_accel_z;        // Linear acceleration (gravity removed)
    float quaternion_w, quaternion_x, quaternion_y, quaternion_z; // Orientation quaternion
    bool absolute_orientation_valid;  // Absolute orientation validity
    bool linear_acceleration_valid;   // Linear acceleration validity
    bool quaternion_valid;            // Quaternion data validity
    uint8_t calibration_status;       // Calibration status (0-3, 3 = fully calibrated)
    uint8_t system_status;            // Sensor system status
    uint8_t self_test_result;         // Self-test result
};
```

#### `IMUData`

```cpp
struct IMUData {
    // Basic IMU data (always available)
    float roll, pitch, yaw;           // Euler angles in degrees
    float accel_x, accel_y, accel_z;  // Raw acceleration in m/s²
    float gyro_x, gyro_y, gyro_z;     // Angular velocity in rad/s
    bool is_valid;                    // Basic data validity

    // Extended data for advanced IMUs
    IMUAbsoluteData absolute_data;    // Absolute-positioning data (when available)
    IMUMode mode;                     // Current operation mode
    bool has_absolute_capability;     // Whether the IMU supports absolute positioning
};
```

### 3. Extended `IIMUInterface`

```cpp
class IIMUInterface {
public:
    virtual bool initialize() = 0;
    virtual IMUData readIMU() = 0;
    virtual bool calibrate() = 0;
    virtual bool isConnected() = 0;

    // Advanced support
    virtual bool setIMUMode(IMUMode mode) = 0;
    virtual IMUMode getIMUMode() const = 0;
    virtual bool hasAbsolutePositioning() const = 0;
    virtual bool getCalibrationStatus(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag) = 0;
    virtual bool runSelfTest() = 0;
    virtual bool resetOrientation() = 0;
};
```

### 4. Auto-Pose Configuration

```cpp
struct IMUPoseParams {
    // ... existing parameters
    bool use_absolute_data;    // Use absolute positioning data when available
    bool prefer_sensor_fusion; // Prefer sensor fusion over library algorithms
};

// Control methods
bool configureIMUMode(bool use_absolute_data, bool prefer_fusion);
uint8_t getIMUCalibrationStatus() const;
bool isUsingAbsoluteData() const;
IMUMode getIMUMode() const;
```

## Basic Usage

### 1. BNO055 Interface Implementation

```cpp
class BNO055IMU : public IIMUInterface {
private:
    IMUMode current_mode_;

public:
    bool initialize() override {
        // Initialize BNO055 hardware
        setIMUMode(IMU_MODE_ABSOLUTE_POS); // Use absolute mode by default
        return true;
    }

    IMUData readIMU() override {
        IMUData data{};
        data.roll = /* read from BNO055 */;
        data.pitch = /* read from BNO055 */;
        data.yaw = /* read from BNO055 */;
        data.mode = current_mode_;
        data.has_absolute_capability = true;

        data.absolute_data.absolute_roll = /* absolute orientation */;
        data.absolute_data.absolute_pitch = /* absolute orientation */;
        data.absolute_data.absolute_yaw = /* absolute orientation */;
        data.absolute_data.linear_accel_x = /* gravity-free acceleration */;
        data.absolute_data.linear_accel_y = /* gravity-free acceleration */;
        data.absolute_data.linear_accel_z = /* gravity-free acceleration */;
        data.absolute_data.absolute_orientation_valid = /* check calibration */;
        data.absolute_data.linear_acceleration_valid = /* check calibration */;
        data.absolute_data.calibration_status = /* read calibration status */;
        return data;
    }

    bool setIMUMode(IMUMode mode) override { current_mode_ = mode; /* configure */ return true; }
    bool hasAbsolutePositioning() const override { return true; }
    // ... implement remaining required methods
};
```

### 2. Locomotion System Configuration

```cpp
void setup() {
    BNO055IMU bno055_imu;
    locomotion_system.initialize(&bno055_imu, &fsr_sensors, &servo_controller);

    IMUAutoPose* auto_pose = locomotion_system.getIMUAutoPose();
    if (auto_pose) {
        auto_pose->configureIMUMode(true, true); // absolute data + sensor fusion

        IMUAutoPose::IMUPoseParams params;
        params.use_absolute_data = true;
        params.prefer_sensor_fusion = true;
        params.orientation_gain = 0.7f;
        params.response_speed = 0.2f;
        params.deadzone_degrees = 1.0f;
        auto_pose->setIMUPoseParams(params);

        auto_pose->setAutoPoseMode(IMUAutoPose::AUTO_POSE_LEVEL);
        auto_pose->setEnabled(true);
    }
}
```

### 3. Status Monitoring

```cpp
IMUAutoPose* auto_pose = locomotion_system.getIMUAutoPose();
if (auto_pose) {
    bool absolute = auto_pose->isUsingAbsoluteData();
    uint8_t calibration = auto_pose->getIMUCalibrationStatus(); // warn if < 3
    IMUMode mode = auto_pose->getIMUMode();
}
```

## Subsystems That Consume Absolute Data

| Subsystem            | File                       | Use of BNO055 data                                                                |
| -------------------- | -------------------------- | --------------------------------------------------------------------------------- |
| Auto-posing          | `imu_auto_pose.h/cpp`      | Absolute orientation + linear acceleration for body posture; no long-term drift   |
| Terrain adaptation   | `terrain_adaptation.h/cpp` | Absolute orientation for walk-plane estimation; linear acceleration for roughness |
| Gait pattern select  | `locomotion_system.cpp`    | Tilt magnitude from absolute orientation drives adaptive pattern selection        |
| Stability assessment | `locomotion_system.cpp`    | Linear-acceleration variance + calibration confidence feed the stability index    |

When `has_absolute_capability` is false, each subsystem falls back to the basic roll/pitch/accel fields.

## Recommended Configurations

```cpp
// Maximum accuracy
params.use_absolute_data = true; params.prefer_sensor_fusion = true;
params.orientation_gain = 0.8f; params.response_speed = 0.15f; params.deadzone_degrees = 0.5f;

// Maximum stability
params.use_absolute_data = true; params.prefer_sensor_fusion = true;
params.orientation_gain = 0.5f; params.response_speed = 0.1f; params.deadzone_degrees = 2.0f;

// Compatibility mode (basic IMUs)
params.use_absolute_data = false; params.prefer_sensor_fusion = false;
```

## Advantages of Absolute Positioning

- **Higher accuracy** — hardware-optimized 9-DOF fusion (accelerometer, gyroscope, magnetometer) with drift compensation.
- **Lower computational load** — fusion executed on the sensor chip frees the main processor.
- **Linear acceleration** — gravity already removed, useful for motion/vibration detection and dynamic control.
- **Absolute orientation** — Earth magnetic-field reference avoids long-term yaw drift.

## Implementation Considerations

- **Calibration:** the BNO055 calibrates 4 components (system, gyroscope, accelerometer, magnetometer). Complete calibration before relying on absolute data and monitor `calibration_status` continuously.
- **Data validity:** always check the validity flags; provide a fallback to raw data when absolute data is invalid.
- **Hardware:** mount the IMU correctly, avoid nearby magnetic interference, and configure axis orientation.

## Compatibility

This implementation is fully backward compatible. IMUs without absolute positioning continue to work using the library algorithms with raw data.
