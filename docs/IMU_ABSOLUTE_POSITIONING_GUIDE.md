# Support for IMUs with Absolute Positioning in HexaMotion

## Introduction

HexaMotion now includes full support for advanced IMUs such as the **BNO055**, which provide absolute orientation data and built-in sensor fusion algorithms. This feature allows you to choose between using sensor-side algorithms or library-side algorithms to get the best accuracy for your specific needs.

## Implemented Features

### 1. IMU Operation Modes

The system now supports three operation modes:

- **`IMU_MODE_RAW_DATA`**: Uses raw sensor data with library algorithms
- **`IMU_MODE_FUSION`**: Uses the IMU's internal sensor fusion
- **`IMU_MODE_ABSOLUTE_POS`**: Uses sensor absolute positioning (for example, BNO055)

### 2. Extended Data Structure

#### `IMUAbsoluteData`

```cpp
struct IMUAbsoluteData {
    float absolute_roll, absolute_pitch, absolute_yaw;  // Absolute orientation in degrees
    float linear_accel_x, linear_accel_y, linear_accel_z;  // Linear acceleration (gravity removed)
    float quaternion_w, quaternion_x, quaternion_y, quaternion_z;  // Orientation quaternion
    bool absolute_orientation_valid;  // Absolute orientation validity
    bool linear_acceleration_valid;   // Linear acceleration validity
    bool quaternion_valid;            // Quaternion data validity
    uint8_t calibration_status;       // Calibration status (0-3, 3=fully calibrated)
    uint8_t system_status;            // Sensor system status
    uint8_t self_test_result;         // Self-test result
};
```

#### Updated `IMUData`

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

The interface now includes additional methods:

```cpp
class IIMUInterface {
public:
    // Existing methods
    virtual bool initialize() = 0;
    virtual IMUData readIMU() = 0;
    virtual bool calibrate() = 0;
    virtual bool isConnected() = 0;

    // New methods for advanced support
    virtual bool setIMUMode(IMUMode mode) = 0;
    virtual IMUMode getIMUMode() const = 0;
    virtual bool hasAbsolutePositioning() const = 0;
    virtual bool getCalibrationStatus(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag) = 0;
    virtual bool runSelfTest() = 0;
    virtual bool resetOrientation() = 0;
};
```

### 4. Improved Auto-Pose Configuration

#### New Parameters

```cpp
struct IMUPoseParams {
    // Existing parameters...
    bool use_absolute_data;    // Use absolute positioning data when available
    bool prefer_sensor_fusion; // Prefer sensor fusion over library algorithms
};
```

#### New Control Methods

```cpp
// Configure IMU operation mode
bool configureIMUMode(bool use_absolute_data, bool prefer_fusion);

// Get calibration status
uint8_t getIMUCalibrationStatus() const;

// Check whether absolute data is currently being used
bool isUsingAbsoluteData() const;

// Get current IMU mode
IMUMode getIMUMode() const;
```

## Basic Usage

### 1. BNO055 IMU Implementation

```cpp
class BNO055IMU : public IIMUInterface {
private:
    IMUMode current_mode_;
    // ... other private members

public:
    bool initialize() override {
        // Initialize BNO055 hardware
        // ...
        setIMUMode(IMU_MODE_ABSOLUTE_POS); // Use absolute mode by default
        return true;
    }

    IMUData readIMU() override {
        IMUData data{};

        // Fill basic data
        data.roll = /* read from BNO055 */;
        data.pitch = /* read from BNO055 */;
        data.yaw = /* read from BNO055 */;
        // ... other basic data

        data.mode = current_mode_;
        data.has_absolute_capability = true;

        // Fill BNO055-specific absolute data
        data.absolute_data.absolute_roll = /* absolute orientation */;
        data.absolute_data.absolute_pitch = /* absolute orientation */;
        data.absolute_data.absolute_yaw = /* absolute orientation */;
        data.absolute_data.linear_accel_x = /* gravity-free acceleration */;
        data.absolute_data.linear_accel_y = /* gravity-free acceleration */;
        data.absolute_data.linear_accel_z = /* gravity-free acceleration */;

        // Validity states
        data.absolute_data.absolute_orientation_valid = /* check calibration */;
        data.absolute_data.linear_acceleration_valid = /* check calibration */;
        data.absolute_data.calibration_status = /* read calibration status */;

        return data;
    }

    bool setIMUMode(IMUMode mode) override {
        current_mode_ = mode;
        // Configure BNO055 according to mode
        switch (mode) {
            case IMU_MODE_RAW_DATA:
                // Configure for raw data
                break;
            case IMU_MODE_FUSION:
                // Configure for sensor fusion
                break;
            case IMU_MODE_ABSOLUTE_POS:
                // Configure for absolute positioning
                break;
        }
        return true;
    }

    bool hasAbsolutePositioning() const override {
        return true; // BNO055 supports absolute positioning
    }

    // ... implement other required methods
};
```

### 2. Locomotion System Configuration

```cpp
void setup() {
    BNO055IMU bno055_imu;
    // ... other components

    // Initialize locomotion system
    locomotion_system.initialize(&bno055_imu, &fsr_sensors, &servo_controller);

    // Configure auto-pose to use absolute data
    IMUAutoPose* auto_pose = locomotion_system.getIMUAutoPose();
    if (auto_pose) {
        // Use absolute data with sensor fusion
        auto_pose->configureIMUMode(true, true);

        // Configure parameters optimized for absolute data
        IMUAutoPose::IMUPoseParams params;
        params.use_absolute_data = true;
        params.prefer_sensor_fusion = true;
        params.orientation_gain = 0.7f; // Higher gain for accurate data
        params.response_speed = 0.2f;   // Faster response
        params.deadzone_degrees = 1.0f; // Smaller deadzone
        auto_pose->setIMUPoseParams(params);

        // Enable auto-pose
        auto_pose->setAutoPoseMode(IMUAutoPose::AUTO_POSE_LEVEL);
        auto_pose->setEnabled(true);
    }
}
```

### 3. Status Monitoring

```cpp
void loop() {
    // Update system
    locomotion_system.update();

    // Check IMU status
    IMUAutoPose* auto_pose = locomotion_system.getIMUAutoPose();
    if (auto_pose) {
        // Check whether absolute data is being used
        if (auto_pose->isUsingAbsoluteData()) {
            Serial.println("Using IMU absolute positioning");
        } else {
            Serial.println("Using library algorithms");
        }

        // Check calibration
        uint8_t calibration = auto_pose->getIMUCalibrationStatus();
        if (calibration < 3) {
            Serial.print("Warning: Incomplete calibration (");
            Serial.print(calibration);
            Serial.println("/3)");
        }

        // Get current mode
        IMUMode mode = auto_pose->getIMUMode();
        Serial.print("IMU mode: ");
        switch (mode) {
            case IMU_MODE_RAW_DATA:
                Serial.println("RAW DATA");
                break;
            case IMU_MODE_FUSION:
                Serial.println("SENSOR FUSION");
                break;
            case IMU_MODE_ABSOLUTE_POS:
                Serial.println("ABSOLUTE POSITIONING");
                break;
        }
    }
}
```

## Advantages of Absolute Positioning

### 1. Higher Accuracy

- BNO055 algorithms are optimized for the hardware
- Automatic calibration and drift compensation
- 9-DOF fusion (accelerometer, gyroscope, magnetometer)

### 2. Lower Computational Load

- Complex calculations are executed on the sensor chip
- Frees main processor resources for other tasks

### 3. Linear Acceleration Data

- Acceleration with gravity already removed
- Useful for motion detection and dynamic control

### 4. Absolute Orientation

- Earth magnetic field references
- Long-term orientation retention without drift

## Recommended Configurations

### For Maximum Accuracy

```cpp
params.use_absolute_data = true;
params.prefer_sensor_fusion = true;
params.orientation_gain = 0.8f;
params.response_speed = 0.15f;
params.deadzone_degrees = 0.5f;
```

### For Maximum Stability

```cpp
params.use_absolute_data = true;
params.prefer_sensor_fusion = true;
params.orientation_gain = 0.5f;
params.response_speed = 0.1f;
params.deadzone_degrees = 2.0f;
```

### For Uneven Terrain

```cpp
params.use_absolute_data = true;
params.prefer_sensor_fusion = true;
params.adaptive_gains = true;
params.stabilization_gain = 1.5f;
```

### Compatibility Mode (Basic IMUs)

```cpp
params.use_absolute_data = false;
params.prefer_sensor_fusion = false;
// Library algorithms will be used with raw data
```

## Implementation Considerations

### 1. Calibration

- BNO055 requires calibration of 4 components: system, gyroscope, accelerometer, and magnetometer
- Calibration should be completed before using absolute data
- Monitor `calibration_status` continuously

### 2. Data Validity

- Always verify validity flags before using absolute data
- Provide a fallback to raw data if absolute data is invalid

### 3. Hardware Configuration

- Ensure the IMU is physically mounted correctly
- Avoid magnetic interference near the sensor
- Configure axis orientation correctly

## Complete Example

See `examples/bno055_absolute_positioning_example.ino` for a full implementation demonstrating:

- Simulated BNO055 setup
- Dynamic mode switching
- Calibration monitoring
- Parameter optimization

## Compatibility

This implementation is fully backward compatible. IMUs without absolute positioning support continue to work normally using library algorithms.
