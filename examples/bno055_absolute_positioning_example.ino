/**
 * @file bno055_absolute_positioning_example.ino
 * @brief Example demonstrating BNO055-style absolute positioning IMU support
 *
 * This example shows how to use HexaMotion with IMUs that provide absolute positioning
 * data, such as the BNO055. It demonstrates:
 * - Configuration of IMU operation modes
 * - Using sensor's built-in fusion algorithms vs. library algorithms
 * - Monitoring calibration status
 * - Switching between absolute and raw data modes
 */

#include <Arduino.h>
#include <imu_auto_pose.h>
#include <locomotion_system.h>

/**
 * @brief Example BNO055-style IMU implementation
 *
 * This class simulates a BNO055 IMU that can provide:
 * - Raw sensor data (accelerometer, gyroscope, magnetometer)
 * - Sensor fusion data (using internal algorithms)
 * - Absolute position data (gravity-compensated linear acceleration)
 * - Calibration status monitoring
 */
class BNO055IMU : public IIMUInterface {
  private:
    IMUMode current_mode_;
    uint8_t calibration_status_[4]; // system, gyro, accel, mag
    bool is_initialized_;
    double simulated_roll_, simulated_pitch_, simulated_yaw_;

  public:
    BNO055IMU() : current_mode_(IMU_MODE_RAW_DATA), is_initialized_(false),
                  simulated_roll_(0), simulated_pitch_(0), simulated_yaw_(0) {
        // Initialize calibration status
        calibration_status_[0] = 0; // system
        calibration_status_[1] = 0; // gyro
        calibration_status_[2] = 0; // accel
        calibration_status_[3] = 0; // mag
    }

    bool initialize() override {
        Serial.println("BNO055: Initializing...");

        // Simulate initialization sequence
        delay(100);

        // Set initial mode
        setIMUMode(IMU_MODE_ABSOLUTE_POS);

        is_initialized_ = true;
        Serial.println("BNO055: Initialization complete");
        return true;
    }

    bool isConnected() override {
        return is_initialized_;
    }

    bool calibrate() override {
        Serial.println("BNO055: Starting calibration...");

        // Simulate calibration process
        for (int i = 0; i < 4; i++) {
            calibration_status_[i] = 3; // Fully calibrated
        }

        Serial.println("BNO055: Calibration complete");
        return true;
    }

    IMUData readIMU() override {
        IMUData data{};

        // Simulate some movement
        simulated_roll_ += 0.1f * sin(millis() * 0.001f);
        simulated_pitch_ += 0.05f * cos(millis() * 0.002f);
        simulated_yaw_ += 0.02f;

        // Basic IMU data (always available)
        data.roll = simulated_roll_;
        data.pitch = simulated_pitch_;
        data.yaw = simulated_yaw_;
        data.accel_x = 0.1f * sin(millis() * 0.003f);
        data.accel_y = 0.1f * cos(millis() * 0.003f);
        data.accel_z = 9.81f + 0.05f * sin(millis() * 0.005f);
        data.gyro_x = 0.01f * sin(millis() * 0.004f);
        data.gyro_y = 0.01f * cos(millis() * 0.004f);
        data.gyro_z = 0.005f * sin(millis() * 0.002f);
        data.is_valid = true;
        data.mode = current_mode_;
        data.has_absolute_capability = true;

        // Absolute positioning data (BNO055 specialty)
        data.absolute_data.absolute_roll = simulated_roll_;
        data.absolute_data.absolute_pitch = simulated_pitch_;
        data.absolute_data.absolute_yaw = simulated_yaw_;

        // Linear acceleration (gravity compensated)
        data.absolute_data.linear_accel_x = data.accel_x; // Gravity already removed
        data.absolute_data.linear_accel_y = data.accel_y;
        data.absolute_data.linear_accel_z = data.accel_z - 9.81f;

        // Orientation quaternion
        double roll_rad = data.absolute_data.absolute_roll * PI / 180.0f;
        double pitch_rad = data.absolute_data.absolute_pitch * PI / 180.0f;
        double yaw_rad = data.absolute_data.absolute_yaw * PI / 180.0f;

        double cr = cos(roll_rad * 0.5f);
        double sr = sin(roll_rad * 0.5f);
        double cp = cos(pitch_rad * 0.5f);
        double sp = sin(pitch_rad * 0.5f);
        double cy = cos(yaw_rad * 0.5f);
        double sy = sin(yaw_rad * 0.5f);

        data.absolute_data.quaternion_w = cr * cp * cy + sr * sp * sy;
        data.absolute_data.quaternion_x = sr * cp * cy - cr * sp * sy;
        data.absolute_data.quaternion_y = cr * sp * cy + sr * cp * sy;
        data.absolute_data.quaternion_z = cr * cp * sy - sr * sp * cy;

        // Data validity flags
        data.absolute_data.absolute_orientation_valid = (calibration_status_[0] >= 2);
        data.absolute_data.linear_acceleration_valid = (calibration_status_[2] >= 2);
        data.absolute_data.quaternion_valid = (calibration_status_[0] >= 3);

        // Status information
        data.absolute_data.calibration_status = calibration_status_[0];
        data.absolute_data.system_status = is_initialized_ ? 5 : 0;
        data.absolute_data.self_test_result = 0x0F; // All tests pass

        return data;
    }

    bool setIMUMode(IMUMode mode) override {
        current_mode_ = mode;

        switch (mode) {
        case IMU_MODE_RAW_DATA:
            Serial.println("BNO055: Switched to RAW DATA mode");
            break;
        case IMU_MODE_FUSION:
            Serial.println("BNO055: Switched to SENSOR FUSION mode");
            break;
        case IMU_MODE_ABSOLUTE_POS:
            Serial.println("BNO055: Switched to ABSOLUTE POSITIONING mode");
            break;
        }

        return true;
    }

    IMUMode getIMUMode() const override {
        return current_mode_;
    }

    bool hasAbsolutePositioning() const override {
        return true; // BNO055 supports absolute positioning
    }

    bool getCalibrationStatus(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag) override {
        if (system)
            *system = calibration_status_[0];
        if (gyro)
            *gyro = calibration_status_[1];
        if (accel)
            *accel = calibration_status_[2];
        if (mag)
            *mag = calibration_status_[3];
        return true;
    }

    bool runSelfTest() override {
        Serial.println("BNO055: Running self-test...");
        // Simulate self-test
        delay(50);
        Serial.println("BNO055: Self-test passed");
        return true;
    }

    bool resetOrientation() override {
        Serial.println("BNO055: Resetting orientation...");
        simulated_roll_ = 0;
        simulated_pitch_ = 0;
        simulated_yaw_ = 0;
        return true;
    }

    bool update() override {
        // BNO055 parallel sensor update implementation
        // This would trigger non-blocking sensor data acquisition
        // and update internal registers for synchronized operation
        // Simulated for demonstration purposes
        return true;
    }
};

// Example FSR implementation (simplified)
class ExampleFSR : public IFSRInterface {
  public:
    bool initialize() override { return true; }
    FSRData readFSR(int /*leg_index*/) override {
        FSRData data{};
        data.pressure = 0.0f;
        data.in_contact = false;
        data.contact_time = 0.0f;
        return data;
    }
    bool calibrateFSR(int /*leg_index*/) override { return true; }
    double getRawReading(int /*leg_index*/) override { return 0.0f; }
    bool update() override { return true; }
};

// Example servo implementation (simplified)
class ExampleServo : public IServoInterface {
  public:
    bool initialize() override { return true; }

    bool hasBlockingStatusFlags(int /*leg_index*/, int /*joint_index*/, uint8_t *active_flags = nullptr) override {
        if (active_flags)
            *active_flags = 0;
        return false;
    }

    bool setJointAngleAndSpeed(int /*leg_index*/, int /*joint_index*/, double /*angle*/, double /*speed*/) override { return true; }
    double getJointAngle(int /*leg_index*/, int /*joint_index*/) override { return 0.0f; }
    bool isJointMoving(int /*leg_index*/, int /*joint_index*/) override { return false; }
    bool enableTorque(int /*leg_index*/, int /*joint_index*/, bool /*enable*/) override { return true; }
};

// Global objects
BNO055IMU bno055_imu;
ExampleFSR fsr_sensors;
ExampleServo servo_controller;
LocomotionSystem locomotion_system;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("=== BNO055 Absolute Positioning Example ===");

    // Initialize hardware interfaces
    Serial.println("Initializing hardware interfaces...");

    if (!bno055_imu.initialize()) {
        Serial.println("ERROR: Failed to initialize BNO055 IMU");
        return;
    }

    if (!fsr_sensors.initialize()) {
        Serial.println("ERROR: Failed to initialize FSR sensors");
        return;
    }

    if (!servo_controller.initialize()) {
        Serial.println("ERROR: Failed to initialize servo controller");
        return;
    }

    // Initialize locomotion system
    Serial.println("Initializing locomotion system...");
    if (!locomotion_system.initialize(&bno055_imu, &fsr_sensors, &servo_controller)) {
        Serial.println("ERROR: Failed to initialize locomotion system");
        return;
    }

    // Configure IMU auto-pose system
    IMUAutoPose *auto_pose = locomotion_system.getIMUAutoPose();
    if (auto_pose) {
        Serial.println("Configuring IMU auto-pose system...");

        // Configure to use absolute data with sensor fusion
        auto_pose->configureIMUMode(true, true);

        // Set auto-pose parameters optimized for absolute positioning
        IMUAutoPose::IMUPoseParams params;
        params.use_absolute_data = true;
        params.prefer_sensor_fusion = true;
        params.orientation_gain = 0.7f; // Higher gain for precise absolute data
        params.response_speed = 0.2f;   // Faster response
        params.deadzone_degrees = 1.0f; // Smaller deadzone for precision
        auto_pose->setIMUPoseParams(params);

        // Enable level mode
        auto_pose->setAutoPoseMode(IMUAutoPose::AUTO_POSE_LEVEL);
        auto_pose->setEnabled(true);

        Serial.println("IMU auto-pose configured for absolute positioning");
    }

    Serial.println("Setup complete!");
    Serial.println();

    // Run calibration check
    runCalibrationCheck();
}

void loop() {
    static unsigned long last_status_print = 0;
    static unsigned long last_mode_switch = 0;
    static int mode_index = 0;

    // Update locomotion system
    locomotion_system.update();

    // Print status every 2 seconds
    if (millis() - last_status_print > 2000) {
        printIMUStatus();
        last_status_print = millis();
    }

    // Demo: Switch between IMU modes every 10 seconds
    if (millis() - last_mode_switch > 10000) {
        switchIMUMode(mode_index);
        mode_index = (mode_index + 1) % 3;
        last_mode_switch = millis();
    }

    delay(50); // Main loop rate: 20Hz
}

void runCalibrationCheck() {
    Serial.println("=== Calibration Status Check ===");

    uint8_t system, gyro, accel, mag;
    if (bno055_imu.getCalibrationStatus(&system, &gyro, &accel, &mag)) {
        Serial.print("System: ");
        Serial.print(system);
        Serial.print(", Gyro: ");
        Serial.print(gyro);
        Serial.print(", Accel: ");
        Serial.print(accel);
        Serial.print(", Mag: ");
        Serial.println(mag);

        if (system < 3) {
            Serial.println("WARNING: System not fully calibrated");
            Serial.println("Move the robot in various orientations to calibrate");
        } else {
            Serial.println("System fully calibrated!");
        }
    }

    // Run self-test
    if (bno055_imu.runSelfTest()) {
        Serial.println("Self-test: PASSED");
    } else {
        Serial.println("Self-test: FAILED");
    }

    Serial.println();
}

void printIMUStatus() {
    IMUAutoPose *auto_pose = locomotion_system.getIMUAutoPose();
    if (!auto_pose)
        return;

    Serial.println("=== IMU Status ===");

    // Print current mode
    IMUMode mode = auto_pose->getIMUMode();
    Serial.print("Mode: ");
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

    // Print data source
    Serial.print("Using absolute data: ");
    Serial.println(auto_pose->isUsingAbsoluteData() ? "YES" : "NO");

    // Print calibration status
    Serial.print("Calibration: ");
    Serial.print(auto_pose->getIMUCalibrationStatus());
    Serial.println("/3");

    // Print orientation
    Point3D inclination = auto_pose->getInclinationAngles();
    Serial.print("Orientation (R/P/Y): ");
    Serial.print(inclination.x * 180.0f / PI, 1);
    Serial.print("°, ");
    Serial.print(inclination.y * 180.0f / PI, 1);
    Serial.print("°, ");
    Serial.print(inclination.z * 180.0f / PI, 1);
    Serial.println("°");

    // Print auto-pose state
    const IMUAutoPose::AutoPoseState &state = auto_pose->getAutoPoseState();
    Serial.print("Auto-pose active: ");
    Serial.print(state.pose_active ? "YES" : "NO");
    Serial.print(", Confidence: ");
    Serial.print(state.confidence, 2);
    Serial.println();

    Serial.println();
}

void switchIMUMode(int mode_index) {
    Serial.println("=== Switching IMU Mode ===");

    IMUMode new_mode;
    bool use_absolute;
    bool prefer_fusion;

    switch (mode_index) {
    case 0:
        new_mode = IMU_MODE_RAW_DATA;
        use_absolute = false;
        prefer_fusion = false;
        Serial.println("Switching to: RAW DATA mode (library algorithms)");
        break;
    case 1:
        new_mode = IMU_MODE_FUSION;
        use_absolute = false;
        prefer_fusion = true;
        Serial.println("Switching to: SENSOR FUSION mode (sensor algorithms)");
        break;
    case 2:
    default:
        new_mode = IMU_MODE_ABSOLUTE_POS;
        use_absolute = true;
        prefer_fusion = true;
        Serial.println("Switching to: ABSOLUTE POSITIONING mode (sensor algorithms)");
        break;
    }

    // Apply mode change
    bno055_imu.setIMUMode(new_mode);

    IMUAutoPose *auto_pose = locomotion_system.getIMUAutoPose();
    if (auto_pose) {
        auto_pose->configureIMUMode(use_absolute, prefer_fusion);
    }

    Serial.println();
}
