/**
 * @file state_machine_example.ino
 * @brief Example demonstrating integration of StateController with HexaMotion
 * @author HexaMotion Team
 * @version 1.0
 * @date 2024
 *
 * This example shows how to integrate the OpenSHC-equivalent state machine
 * with the HexaMotion locomotion system for comprehensive hexapod control.
 */

#include <Arduino.h>
#include <locomotion_system.h>
#include <state_controller.h>

// Hardware interfaces (use your actual implementations)
class MyIMU : public IIMUInterface {
  private:
    IMUMode current_mode_;
    bool supports_absolute_;

  public:
    MyIMU() : current_mode_(IMU_MODE_RAW_DATA), supports_absolute_(false) {}

    bool initialize() override { return true; }
    bool isConnected() override { return true; }
    bool calibrate() override { return true; }

    IMUData readIMU() override {
        IMUData data{};
        // Basic IMU data (always available)
        data.roll = 0.0f;
        data.pitch = 0.0f;
        data.yaw = 0.0f;
        data.accel_x = 0.0f;
        data.accel_y = 0.0f;
        data.accel_z = 9.81f;
        data.gyro_x = 0.0f;
        data.gyro_y = 0.0f;
        data.gyro_z = 0.0f;
        data.is_valid = true;
        data.mode = current_mode_;
        data.has_absolute_capability = supports_absolute_;

        // For BNO055-like sensors, populate absolute data
        if (supports_absolute_) {
            data.absolute_data.absolute_roll = 0.0f;
            data.absolute_data.absolute_pitch = 0.0f;
            data.absolute_data.absolute_yaw = 0.0f;
            data.absolute_data.linear_accel_x = 0.0f;
            data.absolute_data.linear_accel_y = 0.0f;
            data.absolute_data.linear_accel_z = 0.0f;
            data.absolute_data.quaternion_w = 1.0f;
            data.absolute_data.quaternion_x = 0.0f;
            data.absolute_data.quaternion_y = 0.0f;
            data.absolute_data.quaternion_z = 0.0f;
            data.absolute_data.absolute_orientation_valid = true;
            data.absolute_data.linear_acceleration_valid = true;
            data.absolute_data.quaternion_valid = true;
            data.absolute_data.calibration_status = 3; // Fully calibrated
            data.absolute_data.system_status = 5;
            data.absolute_data.self_test_result = 0x0F;
        }

        return data;
    }

    bool setIMUMode(IMUMode mode) override {
        current_mode_ = mode;
        return true;
    }

    IMUMode getIMUMode() const override {
        return current_mode_;
    }

    bool hasAbsolutePositioning() const override {
        return supports_absolute_;
    }

    bool getCalibrationStatus(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag) override {
        if (system)
            *system = 3;
        if (gyro)
            *gyro = 3;
        if (accel)
            *accel = 3;
        if (mag)
            *mag = 3;
        return true;
    }

    bool runSelfTest() override { return true; }
    bool resetOrientation() override { return true; }

    bool update() override {
        // Update IMU readings for parallel sensor operation
        // This method enables synchronized reading with FSR sensors
        return true;
    }

    // Helper to simulate BNO055 capabilities
    void enableAbsoluteMode(bool enable) { supports_absolute_ = enable; }
};

class MyFSR : public IFSRInterface {
  public:
    bool initialize() override { return true; }
    bool calibrateFSR(int leg_index) override { return true; }
    FSRData readFSR(int leg_index) override {
        FSRData data;
        // Read actual FSR data here
        data.pressure = 0.0f;
        data.in_contact = false;
        data.contact_time = 0.0f;
        return data;
    }
    double getRawReading(int leg_index) override { return 0.0f; }
    bool update() override {
        // Update FSR readings using AdvancedAnalog DMA
        // This method should trigger simultaneous ADC reads on all FSR channels
        // and update internal registers with the latest values
        return true;
    }
};

class MyServo : public IServoInterface {
  public:
    bool initialize() override { return true; }

    bool hasBlockingStatusFlags(int leg_index, int joint_index, uint8_t *active_flags = nullptr) override {
        // Mock implementation - no servos are blocked
        if (active_flags) {
            *active_flags = 0; // No flags active
        }
        return false; // No blocking flags
    }

    bool setJointAngleAndSpeed(int leg_index, int joint_index, double angle, double speed) override {
        // Send actual servo commands here with both angle and speed
        (void)leg_index;
        (void)joint_index;
        (void)angle;
        (void)speed;
        return true;
    }
    double getJointAngle(int leg_index, int joint_index) override {
        // Read actual servo positions here
        (void)leg_index;
        (void)joint_index;
        return 0.0f;
    }
    bool isJointMoving(int leg_index, int joint_index) override { return false; }
    bool enableTorque(int leg_index, int joint_index, bool enable) override { return true; }
};

// Global objects
Parameters params;
LocomotionSystem *locomotion_system;
StateController *state_controller;
MyIMU imu;
MyFSR fsr;
MyServo servos;

void setup() {
    Serial.begin(115200);
    Serial.println("HexaMotion StateController Example Starting...");

    // Configure robot parameters
    params.hexagon_radius = 150;
    params.coxa_length = 50;
    params.femur_length = 100;
    params.tibia_length = 120;
    params.robot_height = 80;
    params.robot_weight = 2.5f;
    params.control_frequency = 50;

    // Set joint limits (required!)
    params.coxa_angle_limits[0] = -90;
    params.coxa_angle_limits[1] = 90;
    params.femur_angle_limits[0] = -90;
    params.femur_angle_limits[1] = 90;
    params.tibia_angle_limits[0] = -90;
    params.tibia_angle_limits[1] = 90;

    // Create locomotion system
    locomotion_system = new LocomotionSystem(params);

    // Initialize hardware interfaces
    if (!locomotion_system->initialize(&imu, &fsr, &servos)) {
        Serial.println("Failed to initialize locomotion system!");
        while (1)
            ;
    }

    if (!locomotion_system->calibrateSystem()) {
        Serial.println("Failed to calibrate system!");
        while (1)
            ;
    }

    // Configure state machine
    StateMachineConfig state_config;
    state_config.enable_startup_sequence = true;
    state_config.enable_direct_startup = false;
    state_config.transition_timeout = 10.0f;
    state_config.enable_cruise_control = true;
    state_config.max_manual_legs = 2;

    // Create state controller
    state_controller = new StateController(*locomotion_system, state_config);

    // Initialize state controller
    if (!state_controller->initialize()) {
        Serial.println("Failed to initialize state controller!");
        while (1)
            ;
    }

    Serial.println("System initialized successfully!");
    Serial.println("Current robot state: " + String(state_controller->getRobotState()));
    Serial.println("Current system state: " + String(state_controller->getSystemState()));

    // Start the robot - request transition to READY state
    if (state_controller->requestRobotState(ROBOT_READY)) {
        Serial.println("Requested transition to READY state");
    }
}

void loop() {
    // Calculate delta time
    static unsigned long last_time = millis();
    unsigned long current_time = millis();
    double dt = (current_time - last_time) / 1000.0f;
    last_time = current_time;

    // Limit dt to avoid large jumps
    if (dt > 0.1f)
        dt = 0.1f;

    // Update state machine
    state_controller->update(dt);

    // Update locomotion system
    locomotion_system->update();

    // Example state machine interactions
    static unsigned long demo_timer = millis();
    static int demo_phase = 0;

    if (current_time - demo_timer > 5000) { // Every 5 seconds
        demo_timer = current_time;

        switch (demo_phase) {
        case 0:
            // Transition to running state
            if (state_controller->getRobotState() == ROBOT_READY) {
                Serial.println("Transitioning to RUNNING state...");
                state_controller->requestRobotState(ROBOT_RUNNING);
                demo_phase = 1;
            }
            break;

        case 1:
            // Start walking forward
            if (state_controller->isReadyForOperation()) {
                Serial.println("Setting forward velocity...");
                Eigen::Vector2d linear_vel(30.0f, 0.0f); // 30mm/s forward
                state_controller->setDesiredVelocity(linear_vel, 0.0f);
                demo_phase = 2;
            }
            break;

        case 2:
            // Enable cruise control
            Serial.println("Enabling cruise control...");
            Eigen::Vector3d cruise_vel(20.0f, 0.0f, 0.0f);
            state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, cruise_vel);
            demo_phase = 3;
            break;

        case 3:
            // Stop and return to ready
            Serial.println("Stopping and returning to READY...");
            state_controller->setCruiseControlMode(CRUISE_CONTROL_OFF);
            Eigen::Vector2d zero_vel(0.0f, 0.0f);
            state_controller->setDesiredVelocity(zero_vel, 0.0f);
            state_controller->requestRobotState(ROBOT_READY);
            demo_phase = 0;
            break;
        }
    }

    // Print status every 2 seconds
    static unsigned long status_timer = millis();
    if (current_time - status_timer > 2000) {
        status_timer = current_time;

        Serial.println("=== System Status ===");
        Serial.println("System State: " + String(state_controller->getSystemState()));
        Serial.println("Robot State: " + String(state_controller->getRobotState()));
        Serial.println("Walk State: " + String(state_controller->getWalkState()));
        Serial.println("Posing Mode: " + String(state_controller->getPosingMode()));
        Serial.println("Cruise Control: " + String(state_controller->getCruiseControlMode()));
        Serial.println("Initialized: " + String(state_controller->isInitialized()));
        Serial.println("Ready for Operation: " + String(state_controller->isReadyForOperation()));

        if (state_controller->isTransitioning()) {
            TransitionProgress progress = state_controller->getTransitionProgress();
            Serial.println("Transition Progress: " + String(progress.completion_percentage) + "%");
            if (progress.has_error) {
                Serial.println("Transition Error: " + progress.error_message);
            }
        }

        Serial.println("Manual Leg Count: " + String(state_controller->getManualLegCount()));
        Serial.println("===================");
    }

    // Handle errors
    if (state_controller->hasError()) {
        Serial.println("State Controller Error: " + state_controller->getLastErrorMessage());
    }

    if (locomotion_system->getLastError() != LocomotionSystem::NO_ERROR) {
        Serial.println("Locomotion System Error: " +
                       locomotion_system->getErrorMessage(locomotion_system->getLastError()));
    }

    // Small delay to prevent overwhelming the serial output
    delay(20);
}

/**
 * @brief Example of manual leg control
 */
void demonstrateManualLegControl() {
    if (!state_controller->isReadyForOperation())
        return;

    // Set leg 0 to manual mode
    if (state_controller->setLegState(0, LEG_MANUAL)) {
        Serial.println("Set leg 0 to manual mode");

        // Set manual tip velocity for leg 0
        Eigen::Vector3d tip_velocity(10.0f, 0.0f, 5.0f); // mm/s
        state_controller->setLegTipVelocity(0, tip_velocity);

        delay(2000); // Move for 2 seconds

        // Return leg to walking mode
        state_controller->setLegState(0, LEG_WALKING);
        Serial.println("Returned leg 0 to walking mode");
    }
}

/**
 * @brief Example of pose control
 */
void demonstratePoseControl() {
    if (!state_controller->isReadyForOperation())
        return;

    // Enable manual posing mode
    if (state_controller->setPosingMode(POSING_X_Y)) {
        Serial.println("Enabled X-Y posing mode");

        // Set desired body pose
        Eigen::Vector3d position(10.0f, 5.0f, 0.0f);    // mm
        Eigen::Vector3d orientation(0.0f, 0.0f, 15.0f); // degrees
        state_controller->setDesiredPose(position, orientation);

        delay(3000); // Hold pose for 3 seconds

        // Reset pose
        state_controller->setPoseResetMode(POSE_RESET_ALL);
        state_controller->setPosingMode(POSING_NONE);
        Serial.println("Reset pose and disabled posing");
    }
}

/**
 * @brief Example of gait changing
 */
void demonstrateGaitChanging() {
    if (!state_controller->isReadyForOperation())
        return;

    Serial.println("Changing to wave gait...");
    if (state_controller->changeGait(WAVE_GAIT)) {
        delay(5000); // Walk with wave gait for 5 seconds

        Serial.println("Changing back to tripod gait...");
        state_controller->changeGait(TRIPOD_GAIT);
    }
}
