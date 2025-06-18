/**
 * @file servo_interface_test.ino
 * @brief Test the new combined servo interface (angle + speed)
 *
 * This example demonstrates the updated IServoInterface that requires
 * setting both angle and speed parameters simultaneously.
 */

#include "HexaModel.h"
#include "locomotion_system.h"

// Example implementation of the new servo interface
class TestServo : public IServoInterface {
  private:
    float current_angles[NUM_LEGS][DOF_PER_LEG];
    float current_speeds[NUM_LEGS][DOF_PER_LEG];

  public:
    TestServo() {
        // Initialize arrays
        for (int i = 0; i < NUM_LEGS; i++) {
            for (int j = 0; j < DOF_PER_LEG; j++) {
                current_angles[i][j] = 0.0f;
                current_speeds[i][j] = 1.0f;
            }
        }
    }

    bool initialize() override {
        Serial.println("TestServo: Initializing servo interface...");
        return true;
    }

    // NEW: Combined angle and speed control
    bool setJointAngleAndSpeed(int leg_index, int joint_index, float angle, float speed) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return false;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return false;

        // Store both parameters
        current_angles[leg_index][joint_index] = angle;
        current_speeds[leg_index][joint_index] = speed;

        // In real implementation, you would send both parameters to the servo
        Serial.print("Servo L");
        Serial.print(leg_index);
        Serial.print("J");
        Serial.print(joint_index);
        Serial.print(": Angle=");
        Serial.print(angle, 1);
        Serial.print("°, Speed=");
        Serial.println(speed, 1);

        return true;
    }

    float getJointAngle(int leg_index, int joint_index) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return 0.0f;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return 0.0f;

        return current_angles[leg_index][joint_index];
    }

    bool isJointMoving(int leg_index, int joint_index) override {
        // Simulate movement detection
        return false;
    }

    bool enableTorque(int leg_index, int joint_index, bool enable) override {
        Serial.print("Servo L");
        Serial.print(leg_index);
        Serial.print("J");
        Serial.print(joint_index);
        Serial.print(": Torque ");
        Serial.println(enable ? "ON" : "OFF");
        return true;
    }

    // Method to get current speed (for testing)
    float getCurrentSpeed(int leg_index, int joint_index) {
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return 0.0f;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return 0.0f;
        return current_speeds[leg_index][joint_index];
    }
};

// Simple mock implementations for other interfaces
class MockIMU : public IIMUInterface {
  public:
    bool initialize() override { return true; }
    IMUData readIMU() override {
        IMUData data{};
        data.roll = 0.0f;
        data.pitch = 0.0f;
        data.yaw = 0.0f;
        data.is_valid = true;
        return data;
    }
    bool calibrate() override { return true; }
    bool isConnected() override { return true; }
    bool setIMUMode(IMUMode mode) override { return true; }
    IMUMode getIMUMode() const override { return IMU_MODE_RAW_DATA; }
    bool hasAbsolutePositioning() const override { return false; }
    bool getCalibrationStatus(uint8_t *s, uint8_t *g, uint8_t *a, uint8_t *m) override { return true; }
    bool runSelfTest() override { return true; }
    bool resetOrientation() override { return true; }
    bool update() override { return true; }
};

class MockFSR : public IFSRInterface {
  public:
    bool initialize() override { return true; }
    FSRData readFSR(int leg_index) override {
        FSRData data{};
        data.in_contact = false;
        data.pressure = 0.0f;
        return data;
    }
    bool calibrateFSR(int leg_index) override { return true; }
    float getRawReading(int leg_index) override { return 0.0f; }
    bool update() override { return true; }
};

// Global objects
TestServo test_servo;
MockIMU mock_imu;
MockFSR mock_fsr;
LocomotionSystem locomotion_system;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("=== Servo Interface Test ===");
    Serial.println("Testing new combined angle+speed interface");
    Serial.println();

    // Configure robot parameters
    Parameters params;
    params.hexagon_radius = 80.0f;
    params.coxa_length = 30.0f;
    params.femur_length = 50.0f;
    params.tibia_length = 80.0f;
    params.robot_height = 100.0f;

    // Set joint limits
    params.coxa_angle_limits[0] = -90.0f;
    params.coxa_angle_limits[1] = 90.0f;
    params.femur_angle_limits[0] = -90.0f;
    params.femur_angle_limits[1] = 90.0f;
    params.tibia_angle_limits[0] = -90.0f;
    params.tibia_angle_limits[1] = 90.0f;

    // Set default servo speed
    params.default_servo_speed = 1.2f;

    // Initialize locomotion system
    locomotion_system.setParams(params);
    if (!locomotion_system.initialize(&mock_imu, &mock_fsr, &test_servo)) {
        Serial.println("ERROR: Failed to initialize locomotion system");
        return;
    }

    Serial.println("Locomotion system initialized successfully");
    Serial.print("Default servo speed: ");
    Serial.println(params.default_servo_speed);
    Serial.println();

    // Test direct servo interface
    Serial.println("=== Direct Servo Interface Test ===");

    // Test various speed settings
    test_servo.setJointAngleAndSpeed(0, 0, 45.0f, 0.5f);  // Slow movement
    test_servo.setJointAngleAndSpeed(0, 1, -30.0f, 1.0f); // Normal speed
    test_servo.setJointAngleAndSpeed(0, 2, 60.0f, 2.0f);  // Fast movement

    Serial.println();

    // Test locomotion system integration
    Serial.println("=== Locomotion System Integration Test ===");

    // Test setting leg position (should use default speed)
    JointAngles test_angles;
    test_angles.coxa = 20.0f;
    test_angles.femur = -45.0f;
    test_angles.tibia = 30.0f;

    Serial.print("Setting leg 0 to: Coxa=");
    Serial.print(test_angles.coxa);
    Serial.print("°, Femur=");
    Serial.print(test_angles.femur);
    Serial.print("°, Tibia=");
    Serial.print(test_angles.tibia);
    Serial.println("°");

    locomotion_system.setLegJointAngles(0, test_angles);

    Serial.println();
    Serial.println("=== Test Complete ===");
    Serial.println("All servo commands should show both angle and speed parameters");
}

void loop() {
    // Test different speed settings every 3 seconds
    static unsigned long last_test = 0;
    static int test_case = 0;

    if (millis() - last_test > 3000) {
        last_test = millis();

        Serial.print("Test case ");
        Serial.print(test_case);
        Serial.print(": ");

        switch (test_case % 4) {
        case 0:
            Serial.println("Slow movement test");
            test_servo.setJointAngleAndSpeed(1, 0, 30.0f, 0.3f);
            test_servo.setJointAngleAndSpeed(1, 1, -20.0f, 0.3f);
            break;
        case 1:
            Serial.println("Normal movement test");
            test_servo.setJointAngleAndSpeed(1, 0, -30.0f, 1.0f);
            test_servo.setJointAngleAndSpeed(1, 1, 20.0f, 1.0f);
            break;
        case 2:
            Serial.println("Fast movement test");
            test_servo.setJointAngleAndSpeed(1, 0, 45.0f, 2.5f);
            test_servo.setJointAngleAndSpeed(1, 1, -45.0f, 2.5f);
            break;
        case 3:
            Serial.println("Mixed speed test");
            test_servo.setJointAngleAndSpeed(1, 0, 0.0f, 0.8f);
            test_servo.setJointAngleAndSpeed(1, 1, 0.0f, 1.5f);
            break;
        }

        test_case++;
        Serial.println();
    }

    // Update locomotion system
    locomotion_system.update();

    delay(50);
}
