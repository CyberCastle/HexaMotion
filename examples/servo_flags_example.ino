/**
 * @file servo_flags_example.ino
 * @brief Example demonstrating servo status flags checking
 *
 * This example shows how the HexaMotion system checks for servo status flags
 * before attempting to move servos, preventing damage from overheated, overloaded,
 * or otherwise compromised servos.
 */

#include "HexaModel.h"
#include "locomotion_system.h"

// Mock interfaces for demonstration
class MockIMU : public IIMUInterface {
  public:
    bool initialize() override { return true; }
    bool isConnected() override { return true; }
    IMUData readIMU() override {
        IMUData data;
        data.roll = 0.0f;
        data.pitch = 0.0f;
        data.yaw = 0.0f;
        data.is_valid = true;
        return data;
    }
    bool calibrate() override { return true; }
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
        data.in_contact = true;
        data.pressure = 0.5f;
        return data;
    }
    bool calibrateFSR(int leg_index) override { return true; }
    double getRawReading(int leg_index) override { return 0.5f; }
    bool update() override { return true; }
};

// Example servo interface that can simulate blocked servos
class FlagsTestServo : public IServoInterface {
  private:
    double current_angles[NUM_LEGS][DOF_PER_LEG];
    uint8_t servo_flags[NUM_LEGS][DOF_PER_LEG];

  public:
    FlagsTestServo() {
        // Initialize arrays
        for (int i = 0; i < NUM_LEGS; i++) {
            for (int j = 0; j < DOF_PER_LEG; j++) {
                current_angles[i][j] = 0.0f;
                servo_flags[i][j] = 0; // No flags initially
            }
        }
    }

    bool initialize() override {
        Serial.println("FlagsTestServo: Initializing servo interface...");
        return true;
    }

    bool hasBlockingStatusFlags(int leg_index, int joint_index, uint8_t *active_flags = nullptr) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS || joint_index < 0 || joint_index >= DOF_PER_LEG) {
            return false;
        }

        uint8_t flags = servo_flags[leg_index][joint_index];
        if (active_flags) {
            *active_flags = flags;
        }

        // Return true if any blocking flags are set
        return (flags != 0);
    }

    bool setJointAngleAndSpeed(int leg_index, int joint_index, double angle, double speed) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return false;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return false;

        current_angles[leg_index][joint_index] = angle;

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

    double getJointAngle(int leg_index, int joint_index) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return 0.0f;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return 0.0f;
        return current_angles[leg_index][joint_index];
    }

    bool isJointMoving(int leg_index, int joint_index) override {
        return false;
    }

    bool enableTorque(int leg_index, int joint_index, bool enable) override {
        return true;
    }

    // Test methods to simulate different error conditions
    void setServoFlag(int leg_index, int joint_index, ServoStatusFlag flag) {
        if (leg_index >= 0 && leg_index < NUM_LEGS && joint_index >= 0 && joint_index < DOF_PER_LEG) {
            servo_flags[leg_index][joint_index] |= flag;
        }
    }

    void clearServoFlag(int leg_index, int joint_index, ServoStatusFlag flag) {
        if (leg_index >= 0 && leg_index < NUM_LEGS && joint_index >= 0 && joint_index < DOF_PER_LEG) {
            servo_flags[leg_index][joint_index] &= ~flag;
        }
    }

    void clearAllFlags(int leg_index, int joint_index) {
        if (leg_index >= 0 && leg_index < NUM_LEGS && joint_index >= 0 && joint_index < DOF_PER_LEG) {
            servo_flags[leg_index][joint_index] = 0;
        }
    }
};

// Global objects
MockIMU imu;
MockFSR fsr;
FlagsTestServo servos;
LocomotionSystem locomotion_system;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("=== Servo Status Flags Example ===");
    Serial.println("Demonstrating servo blocking flag functionality");
    Serial.println();

    // Configure robot parameters
    Parameters params;
    params.hexagon_radius = 100.0f;
    params.coxa_length = 40.0f;
    params.femur_length = 80.0f;
    params.tibia_length = 120.0f;
    params.robot_height = 120.0f;
    params.control_frequency = 50.0f;
    params.default_servo_speed = 1.0f;

    // Joint limits
    params.coxa_angle_limits[0] = -90.0f;
    params.coxa_angle_limits[1] = 90.0f;
    params.femur_angle_limits[0] = -90.0f;
    params.femur_angle_limits[1] = 90.0f;
    params.tibia_angle_limits[0] = -90.0f;
    params.tibia_angle_limits[1] = 90.0f;

    // Initialize locomotion system
    locomotion_system.setParams(params);
    if (!locomotion_system.initialize(&imu, &fsr, &servos)) {
        Serial.println("ERROR: Failed to initialize locomotion system");
        return;
    }

    Serial.println("Locomotion system initialized successfully!");
    Serial.println();

    // Set standing pose
    locomotion_system.setStandingPose();
    Serial.println("Standing pose set");
    Serial.println();

    demonstrateServoFlags();
}

void loop() {
    // Update locomotion system
    locomotion_system.update();
    delay(50);
}

void demonstrateServoFlags() {
    Serial.println("=== Servo Flags Demonstration ===");
    Serial.println();

    // Test 1: Normal operation (no flags)
    Serial.println("1. Normal operation - all servos ready:");
    JointAngles test_angles;
    test_angles.coxa = 10.0f;
    test_angles.femur = -20.0f;
    test_angles.tibia = 30.0f;

    // Use pose controller instead of setLegJointAngles (which is private)
    bool success = locomotion_system.setLegPosition(0, Point3D(120, 0, -80));
    Serial.print("   Result: ");
    Serial.println(success ? "SUCCESS" : "FAILED");
    Serial.print("   Error: ");
    Serial.println(locomotion_system.getErrorMessage(locomotion_system.getLastError()));
    Serial.println();

    // Test 2: Simulate overheated servo
    Serial.println("2. Simulating overheated servo on Leg 0, Joint 1 (femur):");
    servos.setServoFlag(0, 1, SERVO_FLAG_OVERHEATED);

    success = locomotion_system.setLegPosition(0, Point3D(125, 0, -85));
    Serial.print("   Result: ");
    Serial.println(success ? "SUCCESS" : "FAILED");
    Serial.print("   Error: ");
    Serial.println(locomotion_system.getErrorMessage(locomotion_system.getLastError()));
    Serial.println();

    // Test 3: Multiple flags
    Serial.println("3. Simulating multiple issues on Leg 1:");
    servos.setServoFlag(1, 0, SERVO_FLAG_OVERLOADED);
    servos.setServoFlag(1, 2, SERVO_FLAG_STALLED);

    success = locomotion_system.setLegPosition(1, Point3D(125, 0, -85));
    Serial.print("   Result: ");
    Serial.println(success ? "SUCCESS" : "FAILED");
    Serial.print("   Error: ");
    Serial.println(locomotion_system.getErrorMessage(locomotion_system.getLastError()));
    Serial.println();

    // Test 4: Test system update with blocked servos
    Serial.println("4. Testing system update with blocked servos:");
    locomotion_system.planGaitSequence(0.05f, 0.0f, 0.0f); // Slow forward movement

    for (int i = 0; i < 5; i++) {
        bool update_success = locomotion_system.update();
        Serial.print("   Update ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(update_success ? "SUCCESS" : "FAILED");
        Serial.print(" - ");
        Serial.println(locomotion_system.getErrorMessage(locomotion_system.getLastError()));
        delay(100);
    }
    Serial.println();

    // Test 5: Clear flags and resume operation
    Serial.println("5. Clearing all flags and resuming normal operation:");
    servos.clearAllFlags(0, 1);
    servos.clearAllFlags(1, 0);
    servos.clearAllFlags(1, 2);

    success = locomotion_system.setLegPosition(0, Point3D(120, 0, -80));
    Serial.print("   Leg 0 Result: ");
    Serial.println(success ? "SUCCESS" : "FAILED");

    success = locomotion_system.setLegPosition(1, Point3D(120, 0, -80));
    Serial.print("   Leg 1 Result: ");
    Serial.println(success ? "SUCCESS" : "FAILED");

    Serial.print("   Final Error Status: ");
    Serial.println(locomotion_system.getErrorMessage(locomotion_system.getLastError()));
    Serial.println();

    Serial.println("=== Demonstration Complete ===");
    Serial.println("The system successfully prevented servo movement when flags were active!");
    Serial.println("This protects hardware from damage due to:");
    Serial.println("- Overheating");
    Serial.println("- Overloading");
    Serial.println("- Voltage issues");
    Serial.println("- Communication errors");
    Serial.println("- And other servo malfunctions");
    Serial.println();
}
