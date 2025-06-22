/**
 * @file smooth_movement_example.ino
 * @brief Example demonstrating smooth trajectory interpolation from current servo positions
 *
 * This example shows how to configure and use the new smooth movement feature
 * that uses current servo positions as starting points for trajectories,
 * equivalent to OpenSHC's smooth movement approach.
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
        return data;
    }
    bool update() override { return true; }
};

class MockFSR : public IFSRInterface {
  public:
    bool initialize() override { return true; }
    bool isConnected() override { return true; }
    float readForce(int leg_index) override { return 0.5f; }
    bool isGroundContact(int leg_index) override { return true; }
    bool update() override { return true; }
};

// Example servo interface that simulates realistic servo positions
class SmoothServo : public IServoInterface {
  private:
    float current_angles[NUM_LEGS][DOF_PER_LEG];
    float target_angles[NUM_LEGS][DOF_PER_LEG];
    float speeds[NUM_LEGS][DOF_PER_LEG];

  public:
    SmoothServo() {
        // Initialize with realistic starting positions
        for (int i = 0; i < NUM_LEGS; i++) {
            current_angles[i][0] = i * 15.0f - 45.0f; // Coxa: spread around body
            current_angles[i][1] = 30.0f;             // Femur: moderate angle
            current_angles[i][2] = -60.0f;            // Tibia: moderate angle

            target_angles[i][0] = current_angles[i][0];
            target_angles[i][1] = current_angles[i][1];
            target_angles[i][2] = current_angles[i][2];

            speeds[i][0] = 1.0f;
            speeds[i][1] = 1.0f;
            speeds[i][2] = 1.0f;
        }
    }

    bool initialize() override {
        Serial.println("SmoothServo: Initializing servo interface...");
        return true;
    }

    bool hasBlockingStatusFlags(int leg_index, int joint_index, uint8_t *active_flags = nullptr) override {
        // Mock implementation - no servos are blocked
        if (active_flags) {
            *active_flags = 0; // No flags active
        }
        return false; // No blocking flags
    }

    bool setJointAngleAndSpeed(int leg_index, int joint_index, float angle, float speed) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return false;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return false;

        target_angles[leg_index][joint_index] = angle;
        speeds[leg_index][joint_index] = speed;

        Serial.print("Servo L");
        Serial.print(leg_index);
        Serial.print("J");
        Serial.print(joint_index);
        Serial.print(": Target=");
        Serial.print(angle, 1);
        Serial.print("°, Speed=");
        Serial.print(speed, 2);
        Serial.print(", Current=");
        Serial.print(current_angles[leg_index][joint_index], 1);
        Serial.println("°");

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
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return false;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return false;

        float diff = abs(target_angles[leg_index][joint_index] - current_angles[leg_index][joint_index]);
        return diff > 0.5f; // Consider moving if difference > 0.5 degrees
    }

    bool enableTorque(int leg_index, int joint_index, bool enable) override {
        return true;
    }

    // Simulate servo movement towards targets
    void updateServoPositions() {
        for (int i = 0; i < NUM_LEGS; i++) {
            for (int j = 0; j < DOF_PER_LEG; j++) {
                float diff = target_angles[i][j] - current_angles[i][j];
                if (abs(diff) > 0.1f) {
                    // Move towards target at speed-controlled rate
                    float step = diff * speeds[i][j] * 0.1f; // Scale factor for simulation
                    current_angles[i][j] += step;
                }
            }
        }
    }
};

// Global objects
MockIMU imu;
MockFSR fsr;
SmoothServo servos;
LocomotionSystem locomotion_system;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("=== Smooth Movement Example ===");
    Serial.println("Demonstrating smooth trajectory interpolation from current servo positions");
    Serial.println("(Equivalent to OpenSHC smooth movement approach)");
    Serial.println();

    // Configure robot parameters
    Parameters params;
    params.hexagon_radius = 100.0f;
    params.coxa_length = 40.0f;
    params.femur_length = 80.0f;
    params.tibia_length = 120.0f;
    params.robot_height = 100.0f;
    params.control_frequency = 50.0f;
    params.default_servo_speed = 1.0f;

    // Joint limits
    params.coxa_angle_limits[0] = -90.0f;
    params.coxa_angle_limits[1] = 90.0f;
    params.femur_angle_limits[0] = -90.0f;
    params.femur_angle_limits[1] = 90.0f;
    params.tibia_angle_limits[0] = -90.0f;
    params.tibia_angle_limits[1] = 90.0f;

    // Configure smooth trajectory (this is the new feature!)
    params.smooth_trajectory.use_current_servo_positions = true; // Enable OpenSHC-style movement
    params.smooth_trajectory.enable_pose_interpolation = true;   // Enable smooth interpolation
    params.smooth_trajectory.interpolation_speed = 0.15f;        // Smooth interpolation speed
    params.smooth_trajectory.max_interpolation_steps = 15;       // Max steps for interpolation
    params.smooth_trajectory.position_tolerance_mm = 2.0f;       // Position tolerance
    params.smooth_trajectory.use_quaternion_slerp = true;        // Use quaternion interpolation

    // Initialize locomotion system
    locomotion_system.setParams(params);
    if (!locomotion_system.initialize(&imu, &fsr, &servos)) {
        Serial.println("ERROR: Failed to initialize locomotion system");
        return;
    }

    Serial.println("Locomotion system initialized successfully!");
    Serial.println();

    // Configure smooth movement parameters at runtime
    Serial.println("=== Configuring Smooth Movement ===");
    locomotion_system.configureSmoothMovement(
        true, // Enable smooth movement
        0.2f, // Interpolation speed (0.01-1.0, where 0.2 is moderately smooth)
        20    // Maximum interpolation steps
    );
    Serial.println("Smooth movement configured: Enabled, Speed=0.2, MaxSteps=20");
    Serial.println();

    // Show current servo positions (starting point)
    Serial.println("=== Current Servo Positions (Starting Point) ===");
    for (int i = 0; i < NUM_LEGS; i++) {
        Serial.print("Leg ");
        Serial.print(i);
        Serial.print(": Coxa=");
        Serial.print(servos.getJointAngle(i, 0), 1);
        Serial.print("°, Femur=");
        Serial.print(servos.getJointAngle(i, 1), 1);
        Serial.print("°, Tibia=");
        Serial.print(servos.getJointAngle(i, 2), 1);
        Serial.println("°");
    }
    Serial.println();

    Serial.println("=== Starting Smooth Movement Demonstration ===");
    Serial.println("Will demonstrate different pose changes with smooth trajectories...");
    Serial.println();
}

void loop() {
    static unsigned long last_demo_time = 0;
    static int demo_step = 0;

    // Update servo simulation
    servos.updateServoPositions();

    // Update locomotion system
    locomotion_system.update();

    // Run demo every 4 seconds
    if (millis() - last_demo_time > 4000) {
        last_demo_time = millis();

        Serial.print("Demo Step ");
        Serial.print(demo_step + 1);
        Serial.print(": ");

        switch (demo_step % 6) {
        case 0: {
            Serial.println("Smooth body height increase");
            Eigen::Vector3f pos(0, 0, 120.0f); // Raise body
            Eigen::Vector3f orient(0, 0, 0);
            locomotion_system.setBodyPoseSmooth(pos, orient);
            break;
        }

        case 1: {
            Serial.println("Smooth forward lean");
            Eigen::Vector3f pos(0, 0, 120.0f);
            Eigen::Vector3f orient(0, 10.0f, 0); // 10° pitch forward
            locomotion_system.setBodyPoseSmooth(pos, orient);
            break;
        }

        case 2: {
            Serial.println("Smooth lateral tilt");
            Eigen::Vector3f pos(0, 0, 120.0f);
            Eigen::Vector3f orient(15.0f, 0, 0); // 15° roll right
            locomotion_system.setBodyPoseSmooth(pos, orient);
            break;
        }

        case 3: {
            Serial.println("Smooth rotation");
            Eigen::Vector3f pos(0, 0, 120.0f);
            Eigen::Vector3f orient(0, 0, 20.0f); // 20° yaw rotation
            locomotion_system.setBodyPoseSmooth(pos, orient);
            break;
        }

        case 4: {
            Serial.println("Smooth crouch position");
            Eigen::Vector3f pos(0, 0, 80.0f); // Lower body
            Eigen::Vector3f orient(0, 0, 0);
            locomotion_system.setBodyPoseSmooth(pos, orient);
            break;
        }

        case 5: {
            Serial.println("Return to neutral (compare with immediate movement)");
            Eigen::Vector3f pos(0, 0, 100.0f);
            Eigen::Vector3f orient(0, 0, 0);
            // Use immediate movement to show the difference
            locomotion_system.setBodyPoseImmediate(pos, orient);
            break;
        }
        }

        Serial.print("  Trajectory in progress: ");
        Serial.println(locomotion_system.isSmoothMovementInProgress() ? "YES" : "NO");
        Serial.println();

        demo_step++;
    }

    delay(50); // 20 Hz update rate
}
