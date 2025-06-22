/**
 * @file pose_controller_flags_example.ino
 * @brief Example demonstrating how PoseController respects servo status flags
 */

#include "../src/HexaModel.h"
#include "../src/pose_controller.h"
#include "mock_interfaces.h"

MockServoInterface servos;
RobotModel model;
PoseController pose_controller(model, &servos);

Point3D leg_positions[NUM_LEGS];
JointAngles joint_angles[NUM_LEGS];

void setup() {
    Serial.begin(115200);
    Serial.println("PoseController Servo Flags Example");

    // Initialize with default pose
    pose_controller.initializeDefaultPose(leg_positions, joint_angles, 100.0f, 80.0f);
    Serial.println("Default pose initialized");

    // Test 1: Normal operation (no flags set)
    Serial.println("\n=== Test 1: Normal Movement (No Flags) ===");
    servos.setBlockingFlag(0, 0, false); // Ensure no flags are set

    Eigen::Vector3f position(0.0f, 0.0f, 90.0f);
    Eigen::Vector3f orientation(0.0f, 0.0f, 0.0f);

    bool result = pose_controller.setBodyPose(position, orientation, leg_positions, joint_angles);
    Serial.print("Body pose set result: ");
    Serial.println(result ? "SUCCESS" : "FAILED");

    // Test 2: Blocked servo (specific joint)
    Serial.println("\n=== Test 2: Blocked Servo (Leg 0, Joint 1) ===");
    servos.setBlockingFlag(0, 1, true); // Block femur joint on leg 0

    result = pose_controller.setBodyPose(position, orientation, leg_positions, joint_angles);
    Serial.print("Body pose with blocked servo result: ");
    Serial.println(result ? "SUCCESS (UNEXPECTED)" : "BLOCKED (EXPECTED)");

    // Test 3: Single leg movement with blocking flag
    Serial.println("\n=== Test 3: Single Leg Movement with Blocking Flag ===");
    // Keep the blocking flag on leg 0, joint 1
    Point3D new_position(120.0f, 50.0f, -70.0f);

    result = pose_controller.setLegPosition(0, new_position, leg_positions, joint_angles);
    Serial.print("Leg 0 movement with blocked servo result: ");
    Serial.println(result ? "SUCCESS (UNEXPECTED)" : "BLOCKED (EXPECTED)");

    // Test 4: Movement of different leg (should succeed)
    Serial.println("\n=== Test 4: Movement of Different Leg (Should Succeed) ===");
    result = pose_controller.setLegPosition(1, new_position, leg_positions, joint_angles);
    Serial.print("Leg 1 movement result: ");
    Serial.println(result ? "SUCCESS (EXPECTED)" : "BLOCKED (UNEXPECTED)");

    // Test 5: Clear flags and retry
    Serial.println("\n=== Test 5: Clear Flags and Retry ===");
    servos.setBlockingFlag(0, 1, false); // Clear the blocking flag

    result = pose_controller.setBodyPose(position, orientation, leg_positions, joint_angles);
    Serial.print("Body pose after clearing flags result: ");
    Serial.println(result ? "SUCCESS (EXPECTED)" : "BLOCKED (UNEXPECTED)");

    // Test 6: Immediate pose setting with flags
    Serial.println("\n=== Test 6: Immediate Pose with Blocking Flags ===");
    servos.setBlockingFlag(2, 0, true); // Block coxa joint on leg 2

    result = pose_controller.setBodyPoseImmediate(position, orientation, leg_positions, joint_angles);
    Serial.print("Immediate body pose with blocked servo result: ");
    Serial.println(result ? "SUCCESS (UNEXPECTED)" : "BLOCKED (EXPECTED)");

    // Clear flag for cleanup
    servos.setBlockingFlag(2, 0, false);

    Serial.println("\n=== Example Completed ===");
    Serial.println("All tests demonstrate that PoseController properly");
    Serial.println("respects servo status flags and prevents movement");
    Serial.println("when servos are blocked.");
}

void loop() {
    // Test smooth trajectory with flags (if enabled)
    static unsigned long last_test = 0;
    static bool flag_test_done = false;

    if (millis() - last_test > 5000 && !flag_test_done) {
        Serial.println("\n=== Periodic Test: Smooth Trajectory with Flags ===");

        // Enable smooth trajectory
        pose_controller.configureSmoothTrajectory(true, 0.1f, 10);

        // Set a blocking flag
        servos.setBlockingFlag(1, 2, true); // Block tibia joint on leg 1

        Eigen::Vector3f target_pos(10.0f, 10.0f, 85.0f);
        Eigen::Vector3f target_orient(5.0f, 0.0f, 0.0f);

        bool result = pose_controller.setBodyPose(target_pos, target_orient, leg_positions, joint_angles);
        Serial.print("Smooth trajectory with blocked servo result: ");
        Serial.println(result ? "SUCCESS (UNEXPECTED)" : "BLOCKED (EXPECTED)");

        // Clear flag
        servos.setBlockingFlag(1, 2, false);

        // Try again
        result = pose_controller.setBodyPose(target_pos, target_orient, leg_positions, joint_angles);
        Serial.print("Smooth trajectory after clearing flags: ");
        Serial.println(result ? "SUCCESS (EXPECTED)" : "BLOCKED (UNEXPECTED)");

        flag_test_done = true;
        last_test = millis();
    }

    delay(100);
}
