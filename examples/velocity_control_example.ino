/**
 * @file velocity_control_example.ino
 * @brief Example demonstrating Cartesian velocity control equivalent to OpenSHC
 *
 * This example shows how servo speeds are dynamically adjusted based on commanded
 * Cartesian velocities (linear and angular), providing equivalent functionality
 * to OpenSHC's velocity control system.
 *
 * Key Features Demonstrated:
 * - Servo speed scaling based on linear velocity
 * - Angular velocity compensation for turning
 * - Gait-specific speed adjustments
 * - Per-leg speed compensation
 * - Velocity control enable/disable
 */

#include "locomotion_system.h"
#include "mock_interfaces.h"

// Mock interface implementations
MockIMUInterface mock_imu;
MockFSRInterface mock_fsr;
MockServoInterface mock_servo;

// Robot parameters - same as used in tests
Parameters setupTestParameters() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 100;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;
    p.default_servo_speed = 1.0f;
    p.max_velocity = 200.0f;        // mm/s
    p.max_angular_velocity = 90.0f; // degrees/s
    return p;
}

LocomotionSystem locomotion(setupTestParameters());

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("=== HexaMotion Cartesian Velocity Control Demo ===");
    Serial.println("OpenSHC-equivalent servo speed control system");
    Serial.println();

    // Initialize locomotion system
    if (!locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo)) {
        Serial.println("Failed to initialize locomotion system!");
        return;
    }

    // Set standing pose
    locomotion.setStandingPose();

    Serial.println("System initialized successfully!");
    Serial.println();

    demonstrateVelocityControl();
}

void loop() {
    // Update locomotion system
    locomotion.update();
    delay(20); // 50Hz update rate
}

void demonstrateVelocityControl() {
    CartesianVelocityController *velocity_ctrl = locomotion.getVelocityController();
    if (!velocity_ctrl) {
        Serial.println("Velocity controller not available!");
        return;
    }

    Serial.println("--- Velocity Control Demonstration ---");

    // Test 1: Low velocity forward motion
    Serial.println("\n1. Low velocity forward motion (0.05 m/s):");
    locomotion.planGaitSequence(0.05f, 0.0f, 0.0f);
    printServoSpeeds("Low forward velocity");

    // Test 2: High velocity forward motion
    Serial.println("\n2. High velocity forward motion (0.15 m/s):");
    locomotion.planGaitSequence(0.15f, 0.0f, 0.0f);
    printServoSpeeds("High forward velocity");

    // Test 3: Pure rotation
    Serial.println("\n3. Pure rotation (45 deg/s):");
    locomotion.planGaitSequence(0.0f, 0.0f, 0.785f); // 45 deg/s = 0.785 rad/s
    printServoSpeeds("Pure rotation");

    // Test 4: Combined linear and angular motion
    Serial.println("\n4. Combined motion (0.1 m/s forward + 30 deg/s rotation):");
    locomotion.planGaitSequence(0.1f, 0.0f, 0.523f); // 30 deg/s = 0.523 rad/s
    printServoSpeeds("Combined motion");

    // Test 5: Diagonal motion
    Serial.println("\n5. Diagonal motion (0.08 m/s at 45°):");
    float diagonal_vel = 0.08f / sqrt(2.0f); // Split between X and Y
    locomotion.planGaitSequence(diagonal_vel, diagonal_vel, 0.0f);
    printServoSpeeds("Diagonal motion");

    // Test 6: Different gait types
    Serial.println("\n6. Gait-specific speed adjustments:");

    // Tripod gait (fast)
    locomotion.setGaitType(TRIPOD_GAIT);
    locomotion.planGaitSequence(0.1f, 0.0f, 0.0f);
    printServoSpeeds("Tripod gait");

    // Wave gait (slow, stable)
    locomotion.setGaitType(WAVE_GAIT);
    locomotion.planGaitSequence(0.1f, 0.0f, 0.0f);
    printServoSpeeds("Wave gait");

    // Test 7: Velocity control disable/enable
    Serial.println("\n7. Velocity control disable/enable:");

    // Disable velocity control
    locomotion.setVelocityControlEnabled(false);
    locomotion.planGaitSequence(0.15f, 0.0f, 0.0f);
    printServoSpeeds("Velocity control DISABLED");

    // Re-enable velocity control
    locomotion.setVelocityControlEnabled(true);
    locomotion.planGaitSequence(0.15f, 0.0f, 0.0f);
    printServoSpeeds("Velocity control ENABLED");

    // Test 8: Custom velocity scaling
    Serial.println("\n8. Custom velocity scaling:");
    CartesianVelocityController::VelocityScaling custom_scaling;
    custom_scaling.linear_velocity_scale = 3.0f; // More aggressive scaling
    custom_scaling.minimum_speed_ratio = 0.3f;   // Higher minimum speed
    custom_scaling.maximum_speed_ratio = 2.5f;   // Higher maximum speed

    locomotion.setVelocityScaling(custom_scaling);
    locomotion.planGaitSequence(0.1f, 0.0f, 0.0f);
    printServoSpeeds("Custom aggressive scaling");

    Serial.println("\n--- Demonstration Complete ---");
    Serial.println("Servo speeds dynamically adjust based on commanded velocities!");
    Serial.println("Higher velocities = faster servo speeds for responsive motion.");
}

void printServoSpeeds(const char *scenario) {
    Serial.print("Scenario: ");
    Serial.println(scenario);

    // Print servo speeds for first 3 legs (representative sample)
    for (int leg = 0; leg < 3; leg++) {
        Serial.print("  Leg ");
        Serial.print(leg);
        Serial.print(": ");

        for (int joint = 0; joint < DOF_PER_LEG; joint++) {
            float speed = locomotion.getCurrentServoSpeed(leg, joint);
            const char *joint_names[] = {"Coxa", "Femur", "Tibia"};
            Serial.print(joint_names[joint]);
            Serial.print("=");
            Serial.print(speed, 2);
            if (joint < DOF_PER_LEG - 1)
                Serial.print(", ");
        }
        Serial.println();
    }

    // Show total velocity magnitude
    CartesianVelocityController *velocity_ctrl = locomotion.getVelocityController();
    if (velocity_ctrl) {
        Serial.print("  Total velocity magnitude: ");
        Serial.print(velocity_ctrl->getCurrentVelocityMagnitude(), 3);
        Serial.println(" m/s");
    }

    delay(1000); // Pause between tests
}
