/**
 * @file example_derivative_admittance.ino
 * @brief Example usage of derivative-based admittance control
 *
 * This example demonstrates how to enable and configure the physics-accurate
 * derivative-based admittance control system, equivalent to OpenSHC's approach
 * but with configurable precision levels.
 *
 * Based on AGENTS.md guidelines for HexaMotion development.
 */

#include "HexaModel.h"
#include "admittance_controller.h"
#include "locomotion_system.h"
#include "precision_config.h"

// Test robot model and interfaces
class MockIMU : public IIMUInterface {
  public:
    IMUData readIMU() override {
        return {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
};

class MockFSR : public IFSRInterface {
  public:
    FSRData readFSR(int leg_index) override {
        return {true, 5.0f}; // Simulate contact
    }
};

class MockServo : public IServoInterface {
  public:
    void setJointAngle(int leg_index, int joint_index, float angle) override {}
    float getJointAngle(int leg_index, int joint_index) override { return 0.0f; }
    void enableJoint(int leg_index, int joint_index, bool enable) override {}
    bool isJointEnabled(int leg_index, int joint_index) override { return true; }
};

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("=== HexaMotion Derivative-Based Admittance Control Example ===");

    // Initialize robot model with realistic parameters from AGENTS.md
    Parameters params;
    params.hexagon_radius = 400;   // 400mm hexagon
    params.coxa_length = 50;       // 50mm coxa
    params.femur_length = 101;     // 101mm femur
    params.tibia_length = 208;     // 208mm tibia
    params.robot_height = 90;      // 90mm height
    params.control_frequency = 50; // 50Hz control

    // Set joint angle limits
    params.coxa_angle_limits[0] = -65;
    params.coxa_angle_limits[1] = 65;
    params.femur_angle_limits[0] = -75;
    params.femur_angle_limits[1] = 75;
    params.tibia_angle_limits[0] = -45;
    params.tibia_angle_limits[1] = 45;

    RobotModel model(params);
    MockIMU imu;
    MockFSR fsr;
    MockServo servo;

    Serial.println("Robot model initialized with AGENTS.md parameters");

    // ==============================
    // EXAMPLE 1: Basic Usage
    // ==============================
    Serial.println("\n--- Example 1: Basic Derivative-Based Admittance ---");

    // Create controller with high precision for maximum accuracy
    AdmittanceController controller(model, &imu, &fsr, ComputeConfig::high());
    controller.initialize();

    // Enable derivative-based integration (physics-accurate)
    controller.setDerivativeBasedIntegration(true);
    Serial.println("✓ Derivative-based integration enabled");

    // Configure virtual leg dynamics
    float virtual_mass = 0.5f;        // 500g virtual mass per leg
    float virtual_damping = 2.0f;     // Light damping for responsiveness
    float virtual_stiffness = 100.0f; // Moderate stiffness

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        controller.setLegAdmittance(leg, virtual_mass, virtual_damping, virtual_stiffness);
    }
    Serial.println("✓ Admittance parameters set for all legs");

    // Simulate external forces and compute response
    Point3D forces[NUM_LEGS];
    Point3D deltas[NUM_LEGS];

    // Apply downward force to leg 0 (simulating ground contact)
    forces[0] = Point3D(0, 0, -10.0f); // 10N downward
    for (int i = 1; i < NUM_LEGS; i++) {
        forces[i] = Point3D(0, 0, 0);
    }

    controller.updateAllLegs(forces, deltas);

    Serial.print("Force applied: (0, 0, -10N) → Position delta: (");
    Serial.print(deltas[0].x, 4);
    Serial.print(", ");
    Serial.print(deltas[0].y, 4);
    Serial.print(", ");
    Serial.print(deltas[0].z, 4);
    Serial.println(")");

    // ==============================
    // EXAMPLE 2: Precision Comparison
    // ==============================
    Serial.println("\n--- Example 2: Precision Level Comparison ---");

    ComputeConfig configs[] = {
        ComputeConfig::low(),    // Fast, basic (Euler integration)
        ComputeConfig::medium(), // Balanced (RK2 integration)
        ComputeConfig::high()    // Maximum accuracy (RK4 integration)
    };

    const char *precision_names[] = {"LOW (Euler)", "MEDIUM (RK2)", "HIGH (RK4)"};

    for (int i = 0; i < 3; i++) {
        AdmittanceController test_controller(model, &imu, &fsr, configs[i]);
        test_controller.initialize();
        test_controller.setDerivativeBasedIntegration(true);
        test_controller.setLegAdmittance(0, 0.5f, 2.0f, 100.0f);

        Point3D test_force(0, 0, -5.0f);
        Point3D test_delta = test_controller.applyForceAndIntegrate(0, test_force);

        Serial.print(precision_names[i]);
        Serial.print(" precision: ");
        Serial.print("Δz = ");
        Serial.println(test_delta.z, 6);
    }

    // ==============================
    // EXAMPLE 3: Dynamic Stiffness
    // ==============================
    Serial.println("\n--- Example 3: Dynamic Stiffness Adaptation ---");

    // Enable dynamic stiffness for gait-adaptive compliance
    controller.setDynamicStiffness(true, 0.5f, 1.5f);
    Serial.println("✓ Dynamic stiffness enabled (swing: 0.5x, load: 1.5x)");

    // Simulate gait states
    LegState leg_states[NUM_LEGS];
    Point3D leg_positions[NUM_LEGS];

    // Leg 0 is swinging, others are in stance
    leg_states[0] = SWING_PHASE;
    for (int i = 1; i < NUM_LEGS; i++) {
        leg_states[i] = STANCE_PHASE;
    }

    // Set leg positions
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_positions[i] = Point3D(0, 0, 0);
    }

    controller.updateStiffness(leg_states, leg_positions, 40.0f);

    // Apply same force to swing leg (should have different response due to reduced stiffness)
    Point3D swing_delta = controller.applyForceAndIntegrate(0, Point3D(0, 0, -5.0f));

    Serial.print("Swing leg response with dynamic stiffness: Δz = ");
    Serial.println(swing_delta.z, 6);

    // ==============================
    // EXAMPLE 4: Traditional vs Derivative Comparison
    // ==============================
    Serial.println("\n--- Example 4: Traditional vs Derivative Methods ---");

    // Traditional simplified method
    AdmittanceController traditional(model, &imu, &fsr, ComputeConfig::medium());
    traditional.initialize();
    traditional.setDerivativeBasedIntegration(false); // Use traditional simplified method
    traditional.setLegAdmittance(0, 0.5f, 2.0f, 100.0f);

    // Derivative-based method
    AdmittanceController derivative(model, &imu, &fsr, ComputeConfig::medium());
    derivative.initialize();
    derivative.setDerivativeBasedIntegration(true); // Use physics-accurate method
    derivative.setLegAdmittance(0, 0.5f, 2.0f, 100.0f);

    // Apply identical forces
    Point3D test_force(0, 0, -8.0f);
    Point3D traditional_delta = traditional.applyForceAndIntegrate(0, test_force);
    Point3D derivative_delta = derivative.applyForceAndIntegrate(0, test_force);

    Serial.println("Comparison for 8N downward force:");
    Serial.print("  Traditional (simplified): Δz = ");
    Serial.println(traditional_delta.z, 6);
    Serial.print("  Derivative (physics):     Δz = ");
    Serial.println(derivative_delta.z, 6);

    float difference = std::abs(derivative_delta.z - traditional_delta.z);
    float percentage = (difference / std::abs(derivative_delta.z)) * 100.0f;
    Serial.print("  Difference: ");
    Serial.print(difference, 6);
    Serial.print(" (");
    Serial.print(percentage, 1);
    Serial.println("%)");

    Serial.println("\n=== Example Complete ===");
    Serial.println("The derivative-based admittance control provides:");
    Serial.println("• Physics-accurate differential equation solving");
    Serial.println("• Configurable precision (Euler/RK2/RK4)");
    Serial.println("• Better numerical stability");
    Serial.println("• Equivalent functionality to OpenSHC admittance");
    Serial.println("• Compatible with existing HexaMotion architecture");
}

void loop() {
    // In a real application, you would continuously:
    // 1. Read sensor data (IMU, FSR)
    // 2. Compute external forces
    // 3. Update admittance control
    // 4. Apply position corrections to locomotion

    delay(1000);

    static int counter = 0;
    if (counter % 10 == 0) {
        Serial.println("Real-time admittance control would run here...");
        Serial.println("Use ComputeConfig::high() for maximum precision");
        Serial.println("Use setDerivativeBasedIntegration(true) for physics accuracy");
    }
    counter++;
}
