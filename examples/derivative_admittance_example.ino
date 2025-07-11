/**
 * @file derivative_admittance_example.ino
 * @brief Example usage of derivative-based admittance control
 *
 * This example demonstrates the physics-accurate derivative-based
 * admittance control system using math_utils functions for integration.
 *
 * All admittance control now uses standardized Runge-Kutta and Euler
 * methods from math_utils to avoid code duplication.
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
    bool initialize() override { return true; }
    bool calibrate() override { return true; }
    bool isConnected() override { return true; }
    bool setIMUMode(IMUMode mode) override { return true; }
    IMUMode getIMUMode() const override { return IMU_MODE_RAW_DATA; }
    bool hasAbsolutePositioning() const override { return false; }
    bool getCalibrationStatus(uint8_t *s, uint8_t *g, uint8_t *a, uint8_t *m) override { return true; }
    bool runSelfTest() override { return true; }
    bool resetOrientation() override { return true; }
    bool update() override { return true; } // Parallel sensor reading support

    IMUData readIMU() override {
        return {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
};

class MockFSR : public IFSRInterface {
  public:
    bool initialize() override { return true; }
    bool calibrateFSR(int leg_index) override { return true; }
    double getRawReading(int leg_index) override { return 5.0f; }
    bool update() override { return true; } // AdvancedAnalog DMA support

    FSRData readFSR(int leg_index) override {
        return {true, 5.0f}; // Simulate contact
    }
};

class MockServo : public IServoInterface {
  public:
    bool initialize() override { return true; }

    bool hasBlockingStatusFlags(int leg_index, int joint_index, uint8_t *active_flags = nullptr) override {
        if (active_flags)
            *active_flags = 0;
        return false;
    }

    bool setJointAngleAndSpeed(int leg_index, int joint_index, double angle, double speed) override {
        (void)leg_index;
        (void)joint_index;
        (void)angle;
        (void)speed;
        return true;
    }
    double getJointAngle(int leg_index, int joint_index) override { return 0.0f; }
    bool isJointMoving(int leg_index, int joint_index) override { return false; }
    bool enableTorque(int leg_index, int joint_index, bool enable) override { return true; }
};

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("=== HexaMotion Derivative-Based Admittance Control Example ===");

    // Initialize robot model with realistic parameters from AGENTS.md
    Parameters params;
    params.hexagon_radius = 200;   // 200mm hexagon
    params.coxa_length = 50;       // 50mm coxa
    params.femur_length = 101;     // 101mm femur
    params.tibia_length = 208;     // 208mm tibia
    params.robot_height = 208;     // 208mm height
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

    Serial.println("✓ Derivative-based integration using math_utils functions");

    // Configure virtual leg dynamics
    double virtual_mass = 0.5f;        // 500g virtual mass per leg
    double virtual_damping = 2.0f;     // Light damping for responsiveness
    double virtual_stiffness = 100.0f; // Moderate stiffness

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
    // EXAMPLE 4: Precision Levels Performance
    // ==============================
    Serial.println("\n--- Example 4: Integration Method Performance ---");

    // Test different precision levels to show math_utils integration methods
    for (int i = 0; i < 3; i++) {
        AdmittanceController precision_test(model, &imu, &fsr, configs[i]);
        precision_test.initialize();
        precision_test.setLegAdmittance(0, 0.5f, 2.0f, 100.0f);

        // Apply multiple forces to show integration behavior
        Point3D cumulative_delta(0, 0, 0);
        Point3D test_force(0, 0, -3.0f);

        for (int step = 0; step < 5; step++) {
            Point3D step_delta = precision_test.applyForceAndIntegrate(0, test_force);
            cumulative_delta = cumulative_delta + step_delta;
        }

        Serial.print(precision_names[i]);
        Serial.print(" cumulative response: Δz = ");
        Serial.println(cumulative_delta.z, 6);
    }

    Serial.println("\n=== Example Complete ===");
    Serial.println("The derivative-based admittance control provides:");
    Serial.println("• Physics-accurate differential equation solving");
    Serial.println("• Configurable precision (Euler/RK2/RK4) via math_utils");
    Serial.println("• Better numerical stability");
    Serial.println("• No code duplication - uses standardized math functions");
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
        Serial.println("Use ComputeConfig::high() for maximum precision with RK4");
        Serial.println("All integration uses standardized math_utils functions");
    }
    counter++;
}
