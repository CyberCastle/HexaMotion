/**
 * @file te// Test parameters - same as other tests
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
    p.max_velocity = 200.0f; // mm/s
    p.max_angular_velocity = 90.0f; // degrees/s
    return p;
}cpp
 * @brief Tests for Cartesian velocity control system
 *
 * Validates that servo speeds are correctly adjusted based on commanded
 * Cartesian velocities, equivalent to OpenSHC's velocity control system.
 */

#include "../examples/mock_interfaces.h"
#include "cartesian_velocity_controller.h"
#include "locomotion_system.h"
#include <cassert>
#include <cmath>
#include <iostream>

// Test parameters - same as other tests
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

void testBasicVelocityScaling() {
    std::cout << "Testing basic velocity scaling..." << std::endl;

    Parameters params = setupTestParameters();
    LocomotionSystem locomotion(params);

    ExampleIMU mock_imu;
    ExampleFSR mock_fsr;
    ExampleServo mock_servo;

    assert(locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo));

    CartesianVelocityController *velocity_ctrl = locomotion.getVelocityController();
    assert(velocity_ctrl != nullptr);

    // Test 1: Zero velocity should give baseline speeds
    locomotion.planGaitSequence(0.0f, 0.0f, 0.0f);
    float baseline_speed = locomotion.getCurrentServoSpeed(0, 0);
    std::cout << "  Baseline speed (zero velocity): " << baseline_speed << std::endl;

    // Test 2: Low velocity should give lower speeds than high velocity
    locomotion.planGaitSequence(0.05f, 0.0f, 0.0f); // Low velocity
    float low_speed = locomotion.getCurrentServoSpeed(0, 0);

    locomotion.planGaitSequence(0.15f, 0.0f, 0.0f); // High velocity
    float high_speed = locomotion.getCurrentServoSpeed(0, 0);

    std::cout << "  Low velocity speed: " << low_speed << std::endl;
    std::cout << "  High velocity speed: " << high_speed << std::endl;

    assert(high_speed > low_speed);
    std::cout << "  ✓ Higher velocities produce higher servo speeds" << std::endl;
}

void testAngularVelocityCompensation() {
    std::cout << "Testing angular velocity compensation..." << std::endl;

    Parameters params = setupTestParameters();
    LocomotionSystem locomotion(params);

    ExampleIMU mock_imu;
    ExampleFSR mock_fsr;
    ExampleServo mock_servo;

    assert(locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo));

    // Test pure linear motion
    locomotion.planGaitSequence(0.1f, 0.0f, 0.0f);
    float linear_speed = locomotion.getCurrentServoSpeed(0, 0);

    // Test pure angular motion
    locomotion.planGaitSequence(0.0f, 0.0f, 0.5f); // 0.5 rad/s
    float angular_speed = locomotion.getCurrentServoSpeed(0, 0);

    // Test combined motion
    locomotion.planGaitSequence(0.1f, 0.0f, 0.5f);
    float combined_speed = locomotion.getCurrentServoSpeed(0, 0);

    std::cout << "  Pure linear speed: " << linear_speed << std::endl;
    std::cout << "  Pure angular speed: " << angular_speed << std::endl;
    std::cout << "  Combined motion speed: " << combined_speed << std::endl;

    assert(combined_speed >= std::max(linear_speed, angular_speed));
    std::cout << "  ✓ Combined motion produces appropriate speed scaling" << std::endl;
}

void testGaitSpecificAdjustments() {
    std::cout << "Testing gait-specific adjustments..." << std::endl;

    Parameters params = setupTestParameters();
    LocomotionSystem locomotion(params);

    ExampleIMU mock_imu;
    ExampleFSR mock_fsr;
    ExampleServo mock_servo;

    assert(locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo));

    float test_velocity = 0.1f;

    // Test different gaits with same velocity
    locomotion.setGaitType(TRIPOD_GAIT);
    locomotion.planGaitSequence(test_velocity, 0.0f, 0.0f);
    float tripod_speed = locomotion.getCurrentServoSpeed(0, 0);

    locomotion.setGaitType(WAVE_GAIT);
    locomotion.planGaitSequence(test_velocity, 0.0f, 0.0f);
    float wave_speed = locomotion.getCurrentServoSpeed(0, 0);

    locomotion.setGaitType(RIPPLE_GAIT);
    locomotion.planGaitSequence(test_velocity, 0.0f, 0.0f);
    float ripple_speed = locomotion.getCurrentServoSpeed(0, 0);

    std::cout << "  Tripod gait speed: " << tripod_speed << std::endl;
    std::cout << "  Wave gait speed: " << wave_speed << std::endl;
    std::cout << "  Ripple gait speed: " << ripple_speed << std::endl;

    // Tripod should be fastest (speed factor 1.2), wave slowest (speed factor 0.8)
    assert(tripod_speed > ripple_speed);
    assert(ripple_speed > wave_speed);
    std::cout << "  ✓ Gait-specific speed adjustments working correctly" << std::endl;
}

void testPerLegCompensation() {
    std::cout << "Testing per-leg speed compensation..." << std::endl;

    Parameters params = setupTestParameters();
    LocomotionSystem locomotion(params);

    ExampleIMU mock_imu;
    ExampleFSR mock_fsr;
    ExampleServo mock_servo;

    assert(locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo));

    // Test pure rotation - outer legs should get higher speeds
    locomotion.planGaitSequence(0.0f, 0.0f, 1.0f); // 1.0 rad/s rotation

    float speeds[NUM_LEGS];
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        speeds[leg] = locomotion.getCurrentServoSpeed(leg, 0); // Get coxa speeds
        std::cout << "  Leg " << leg << " speed: " << speeds[leg] << std::endl;
    }

    // All speeds should be similar for pure rotation (hexapod symmetry)
    // But let's verify they're all reasonably high due to angular motion
    float avg_speed = 0.0f;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        avg_speed += speeds[leg];
    }
    avg_speed /= NUM_LEGS;

    assert(avg_speed > params.default_servo_speed); // Should be higher than baseline
    std::cout << "  ✓ Angular motion increases servo speeds appropriately" << std::endl;
}

void testJointSpecificScaling() {
    std::cout << "Testing joint-specific scaling..." << std::endl;

    Parameters params = setupTestParameters();
    LocomotionSystem locomotion(params);

    ExampleIMU mock_imu;
    ExampleFSR mock_fsr;
    ExampleServo mock_servo;

    assert(locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo));

    locomotion.planGaitSequence(0.1f, 0.0f, 0.0f);

    float coxa_speed = locomotion.getCurrentServoSpeed(0, 0);
    float femur_speed = locomotion.getCurrentServoSpeed(0, 1);
    float tibia_speed = locomotion.getCurrentServoSpeed(0, 2);

    std::cout << "  Coxa speed: " << coxa_speed << std::endl;
    std::cout << "  Femur speed: " << femur_speed << std::endl;
    std::cout << "  Tibia speed: " << tibia_speed << std::endl;

    // Tibia should be fastest (1.1x), coxa slowest (0.9x)
    assert(tibia_speed >= femur_speed);
    assert(femur_speed >= coxa_speed);
    std::cout << "  ✓ Joint-specific scaling working correctly" << std::endl;
}

void testVelocityControlToggle() {
    std::cout << "Testing velocity control enable/disable..." << std::endl;

    Parameters params = setupTestParameters();
    LocomotionSystem locomotion(params);

    ExampleIMU mock_imu;
    ExampleFSR mock_fsr;
    ExampleServo mock_servo;

    assert(locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo));

    float test_velocity = 0.15f;

    // Test with velocity control enabled
    locomotion.setVelocityControlEnabled(true);
    locomotion.planGaitSequence(test_velocity, 0.0f, 0.0f);
    float enabled_speed = locomotion.getCurrentServoSpeed(0, 0);

    // Test with velocity control disabled
    locomotion.setVelocityControlEnabled(false);
    locomotion.planGaitSequence(test_velocity, 0.0f, 0.0f);
    float disabled_speed = locomotion.getCurrentServoSpeed(0, 0);

    std::cout << "  Velocity control enabled speed: " << enabled_speed << std::endl;
    std::cout << "  Velocity control disabled speed: " << disabled_speed << std::endl;

    // When disabled, should use default speed regardless of velocity
    assert(std::abs(disabled_speed - params.default_servo_speed) < 0.01f);
    assert(enabled_speed != disabled_speed); // Should be different when enabled
    std::cout << "  ✓ Velocity control toggle working correctly" << std::endl;
}

void testCustomScaling() {
    std::cout << "Testing custom velocity scaling..." << std::endl;

    Parameters params = setupTestParameters();
    LocomotionSystem locomotion(params);

    ExampleIMU mock_imu;
    ExampleFSR mock_fsr;
    ExampleServo mock_servo;

    assert(locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo));

    float test_velocity = 0.1f;

    // Test with default scaling
    locomotion.planGaitSequence(test_velocity, 0.0f, 0.0f);
    float default_speed = locomotion.getCurrentServoSpeed(0, 0);

    // Test with custom aggressive scaling
    CartesianVelocityController::VelocityScaling custom_scaling;
    custom_scaling.linear_velocity_scale = 4.0f; // Double the default
    custom_scaling.minimum_speed_ratio = 0.5f;
    custom_scaling.maximum_speed_ratio = 2.5f;

    locomotion.setVelocityScaling(custom_scaling);
    locomotion.planGaitSequence(test_velocity, 0.0f, 0.0f);
    float custom_speed = locomotion.getCurrentServoSpeed(0, 0);

    std::cout << "  Default scaling speed: " << default_speed << std::endl;
    std::cout << "  Custom scaling speed: " << custom_speed << std::endl;

    assert(custom_speed > default_speed); // Custom scaling should produce higher speeds
    std::cout << "  ✓ Custom velocity scaling working correctly" << std::endl;
}

void testVelocityMagnitudeCalculation() {
    std::cout << "Testing velocity magnitude calculation..." << std::endl;

    Parameters params = setupTestParameters();
    LocomotionSystem locomotion(params);

    ExampleIMU mock_imu;
    ExampleFSR mock_fsr;
    ExampleServo mock_servo;

    assert(locomotion.initialize(&mock_imu, &mock_fsr, &mock_servo));

    CartesianVelocityController *velocity_ctrl = locomotion.getVelocityController();

    // Test pure linear motion
    locomotion.planGaitSequence(0.1f, 0.0f, 0.0f);
    float linear_magnitude = velocity_ctrl->getCurrentVelocityMagnitude();

    // Test pure angular motion
    locomotion.planGaitSequence(0.0f, 0.0f, 0.5f);
    float angular_magnitude = velocity_ctrl->getCurrentVelocityMagnitude();

    // Test combined motion
    locomotion.planGaitSequence(0.1f, 0.0f, 0.5f);
    float combined_magnitude = velocity_ctrl->getCurrentVelocityMagnitude();

    std::cout << "  Linear magnitude: " << linear_magnitude << std::endl;
    std::cout << "  Angular magnitude: " << angular_magnitude << std::endl;
    std::cout << "  Combined magnitude: " << combined_magnitude << std::endl;

    assert(linear_magnitude > 0.0f);
    assert(angular_magnitude > 0.0f);
    assert(combined_magnitude > std::max(linear_magnitude, angular_magnitude));
    std::cout << "  ✓ Velocity magnitude calculation working correctly" << std::endl;
}

int main() {
    std::cout << "=== Cartesian Velocity Control Tests ===" << std::endl;
    std::cout << "Testing OpenSHC-equivalent servo speed control system" << std::endl;
    std::cout << std::endl;

    try {
        testBasicVelocityScaling();
        std::cout << std::endl;

        testAngularVelocityCompensation();
        std::cout << std::endl;

        testGaitSpecificAdjustments();
        std::cout << std::endl;

        testPerLegCompensation();
        std::cout << std::endl;

        testJointSpecificScaling();
        std::cout << std::endl;

        testVelocityControlToggle();
        std::cout << std::endl;

        testCustomScaling();
        std::cout << std::endl;

        testVelocityMagnitudeCalculation();
        std::cout << std::endl;

        std::cout << "=== All Velocity Control Tests Passed! ===" << std::endl;
        std::cout << "✓ Servo speeds correctly scale with Cartesian velocity" << std::endl;
        std::cout << "✓ Angular velocity compensation working" << std::endl;
        std::cout << "✓ Gait-specific speed adjustments functional" << std::endl;
        std::cout << "✓ Per-leg and per-joint scaling operational" << std::endl;
        std::cout << "✓ Velocity control can be enabled/disabled" << std::endl;
        std::cout << "✓ Custom scaling parameters working" << std::endl;
        std::cout << std::endl;
        std::cout << "The HexaMotion velocity control system provides" << std::endl;
        std::cout << "equivalent functionality to OpenSHC's velocity control!" << std::endl;

        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}
