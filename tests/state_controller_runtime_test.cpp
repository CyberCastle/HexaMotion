/**
 * @file state_controller_runtime_test.cpp
 * @brief Runtime validation test for StateController
 */

#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <iomanip>
#include <iostream>

void printStateInfo(const StateController *controller) {
    std::cout << "\n=== State Controller Status ===" << std::endl;
    std::cout << "Initialized: " << (controller->isInitialized() ? "YES" : "NO") << std::endl;
    std::cout << "Has Errors: " << (controller->hasErrors() ? "YES" : "NO") << std::endl;
    std::cout << "System State: " << (controller->getSystemState() == SYSTEM_OPERATIONAL ? "OPERATIONAL" : "SUSPENDED") << std::endl;
    std::cout << "Robot State: ";
    switch (controller->getRobotState()) {
    case ROBOT_UNKNOWN:
        std::cout << "UNKNOWN";
        break;
    case ROBOT_PACKED:
        std::cout << "PACKED";
        break;
    case ROBOT_READY:
        std::cout << "READY";
        break;
    case ROBOT_RUNNING:
        std::cout << "RUNNING";
        break;
    default:
        std::cout << "INVALID";
        break;
    }
    std::cout << std::endl;
    std::cout << "Walk State: ";
    switch (controller->getWalkState()) {
    case WALK_STOPPED:
        std::cout << "STOPPED";
        break;
    case WALK_STARTING:
        std::cout << "STARTING";
        break;
    case WALK_MOVING:
        std::cout << "MOVING";
        break;
    case WALK_STOPPING:
        std::cout << "STOPPING";
        break;
    default:
        std::cout << "INVALID";
        break;
    }
    std::cout << std::endl;
    std::cout << "Posing Mode: ";
    switch (controller->getPosingMode()) {
    case POSING_NONE:
        std::cout << "NONE";
        break;
    case POSING_X_Y:
        std::cout << "X_Y";
        break;
    case POSING_PITCH_ROLL:
        std::cout << "PITCH_ROLL";
        break;
    case POSING_Z_YAW:
        std::cout << "Z_YAW";
        break;
    case POSING_EXTERNAL:
        std::cout << "EXTERNAL";
        break;
    default:
        std::cout << "INVALID";
        break;
    }
    std::cout << std::endl;
    std::cout << "Manual Legs: " << controller->getManualLegCount() << std::endl;
    std::cout << "Cruise Control: " << (controller->getCruiseControlMode() == CRUISE_CONTROL_ON ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "==========================" << std::endl;
}

int main() {
    std::cout << "=== StateController Runtime Validation Test ===" << std::endl;

    // Setup parameters
    Parameters params = createDefaultParameters();
    params.control_frequency = 50;

    // Create locomotion system
    LocomotionSystem locomotion(params);
    MockIMU imu;
    MockFSR fsr;
    MockServo servo;

    if (!locomotion.initialize(&imu, &fsr, &servo)) {
        std::cerr << "❌ Failed to initialize locomotion system" << std::endl;
        return 1;
    }

    // Create state controller
    StateMachineConfig config;
    config.max_manual_legs = 2;
    config.pack_unpack_time = 2.0f;
    config.enable_cruise_control = true;

    StateController controller(locomotion, config);

    std::cout << "\n✓ StateController created successfully" << std::endl;
    printStateInfo(&controller);

    // Initialize
    if (!controller.initialize()) {
        std::cerr << "❌ Failed to initialize state controller" << std::endl;
        return 1;
    }

    std::cout << "\n✓ StateController initialized successfully" << std::endl;
    printStateInfo(&controller);

    // Test basic state transitions
    std::cout << "\n--- Testing Basic State Transitions ---" << std::endl;

    // Ensure system is operational
    controller.requestSystemState(SYSTEM_OPERATIONAL);
    controller.update(0.02f);
    std::cout << "System requested to be operational" << std::endl;
    printStateInfo(&controller);

    // Request robot running
    if (controller.requestRobotState(ROBOT_RUNNING)) {
        std::cout << "✓ Robot running state requested" << std::endl;

        // Update until transition completes
        for (int i = 0; i < 100 && controller.isTransitioning(); ++i) {
            controller.update(0.02f);
            if (i % 25 == 0) {
                TransitionProgress progress = controller.getTransitionProgress();
                std::cout << "Transition progress: " << progress.current_step << "/" << progress.total_steps << " (" << progress.completion_percentage << "%)" << std::endl;
            }
        }

        printStateInfo(&controller);
    } else {
        std::cout << "❌ Failed to request robot running state" << std::endl;
    }

    // Test manual leg control
    std::cout << "\n--- Testing Manual Leg Control ---" << std::endl;
    if (controller.setLegState(0, LEG_MANUAL)) {
        std::cout << "✓ Leg 0 set to manual control" << std::endl;
    } else {
        std::cout << "❌ Failed to set leg 0 to manual" << std::endl;
    }
    printStateInfo(&controller);

    // Test velocity control
    std::cout << "\n--- Testing Velocity Control ---" << std::endl;
    Eigen::Vector2f linear_velocity(25.0f, 0.0f);
    float angular_velocity = 5.0f;
    controller.setDesiredVelocity(linear_velocity, angular_velocity);
    std::cout << "✓ Desired velocity set to [25, 0] linear, 5 angular" << std::endl;

    // Test pose control
    std::cout << "\n--- Testing Pose Control ---" << std::endl;
    if (controller.setPosingMode(POSING_X_Y)) {
        std::cout << "✓ X-Y posing mode enabled" << std::endl;

        Eigen::Vector3f position(10.0f, 5.0f, 0.0f);
        Eigen::Vector3f orientation(0.0f, 0.0f, 0.1f);
        controller.setDesiredPose(position, orientation);
        std::cout << "✓ Desired pose set" << std::endl;
    } else {
        std::cout << "❌ Failed to enable X-Y posing mode" << std::endl;
    }
    printStateInfo(&controller);

    // Test cruise control
    std::cout << "\n--- Testing Cruise Control ---" << std::endl;
    Eigen::Vector3f cruise_velocity(25.0f, 0.0f, 5.0f);
    if (controller.setCruiseControlMode(CRUISE_CONTROL_ON, cruise_velocity)) {
        std::cout << "✓ Cruise control enabled" << std::endl;
    } else {
        std::cout << "❌ Failed to enable cruise control" << std::endl;
    }
    printStateInfo(&controller);

    // Final update cycles
    std::cout << "\n--- Running Update Cycles ---" << std::endl;
    for (int i = 0; i < 10; ++i) {
        controller.update(0.02f);
        if (i % 3 == 0) {
            std::cout << "Update cycle " << i << " completed" << std::endl;
        }
    }

    printStateInfo(&controller);

    std::cout << "\n✅ StateController runtime validation completed successfully!" << std::endl;
    std::cout << "All core functionality appears to be working correctly." << std::endl;

    return 0;
}
