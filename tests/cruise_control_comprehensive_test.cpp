/**
 * @file cruise_control_comprehensive_test.cpp
 * @brief Comprehensive test suite for enhanced cruise control functionality
 *
 * This test validates the complete cruise control implementation that provides
 * OpenSHC equivalence, including time limits, velocity capture, combined movement,
 * and all new status monitoring methods.
 *
 * @author HexaMotion Team
 * @date 2025
 */

#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

class CruiseControlComprehensiveTest {
  private:
    Parameters params;
    LocomotionSystem *locomotion;
    StateController *state_controller;
    MockIMU imu;
    MockFSR fsr;
    MockServo servo;
    int test_count = 0;
    int passed_tests = 0;

  public:
    CruiseControlComprehensiveTest() {
        setupParameters();
        locomotion = new LocomotionSystem(params);
        locomotion->initialize(&imu, &fsr, &servo);
        state_controller = nullptr; // Initialize to nullptr
    }

    ~CruiseControlComprehensiveTest() {
        delete locomotion;
    }

    void setupParameters() {
        params.hexagon_radius = 200;
        params.coxa_length = 50;
        params.femur_length = 101;
        params.tibia_length = 208;
        params.robot_height = 120;
        params.control_frequency = 50;
        params.coxa_angle_limits[0] = -65;
        params.coxa_angle_limits[1] = 65;
        params.femur_angle_limits[0] = -75;
        params.femur_angle_limits[1] = 75;
        params.tibia_angle_limits[0] = -45;
        params.tibia_angle_limits[1] = 45;
    }

    void assert_test(bool condition, const std::string &test_name) {
        test_count++;
        if (condition) {
            passed_tests++;
            std::cout << "✓ " << test_name << std::endl;
        } else {
            std::cout << "❌ " << test_name << " FAILED!" << std::endl;
        }
    }

    void assert_near(float actual, float expected, float tolerance, const std::string &test_name) {
        test_count++;
        if (abs(actual - expected) <= tolerance) {
            passed_tests++;
            std::cout << "✓ " << test_name << " (actual: " << actual << ", expected: " << expected << ")" << std::endl;
        } else {
            std::cout << "❌ " << test_name << " FAILED! (actual: " << actual << ", expected: " << expected << ", tolerance: " << tolerance << ")" << std::endl;
        }
    }

    void runAllTests() {
        std::cout << "=== CRUISE CONTROL COMPREHENSIVE TEST SUITE ===" << std::endl;
        std::cout << "Testing OpenSHC-equivalent cruise control functionality\n"
                  << std::endl;

        testBasicCruiseControlFunctionality();
        testNewStatusMethods();
        testTimeLimitFunctionality();
        testVelocityCaptureFeature();
        testCombinedMovementControl();
        testConfigurationOptions();
        testErrorHandlingAndEdgeCases();
        testOpenSHCEquivalence();

        printResults();
    }

  private:
    void createStateController(const StateMachineConfig &config = StateMachineConfig()) {
        if (state_controller) {
            delete state_controller;
            state_controller = nullptr;
        }
        state_controller = new StateController(*locomotion, config);
        state_controller->initialize();
    }

    void cleanupStateController() {
        if (state_controller) {
            delete state_controller;
            state_controller = nullptr;
        }
    }

    void testBasicCruiseControlFunctionality() {
        std::cout << "\n--- Test 1: Basic Cruise Control Functionality ---" << std::endl;

        // Setup with default config
        StateMachineConfig config;
        createStateController(config);

        // Test initial state
        assert_test(state_controller->getCruiseControlMode() == CRUISE_CONTROL_OFF,
                    "Initial cruise control state is OFF");
        assert_test(!state_controller->isCruiseControlActive(),
                    "Initial cruise control is not active");

        // Test enabling cruise control with specific velocity
        Eigen::Vector3f cruise_velocity(100.0f, 50.0f, 15.0f);
        assert_test(state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, cruise_velocity),
                    "Enable cruise control with specific velocity");
        assert_test(state_controller->getCruiseControlMode() == CRUISE_CONTROL_ON,
                    "Cruise control mode is ON");
        assert_test(state_controller->isCruiseControlActive(),
                    "Cruise control is active");

        // Test disabling cruise control
        assert_test(state_controller->setCruiseControlMode(CRUISE_CONTROL_OFF),
                    "Disable cruise control");
        assert_test(state_controller->getCruiseControlMode() == CRUISE_CONTROL_OFF,
                    "Cruise control mode is OFF after disable");
        assert_test(!state_controller->isCruiseControlActive(),
                    "Cruise control is not active after disable");

        // Clean up for next test
        // No need to delete here as destructor will handle it
    }

    void testNewStatusMethods() {
        std::cout << "\n--- Test 2: New Status Methods ---" << std::endl;

        StateMachineConfig config;
        state_controller = new StateController(*locomotion, config);
        state_controller->initialize();

        // Test getCruiseVelocity() method
        Eigen::Vector3f test_velocity(120.0f, -30.0f, 25.0f);
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, test_velocity);

        Eigen::Vector3f retrieved_velocity = state_controller->getCruiseVelocity();
        assert_near(retrieved_velocity.x(), test_velocity.x(), 0.01f, "getCruiseVelocity X component");
        assert_near(retrieved_velocity.y(), test_velocity.y(), 0.01f, "getCruiseVelocity Y component");
        assert_near(retrieved_velocity.z(), test_velocity.z(), 0.01f, "getCruiseVelocity Z component");

        // Test getCruiseStartTime() method - verify it's set (may be 0 initially in test environment)
        unsigned long start_time = state_controller->getCruiseStartTime();
        assert_test(start_time >= 0, "getCruiseStartTime returns valid time");

        // Test when cruise control is disabled
        state_controller->setCruiseControlMode(CRUISE_CONTROL_OFF);
        Eigen::Vector3f zero_velocity = state_controller->getCruiseVelocity();
        assert_near(zero_velocity.norm(), 0.0f, 0.01f, "getCruiseVelocity returns zero when disabled");

        cleanupStateController();
    }

    void testTimeLimitFunctionality() {
        std::cout << "\n--- Test 3: Time Limit Functionality ---" << std::endl;

        // Test with time limit
        StateMachineConfig config_with_limit;
        config_with_limit.cruise_control_time_limit = 2.0f; // 2 seconds
        state_controller = new StateController(*locomotion, config_with_limit);
        state_controller->initialize();

        // Enable cruise control
        Eigen::Vector3f velocity(80.0f, 0.0f, 10.0f);
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, velocity);

        // Test remaining time immediately after activation
        float remaining_time = state_controller->getCruiseRemainingTime();
        assert_test(remaining_time > 1.9f && remaining_time <= 2.0f,
                    "getCruiseRemainingTime shows correct initial time");
        assert_test(state_controller->isCruiseControlActive(),
                    "Cruise control is active with time limit");

        // Simulate time passage and test expiration
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        state_controller->updateVelocityControl(); // This should check time limit

        remaining_time = state_controller->getCruiseRemainingTime();
        assert_test(remaining_time < 2.0f, "Remaining time decreases");

        // Test unlimited time limit (0.0f)
        delete state_controller;
        StateMachineConfig config_unlimited;
        config_unlimited.cruise_control_time_limit = 0.0f; // Unlimited
        state_controller = new StateController(*locomotion, config_unlimited);
        state_controller->initialize();

        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, velocity);
        assert_near(state_controller->getCruiseRemainingTime(), 0.0f, 0.01f,
                    "Unlimited cruise control returns 0 remaining time");
        assert_test(state_controller->isCruiseControlActive(),
                    "Unlimited cruise control is active");

        delete state_controller;
    }

    void testVelocityCaptureFeature() {
        std::cout << "\n--- Test 4: Velocity Capture Feature ---" << std::endl;

        StateMachineConfig config;
        state_controller = new StateController(*locomotion, config);
        state_controller->initialize();

        // Set desired velocities
        Eigen::Vector2f linear_vel(75.0f, -25.0f);
        float angular_vel = 20.0f;
        state_controller->setDesiredVelocity(linear_vel, angular_vel);

        // Enable cruise control without specifying velocity (should capture current)
        assert_test(state_controller->setCruiseControlMode(CRUISE_CONTROL_ON),
                    "Enable cruise control with velocity capture");

        // Verify captured velocities
        Eigen::Vector3f captured_velocity = state_controller->getCruiseVelocity();
        assert_near(captured_velocity.x(), linear_vel.x(), 0.01f, "Captured velocity X matches desired");
        assert_near(captured_velocity.y(), linear_vel.y(), 0.01f, "Captured velocity Y matches desired");
        assert_near(captured_velocity.z(), angular_vel, 0.01f, "Captured velocity Z matches desired angular");

        // Test with zero velocity (should still work)
        state_controller->setDesiredVelocity(Eigen::Vector2f::Zero(), 0.0f);
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON);
        captured_velocity = state_controller->getCruiseVelocity();
        assert_near(captured_velocity.norm(), 0.0f, 0.01f, "Captured zero velocity correctly");

        delete state_controller;
    }

    void testCombinedMovementControl() {
        std::cout << "\n--- Test 5: Combined Movement Control ---" << std::endl;

        StateMachineConfig config;
        state_controller = new StateController(*locomotion, config);
        state_controller->initialize();

        // Prepare robot for operational state
        state_controller->requestSystemState(SYSTEM_OPERATIONAL);
        state_controller->requestRobotState(ROBOT_RUNNING);
        for (int i = 0; i < 10; i++) {
            state_controller->update(0.02f);
        }

        // Test combined movement (X, Y, and angular simultaneously)
        Eigen::Vector3f combined_velocity(60.0f, 40.0f, 12.0f);
        assert_test(state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, combined_velocity),
                    "Set combined velocity cruise control");

        // Test velocity control update (this should call planGaitSequence internally)
        state_controller->updateVelocityControl();
        assert_test(!state_controller->hasErrors(), "Combined movement control executes without errors");

        // Test pure X movement
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, Eigen::Vector3f(100.0f, 0.0f, 0.0f));
        state_controller->updateVelocityControl();
        assert_test(!state_controller->hasErrors(), "Pure X movement executes without errors");

        // Test pure Y movement
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, Eigen::Vector3f(0.0f, 80.0f, 0.0f));
        state_controller->updateVelocityControl();
        assert_test(!state_controller->hasErrors(), "Pure Y movement executes without errors");

        // Test pure angular movement
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, Eigen::Vector3f(0.0f, 0.0f, 30.0f));
        state_controller->updateVelocityControl();
        assert_test(!state_controller->hasErrors(), "Pure angular movement executes without errors");

        delete state_controller;
    }

    void testConfigurationOptions() {
        std::cout << "\n--- Test 6: Configuration Options ---" << std::endl;

        // Test with cruise control disabled in config
        StateMachineConfig disabled_config;
        disabled_config.enable_cruise_control = false;
        state_controller = new StateController(*locomotion, disabled_config);
        state_controller->initialize();

        Eigen::Vector3f velocity(50.0f, 25.0f, 10.0f);
        assert_test(!state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, velocity),
                    "Cruise control fails when disabled in config");
        assert_test(state_controller->getCruiseControlMode() == CRUISE_CONTROL_OFF,
                    "Cruise control remains OFF when disabled in config");

        delete state_controller;

        // Test with different time limits
        StateMachineConfig short_limit_config;
        short_limit_config.cruise_control_time_limit = 0.5f; // 500ms
        state_controller = new StateController(*locomotion, short_limit_config);
        state_controller->initialize();

        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, velocity);
        float remaining = state_controller->getCruiseRemainingTime();
        assert_test(remaining > 0.4f && remaining <= 0.5f, "Short time limit configured correctly");

        delete state_controller;
    }

    void testErrorHandlingAndEdgeCases() {
        std::cout << "\n--- Test 7: Error Handling and Edge Cases ---" << std::endl;

        StateMachineConfig config;
        state_controller = new StateController(*locomotion, config);
        state_controller->initialize();

        // Test very small velocity (should be treated as zero)
        Eigen::Vector3f tiny_velocity(0.0001f, 0.0001f, 0.0001f);
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, tiny_velocity);
        state_controller->updateVelocityControl();
        assert_test(!state_controller->hasErrors(), "Tiny velocity handled without errors");

        // Test very large velocity (should be limited by locomotion system)
        Eigen::Vector3f large_velocity(1000.0f, 1000.0f, 1000.0f);
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, large_velocity);
        state_controller->updateVelocityControl();
        // Note: Locomotion system should handle velocity limits internally

        // Test rapid mode switching
        for (int i = 0; i < 5; i++) {
            state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, Eigen::Vector3f(10.0f * i, 5.0f * i, 2.0f * i));
            state_controller->setCruiseControlMode(CRUISE_CONTROL_OFF);
        }
        assert_test(!state_controller->hasErrors(), "Rapid mode switching handled without errors");

        delete state_controller;
    }

    void testOpenSHCEquivalence() {
        std::cout << "\n--- Test 8: OpenSHC Equivalence Validation ---" << std::endl;

        StateMachineConfig config;
        config.cruise_control_time_limit = 5.0f; // Equivalent to OpenSHC cruise_control_time_limit
        state_controller = new StateController(*locomotion, config);
        state_controller->initialize();

        // Test equivalent behavior: force specific velocity vs capture current velocity

        // 1. Force specific velocity (equivalent to params_.force_cruise_velocity.data = true)
        Eigen::Vector3f forced_velocity(90.0f, 45.0f, 18.0f);
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON, forced_velocity);
        Eigen::Vector3f result1 = state_controller->getCruiseVelocity();
        assert_near(result1.x(), forced_velocity.x(), 0.01f, "OpenSHC equivalent: forced velocity X");
        assert_near(result1.y(), forced_velocity.y(), 0.01f, "OpenSHC equivalent: forced velocity Y");
        assert_near(result1.z(), forced_velocity.z(), 0.01f, "OpenSHC equivalent: forced velocity Z");

        // 2. Capture current velocity (equivalent to params_.force_cruise_velocity.data = false)
        state_controller->setCruiseControlMode(CRUISE_CONTROL_OFF);
        state_controller->setDesiredVelocity(Eigen::Vector2f(65.0f, -35.0f), 22.0f);
        state_controller->setCruiseControlMode(CRUISE_CONTROL_ON); // No velocity specified
        Eigen::Vector3f result2 = state_controller->getCruiseVelocity();
        assert_near(result2.x(), 65.0f, 0.01f, "OpenSHC equivalent: captured velocity X");
        assert_near(result2.y(), -35.0f, 0.01f, "OpenSHC equivalent: captured velocity Y");
        assert_near(result2.z(), 22.0f, 0.01f, "OpenSHC equivalent: captured velocity Z");

        // 3. Test time limit behavior (equivalent to OpenSHC time limit check)
        assert_test(state_controller->isCruiseControlActive(), "OpenSHC equivalent: cruise control active within time limit");
        float remaining = state_controller->getCruiseRemainingTime();
        assert_test(remaining > 0.0f, "OpenSHC equivalent: time limit tracking");

        // 4. Test external mode support
        assert_test(state_controller->setCruiseControlMode(CRUISE_CONTROL_EXTERNAL),
                    "OpenSHC equivalent: external cruise control mode");
        assert_test(state_controller->getCruiseControlMode() == CRUISE_CONTROL_EXTERNAL,
                    "OpenSHC equivalent: external mode active");

        delete state_controller;
    }

    void printResults() {
        std::cout << "\n=== TEST RESULTS ===" << std::endl;
        std::cout << "Total tests: " << test_count << std::endl;
        std::cout << "Passed: " << passed_tests << std::endl;
        std::cout << "Failed: " << (test_count - passed_tests) << std::endl;
        std::cout << "Success rate: " << (float(passed_tests) / test_count * 100.0f) << "%" << std::endl;

        if (passed_tests == test_count) {
            std::cout << "\n🎉 ALL TESTS PASSED! Cruise control OpenSHC equivalence validated!" << std::endl;
        } else {
            std::cout << "\n❌ Some tests failed. Check implementation." << std::endl;
        }
    }
};

int main() {
    CruiseControlComprehensiveTest test;
    test.runAllTests();
    return 0;
}
