#include "../include/locomotion_system.h"
#include "test_stubs.h"
#include <iomanip>
#include <iostream>

void testTripodGaitWithIKConfig(bool use_multiple_starts) {
    std::cout << "\n=========================================" << std::endl;
    std::cout << "TRIPOD GAIT TEST - IK Multiple Starts: " << (use_multiple_starts ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "=========================================" << std::endl;

    // Initialize robot parameters for main simulation (use 150mm height)
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 150;
    p.height_offset = 0;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    // Configure IK behavior
    p.ik.use_multiple_starts = use_multiple_starts;

    // Initialize locomotion system
    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    assert(sys.initialize(&imu, &fsr, &servos));
    assert(sys.calibrateSystem());
    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.walkForward(400.0f));

    // Track previous joint angles to detect changes
    JointAngles prev[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev[i] = sys.getJointAngles(i);
    }

    bool all_legs_moved = true;
    int movement_checks[NUM_LEGS] = {0}; // Track movement for each leg

    // Run simulation for 20 steps to check movement
    for (int s = 0; s < 20; ++s) {
        sys.update();

        // Check if angles changed for each leg
        for (int i = 0; i < NUM_LEGS; ++i) {
            JointAngles q = sys.getJointAngles(i);
            if (std::abs(q.coxa - prev[i].coxa) > 0.1f ||
                std::abs(q.femur - prev[i].femur) > 0.1f ||
                std::abs(q.tibia - prev[i].tibia) > 0.1f) {
                movement_checks[i]++;
            }
            prev[i] = q;
        }
    }

    // Check that all legs showed movement at some point
    std::cout << "Leg movement check:" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        bool leg_moved = movement_checks[i] > 0;
        std::cout << "  Leg " << (i + 1) << ": " << (leg_moved ? "MOVED" : "STATIC")
                  << " (" << movement_checks[i] << " movements detected)" << std::endl;
        if (!leg_moved) {
            all_legs_moved = false;
        }
    }

    std::cout << "\nResult: " << (all_legs_moved ? "✅ ALL LEGS MOVING" : "❌ SOME LEGS STATIC") << std::endl;
    std::cout << "IK Configuration: " << (use_multiple_starts ? "Multiple starts" : "Single start (CSIRO)") << std::endl;
}

int main() {
    std::cout << "=== TRIPOD GAIT VALIDATION WITH IK CONFIGURATION TEST ===" << std::endl;
    std::cout << "Verifying that tripod gait corrections work with both IK approaches" << std::endl;

    // Test with multiple starts enabled
    testTripodGaitWithIKConfig(true);

    // Test with multiple starts disabled (original CSIRO)
    testTripodGaitWithIKConfig(false);

    std::cout << "\n=========================================" << std::endl;
    std::cout << "VALIDATION COMPLETE" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Both IK configurations should maintain the tripod gait" << std::endl;
    std::cout << "corrections with all 6 legs participating actively." << std::endl;

    return 0;
}
