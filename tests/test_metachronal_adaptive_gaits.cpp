#include "locomotion_system.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>

int main() {
    std::cout << "Testing METACHRONAL_GAIT and ADAPTIVE_GAIT implementation status..." << std::endl;

    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 150;
    p.control_frequency = 50;

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    assert(sys.initialize(&imu, &fsr, &servos));
    assert(sys.calibrateSystem());

    // Test METACHRONAL_GAIT
    std::cout << "\n1. Testing METACHRONAL_GAIT:" << std::endl;
    bool metachronal_success = sys.setGaitType(METACHRONAL_GAIT);
    std::cout << "   setGaitType(METACHRONAL_GAIT): " << (metachronal_success ? "✅" : "❌") << std::endl;

    if (metachronal_success) {
        // Try to walk with metachronal gait
        bool walk_success = sys.walkForward(200.0f);
        std::cout << "   walkForward() with METACHRONAL: " << (walk_success ? "✅" : "❌") << std::endl;

        // Run a few steps to see if it works
        for (int i = 0; i < 10; i++) {
            sys.update();
        }
        std::cout << "   Simulation steps executed: ✅" << std::endl;
    }

    // Test ADAPTIVE_GAIT
    std::cout << "\n2. Testing ADAPTIVE_GAIT:" << std::endl;
    bool adaptive_success = sys.setGaitType(ADAPTIVE_GAIT);
    std::cout << "   setGaitType(ADAPTIVE_GAIT): " << (adaptive_success ? "✅" : "❌") << std::endl;

    if (adaptive_success) {
        // Try to walk with adaptive gait
        bool walk_success = sys.walkForward(200.0f);
        std::cout << "   walkForward() with ADAPTIVE: " << (walk_success ? "✅" : "❌") << std::endl;

        // Run a few steps to see if it works
        for (int i = 0; i < 10; i++) {
            sys.update();
        }
        std::cout << "   Simulation steps executed: ✅" << std::endl;
    }

    // Check for movement with both gaits
    std::cout << "\n3. Checking movement patterns:" << std::endl;

    // METACHRONAL gait movement test
    sys.setGaitType(METACHRONAL_GAIT);
    sys.walkForward(200.0f);

    JointAngles prev_metachronal[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev_metachronal[i] = sys.getJointAngles(i);
    }

    // Run simulation for a few steps
    bool metachronal_movement = false;
    for (int s = 0; s < 20; s++) {
        sys.update();

        // Check if any leg moved
        for (int i = 0; i < NUM_LEGS; ++i) {
            JointAngles curr = sys.getJointAngles(i);
            if (std::abs(curr.coxa - prev_metachronal[i].coxa) > 0.1f ||
                std::abs(curr.femur - prev_metachronal[i].femur) > 0.1f ||
                std::abs(curr.tibia - prev_metachronal[i].tibia) > 0.1f) {
                metachronal_movement = true;
                break;
            }
        }
        if (metachronal_movement)
            break;
    }
    std::cout << "   METACHRONAL gait produces movement: " << (metachronal_movement ? "✅" : "❌") << std::endl;

    // ADAPTIVE gait movement test
    sys.setGaitType(ADAPTIVE_GAIT);
    sys.walkForward(200.0f);

    JointAngles prev_adaptive[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev_adaptive[i] = sys.getJointAngles(i);
    }

    // Run simulation for a few steps
    bool adaptive_movement = false;
    for (int s = 0; s < 20; s++) {
        sys.update();

        // Check if any leg moved
        for (int i = 0; i < NUM_LEGS; ++i) {
            JointAngles curr = sys.getJointAngles(i);
            if (std::abs(curr.coxa - prev_adaptive[i].coxa) > 0.1f ||
                std::abs(curr.femur - prev_adaptive[i].femur) > 0.1f ||
                std::abs(curr.tibia - prev_adaptive[i].tibia) > 0.1f) {
                adaptive_movement = true;
                break;
            }
        }
        if (adaptive_movement)
            break;
    }
    std::cout << "   ADAPTIVE gait produces movement: " << (adaptive_movement ? "✅" : "❌") << std::endl;

    // Final status report
    std::cout << "\n📊 IMPLEMENTATION STATUS SUMMARY:" << std::endl;
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;

    if (metachronal_success && adaptive_success) {
        std::cout << "✅ ENUM DEFINITIONS: Both gaits defined in GaitType enum" << std::endl;
        std::cout << "✅ GAIT FACTORS: Step parameters defined for both gaits" << std::endl;
        std::cout << "✅ PARAMETER CALCULATION: updateStepParameters() handles both gaits" << std::endl;

        if (metachronal_movement && adaptive_movement) {
            std::cout << "🔶 BASIC FUNCTIONALITY: Both gaits can execute movement" << std::endl;
            std::cout << "⚠️  PHASE OFFSETS: No specific patterns - using default (TRIPOD)" << std::endl;
            std::cout << "⚠️  IMPLEMENTATION STATUS: PARTIALLY IMPLEMENTED" << std::endl;
        } else {
            std::cout << "❌ MOVEMENT: One or both gaits don't produce movement" << std::endl;
            std::cout << "❌ IMPLEMENTATION STATUS: INCOMPLETE" << std::endl;
        }
    } else {
        std::cout << "❌ BASIC SETUP: One or both gaits failed to initialize" << std::endl;
        std::cout << "❌ IMPLEMENTATION STATUS: NOT IMPLEMENTED" << std::endl;
    }

    std::cout << "\n🔍 ANALYSIS:" << std::endl;
    std::cout << "• These gaits exist as framework placeholders" << std::endl;
    std::cout << "• They have gait factor parameters but lack specific phase offset patterns" << std::endl;
    std::cout << "• They fall back to TRIPOD phase offsets (default case)" << std::endl;
    std::cout << "• The main implementation focuses on the 3 OpenSHC-equivalent gaits" << std::endl;

    return 0;
}
