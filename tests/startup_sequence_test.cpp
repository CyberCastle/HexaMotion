#include "robot_model.h"
#include "../src/locomotion_system.h"
#include "../src/body_pose_config_factory.h"
#include "test_stubs.h"
#include <iostream>

int main() {
    std::cout << "=== Startup Sequence Test ===" << std::endl;

    // Initialize robot parameters
    Parameters p = createDefaultParameters();

    // Initialize locomotion system
    LocomotionSystem sys(p);

    // Create mock interfaces
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    // Create body pose configuration
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);

    std::cout << "Initializing locomotion system..." << std::endl;

    // Initialize the locomotion system
    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cout << "❌ ERROR: Failed to initialize locomotion system" << std::endl;
        return 1;
    }

    std::cout << "Setting standing pose..." << std::endl;

    // Set initial standing pose
    if (!sys.setStandingPose()) {
        std::cout << "❌ ERROR: Failed to set standing pose" << std::endl;
        return 1;
    }

    std::cout << "Testing executeStartupSequence()..." << std::endl;

    // Test startup sequence with a limit
    int startup_steps = 0;
    const int max_startup_steps = 1000;

    while (!sys.executeStartupSequence() && startup_steps < max_startup_steps) {
        startup_steps++;
        if (startup_steps % 10 == 0) {
            std::cout << "Startup step " << startup_steps << std::endl;
        }
    }

    if (startup_steps >= max_startup_steps) {
        std::cout << "❌ ERROR: Startup sequence took too long (" << startup_steps << " steps)" << std::endl;
        return 1;
    }

    std::cout << "✓ Startup sequence completed in " << startup_steps << " steps" << std::endl;

    std::cout << "Testing gait setup..." << std::endl;

    // Setup gait
    if (!sys.setGaitType(TRIPOD_GAIT)) {
        std::cout << "❌ ERROR: Failed to set gait type" << std::endl;
        return 1;
    }

    if (!sys.planGaitSequence(400.0, 0.0, 0.0)) {
        std::cout << "❌ ERROR: Failed to plan gait sequence" << std::endl;
        return 1;
    }

    std::cout << "✓ Gait setup completed successfully" << std::endl;

    std::cout << "Testing update() method..." << std::endl;

    // Test update method a few times
    for (int i = 0; i < 10; i++) {
        bool success = sys.update();
        if (!success) {
            std::cout << "❌ ERROR: Update failed at step " << i + 1 << std::endl;
            return 1;
        }
        std::cout << "Update " << i + 1 << "/10 completed" << std::endl;
    }

    std::cout << "✅ SUCCESS: All tests passed" << std::endl;
    return 0;
}