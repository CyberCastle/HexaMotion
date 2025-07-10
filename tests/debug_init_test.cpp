#include "robot_model.h"
#include "../src/locomotion_system.h"
#include "../src/body_pose_config_factory.h"
#include "test_stubs.h"
#include <iostream>

int main() {
    std::cout << "=== Debug Initialization Test ===" << std::endl;

    std::cout << "Step 1: Creating parameters..." << std::endl;
    Parameters p = createDefaultParameters();
    std::cout << "✓ Parameters created" << std::endl;

    std::cout << "Step 2: Creating locomotion system..." << std::endl;
    LocomotionSystem sys(p);
    std::cout << "✓ LocomotionSystem created" << std::endl;

    std::cout << "Step 3: Creating mock interfaces..." << std::endl;
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    std::cout << "✓ Mock interfaces created" << std::endl;

    std::cout << "Step 4: Creating body pose configuration..." << std::endl;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    std::cout << "✓ Body pose configuration created" << std::endl;

    std::cout << "Step 5: Initializing locomotion system..." << std::endl;
    std::cout << "  - About to call sys.initialize()..." << std::endl;

    bool init_result = sys.initialize(&imu, &fsr, &servos, pose_config);

    if (!init_result) {
        std::cout << "❌ ERROR: Failed to initialize locomotion system" << std::endl;
        return 1;
    }

    std::cout << "✓ LocomotionSystem initialized successfully" << std::endl;

    std::cout << "Step 6: Setting standing pose..." << std::endl;
    bool pose_result = sys.setStandingPose();

    if (!pose_result) {
        std::cout << "❌ ERROR: Failed to set standing pose" << std::endl;
        return 1;
    }

    std::cout << "✓ Standing pose set successfully" << std::endl;

    std::cout << "✅ SUCCESS: All initialization steps completed" << std::endl;
    return 0;
}