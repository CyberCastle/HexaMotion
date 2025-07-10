#include "robot_model.h"
#include "../src/body_pose_controller.h"
#include "../src/body_pose_config_factory.h"
#include "../src/leg.h"
#include "test_stubs.h"
#include <iostream>

int main() {
    std::cout << "=== Debug stepToNewStance Test ===" << std::endl;

    // Initialize robot parameters
    Parameters p = createDefaultParameters();
    RobotModel model(p);

    // Create legs
    std::vector<Leg> legs;
    Pose default_stance(Point3D(0, 0, -p.robot_height), Eigen::Vector3d(0, 0, 0));
    for (int i = 0; i < NUM_LEGS; i++) {
        legs.emplace_back(i, model);
        legs[i].initialize(model, default_stance);
    }

    // Create body pose configuration
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);

    // Create BodyPoseController
    BodyPoseController controller(model, pose_config);
    // Convert vector to array for API compatibility
    controller.initializeLegPosers(legs.data());

    std::cout << "BodyPoseController created and initialized" << std::endl;

    // Test stepToNewStance
    double step_height = 20.0;
    double step_time = 1.0;

    std::cout << "Testing stepToNewStance..." << std::endl;
    std::cout << "Step height: " << step_height << "mm" << std::endl;
    std::cout << "Step time: " << step_time << "s" << std::endl;

    int step_count = 0;
    const int max_steps = 200;

    while (!controller.stepToNewStance(legs.data(), step_height, step_time) && step_count < max_steps) {
        step_count++;
        if (step_count % 10 == 0) {
            std::cout << "Step " << step_count << ": stepToNewStance still in progress" << std::endl;

            // Print current positions of all legs
            for (int i = 0; i < NUM_LEGS; i++) {
                Point3D pos = legs[i].getCurrentTipPositionGlobal();
                std::cout << "  Leg " << i << ": (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
            }
        }
    }

    if (step_count >= max_steps) {
        std::cout << "❌ ERROR: stepToNewStance took too long (" << step_count << " steps)" << std::endl;
        return 1;
    }

    std::cout << "✓ stepToNewStance completed in " << step_count << " steps" << std::endl;

    // Print final positions
    std::cout << "Final leg positions:" << std::endl;
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D pos = legs[i].getCurrentTipPositionGlobal();
        std::cout << "  Leg " << i << ": (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }

    std::cout << "✅ SUCCESS: stepToNewStance test passed" << std::endl;
    return 0;
}