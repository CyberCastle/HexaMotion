#include "robot_model.h"
#include "../src/leg_poser.h"
#include "../src/leg.h"
#include "test_stubs.h"
#include <iostream>

int main() {
    std::cout << "=== Debug LegPoser Test ===" << std::endl;

    // Initialize robot parameters
    Parameters p = createDefaultParameters();
    RobotModel model(p);

    // Create a leg
    Pose default_stance(Point3D(0, 0, -p.robot_height), Eigen::Vector3d(0, 0, 0));
    Leg leg(0, model);
    leg.initialize(model, default_stance);

    // Create LegPoser
    LegPoser poser(0, leg, model);

    std::cout << "LegPoser created successfully" << std::endl;

    // Test stepToPosition with a simple movement
    Point3D target_position(50, 0, -p.robot_height); // Move 50mm in X direction
    double step_height = 20.0; // 20mm lift height
    double step_time = 1.0; // 1 second

    std::cout << "Starting stepToPosition test..." << std::endl;
    std::cout << "Target position: (" << target_position.x << ", " << target_position.y << ", " << target_position.z << ")" << std::endl;
    std::cout << "Step height: " << step_height << "mm" << std::endl;
    std::cout << "Step time: " << step_time << "s" << std::endl;

    int step_count = 0;
    const int max_steps = 100;

    while (!poser.stepToPosition(target_position, step_height, step_time) && step_count < max_steps) {
        step_count++;
        if (step_count % 10 == 0) {
            Point3D current_pos = poser.getCurrentPosition();
            std::cout << "Step " << step_count << ": Current position = ("
                      << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << ")" << std::endl;
        }
    }

    if (step_count >= max_steps) {
        std::cout << "❌ ERROR: stepToPosition took too long (" << step_count << " steps)" << std::endl;
        return 1;
    }

    std::cout << "✓ stepToPosition completed in " << step_count << " steps" << std::endl;

    Point3D final_pos = poser.getCurrentPosition();
    std::cout << "Final position: (" << final_pos.x << ", " << final_pos.y << ", " << final_pos.z << ")" << std::endl;

    std::cout << "✅ SUCCESS: LegPoser test passed" << std::endl;
    return 0;
}