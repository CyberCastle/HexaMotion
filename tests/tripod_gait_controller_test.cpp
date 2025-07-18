#include "tripod_gait_controller.h"
#include "analytic_robot_model.h"
#include "test_stubs.h"
#include <iomanip>
#include <iostream>

int main() {
    Parameters params = createDefaultParameters();
    TripodGaitController controller(params);
    AnalyticRobotModel model(params);

    controller.moveForward(40.0);
    controller.start();

    std::array<JointAngles, NUM_LEGS> angles{};
    double dt = 1.0 / params.control_frequency;
    const int cycles = 25; // ~1m at 40mm per cycle

    for (int c = 0; c < cycles; ++c) {
        for (int i = 0; i < static_cast<int>(params.control_frequency); ++i) {
            controller.update(dt, angles);
        }

        std::cout << "Cycle " << (c + 1) << std::endl;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            Point3D pos = model.forwardKinematicsGlobalCoordinatesAnalytic(leg, angles[leg]);
            bool swing = pos.z > -150.0 + 1e-3;
            std::string phase = swing ? "SWING" : "STANCE";
            std::cout << "  Leg " << (leg + 1) << " " << phase << " Pos("
                      << std::fixed << std::setprecision(1)
                      << pos.x << ", " << pos.y << ", " << pos.z << ") "
                      << "Angles("
                      << math_utils::radiansToDegrees(angles[leg].coxa) << ", "
                      << math_utils::radiansToDegrees(angles[leg].femur) << ", "
                      << math_utils::radiansToDegrees(angles[leg].tibia) << ")"
                      << std::endl;
        }
    }

    return 0;
}
