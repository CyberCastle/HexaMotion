#include "robot_model.h"
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.coxa_angle_limits[0] = -90;
    p.coxa_angle_limits[1] = 90;
    p.femur_angle_limits[0] = -90;
    p.femur_angle_limits[1] = 90;
    p.tibia_angle_limits[0] = -90;
    p.tibia_angle_limits[1] = 90;
    p.ik.clamp_joints = true;

    RobotModel model(p);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== Joint Angle Range Analysis ===" << std::endl;
    std::cout << "Expected ranges: Coxa [-90°, 90°], Femur [-90°, 90°], Tibia [-90°, 90°]" << std::endl;
    std::cout << std::endl;

    // Test various targets and check if angles exceed limits
    std::vector<Point3D> test_targets = {
        Point3D{759, 0, 0},      // Horizontal straight
        Point3D{600, 300, -50},  // Forward-right, down
        Point3D{200, 200, -100}, // Diagonal, down
        Point3D{200, 500, 50},   // Near, right, up
        Point3D{700, -200, -30}, // Forward-left, down
        Point3D{300, -200, 80},  // Near-left, up
        Point3D{500, 0, -150},   // Forward, far down
        Point3D{450, 350, 100}   // Forward-right, up
    };

    int violations = 0;
    for (size_t i = 0; i < test_targets.size(); ++i) {
        Point3D target = test_targets[i];

        std::cout << "Target " << (i + 1) << ": (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

        JointAngles angles = model.inverseKinematicsGlobalCoordinates(0, target);

        std::cout << "  Result: (" << angles.coxa << "°, " << angles.femur << "°, " << angles.tibia << "°)" << std::endl;

        // Check for violations
        bool coxa_violation = (angles.coxa < p.coxa_angle_limits[0] || angles.coxa > p.coxa_angle_limits[1]);
        bool femur_violation = (angles.femur < p.femur_angle_limits[0] || angles.femur > p.femur_angle_limits[1]);
        bool tibia_violation = (angles.tibia < p.tibia_angle_limits[0] || angles.tibia > p.tibia_angle_limits[1]);

        if (coxa_violation || femur_violation || tibia_violation) {
            violations++;
            std::cout << "  *** VIOLATION DETECTED ***" << std::endl;
            if (coxa_violation) {
                std::cout << "    Coxa: " << angles.coxa << "° outside [-90°, 90°]" << std::endl;
            }
            if (femur_violation) {
                std::cout << "    Femur: " << angles.femur << "° outside [-90°, 90°]" << std::endl;
            }
            if (tibia_violation) {
                std::cout << "    Tibia: " << angles.tibia << "° outside [-90°, 90°]" << std::endl;
            }
        }

        // Verify FK accuracy
        Point3D fk_result = model.forwardKinematics(0, angles);
        double error = sqrt(pow(target.x - fk_result.x, 2) +
                           pow(target.y - fk_result.y, 2) +
                           pow(target.z - fk_result.z, 2));
        std::cout << "  FK verify: (" << fk_result.x << ", " << fk_result.y << ", " << fk_result.z << ")" << std::endl;
        std::cout << "  Error: " << error << "mm" << std::endl;
        std::cout << std::endl;
    }

    std::cout << "=== Test Summary ===" << std::endl;
    std::cout << "Total targets tested: " << test_targets.size() << std::endl;
    std::cout << "Joint limit violations: " << violations << std::endl;

    if (violations > 0) {
        std::cout << "*** JOINT LIMIT VIOLATIONS DETECTED - NEEDS FIXING ***" << std::endl;
    } else {
        std::cout << "All joint angles within specified limits." << std::endl;
    }

    return 0;
}
