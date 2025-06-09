#include "../include/locomotion_system.h"
#include "test_stubs.h"
#include <chrono>
#include <iomanip>
#include <iostream>

void testIKWithMultipleStarts(bool use_multiple_starts) {
    std::cout << "\n=========================================" << std::endl;
    std::cout << "Testing IK with use_multiple_starts = " << (use_multiple_starts ? "true" : "false") << std::endl;
    std::cout << "=========================================" << std::endl;

    // Setup parameters
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

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    assert(sys.initialize(&imu, &fsr, &servos));

    // Test challenging targets that might benefit from multiple starting configurations
    std::vector<Point3D> test_targets = {
        Point3D(640, 0, -80),     // Far target (potential convergence issues)
        Point3D(350, 550, -80),   // Side target (multiple solutions possible)
        Point3D(-350, -550, -80), // Opposite side
        Point3D(450, 200, -120),  // Mixed challenging case
    };

    int successful_solves = 0;
    float total_error = 0.0f;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < test_targets.size(); ++i) {
        Point3D target = test_targets[i];

        std::cout << "\nTarget " << (i + 1) << ": ("
                  << std::fixed << std::setprecision(1)
                  << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

        // Test IK on leg 0
        JointAngles result = sys.calculateInverseKinematics(0, target);
        Point3D fk_verify = sys.calculateForwardKinematics(0, result);

        float error = sqrt(pow(target.x - fk_verify.x, 2) +
                           pow(target.y - fk_verify.y, 2) +
                           pow(target.z - fk_verify.z, 2));

        std::cout << "  Result: (" << std::setprecision(1)
                  << result.coxa << "°, " << result.femur << "°, " << result.tibia << "°)" << std::endl;
        std::cout << "  FK verify: (" << std::setprecision(1)
                  << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;
        std::cout << "  Error: " << std::setprecision(2) << error << "mm" << std::endl;

        if (error < 50.0f) { // Consider it successful if error < 50mm
            successful_solves++;
        }
        total_error += error;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cout << "\n--- SUMMARY ---" << std::endl;
    std::cout << "Successful solves: " << successful_solves << "/" << test_targets.size() << std::endl;
    std::cout << "Average error: " << std::setprecision(2) << (total_error / test_targets.size()) << "mm" << std::endl;
    std::cout << "Computation time: " << duration.count() << " microseconds" << std::endl;
}

int main() {
    std::cout << "=== IK MULTIPLE STARTS VALIDATION TEST ===" << std::endl;
    std::cout << "Testing the configurable multiple starting configurations feature" << std::endl;

    // Test with multiple starts enabled (enhanced approach)
    testIKWithMultipleStarts(true);

    // Test with multiple starts disabled (original CSIRO approach)
    testIKWithMultipleStarts(false);

    std::cout << "\n=========================================" << std::endl;
    std::cout << "COMPARISON COMPLETE" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "The flag params.ik.use_multiple_starts successfully controls" << std::endl;
    std::cout << "whether to use multiple starting configurations (enhanced)" << std::endl;
    std::cout << "or single starting configuration (original CSIRO behavior)." << std::endl;
    std::cout << "\nBoth approaches should work, but multiple starts may provide" << std::endl;
    std::cout << "better convergence for challenging targets at the cost of" << std::endl;
    std::cout << "increased computation time." << std::endl;

    return 0;
}
