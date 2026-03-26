#include "body_pose_config.h"
#include "body_pose_config_factory.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "robot_model.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Structure to store a complete solution
struct AngleSolution {
    double femur_deg;
    double tibia_deg;
    double coxa_deg;
    Point3D global_position;
    double height_error;
    double tibia_angle;
    double score;

    AngleSolution() : femur_deg(0), tibia_deg(0), coxa_deg(0),
                      global_position(0, 0, 0), height_error(999),
                      tibia_angle(0), score(999) {}
};

// Structure to store leg information
struct LegConfiguration {
    int leg_index;
    double base_angle_deg;
    JointAngles joint_angles;
    Point3D global_position;
    double height_error;
    double tibia_angle;
    double score;
};

// Calculates the tibia angle relative to the ground (XY plane)
double getTibiaAngleToGround(const RobotModel &model, int leg, const JointAngles &angles) {
    Eigen::Matrix4d transform = model.legTransform(leg, angles);
    Eigen::Vector3d tibia_direction(transform(0, 2), transform(1, 2), transform(2, 2));
    Eigen::Vector3d ground_normal(0, 0, 1);
    double dot_product = tibia_direction.dot(ground_normal) / (tibia_direction.norm() * ground_normal.norm());
    double angle_rad = acos(std::max(-1.0, std::min(1.0, dot_product))); // Clamp to avoid numerical errors
    double angle_deg = math_utils::radiansToDegrees(angle_rad);
    // Angle relative to the XY plane: 90° when the tibia is perpendicular to the ground
    return std::abs(90.0 - angle_deg);
}

// Checks if the tibia is perpendicular to the ground
bool isTibiaPerpendicularToGround(const RobotModel &model, int leg, const JointAngles &angles, double tolerance_deg = 2.0) {
    double angle = getTibiaAngleToGround(model, leg, angles);
    return std::abs(angle - 90.0) <= tolerance_deg;
}

// Validates a leg configuration using inverse kinematics
bool validateLegConfigurationWithIK(const RobotModel &model, const LegConfiguration &config,
                                    double tolerance_mm = 1.0) {
    // Get the target position from the configuration
    Point3D target_position = config.global_position;
    JointAngles target_angles = config.joint_angles;

    // Apply inverse kinematics to obtain calculated angles
    Point3D target = model.forwardKinematicsGlobalCoordinates(config.leg_index, target_angles);
    JointAngles ik = model.inverseKinematicsCurrentGlobalCoordinates(config.leg_index, target_angles, target);
    // Calculate position using IK angles
    Point3D fk = model.forwardKinematicsGlobalCoordinates(config.leg_index, ik);

    // Calculate position error
    double position_error = std::sqrt(
        std::pow(target.x - fk.x, 2) +
        std::pow(target.y - fk.y, 2) +
        std::pow(target.z - fk.z, 2));

    // Check if the solution is valid
    return position_error <= tolerance_mm;
}

// Finds the optimal angles for a given robot height using brute force
std::vector<AngleSolution> findOptimalAnglesForHeight(const RobotModel &model, double target_height,
                                                      double height_tolerance = 1.0) {
    std::vector<AngleSolution> solutions;
    const Parameters &p = model.getParams();

    std::cout << "Searching for solutions at height " << target_height << "mm..." << std::endl;
    std::cout << "Real robot limitations (AGENTS.md):" << std::endl;
    std::cout << "Coxa range: [" << p.coxa_angle_limits[0] << "°, " << p.coxa_angle_limits[1] << "°]" << std::endl;
    std::cout << "Femur range: [" << p.femur_angle_limits[0] << "°, " << p.femur_angle_limits[1] << "°]" << std::endl;
    std::cout << "Tibia range: [" << p.tibia_angle_limits[0] << "°, " << p.tibia_angle_limits[1] << "°]" << std::endl;

    int total_combinations = 0;
    int valid_combinations = 0;

    // Brute force strategy: iterate only femur and tibia (coxa = 30°)
    double femur_step = 0.1; // Smaller step for higher precision
    double tibia_step = 0.1;
    double coxa_deg = 0.0; // Coxa fixed at 0°

    // We want the minimum height, so we evaluate from the lower limit up to 0, which implies the femur is pointing upward.
    for (double femur_deg = p.femur_angle_limits[0]; femur_deg <= 1; femur_deg += femur_step) {
        // We want the minimum height, so we evaluate from 0 to the upper limit of the tibia.
        for (double tibia_deg = -1; tibia_deg <= p.tibia_angle_limits[1]; tibia_deg += tibia_step) {
            total_combinations++;

            JointAngles test_angles(math_utils::degreesToRadians(coxa_deg),
                                    math_utils::degreesToRadians(femur_deg),
                                    math_utils::degreesToRadians(tibia_deg));

            // Calculate global position
            Point3D fk_result = model.forwardKinematicsGlobalCoordinates(0, test_angles);
            // Compare height magnitude with target
            // Height error: Z magnitude vs target height
            double height_error = std::abs(std::abs(fk_result.z) - target_height);

            // Only consider if height is within tolerance
            if (height_error <= height_tolerance) {

                double tibia_angle = getTibiaAngleToGround(model, 0, test_angles);
                bool is_perpendicular = isTibiaPerpendicularToGround(model, 0, test_angles, 0.0); // Increased tolerance

                // Only save if the tibia is perpendicular
                if (is_perpendicular) {
                    AngleSolution solution;
                    solution.femur_deg = femur_deg;
                    solution.tibia_deg = tibia_deg;
                    solution.coxa_deg = coxa_deg;
                    solution.global_position = fk_result;
                    solution.height_error = height_error;
                    solution.tibia_angle = tibia_angle;
                    solution.score = 10.0 * height_error + std::abs(tibia_angle - 90.0);

                    solutions.push_back(solution);
                    valid_combinations++;
                }
            }
        }
    }

    std::cout << "Evaluated " << total_combinations << " combinations, found "
              << valid_combinations << " valid" << std::endl;

    // Refine the best solutions with finer search
    if (!solutions.empty()) {
        std::sort(solutions.begin(), solutions.end(),
                  [](AngleSolution &a, AngleSolution &b) {
                      return a.score < b.score;
                  });

        std::cout << "Refining the best solutions..." << std::endl;

        // Take the best 3 solutions and refine them (reduced for smaller ranges)
        std::vector<AngleSolution> refined_solutions;
        for (int i = 0; i < std::min(3, (int)solutions.size()); i++) {
            AngleSolution &base = solutions[i];

            // Fine search around the best solution (femur and tibia only)
            for (double df = -2.0; df <= 2.0; df += 0.5) {
                for (double dt = -2.0; dt <= 2.0; dt += 0.5) {
                    double new_femur = base.femur_deg + df;
                    double new_tibia = base.tibia_deg + dt;
                    double new_coxa = 0.0; // Coxa fixed at 0°

                    if (new_femur >= p.femur_angle_limits[0] && new_femur <= p.femur_angle_limits[1] &&
                        new_tibia >= p.tibia_angle_limits[0] && new_tibia <= p.tibia_angle_limits[1]) {

                        JointAngles test_angles(math_utils::degreesToRadians(new_coxa),
                                                math_utils::degreesToRadians(new_femur),
                                                math_utils::degreesToRadians(new_tibia));

                        Point3D fk_result = model.forwardKinematicsGlobalCoordinates(0, test_angles);
                        // Height error: Z magnitude vs target height
                        double height_error = std::abs(std::abs(fk_result.z) - target_height);

                        if (height_error <= height_tolerance) {
                            double tibia_angle = getTibiaAngleToGround(model, 0, test_angles);
                            bool is_perpendicular = isTibiaPerpendicularToGround(model, 0, test_angles, 3.0);

                            if (is_perpendicular) {
                                AngleSolution solution;
                                solution.femur_deg = new_femur;
                                solution.tibia_deg = new_tibia;
                                solution.coxa_deg = new_coxa;
                                solution.global_position = fk_result;
                                solution.height_error = height_error;
                                solution.tibia_angle = tibia_angle;
                                solution.score = 10.0 * height_error + std::abs(tibia_angle - 90.0);

                                refined_solutions.push_back(solution);
                            }
                        }
                    }
                }
            }
        }

        // Combine and reorder all solutions
        solutions.insert(solutions.end(), refined_solutions.begin(), refined_solutions.end());

        std::cout << "Refinement completed. Total solutions: " << solutions.size() << std::endl;
    }

    // Sort by final score
    std::sort(solutions.begin(), solutions.end(),
              [](AngleSolution &a, AngleSolution &b) {
                  return a.score < b.score;
              });

    return solutions;
}

// Calculates configurations for all 6 legs using angular offsets from BASE_THETA_OFFSETS
std::vector<LegConfiguration> calculateAllLegsConfiguration(const RobotModel &model, double target_height,
                                                            AngleSolution &base_solution) {
    std::vector<LegConfiguration> leg_configs;

    std::cout << "\nCalculating configurations for all 6 legs:" << std::endl;
    std::cout << "Leg  | Coxa° | Femur° | Tibia° | Pos_X   | Pos_Y   | Pos_Z   | H_Error | T_Angle" << std::endl;
    std::cout << "-----|-------|--------|--------|---------|---------|---------|---------|--------" << std::endl;

    for (int leg = 0; leg < 6; leg++) {
        LegConfiguration config;
        config.leg_index = leg;
        config.base_angle_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[leg]);

        // Use the base solution for femur and tibia, coxa stays at 0°
        config.joint_angles = JointAngles(
            math_utils::degreesToRadians(base_solution.coxa_deg),
            math_utils::degreesToRadians(base_solution.femur_deg),
            math_utils::degreesToRadians(base_solution.tibia_deg));

        // Calculate the global position for this leg
        config.global_position = model.forwardKinematicsGlobalCoordinates(leg, config.joint_angles);

        // Calculate quality metrics
        config.height_error = std::abs(std::abs(config.global_position.z) - target_height);
        config.tibia_angle = getTibiaAngleToGround(model, leg, config.joint_angles);
        config.score = 10.0 * config.height_error + std::abs(config.tibia_angle - 90.0);

        leg_configs.push_back(config);

        // Display leg information
        std::cout << std::setw(4) << leg << " | "
                  << std::setw(5) << std::setprecision(2) << base_solution.coxa_deg << " | "
                  << std::setw(6) << std::setprecision(2) << base_solution.femur_deg << " | "
                  << std::setw(6) << std::setprecision(2) << base_solution.tibia_deg << " | "
                  << std::setw(7) << std::setprecision(2) << config.global_position.x << " | "
                  << std::setw(7) << std::setprecision(2) << config.global_position.y << " | "
                  << std::setw(7) << std::setprecision(2) << config.global_position.z << " | "
                  << std::setw(7) << std::setprecision(2) << config.height_error << " | "
                  << std::setw(7) << std::setprecision(2) << config.tibia_angle << std::endl;
    }

    return leg_configs;
}

int main() {
    // Robot parameter configuration per AGENTS.md
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.default_height_offset = -208.0; // Set to -tibia_length for explicit configuration
    p.robot_height = 208;
    p.time_delta = 1.0 / 50.0;
    // Real robot limitations per AGENTS.md
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -50;
    p.tibia_angle_limits[1] = 50;

    RobotModel model(p);
    model.workspaceAnalyzerInitializer(); // Initialize WorkspaceAnalyzer

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "=== OPTIMAL ANGLE SEARCH FOR SPECIFIC MINIMUM HEIGHT ===" << std::endl;
    std::cout << "Robot parameters:" << std::endl;
    std::cout << "  - Hexagon radius: " << p.hexagon_radius << "mm" << std::endl;
    std::cout << "  - Coxa length: " << p.coxa_length << "mm" << std::endl;
    std::cout << "  - Femur length: " << p.femur_length << "mm" << std::endl;
    std::cout << "  - Tibia length: " << p.tibia_length << "mm" << std::endl;
    std::cout << "  - Robot height: " << p.robot_height << "mm" << std::endl;

    // Define target heights to test - adjusted for real limitations
    std::vector<double> target_heights = {150.0}; // Range of heights to test

    for (double target_height : target_heights) {
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "TARGET HEIGHT: " << target_height << "mm" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        CalculatedServoAngles calc_angles = RobotModel::calculateServoAnglesForHeight(target_height, p);
        std::cout << "Calculated angles for target height:" << std::endl;
        std::cout << "  - Coxa: " << math_utils::radiansToDegrees(calc_angles.coxa) << "°" << std::endl;
        std::cout << "  - Femur: " << math_utils::radiansToDegrees(calc_angles.femur) << "°" << std::endl;
        std::cout << "  - Tibia: " << math_utils::radiansToDegrees(calc_angles.tibia) << "°" << std::endl;
        std::cout << "  - Valid solution: " << (calc_angles.valid ? "YES" : "NO") << std::endl;

        // Search for solutions at this height with increased tolerance
        std::vector<AngleSolution> solutions = findOptimalAnglesForHeight(model, target_height, 0.1); // Increased tolerance

        if (solutions.empty()) {
            std::cout << "No solutions found for height " << target_height << "mm" << std::endl;
            continue;
        }

        std::cout << "Found " << solutions.size() << " valid solutions" << std::endl;

        // Use the best solution to calculate all 6 legs
        AngleSolution &best_solution = solutions[0];

        std::cout << "\n--- BEST SOLUTION FOR REFERENCE LEG ---" << std::endl;
        std::cout << "Joint angles:" << std::endl;
        std::cout << "  - Coxa: " << best_solution.coxa_deg << "°" << std::endl;
        std::cout << "  - Femur: " << best_solution.femur_deg << "°" << std::endl;
        std::cout << "  - Tibia: " << best_solution.tibia_deg << "°" << std::endl;
        std::cout << "Global tip position (leg 0):" << std::endl;
        std::cout << "  - X: " << best_solution.global_position.x << "mm" << std::endl;
        std::cout << "  - Y: " << best_solution.global_position.y << "mm" << std::endl;
        std::cout << "  - Z: " << best_solution.global_position.z << "mm" << std::endl;
        std::cout << "Quality metrics:" << std::endl;
        std::cout << "  - Height error: " << best_solution.height_error << "mm" << std::endl;
        std::cout << "  - Tibia angle to ground: " << best_solution.tibia_angle << "°" << std::endl;
        std::cout << "  - Score: " << best_solution.score << std::endl;

        // Calculate configurations for all 6 legs
        std::vector<LegConfiguration> all_legs = calculateAllLegsConfiguration(model, target_height, best_solution);

        // Validate configurations with inverse kinematics
        std::cout << "\n--- INVERSE KINEMATICS VALIDATION ---" << std::endl;
        int patas_validadas = 0;
        for (const auto &leg_config : all_legs) {
            bool ik_valid = validateLegConfigurationWithIK(model, leg_config, 1.0);
            if (ik_valid) {
                patas_validadas++;
            }
            std::cout << "Leg " << leg_config.leg_index << " (offset " << leg_config.base_angle_deg << "°): ";
            std::cout << "IK: " << (ik_valid ? "PASSED" : "FAILED")
                      << std::endl;
        }
        std::cout << "Total: " << patas_validadas << "/6 legs validated" << std::endl;

        // Calculate statistics
        double avg_height_error = 0.0;
        double avg_tibia_angle = 0.0;
        double max_height_error = 0.0;
        double min_height_error = std::numeric_limits<double>::max();

        for (const auto &leg : all_legs) {
            avg_height_error += leg.height_error;
            avg_tibia_angle += leg.tibia_angle;
            max_height_error = std::max(max_height_error, leg.height_error);
            min_height_error = std::min(min_height_error, leg.height_error);
        }
        avg_height_error /= 6.0;
        avg_tibia_angle /= 6.0;

        std::cout << "\nStatistics for all 6 legs:" << std::endl;
        std::cout << "  - Average height error: " << avg_height_error << "mm" << std::endl;
        std::cout << "  - Min/max height error: " << min_height_error << "/" << max_height_error << "mm" << std::endl;
        std::cout << "  - Average tibia angle: " << avg_tibia_angle << "°" << std::endl;
        std::cout << "  - All legs use the same joint angles" << std::endl;
        std::cout << "  - Position differences are due to BASE_THETA_OFFSETS" << std::endl;

        // Verify inverse kinematics for leg 0
        JointAngles best_angles(math_utils::degreesToRadians(best_solution.coxa_deg),
                                math_utils::degreesToRadians(best_solution.femur_deg),
                                math_utils::degreesToRadians(best_solution.tibia_deg));
        Point3D verification = model.forwardKinematicsGlobalCoordinates(0, best_angles);
        std::cout << "\nFK verification (leg 0):" << std::endl;
        std::cout << "  - Calculated position: (" << verification.x << ", " << verification.y << ", " << verification.z << ")" << std::endl;
        std::cout << "  - Difference: (" << (verification.x - best_solution.global_position.x) << ", "
                  << (verification.y - best_solution.global_position.y) << ", "
                  << (verification.z - best_solution.global_position.z) << ")" << std::endl;
    }

    return 0;
}
