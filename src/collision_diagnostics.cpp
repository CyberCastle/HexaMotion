#include "hexamotion_constants.h"
#include "workspace_validator.h"
#include <cmath>
#include <iostream>

/**
 * @file collision_diagnostics.cpp
 * @brief Diagnostic tools for analyzing and fixing the HexaMotion collision issue
 *
 * This file provides tools to diagnose the critical issue where legs can collide
 * due to inadequate hexagon_radius calculation that doesn't account for workspace overlap.
 */

class CollisionDiagnostics {
  public:
    /**
     * @brief Analyze the current robot configuration for potential leg collisions
     * @param model Reference to the robot model
     * @return true if configuration is safe, false if collisions are possible
     */
    static bool analyzeCurrentConfiguration(const RobotModel &model) {
        const Parameters &p = model.getParams();
        double leg_reach = p.coxa_length + p.femur_length + p.tibia_length;
        double safe_reach = leg_reach * 0.65f; // Same as in walk_controller.cpp

        std::cout << "=== HexaMotion Collision Analysis ===" << std::endl;
        std::cout << "Current hexagon_radius: " << p.hexagon_radius << " mm" << std::endl;
        std::cout << "Total leg reach: " << leg_reach << " mm" << std::endl;
        std::cout << "Safe reach (65%): " << safe_reach << " mm" << std::endl;

        // Calculate minimum safe radius
        double min_safe_radius = WorkspaceValidator::calculateSafeHexagonRadius(safe_reach, 30.0f);
        std::cout << "Minimum safe radius: " << min_safe_radius << " mm" << std::endl;

        if (p.hexagon_radius < min_safe_radius) {
            std::cout << "⚠️  COLLISION RISK DETECTED!" << std::endl;
            std::cout << "   Current radius is " << (min_safe_radius - p.hexagon_radius)
                      << " mm too small" << std::endl;
            std::cout << "   Legs may collide during normal operation" << std::endl;
            return false;
        } else {
            std::cout << "✅ Configuration appears safe" << std::endl;
            std::cout << "   Safety margin: " << (p.hexagon_radius - min_safe_radius) << " mm" << std::endl;
            return true;
        }
    }

    /**
     * @brief Check workspace overlap between adjacent legs
     * @param model Reference to the robot model
     */
    static void analyzeWorkspaceOverlap(const RobotModel &model) {
        const Parameters &p = model.getParams();
        double leg_reach = p.coxa_length + p.femur_length + p.tibia_length;
        double safe_reach = leg_reach * 0.65f;

        std::cout << "\n=== Workspace Overlap Analysis ===" << std::endl;

        for (int leg = 0; leg < 6; leg++) {
            int next_leg = (leg + 1) % 6;

            // Calculate base positions
            double angle1 = leg * 60.0f;
            double angle2 = next_leg * 60.0f;

            Point3D base1(p.hexagon_radius * cos(angle1 * M_PI / 180.0f),
                          p.hexagon_radius * sin(angle1 * M_PI / 180.0f), 0);
            Point3D base2(p.hexagon_radius * cos(angle2 * M_PI / 180.0f),
                          p.hexagon_radius * sin(angle2 * M_PI / 180.0f), 0);

            bool overlap = WorkspaceValidator::checkWorkspaceOverlap(
                base1, safe_reach, base2, safe_reach, 20.0f);

            double distance = WorkspaceValidator::getDistance2D(base1, base2);
            double workspace_separation = distance - (2 * safe_reach);

            std::cout << "Legs " << leg << "-" << next_leg << ": ";
            if (overlap) {
                std::cout << "⚠️  OVERLAP by " << -workspace_separation << " mm" << std::endl;
            } else {
                std::cout << "✅ Safe separation: " << workspace_separation << " mm" << std::endl;
            }
        }
    }

    /**
     * @brief Recommend optimal hexagon_radius to prevent collisions
     * @param model Reference to the robot model
     * @return Recommended hexagon_radius value
     */
    static double recommendOptimalRadius(const RobotModel &model) {
        const Parameters &p = model.getParams();
        double leg_reach = p.coxa_length + p.femur_length + p.tibia_length;
        double safe_reach = leg_reach * 0.65f;

        // Calculate minimum safe radius with different safety margins
        double conservative_radius = WorkspaceValidator::calculateSafeHexagonRadius(safe_reach, 50.0f);
        double standard_radius = WorkspaceValidator::calculateSafeHexagonRadius(safe_reach, 30.0f);
        double minimum_radius = WorkspaceValidator::calculateSafeHexagonRadius(safe_reach, 20.0f);

        std::cout << "\n=== Radius Recommendations ===" << std::endl;
        std::cout << "Conservative (50mm margin): " << conservative_radius << " mm" << std::endl;
        std::cout << "Standard (30mm margin): " << standard_radius << " mm" << std::endl;
        std::cout << "Minimum (20mm margin): " << minimum_radius << " mm" << std::endl;
        std::cout << "Current value: " << p.hexagon_radius << " mm" << std::endl;

        return standard_radius;
    }

  private:
    static constexpr int NUM_LEGS_COUNT = 6;
};

// Example usage function
void diagnoseHexaMotionCollisions(const RobotModel &model) {
    std::cout << "Diagnosing HexaMotion collision issue..." << std::endl;
    std::cout << "The problem: hexagon_radius calculation in walk_controller.cpp" << std::endl;
    std::cout << "lines 65-66 positions legs geometrically but doesn't prevent" << std::endl;
    std::cout << "workspace overlap, unlike OpenSHC's sophisticated approach." << std::endl;
    std::cout << std::endl;

    bool is_safe = CollisionDiagnostics::analyzeCurrentConfiguration(model);
    CollisionDiagnostics::analyzeWorkspaceOverlap(model);
    double recommended_radius = CollisionDiagnostics::recommendOptimalRadius(model);

    if (!is_safe) {
        std::cout << "\n🔧 RECOMMENDED FIXES:" << std::endl;
        std::cout << "1. Update DEFAULT_HEXAGON_RADIUS in hexamotion_constants.h" << std::endl;
        std::cout << "   from " << model.getParams().hexagon_radius << " mm to "
                  << recommended_radius << " mm" << std::endl;
        std::cout << "2. The new collision avoidance system has been added to" << std::endl;
        std::cout << "   walk_controller.cpp to provide runtime protection" << std::endl;
        std::cout << "3. Consider implementing OpenSHC-style workspace analysis" << std::endl;
        std::cout << "   for more sophisticated collision prevention" << std::endl;
    }
}
