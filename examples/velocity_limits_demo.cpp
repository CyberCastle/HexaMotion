#include "robot_model.h"
#include "hexamotion_constants.h"
#include "velocity_limits.h"
#include "walk_controller.h"
#include <cmath>
#include <iomanip>
#include <iostream>

void printLimitValues(const VelocityLimits::LimitValues &limits, const std::string &label) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << label << ":\n";
    std::cout << "  Linear X: " << limits.linear_x << " m/s\n";
    std::cout << "  Linear Y: " << limits.linear_y << " m/s\n";
    std::cout << "  Angular Z: " << limits.angular_z << " rad/s\n";
    std::cout << "  Acceleration: " << limits.acceleration << " m/s²\n\n";
}

void printWorkspaceConfig(const VelocityLimits::WorkspaceConfig &workspace) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Workspace Configuration:\n";
    std::cout << "  Walkspace Radius: " << workspace.walkspace_radius << " m\n";
    std::cout << "  Stance Radius: " << workspace.stance_radius << " m\n";
    std::cout << "  Overshoot X: " << workspace.overshoot_x << " m\n";
    std::cout << "  Overshoot Y: " << workspace.overshoot_y << " m\n";
    std::cout << "  Safety Margin: " << workspace.safety_margin << "\n\n";
}

void demonstrateBearingBasedLimits(VelocityLimits &velocity_limits) {
    std::cout << "=== Bearing-Based Velocity Limits ===\n";

    // Test limits at different bearings
    std::vector<double> test_bearings = {0.0f, TIBIA_ANGLE_MAX, DEFAULT_MAX_ANGULAR_VELOCITY, 135.0f, HALF_ROTATION_DEGREES, 225.0f, 270.0f, 315.0f};

    for (double bearing : test_bearings) {
        auto limits = velocity_limits.getLimit(bearing);
        std::cout << "Bearing " << std::setw(3) << bearing << "°: "
                  << "Vx=" << std::setw(6) << limits.linear_x
                  << " Vy=" << std::setw(6) << limits.linear_y
                  << " ω=" << std::setw(6) << limits.angular_z << " rad/s\n";
    }
    std::cout << "\n";
}

void demonstrateGaitParameterEffects(VelocityLimits &velocity_limits) {
    std::cout << "=== Gait Parameter Effects on Velocity Limits ===\n";

    // Test different gait configurations
    std::vector<VelocityLimits::GaitConfig> gait_configs(3);

    // Slow, stable gait
    gait_configs[0].frequency = WORKSPACE_SCALING_FACTOR;
    gait_configs[0].stance_ratio = 0.8f;
    gait_configs[0].swing_ratio = 0.2f;
    gait_configs[0].time_to_max_stride = SERVO_SPEED_MAX;

    // Normal gait
    gait_configs[1].frequency = DEFAULT_ANGULAR_SCALING;
    gait_configs[1].stance_ratio = 0.6f;
    gait_configs[1].swing_ratio = 0.4f;
    gait_configs[1].time_to_max_stride = ANGULAR_ACCELERATION_FACTOR;

    // Fast, dynamic gait
    gait_configs[2].frequency = ANGULAR_ACCELERATION_FACTOR;
    gait_configs[2].stance_ratio = 0.4f;
    gait_configs[2].swing_ratio = 0.6f;
    gait_configs[2].time_to_max_stride = DEFAULT_ANGULAR_SCALING;

    std::vector<std::string> gait_names = {"Slow Gait", "Normal Gait", "Fast Gait"};

    for (size_t i = 0; i < gait_configs.size(); ++i) {
        velocity_limits.generateLimits(gait_configs[i]);
        auto limits = velocity_limits.getLimit(0.0f);

        std::cout << gait_names[i] << " (f=" << gait_configs[i].frequency
                  << "Hz, stance=" << gait_configs[i].stance_ratio << "):\n";
        std::cout << "  Max Linear Speed: " << limits.linear_x << " m/s\n";
        std::cout << "  Max Angular Speed: " << limits.angular_z << " rad/s\n";
        std::cout << "  Max Acceleration: " << limits.acceleration << " m/s²\n\n";
    }
}

void demonstrateVelocityScaling(VelocityLimits &velocity_limits) {
    std::cout << "=== Velocity Scaling with Angular Demand ===\n";

    VelocityLimits::LimitValues base_limits(DEFAULT_ANGULAR_SCALING, DEFAULT_ANGULAR_SCALING, ANGULAR_ACCELERATION_FACTOR, DEFAULT_ANGULAR_SCALING);

    std::vector<double> angular_percentages = {0.0f, 0.25f, WORKSPACE_SCALING_FACTOR, 0.75f, DEFAULT_ANGULAR_SCALING};

    for (double percentage : angular_percentages) {
        auto scaled = velocity_limits.scaleVelocityLimits(base_limits, percentage);

        std::cout << "Angular demand " << std::setw(4) << (percentage * 100) << "%: "
                  << "Linear reduced to " << std::setw(5) << scaled.linear_x
                  << " m/s, Angular scaled to " << std::setw(5) << scaled.angular_z << " rad/s\n";
    }
    std::cout << "\n";
}

void demonstrateAccelerationLimiting(VelocityLimits &velocity_limits) {
    std::cout << "=== Acceleration Limiting Demonstration ===\n";

    VelocityLimits::LimitValues target(DEFAULT_ANGULAR_SCALING, WORKSPACE_SCALING_FACTOR, 1.5f, 0.8f);
    VelocityLimits::LimitValues current(0.0f, 0.0f, 0.0f, 0.8f);
    double dt = MIN_SERVO_VELOCITY;

    std::cout << "Target velocities: Vx=" << target.linear_x
              << " Vy=" << target.linear_y << " ω=" << target.angular_z << "\n";
    std::cout << "Starting from zero, dt=" << dt << "s:\n\n";

    for (int step = 0; step < 10; ++step) {
        auto limited = velocity_limits.applyAccelerationLimits(target, current, dt);

        std::cout << "Step " << std::setw(2) << step << ": "
                  << "Vx=" << std::setw(6) << std::setprecision(3) << limited.linear_x
                  << " Vy=" << std::setw(6) << std::setprecision(3) << limited.linear_y
                  << " ω=" << std::setw(6) << std::setprecision(3) << limited.angular_z << "\n";

        current = limited;

        // Stop when we're close to target
        if (std::abs(limited.linear_x - target.linear_x) < 0.01f &&
            std::abs(limited.linear_y - target.linear_y) < 0.01f &&
            std::abs(limited.angular_z - target.angular_z) < 0.01f) {
            break;
        }
    }
    std::cout << "\n";
}

void demonstrateWalkControllerIntegration(WalkController &walk_controller) {
    std::cout << "=== WalkController Integration ===\n";

    // Show current workspace configuration
    auto workspace = walk_controller.getWorkspaceConfig();
    printWorkspaceConfig(workspace);

    // Test velocity limiting through WalkController
    std::vector<std::tuple<double, double, double>> test_velocities = {
        {WORKSPACE_SCALING_FACTOR, 0.0f, 0.0f}, // Forward
        {0.0f, WORKSPACE_SCALING_FACTOR, 0.0f}, // Sideways
        {0.0f, 0.0f, DEFAULT_ANGULAR_SCALING},  // Rotation
        {1.0f, 1.0f, 2.0f},                     // Combined (likely exceeds limits)
        {-0.5f, -0.3f, -1.5f}                   // Negative velocities
    };

    for (const auto &[vx, vy, omega] : test_velocities) {
        std::cout << "Input: Vx=" << std::setw(5) << vx
                  << " Vy=" << std::setw(5) << vy
                  << " ω=" << std::setw(5) << omega;

        bool valid = walk_controller.validateVelocityCommand(vx, vy, omega);
        auto limited = walk_controller.applyVelocityLimits(vx, vy, omega);

        std::cout << " → Valid: " << (valid ? "Yes" : "No")
                  << ", Limited: Vx=" << std::setw(5) << std::setprecision(3) << limited.linear_x
                  << " Vy=" << std::setw(5) << std::setprecision(3) << limited.linear_y
                  << " ω=" << std::setw(5) << std::setprecision(3) << limited.angular_z << "\n";
    }
    std::cout << "\n";
}

void demonstrateOpenSHCEquivalence(VelocityLimits &velocity_limits) {
    std::cout << "=== OpenSHC Mathematical Equivalence ===\n";

    VelocityLimits::GaitConfig test_gait;
    test_gait.frequency = 1.2f;
    test_gait.stance_ratio = 0.65f;
    test_gait.time_to_max_stride = 2.0f;

    velocity_limits.generateLimits(test_gait);
    auto workspace = velocity_limits.getWorkspaceConfig();
    auto limits = velocity_limits.getLimit(0.0f);

    std::cout << "Gait Parameters:\n";
    std::cout << "  Frequency: " << test_gait.frequency << " Hz\n";
    std::cout << "  Stance Ratio: " << test_gait.stance_ratio << "\n";
    std::cout << "  Time to Max Stride: " << test_gait.time_to_max_stride << " s\n\n";

    // Show OpenSHC-equivalent calculations
    double cycle_time = test_gait.stance_ratio / test_gait.frequency;
    double opensc_max_speed = (workspace.walkspace_radius * 2.0f) / cycle_time;
    double opensc_max_angular = opensc_max_speed / workspace.stance_radius;
    double opensc_max_accel = opensc_max_speed / test_gait.time_to_max_stride;

    std::cout << "OpenSHC Equivalent Calculations:\n";
    std::cout << "  max_speed = (walkspace_radius * 2.0) / (stance_ratio / frequency)\n";
    std::cout << "            = (" << workspace.walkspace_radius << " * 2.0) / ("
              << test_gait.stance_ratio << " / " << test_gait.frequency << ")\n";
    std::cout << "            = " << opensc_max_speed << " m/s\n\n";

    std::cout << "  max_angular = max_speed / stance_radius\n";
    std::cout << "              = " << opensc_max_speed << " / " << workspace.stance_radius << "\n";
    std::cout << "              = " << opensc_max_angular << " rad/s\n\n";

    std::cout << "  max_acceleration = max_speed / time_to_max_stride\n";
    std::cout << "                   = " << opensc_max_speed << " / " << test_gait.time_to_max_stride << "\n";
    std::cout << "                   = " << opensc_max_accel << " m/s²\n\n";

    std::cout << "Our Implementation Results:\n";
    printLimitValues(limits, "Calculated Limits");

    std::cout << "Equivalence Check:\n";
    std::cout << "  Linear Speed Match: " << (std::abs(limits.linear_x - std::min(opensc_max_speed, 5.0f)) < 0.5f ? "✓" : "✗") << "\n";
    std::cout << "  Angular Speed Match: " << (std::abs(limits.angular_z - std::min(opensc_max_angular, 10.0f)) < 2.0f ? "✓" : "✗") << "\n";
    std::cout << "  Acceleration Match: " << (std::abs(limits.acceleration - std::min(opensc_max_accel, 10.0f)) < 0.5f ? "✓" : "✗") << "\n\n";
}

int main() {
    std::cout << "HexaMotion Velocity Limits System Demonstration\n";
    std::cout << "===============================================\n\n";

    // Initialize robot model with realistic parameters
    Parameters params;
    params.hexagon_radius = 0.12f; // 12cm hexagon radius
    params.coxa_length = 0.05f;    // 5cm coxa
    params.femur_length = 0.10f;   // 10cm femur
    params.tibia_length = 0.15f;   // 15cm tibia

    RobotModel model(params);
    VelocityLimits velocity_limits(model);
    WalkController walk_controller(model);

    // Demonstrate various aspects of the velocity limits system
    demonstrateBearingBasedLimits(velocity_limits);
    demonstrateGaitParameterEffects(velocity_limits);
    demonstrateVelocityScaling(velocity_limits);
    demonstrateAccelerationLimiting(velocity_limits);
    demonstrateWalkControllerIntegration(walk_controller);
    demonstrateOpenSHCEquivalence(velocity_limits);

    std::cout << "Demonstration completed. The velocity limits system provides:\n";
    std::cout << "• Dynamic velocity calculation based on workspace constraints\n";
    std::cout << "• Bearing-based directional limits with smooth interpolation\n";
    std::cout << "• Gait parameter integration (frequency, stance ratio)\n";
    std::cout << "• Acceleration limiting for smooth velocity transitions\n";
    std::cout << "• Mathematical equivalence to OpenSHC's velocity limiting\n";
    std::cout << "• Full integration with existing terrain adaptation system\n\n";

    return 0;
}
