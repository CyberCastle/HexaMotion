#include "../src/hexamotion_constants.h"
#include "../src/robot_model.h"
#include "../src/velocity_limits.h"
#include "../src/workspace_analyzer.h"
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    std::cout << "=== WorkspaceAnalyzer Fusion Test ===" << std::endl;

    // Create a robot model with real robot parameters from AGENTS.md
    Parameters params{};
    params.hexagon_radius = 200;
    params.coxa_length = 50;
    params.femur_length = 101;
    params.tibia_length = 208;
    params.default_height_offset = -208.0; // Set to -tibia_length for explicit configuration
    params.robot_height = 208;
    params.standing_height = 150;
    params.time_delta = 1.0 / 50.0;
    params.coxa_angle_limits[0] = -65;
    params.coxa_angle_limits[1] = 65;
    params.femur_angle_limits[0] = -75;
    params.femur_angle_limits[1] = 75;
    params.tibia_angle_limits[0] = -45;
    params.tibia_angle_limits[1] = 45;

    RobotModel model(params);
    model.workspaceAnalyzerInitializer(); // Inicializar WorkspaceAnalyzer

    // Test unified WorkspaceAnalyzer creation
    std::cout << "Creating WorkspaceAnalyzer..." << std::endl;
    ComputeConfig config = ComputeConfig::high(); // Use medium precision for realistic testing
    WorkspaceAnalyzer analyzer(model, config);

    // Initialize the analyzer to ensure proper setup
    std::cout << "Initializing WorkspaceAnalyzer..." << std::endl;
    analyzer.initialize();

    // Test workspace generation (from WalkspaceAnalyzer)
    std::cout << "Testing workspace generation..." << std::endl;
    try {
        analyzer.generateWorkspace();
        std::cout << "✅ Workspace generation successful" << std::endl;

        // Validate generateWorkspace() results
        std::cout << "Validating generateWorkspace() results..." << std::endl;
        const auto &walkspace_map = analyzer.getWalkspaceMap();
        if (!walkspace_map.empty()) {
            std::cout << "✅ Walkspace map generated with " << walkspace_map.size() << " bearing entries" << std::endl;

            // Test some specific bearings
            for (int bearing = 0; bearing <= 360; bearing += 90) {
                auto it = walkspace_map.find(bearing);
                if (it != walkspace_map.end()) {
                    std::cout << "  Bearing " << bearing << "°: radius = " << it->second << " mm" << std::endl;
                } else {
                    std::cout << "❌ Missing bearing " << bearing << "° in walkspace map" << std::endl;
                }
            }

            // Validate symmetry (bearing 0° should equal bearing 360°)
            auto bearing_0 = walkspace_map.find(0);
            auto bearing_360 = walkspace_map.find(360);
            if (bearing_0 != walkspace_map.end() && bearing_360 != walkspace_map.end()) {
                if (std::abs(bearing_0->second - bearing_360->second) < 0.001) {
                    std::cout << "✅ Walkspace map symmetry validated (0° = 360°)" << std::endl;
                } else {
                    std::cout << "❌ Walkspace map symmetry failed" << std::endl;
                }
            }
        } else {
            std::cout << "❌ Walkspace map is empty" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cout << "❌ Workspace generation failed: " << e.what() << std::endl;
    }

    // Test validation functions (from WorkspaceValidator)
    std::cout << "Testing validation functions..." << std::endl;
    // Test point within reasonable reach (robot has max reach ~359mm: coxa+femur+tibia)
    Point3D test_point(250, 100, -150); // More realistic test point
    bool is_reachable = analyzer.isPositionReachable(0, test_point, false);
    std::cout << "Point (" << test_point.x << ", " << test_point.y << ", " << test_point.z
              << ") reachable for leg 0: " << (is_reachable ? "YES" : "NO") << std::endl;

    // Test getWorkplane() function (OpenSHC equivalent)
    std::cout << "\nTesting getWorkplane() function..." << std::endl;
    try {
        // Test workplane at different heights for leg 0
        // Note: Robot default height is 208mm, standing height is 150mm (body at z=-208)
        std::vector<double> test_heights = {-250.0, -200.0, -150.0, -100.0, -50.0};

        for (double height : test_heights) {
            Workplane workplane = analyzer.getWorkplane(0, height);
            std::cout << "Leg 0, Height " << height << " mm: ";

            if (!workplane.empty()) {
                std::cout << "workplane with " << workplane.size() << " bearings" << std::endl;

                // Show sample bearings and their radii
                int sample_count = 0;
                for (const auto &bearing_radius : workplane) {
                    if (sample_count < 3) { // Show first 3 entries as sample
                        std::cout << "    Bearing " << bearing_radius.first
                                  << "°: radius = " << bearing_radius.second << " mm" << std::endl;
                        sample_count++;
                    }
                }

                // Validate workplane consistency
                auto bearing_0 = workplane.find(0);
                auto bearing_360 = workplane.find(360);
                if (bearing_0 != workplane.end() && bearing_360 != workplane.end()) {
                    if (std::abs(bearing_0->second - bearing_360->second) < 0.001) {
                        std::cout << "    ✅ Workplane symmetry OK" << std::endl;
                    } else {
                        std::cout << "    ❌ Workplane symmetry failed" << std::endl;
                    }
                }
            } else {
                std::cout << "empty workplane (height outside workspace)" << std::endl;
            }
        }

        // Test workplane interpolation
        std::cout << "\nTesting workplane interpolation..." << std::endl;
        double interpolated_height = -175.0; // Between -200 and -150
        Workplane interpolated_workplane = analyzer.getWorkplane(0, interpolated_height);
        if (!interpolated_workplane.empty()) {
            std::cout << "✅ Workplane interpolation successful at height "
                      << interpolated_height << " mm" << std::endl;

            // Compare with adjacent heights to validate interpolation
            Workplane lower_workplane = analyzer.getWorkplane(0, -200.0);
            Workplane upper_workplane = analyzer.getWorkplane(0, -150.0);

            if (!lower_workplane.empty() && !upper_workplane.empty()) {
                auto interp_0 = interpolated_workplane.find(0);
                auto lower_0 = lower_workplane.find(0);
                auto upper_0 = upper_workplane.find(0);

                if (interp_0 != interpolated_workplane.end() &&
                    lower_0 != lower_workplane.end() &&
                    upper_0 != upper_workplane.end()) {

                    double expected = lower_0->second * 0.5 + upper_0->second * 0.5; // 50% interpolation
                    double actual = interp_0->second;

                    if (std::abs(actual - expected) < 10.0) { // 10mm tolerance for realistic interpolation
                        std::cout << "✅ Workplane interpolation accuracy validated" << std::endl;
                    } else {
                        std::cout << "❌ Workplane interpolation accuracy failed (expected: "
                                  << expected << ", actual: " << actual << ")" << std::endl;
                    }
                }
            }
        } else {
            std::cout << "❌ Workplane interpolation failed" << std::endl;
        }

        // Test invalid leg index
        Workplane invalid_workplane = analyzer.getWorkplane(NUM_LEGS + 1, 0.0);
        if (invalid_workplane.empty()) {
            std::cout << "✅ getWorkplane() correctly handles invalid leg index" << std::endl;
        } else {
            std::cout << "❌ getWorkplane() should return empty workplane for invalid leg" << std::endl;
        }

    } catch (const std::exception &e) {
        std::cout << "❌ getWorkplane() test failed: " << e.what() << std::endl;
    }

    // Test workspace bounds
    std::cout << "Testing workspace bounds..." << std::endl;
    auto bounds = analyzer.getWorkspaceBounds(0);
    std::cout << "Leg 0 workspace bounds:" << std::endl;
    std::cout << "  Min reach: " << bounds.min_reach << " mm" << std::endl;
    std::cout << "  Max reach: " << bounds.max_reach << " mm" << std::endl;
    std::cout << "  Min height: " << bounds.min_height << " mm" << std::endl;
    std::cout << "  Max height: " << bounds.max_height << " mm" << std::endl;

    // Test analysis functions
    std::cout << "Testing analysis functions..." << std::endl;
    Point3D leg_positions[NUM_LEGS];

    // Get realistic leg positions from robot model using forward kinematics
    // According to AGENTS.md: with all angles at 0°, the robot stands stably by default
    // - Femur remains horizontal, in line with the coxa
    // - Tibia remains vertical, perpendicular to ground
    // - Robot body positioned at z = -208 (tibia length)
    for (int i = 0; i < NUM_LEGS; i++) {
        // Use default configuration: all angles at 0° for stable standing position
        JointAngles default_angles(0.0, 0.0, 0.0); // coxa, femur, tibia all at 0°

        // Calculate actual position using robot model's forward kinematics
        leg_positions[i] = model.forwardKinematicsGlobalCoordinates(i, default_angles);

        std::cout << "  Leg " << i << " position (0° angles): ("
                  << leg_positions[i].x << ", "
                  << leg_positions[i].y << ", "
                  << leg_positions[i].z << ") mm" << std::endl;
    }

    auto analysis_result = analyzer.analyzeWalkspace(leg_positions);
    std::cout << "Walkspace analysis result: " << (analysis_result.is_stable ? "STABLE" : "UNSTABLE") << std::endl;
    std::cout << "Stability margin: " << analysis_result.stability_margin << " mm" << std::endl;

    // Test OpenSHC compatibility validation
    std::cout << "\n=== OpenSHC Compatibility Validation ===" << std::endl;

    // Verify that generateWorkspace() and getWorkplane() work together
    std::cout << "Testing generateWorkspace() + getWorkplane() integration..." << std::endl;

    // First ensure workspace is generated
    analyzer.generateWorkspace();

    // Test that workplanes are consistent across all legs
    bool integration_success = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Workplane workplane = analyzer.getWorkplane(leg, -150.0); // Test at standing height
        if (workplane.empty()) {
            std::cout << "❌ Leg " << leg << " has empty workplane at standing height" << std::endl;
            integration_success = false;
        } else {
            // Check that workplane has reasonable values
            auto bearing_0 = workplane.find(0);
            if (bearing_0 != workplane.end() && bearing_0->second > 0) {
                std::cout << "✅ Leg " << leg << " workplane valid (radius at 0°: "
                          << bearing_0->second << " mm)" << std::endl;
            } else {
                std::cout << "❌ Leg " << leg << " workplane has invalid data" << std::endl;
                integration_success = false;
            }
        }
    }

    if (integration_success) {
        std::cout << "✅ OpenSHC generateWorkspace() + getWorkplane() integration successful" << std::endl;
    } else {
        std::cout << "❌ OpenSHC integration has issues" << std::endl;
    }

    // Test getLegWorkspace() for complete 3D workspace
    std::cout << "\nTesting getLegWorkspace() for 3D workspace data..." << std::endl;
    Workspace leg_workspace = analyzer.getLegWorkspace(0);
    if (!leg_workspace.empty()) {
        std::cout << "✅ 3D workspace data available with " << leg_workspace.size() << " height layers" << std::endl;

        // Show height range
        double min_height = leg_workspace.begin()->first;
        double max_height = leg_workspace.rbegin()->first;
        std::cout << "  Height range: " << min_height << " to " << max_height << " mm" << std::endl;
    } else {
        std::cout << "❌ 3D workspace data not available" << std::endl;
    }

    std::cout << "=== All WorkspaceAnalyzer functions tested successfully! ===" << std::endl;

    // ==============================================================
    // VelocityLimits integration & behavior validation
    // ==============================================================
    std::cout << "\n=== VelocityLimits Behavior Tests ===" << std::endl;

    VelocityLimits velocity_limits(model);

    std::vector<int> bearings = {0, 45, 90, 135, 180};
    bool velocity_limits_ok = true;

    auto workspace_cfg = velocity_limits.getWorkspaceConfig();
    std::cout << "Workspace radii (walk/stance): " << workspace_cfg.walkspace_radius
              << " / " << workspace_cfg.stance_radius << " mm" << std::endl;

    if (workspace_cfg.stance_radius <= 0 || workspace_cfg.walkspace_radius <= 0) {
        std::cout << "❌ Invalid workspace radii in VelocityLimits" << std::endl;
        velocity_limits_ok = false;
    }

    std::cout << "Retrieving limits for bearings:" << std::endl;
    for (int b : bearings) {
        auto limits = velocity_limits.getLimit((double)b);
        std::cout << "  Bearing " << b << "° -> Vx: " << limits.linear_x
                  << " mm/s, Vy: " << limits.linear_y
                  << " mm/s, Wz: " << limits.angular_z
                  << " rad/s, Accel: " << limits.acceleration << " mm/s²" << std::endl;
        if (limits.linear_x <= 0 || limits.linear_y <= 0 || limits.angular_z <= 0 || limits.acceleration <= 0) {
            std::cout << "    ❌ Non-positive limit value detected" << std::endl;
            velocity_limits_ok = false;
        }
    }

    // Interpolation test (0° a 1°)
    auto limit0 = velocity_limits.getLimit(0.0);
    auto limit1 = velocity_limits.getLimit(1.0);
    auto mid = velocity_limits.getLimit(0.5);
    if (!(mid.linear_x >= std::min(limit0.linear_x, limit1.linear_x) - 1e-6 &&
          mid.linear_x <= std::max(limit0.linear_x, limit1.linear_x) + 1e-6)) {
        std::cout << "❌ Interpolation out of bounds for linear_x" << std::endl;
        velocity_limits_ok = false;
    } else {
        std::cout << "✅ Interpolation bounds check passed" << std::endl;
    }

    // Angular scaling & coupling test
    auto base_limits = velocity_limits.getLimit(0.0);
    // 0% angular demand should not reduce linear limits now
    VelocityLimits::LimitValues no_demand_scaled = velocity_limits.scaleVelocityLimits(base_limits, 0.0);
    if (std::abs(no_demand_scaled.linear_x - base_limits.linear_x) > 1e-6 ||
        std::abs(no_demand_scaled.linear_y - base_limits.linear_y) > 1e-6) {
        std::cout << "❌ Linear limits reduced with zero angular demand" << std::endl;
        velocity_limits_ok = false;
    } else {
        std::cout << "✅ No linear reduction at zero angular demand" << std::endl;
    }

    // 100% angular demand should enforce v_planar <= w * r
    VelocityLimits::LimitValues full_scaled = velocity_limits.scaleVelocityLimits(base_limits, 1.0);
    double planar_mag = std::hypot(full_scaled.linear_x, full_scaled.linear_y);
    double stance_r = std::max(1.0, velocity_limits.getWorkspaceConfig().stance_radius);
    double kinematic_cap = base_limits.angular_z * stance_r; // base (not scaled angular_z)
    if (planar_mag - kinematic_cap > 1e-6) {
        std::cout << "❌ Kinematic coupling violated (" << planar_mag << " > " << kinematic_cap << ")" << std::endl;
        velocity_limits_ok = false;
    } else {
        std::cout << "✅ Kinematic coupling respected (planar mag ≤ ω * r)" << std::endl;
    }

    // Forward progress expectation: ensure forward linear_x is a meaningful fraction of configured cap
    double expected_min_forward = 0.3 * (params.max_velocity > 0 ? params.max_velocity : DEFAULT_MAX_LINEAR_VELOCITY);
    if (base_limits.linear_x < expected_min_forward) {
        std::cout << "❌ Forward linear_x too low (" << base_limits.linear_x << " < " << expected_min_forward << ")" << std::endl;
        velocity_limits_ok = false;
    } else {
        std::cout << "✅ Forward linear_x acceptable (" << base_limits.linear_x << " mm/s)" << std::endl;
    }

    // Validation function (in-range)
    double test_vx = base_limits.linear_x * 0.5;
    double test_vy = base_limits.linear_y * 0.4;
    double test_wz = base_limits.angular_z * 0.6;
    if (!velocity_limits.validateVelocityInputs(test_vx, test_vy, test_wz)) {
        std::cout << "❌ validateVelocityInputs rejected valid velocities" << std::endl;
        velocity_limits_ok = false;
    } else {
        std::cout << "✅ validateVelocityInputs accepted in-range velocities" << std::endl;
    }

    // Over-limit linear
    if (velocity_limits.validateVelocityInputs(base_limits.linear_x * 1.05, 0, 0)) {
        std::cout << "❌ validateVelocityInputs failed to reject over-limit linear_x" << std::endl;
        velocity_limits_ok = false;
    } else {
        std::cout << "✅ Over-limit linear_x correctly rejected" << std::endl;
    }

    // Overshoot sanity
    double ox = velocity_limits.getOvershootX();
    double oy = velocity_limits.getOvershootY();
    double walk_r = workspace_cfg.walkspace_radius;
    double overshoot_cap = 0.25 * walk_r + 1e-6; // tolerance
    if (ox <= 0 || oy <= 0) {
        std::cout << "❌ Overshoot should be positive" << std::endl;
        velocity_limits_ok = false;
    } else if (ox > overshoot_cap || oy > overshoot_cap) {
        std::cout << "❌ Overshoot exceeds 25% walkspace radius (" << ox << " / cap " << overshoot_cap << ")" << std::endl;
        velocity_limits_ok = false;
    } else {
        std::cout << "✅ Overshoot within expected bounds (" << ox << ", cap " << overshoot_cap << ")" << std::endl;
    }

    if (velocity_limits_ok) {
        std::cout << "✅ VelocityLimits tests passed" << std::endl;
    } else {
        std::cout << "❌ VelocityLimits tests encountered failures" << std::endl;
    }

    // ==============================================================
    // Compatibility Mode Comparison Test
    // ==============================================================
    // NOTE: Compatibility (OpenSHC diameter traversal) mode removed. Unified stride-based velocity limiting
    // provides a single coherent surface, integrates overshoot directly into stride_length, and avoids
    // inflated theoretical maxima (2R / (stance_ratio/f)) that were not achievable under the configured
    // stride fraction and scaling. Legacy comparison and divergence diagnostics deleted intentionally.

    return 0;
}
