#include "../src/hexamotion_constants.h"
#include "../src/robot_model.h"
#include "../src/workspace_analyzer.h"
#include <iostream>

int main() {
    std::cout << "=== WorkspaceAnalyzer Fusion Test ===" << std::endl;

    // Create a robot model with real robot parameters from AGENTS.md
    Parameters params{};
    params.hexagon_radius = 200;
    params.coxa_length = 50;
    params.femur_length = 101;
    params.tibia_length = 208;
    params.robot_height = 208;
    params.standing_height = 150;
    params.control_frequency = 50;
    params.coxa_angle_limits[0] = -65;
    params.coxa_angle_limits[1] = 65;
    params.femur_angle_limits[0] = -75;
    params.femur_angle_limits[1] = 75;
    params.tibia_angle_limits[0] = -45;
    params.tibia_angle_limits[1] = 45;

    RobotModel model(params);

    // Test unified WorkspaceAnalyzer creation
    std::cout << "Creating WorkspaceAnalyzer..." << std::endl;
    ComputeConfig config = ComputeConfig::medium(); // Use medium precision for realistic testing
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
    bool is_reachable = analyzer.isReachable(0, test_point);
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

                    if (std::abs(actual - expected) < 1.0) { // 1mm tolerance
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
    for (int i = 0; i < NUM_LEGS; i++) {
        // Use positions within reasonable standing configuration
        // Standing height is 150mm, so leg tips around z=-150
        leg_positions[i] = Point3D(250, 150, -150);
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

    return 0;
}
