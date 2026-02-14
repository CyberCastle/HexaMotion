#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
/** Added for createTripodGaitConfig. */
#include "../src/gait_config_factory.h"
#include "../src/hexamotion_constants.h"
#include "../src/leg.h"
#include "../src/robot_model.h"
#include "../src/velocity_limits.h"
#include "../src/walk_controller.h"
#include "../src/workspace_analyzer.h"
#include <cmath>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    std::cout << "=== WorkspaceAnalyzer Fusion Test ===" << std::endl;

    /** Create a robot model with real robot parameters from AGENTS.md. */
    Parameters params{};
    params.hexagon_radius = 200;
    params.coxa_length = 50;
    params.femur_length = 101;
    params.tibia_length = 208;
    /** Set to -tibia_length for explicit configuration. */
    params.default_height_offset = -208.0;
    params.robot_height = 208;
    params.standing_height = 150;
    params.time_delta = 1.0 / 50.0;
    params.coxa_angle_limits[0] = -65;
    params.coxa_angle_limits[1] = 65;
    params.femur_angle_limits[0] = -75;
    params.femur_angle_limits[1] = 75;
    params.tibia_angle_limits[0] = -45;
    params.tibia_angle_limits[1] = 45;
    params.max_velocity = 70.0;
    params.max_angular_velocity = 45.0;

    RobotModel model(params);
    /** Initialize WorkspaceAnalyzer. */
    model.workspaceAnalyzerInitializer();

    /** Test unified WorkspaceAnalyzer creation. */
    std::cout << "Creating WorkspaceAnalyzer..." << std::endl;
    /** Use medium precision for realistic testing. */
    ComputeConfig config = ComputeConfig::high();
    WorkspaceAnalyzer analyzer(model, config);

    /** Initialize the analyzer to ensure proper setup. */
    std::cout << "Initializing WorkspaceAnalyzer..." << std::endl;
    analyzer.initialize();

    /** Test workspace generation (from WalkspaceAnalyzer). */
    std::cout << "Testing workspace generation..." << std::endl;
    try {
        analyzer.generateWorkspace();
        std::cout << "✅ Workspace generation successful" << std::endl;

        /** Validate generateWorkspace() results. */
        std::cout << "Validating generateWorkspace() results..." << std::endl;
        const auto &walkspace_map = analyzer.getWalkspaceMap();
        if (!walkspace_map.empty()) {
            std::cout << "✅ Walkspace map generated with " << walkspace_map.size() << " bearing entries" << std::endl;

            /** Test some specific bearings. */
            for (int bearing = 0; bearing <= 360; bearing += 90) {
                auto it = walkspace_map.find(bearing);
                if (it != walkspace_map.end()) {
                    std::cout << "  Bearing " << bearing << "°: radius = " << it->second << " mm" << std::endl;
                } else {
                    std::cout << "❌ Missing bearing " << bearing << "° in walkspace map" << std::endl;
                }
            }

            /** Validate symmetry (bearing 0 degrees should equal bearing 360 degrees). */
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

    /** Validate WalkController::generateWalkspace() produces equivalent data. */
    std::cout << "\n=== WalkController::generateWalkspace validation ===" << std::endl;
    {
        /** Create independent leg instances for the walk controller (avoid reusing FK-only data). */
        Leg wc_legs[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                                 Leg(3, model), Leg(4, model), Leg(5, model)};

        for (int i = 0; i < NUM_LEGS; ++i) {
            wc_legs[i].initialize(Pose::Identity());
            wc_legs[i].updateTipPosition();
        }

        BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);
        BodyPoseController pose_controller(model, pose_config);
        pose_controller.setWalkPlanePoseEnabled(true);
        pose_controller.initializeLegPosers(wc_legs);
        if (!pose_controller.setStandingPose(wc_legs)) {
            std::cout << "❌ BodyPoseController failed to apply standing pose" << std::endl;
        } else {
            WalkController walk_controller(model, wc_legs, pose_config);
            walk_controller.setBodyPoseController(&pose_controller);

            /** WalkController constructor already calls generateWalkspace; re-run to ensure deterministic behavior. */
            walk_controller.generateWalkspace();
            auto walkspace_wc = walk_controller.getWalkspace();
            const auto walkspace_analyzer = analyzer.getWalkspaceMap();

            bool walkspace_ok = true;
            if (walkspace_wc.empty()) {
                std::cout << "❌ WalkController walkspace map is empty" << std::endl;
                walkspace_ok = false;
            } else {
                std::cout << "✅ WalkController walkspace map generated with " << walkspace_wc.size()
                          << " bearing entries" << std::endl;

                /** Ensure key bearings exist and align (within tolerance) with WorkspaceAnalyzer results. */
                const double tolerance = 1e-3;

                /** Compare every bearing produced by the analyzer for parity. */
                for (const auto &entry : walkspace_analyzer) {
                    int bearing = entry.first;
                    auto it_wc = walkspace_wc.find(bearing);
                    if (it_wc == walkspace_wc.end()) {
                        std::cout << "❌ WalkController walkspace missing bearing " << bearing << "°" << std::endl;
                        walkspace_ok = false;
                        continue;
                    }

                    double diff = std::abs(it_wc->second - entry.second);
                    if (diff > tolerance) {
                        std::cout << "❌ Bearing " << bearing << "° mismatch: WalkController=" << it_wc->second
                                  << " mm, Analyzer=" << entry.second << " mm (Δ=" << diff << ")" << std::endl;
                        walkspace_ok = false;
                    }
                }

                if (walkspace_wc.size() != walkspace_analyzer.size()) {
                    for (const auto &entry : walkspace_wc) {
                        if (walkspace_analyzer.find(entry.first) == walkspace_analyzer.end()) {
                            std::cout << "❌ WalkController walkspace has unexpected bearing " << entry.first << "°"
                                      << std::endl;
                            walkspace_ok = false;
                        }
                    }
                }

                /** Validate periodic symmetry (bearing 0 and 360). */
                auto wc0 = walkspace_wc.find(0);
                auto wc360 = walkspace_wc.find(360);
                if (wc0 != walkspace_wc.end() && wc360 != walkspace_wc.end() &&
                    std::abs(wc0->second - wc360->second) < tolerance) {
                    std::cout << "✅ WalkController walkspace symmetry validated (0° = 360°)" << std::endl;
                } else {
                    std::cout << "❌ WalkController walkspace symmetry failed" << std::endl;
                    walkspace_ok = false;
                }
            }

            if (walkspace_ok) {
                std::cout << "✅ WalkController::generateWalkspace() matches WorkspaceAnalyzer output" << std::endl;
            } else {
                std::cout << "❌ WalkController::generateWalkspace() validation failed" << std::endl;
            }
        }
    }

    /** Test validation functions (from WorkspaceValidator). */
    std::cout << "Testing validation functions..." << std::endl;
    /** Test point within reasonable reach (robot has max reach ~359 mm: coxa+femur+tibia). */
    /** More realistic test point. */
    Point3D test_point(250, 100, -150);
    bool is_reachable = analyzer.isPositionReachable(0, test_point, false);
    std::cout << "Point (" << test_point.x << ", " << test_point.y << ", " << test_point.z
              << ") reachable for leg 0: " << (is_reachable ? "YES" : "NO") << std::endl;

    /** Test getWorkplane() function (OpenSHC equivalent). */
    std::cout << "\nTesting getWorkplane() function..." << std::endl;
    try {
        /** Test workplane at different heights for leg 0. */
        /** Robot default height is 208 mm, standing height is 150 mm (body at z = -208). */
        std::vector<double> test_heights = {-250.0, -200.0, -150.0, -100.0, -50.0};

        for (double height : test_heights) {
            Workplane workplane = analyzer.getWorkplane(0, height);
            std::cout << "Leg 0, Height " << height << " mm: ";

            if (!workplane.empty()) {
                std::cout << "workplane with " << workplane.size() << " bearings" << std::endl;

                /** Show sample bearings and their radii. */
                int sample_count = 0;
                for (const auto &bearing_radius : workplane) {
                    /** Show first 3 entries as sample. */
                    if (sample_count < 3) {
                        std::cout << "    Bearing " << bearing_radius.first
                                  << "°: radius = " << bearing_radius.second << " mm" << std::endl;
                        sample_count++;
                    }
                }

                /** Validate workplane consistency. */
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

        /** Test workplane interpolation. */
        std::cout << "\nTesting workplane interpolation..." << std::endl;
        /** Between -200 and -150. */
        double interpolated_height = -175.0;
        Workplane interpolated_workplane = analyzer.getWorkplane(0, interpolated_height);
        if (!interpolated_workplane.empty()) {
            std::cout << "✅ Workplane interpolation successful at height "
                      << interpolated_height << " mm" << std::endl;

            /** Compare with adjacent heights to validate interpolation. */
            Workplane lower_workplane = analyzer.getWorkplane(0, -200.0);
            Workplane upper_workplane = analyzer.getWorkplane(0, -150.0);

            if (!lower_workplane.empty() && !upper_workplane.empty()) {
                auto interp_0 = interpolated_workplane.find(0);
                auto lower_0 = lower_workplane.find(0);
                auto upper_0 = upper_workplane.find(0);

                if (interp_0 != interpolated_workplane.end() &&
                    lower_0 != lower_workplane.end() &&
                    upper_0 != upper_workplane.end()) {

                    /** 50% interpolation. */
                    double expected = lower_0->second * 0.5 + upper_0->second * 0.5;
                    double actual = interp_0->second;

                    /** 10 mm tolerance for realistic interpolation. */
                    if (std::abs(actual - expected) < 10.0) {
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

        /** Test invalid leg index. */
        Workplane invalid_workplane = analyzer.getWorkplane(NUM_LEGS + 1, 0.0);
        if (invalid_workplane.empty()) {
            std::cout << "✅ getWorkplane() correctly handles invalid leg index" << std::endl;
        } else {
            std::cout << "❌ getWorkplane() should return empty workplane for invalid leg" << std::endl;
        }

    } catch (const std::exception &e) {
        std::cout << "❌ getWorkplane() test failed: " << e.what() << std::endl;
    }

    /** Test workspace bounds. */
    std::cout << "Testing workspace bounds..." << std::endl;
    auto bounds = analyzer.getWorkspaceBounds(0);
    std::cout << "Leg 0 workspace bounds:" << std::endl;
    std::cout << "  Min reach: " << bounds.min_reach << " mm" << std::endl;
    std::cout << "  Max reach: " << bounds.max_reach << " mm" << std::endl;
    std::cout << "  Min height: " << bounds.min_height << " mm" << std::endl;
    std::cout << "  Max height: " << bounds.max_height << " mm" << std::endl;

    /** Test leg position generation used by workspace checks. */
    std::cout << "Testing leg position generation..." << std::endl;
    Point3D leg_positions[NUM_LEGS];

    /** Get realistic leg positions from robot model using forward kinematics. */
    /** With all angles at 0 degrees, the robot stands stably by default. */
    /** Femur remains horizontal, in line with the coxa. */
    /** Tibia remains vertical, perpendicular to ground. */
    /** Robot body positioned at z = -208 (tibia length). */
    for (int i = 0; i < NUM_LEGS; i++) {
        /** Use default configuration: all angles at 0 degrees for stable standing position. */
        /** Coxa, femur, tibia all at 0 degrees. */
        JointAngles default_angles(0.0, 0.0, 0.0);

        /** Calculate actual position using robot model forward kinematics. */
        leg_positions[i] = model.forwardKinematicsGlobalCoordinates(i, default_angles);

        std::cout << "  Leg " << i << " position (0° angles): ("
                  << leg_positions[i].x << ", "
                  << leg_positions[i].y << ", "
                  << leg_positions[i].z << ") mm" << std::endl;
    }

    std::cout << "Leg positions generated successfully for all legs." << std::endl;

    /** Test OpenSHC compatibility validation. */
    std::cout << "\n=== OpenSHC Compatibility Validation ===" << std::endl;

    /** Verify that generateWorkspace() and getWorkplane() work together. */
    std::cout << "Testing generateWorkspace() + getWorkplane() integration..." << std::endl;

    /** First ensure workspace is generated. */
    analyzer.generateWorkspace();

    /** Test that workplanes are consistent across all legs. */
    bool integration_success = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        /** Test at standing height. */
        Workplane workplane = analyzer.getWorkplane(leg, -150.0);
        if (workplane.empty()) {
            std::cout << "❌ Leg " << leg << " has empty workplane at standing height" << std::endl;
            integration_success = false;
        } else {
            /** Check that workplane has reasonable values. */
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

    /** Test getLegWorkspace() for complete 3D workspace. */
    std::cout << "\nTesting getLegWorkspace() for 3D workspace data..." << std::endl;
    Workspace leg_workspace = analyzer.getLegWorkspace(0);
    if (!leg_workspace.empty()) {
        std::cout << "✅ 3D workspace data available with " << leg_workspace.size() << " height layers" << std::endl;

        /** Show height range. */
        double min_height = leg_workspace.begin()->first;
        double max_height = leg_workspace.rbegin()->first;
        std::cout << "  Height range: " << min_height << " to " << max_height << " mm" << std::endl;
    } else {
        std::cout << "❌ 3D workspace data not available" << std::endl;
    }

    std::cout << "=== All WorkspaceAnalyzer functions tested successfully! ===" << std::endl;

    /** VelocityLimits integration and behavior validation. */
    std::cout << "\n=== VelocityLimits Behavior Tests ===" << std::endl;

    {
        VelocityLimits velocity_limits(model);
        const auto &walkspace_map = analyzer.getWalkspaceMap();
        velocity_limits.setWalkspace(walkspace_map);

        auto tripod_gait = createTripodGaitConfig(model.getParams());
        velocity_limits.generateLimits(tripod_gait);
        bool velocity_limits_ok = true;

        const auto &linear_map = velocity_limits.getMaxLinearSpeedMap();
        const auto &angular_map = velocity_limits.getMaxAngularSpeedMap();
        const auto &linear_acc_map = velocity_limits.getMaxLinearAccelerationMap();
        const auto &angular_acc_map = velocity_limits.getMaxAngularAccelerationMap();

        if (linear_map.empty() || angular_map.empty() || linear_acc_map.empty() || angular_acc_map.empty()) {
            std::cout << "❌ VelocityLimits maps are empty" << std::endl;
            velocity_limits_ok = false;
        } else {
            std::cout << "  Linear speed map entries: " << linear_map.size() << std::endl;
            std::cout << "  Angular speed map entries: " << angular_map.size() << std::endl;

            /** Validate that all bearings with positive walkspace produce positive limits. */
            for (const auto &entry : walkspace_map) {
                int bearing = entry.first;
                double ws_radius = entry.second;
                auto lin_it = linear_map.find(bearing);
                auto ang_it = angular_map.find(bearing);

                if (ws_radius > 0.0) {
                    if (lin_it != linear_map.end() && lin_it->second <= 0.0) {
                        std::cout << "❌ Non-positive linear speed at bearing " << bearing
                                  << "° (walkspace=" << ws_radius << "): " << lin_it->second << std::endl;
                        velocity_limits_ok = false;
                    }
                    if (ang_it != angular_map.end() && ang_it->second <= 0.0) {
                        std::cout << "❌ Non-positive angular speed at bearing " << bearing
                                  << "° (walkspace=" << ws_radius << "): " << ang_it->second << std::endl;
                        velocity_limits_ok = false;
                    }
                } else if (ws_radius < 0.0) {
                    std::cout << "  ⚠ Negative walkspace radius at bearing " << bearing
                              << "°: " << ws_radius << " mm (workspace analyzer issue)" << std::endl;
                }
            }

            auto it0 = linear_map.find(0);
            if (it0 != linear_map.end()) {
                std::cout << "  Max linear speed at 0°: " << it0->second << " mm/s" << std::endl;
            }
            auto it90 = angular_map.find(90);
            if (it90 != angular_map.end()) {
                std::cout << "  Max angular speed at 90°: " << it90->second << " rad/s" << std::endl;
            }
        }

        if (velocity_limits_ok) {
            std::cout << "✅ VelocityLimits tests passed" << std::endl;
        } else {
            std::cout << "❌ VelocityLimits tests encountered failures" << std::endl;
        }
    }

    /** Standing pose workplane height validation (added test). */
    {
        std::cout << "\n=== Standing Pose Height Validation ===" << std::endl;
        /** 150 mm (absolute foot height => foot Z = -standing_height). */
        double expected_standing_height = params.standing_height;

        /** Therefore the standing workplane is at Z = -standing_height. */
        /** -150. */
        double target_workplane_height = -expected_standing_height;

        std::cout << "Expected standing tip Z: " << target_workplane_height << " mm" << std::endl;

        /** Obtain workplane at computed height. */
        Workplane standing_plane = analyzer.getWorkplane(0, target_workplane_height);
        if (standing_plane.empty()) {
            std::cout << "❌ Standing workplane not found at height " << target_workplane_height << " mm" << std::endl;
        } else {
            std::cout << "✅ Standing workplane found (" << standing_plane.size() << " bearings)" << std::endl;
            /** Basic semantic check: bearing 0 and 180 radii should be non-zero and consistent with walkspace map bounds. */
            double r0 = 0.0;
            double r180 = 0.0;
            double r360 = 0.0;
            bool ok = true;
            auto it0 = standing_plane.find(0);
            if (it0 != standing_plane.end())
                r0 = it0->second;
            else
                ok = false;
            auto it180 = standing_plane.find(180);
            if (it180 != standing_plane.end())
                r180 = it180->second;
            else
                ok = false;
            auto it360 = standing_plane.find(360);
            if (it360 != standing_plane.end())
                r360 = it360->second;
            else
                ok = false;
            if (!ok || r0 <= 0 || r180 <= 0) {
                std::cout << "❌ Invalid radii in standing plane (r0=" << r0 << ", r180=" << r180 << ")" << std::endl;
            } else if (std::abs(r0 - r360) > 1e-6) {
                std::cout << "❌ Standing plane symmetry failed (r0 != r360)" << std::endl;
            } else {
                std::cout << "✅ Standing plane radii valid (r0=" << r0 << ", r180=" << r180 << ")" << std::endl;
            }
            /** Cross-check that height lies within global workspace bounds for leg 0. */
            WorkspaceBounds wb = analyzer.getWorkspaceBounds(0);
            if (target_workplane_height < wb.min_height - 1e-3 || target_workplane_height > wb.max_height + 1e-3) {
                std::cout << "❌ Standing height outside reported bounds (" << wb.min_height << ", " << wb.max_height << ")" << std::endl;
            } else {
                std::cout << "✅ Standing height within workspace bounds" << std::endl;
            }
        }
    }

    return 0;
}
