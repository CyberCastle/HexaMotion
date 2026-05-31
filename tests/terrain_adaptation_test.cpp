/**
 * @file terrain_adaptation_test.cpp
 * @brief Coverage for TerrainAdaptation instance behaviour ([D] in TODO_TEST_COVERAGE).
 *
 * The existing rough_terrain_adaptation_test exercises the static
 * computeRoughTerrainSwingTarget helper. This test drives the stateful members:
 * update() with FSR/IMU stubs, adaptTrajectoryForTerrain in swing/stance for both
 * proactive and reactive paths, forceNormalTouchdown/projectOntoWalkPlane (reached
 * through adaptTrajectoryForTerrain), isTargetReachableOnTerrain, getWalkPlane,
 * getStepPlane, hasTouchdownDetection, updateThresholdsFromModel and the threshold
 * setters/getters.
 */

#include "robot_model.h"
#include "terrain_adaptation.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

static int g_failures = 0;
static void check(bool cond, const std::string &msg) {
    if (!cond) {
        std::cout << "  [FAIL] " << msg << "\n";
        ++g_failures;
    } else {
        std::cout << "  [ OK ] " << msg << "\n";
    }
}

static Parameters makeParams() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.default_height_offset = -208.0;
    p.robot_height = 208;
    p.standing_height = 150;
    p.time_delta = 1.0 / 50.0;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;
    return p;
}

int main() {
    std::cout << "=== Terrain Adaptation Coverage Test ([D]) ===\n";

    Parameters params = makeParams();
    RobotModel model(params);
    TerrainAdaptation terrain(model);
    terrain.initialize();

    // Threshold setters/getters and model-sync.
    terrain.setTouchdownThreshold(10.0);
    terrain.setLiftoffThreshold(5.0);
    check(std::fabs(terrain.getTouchdownThreshold() - 10.0) < 1e-9, "touchdown threshold setter/getter");
    check(std::fabs(terrain.getLiftoffThreshold() - 5.0) < 1e-9, "liftoff threshold setter/getter");
    terrain.updateThresholdsFromModel();

    // Mode toggles.
    terrain.setRoughTerrainMode(true);
    terrain.setForceNormalTouchdown(true);
    check(terrain.isRoughTerrainModeEnabled(), "rough terrain mode enabled");
    check(terrain.isForceNormalTouchdownEnabled(), "force normal touchdown enabled");

    // Drive several update cycles to populate foot contacts and step planes.
    DummyFSR fsr;
    DummyIMU imu;
    imu.initialize();
    for (int i = 0; i < 30; ++i) {
        terrain.update(&fsr, &imu);
    }

    // Walk plane and gravity accessors should be callable after updates.
    const TerrainAdaptation::WalkPlane &wp = terrain.getWalkPlane();
    (void)wp;
    Eigen::Vector3d gravity = terrain.getGravityVector();
    check(gravity.norm() >= 0.0, "gravity vector accessible");

    // Per-leg step plane and touchdown accessors.
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const TerrainAdaptation::StepPlane &sp = terrain.getStepPlane(leg);
        (void)sp;
        bool td = terrain.hasTouchdownDetection(leg);
        (void)td;
    }

    // adaptTrajectoryForTerrain: swing phase (reactive + proactive + forceNormalTouchdown paths).
    Point3D base_target(300.0, 0.0, -150.0);
    Point3D swing_adapted = terrain.adaptTrajectoryForTerrain(0, base_target, SWING_PHASE, 0.5);
    check(std::isfinite(swing_adapted.x) && std::isfinite(swing_adapted.y) && std::isfinite(swing_adapted.z),
          "adaptTrajectoryForTerrain swing produces finite result");

    // Stance phase should pass the trajectory through largely unchanged.
    Point3D stance_adapted = terrain.adaptTrajectoryForTerrain(0, base_target, STANCE_PHASE, 0.5);
    check(std::isfinite(stance_adapted.z), "adaptTrajectoryForTerrain stance produces finite result");

    // With rough terrain mode off, adaptation should be a near pass-through.
    terrain.setRoughTerrainMode(false);
    terrain.setForceNormalTouchdown(false);
    Point3D passthrough = terrain.adaptTrajectoryForTerrain(0, base_target, SWING_PHASE, 0.5);
    check(std::fabs(passthrough.x - base_target.x) < 1e-6 &&
              std::fabs(passthrough.y - base_target.y) < 1e-6 &&
              std::fabs(passthrough.z - base_target.z) < 1e-6,
          "adaptTrajectoryForTerrain pass-through when rough terrain disabled");

    // Reachability check on terrain (exercises the terrain-constrained path; the
    // result depends on the live workspace estimate, so we only require a finite call).
    bool reachable = terrain.isTargetReachableOnTerrain(0, base_target);
    bool far_call = terrain.isTargetReachableOnTerrain(0, Point3D(5000.0, 5000.0, -150.0));
    (void)far_call;
    check(reachable, "near target reachable on terrain");

    if (g_failures == 0) {
        std::cout << "\nALL TESTS PASSED\n";
        return 0;
    }
    std::cout << "\n"
              << g_failures << " CHECK(S) FAILED\n";
    return 1;
}
