#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
#include "../src/gait_config_factory.h"
#include "../src/hexamotion_constants.h"
#include "../src/robot_model.h"
#include "../src/walk_controller.h"
#include "test_pose_helpers.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iostream>

/**
 * @file walk_controller_accessor_test.cpp
 * @brief Coverage test for the residual WalkController accessors documented in
 *        TODO_TEST_COVERAGE.md section [B]: gait setters, terrain-mode toggles,
 *        step/timing/walk-plane getters, odometry and leg trajectory info.
 */

/** Build the standard HexaMotion test parameter set. */
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
    std::cout << "=== [B] WalkController residual accessors ===" << std::endl;

    Parameters p = makeParams();
    RobotModel model(p);
    model.workspaceAnalyzerInitializer();

    Leg legs[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                          Leg(3, model), Leg(4, model), Leg(5, model)};
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].initialize(Pose::Identity());
        legs[i].updateTipPosition();
    }

    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    BodyPoseController pose_controller(model, pose_config);
    pose_controller.initializeLegPosers(legs);
    assert(testSetStandingPose(pose_controller, model, legs));

    WalkController wc(model, legs, pose_config);
    wc.setBodyPoseController(&pose_controller);

    // ── setGait overloads ───────────────────────────────────────────────────
    GaitConfiguration tripod = createTripodGaitConfig(p);
    assert(wc.setGait(tripod));
    assert(wc.getCurrentGaitName() == tripod.gait_name);
    assert(wc.setGait(WAVE_GAIT));
    assert(!wc.getCurrentGaitName().empty());
    // Restore tripod for the remaining checks.
    assert(wc.setGait(TRIPOD_GAIT));
    wc.generateWalkspace();

    // ── Step / timing accessors ─────────────────────────────────────────────
    assert(wc.getStepClearance() > 0.0);
    assert(wc.getStepDepth() >= 0.0);
    assert(wc.getTimeDelta() > 0.0);
    assert(wc.getSwingDuration() > 0.0);
    assert(wc.getCycleFrequency() > 0.0);
    StepCycle sc = wc.getStepCycle();
    assert(sc.period_ > 0);

    // ── Walk-plane / state accessors ────────────────────────────────────────
    Point3D walk_plane = wc.getWalkPlane();
    Point3D walk_plane_normal = wc.getWalkPlaneNormal();
    (void)walk_plane;
    // Normal should be a unit-ish vector pointing up by default.
    assert(walk_plane_normal.z != 0.0);
    assert(wc.getWalkState() == WALK_STOPPED);
    Pose odometry_ideal = wc.getOdometryIdeal();
    (void)odometry_ideal;

    // ── Terrain-mode toggles and accessors ──────────────────────────────────
    wc.enableRoughTerrainMode(true, true, true);
    wc.enableForceNormalTouchdown(true);
    const TerrainAdaptation::WalkPlane &terrain_plane = wc.getTerrainWalkPlane();
    (void)terrain_plane;
    const TerrainAdaptation::StepPlane &step_plane = wc.getStepPlane(0);
    (void)step_plane;
    bool touchdown = wc.hasTouchdownDetection(0);
    (void)touchdown;
    // Disable again to exercise the false branch.
    wc.enableForceNormalTouchdown(false);
    wc.enableRoughTerrainMode(false, false, false);

    // updateTerrainAdaptation with dummy sensors.
    DummyFSR fsr;
    DummyIMU imu;
    wc.updateTerrainAdaptation(&fsr, &imu);

    // ── Drive walking to populate odometry and leg trajectory info ──────────
    Eigen::Vector3d body_pos(0.0, 0.0, -p.standing_height);
    Eigen::Vector3d body_orient(0.0, 0.0, 0.0);
    for (int step = 0; step < 10; ++step) {
        wc.updateWalk(Point3D(30.0, 0.0, 0.0), 0.0, body_pos, body_orient);
    }
    assert(wc.getWalkState() != WALK_STOPPED);

    Pose odom = wc.calculateOdometry(p.time_delta);
    (void)odom;

    for (int i = 0; i < NUM_LEGS; ++i) {
        WalkController::LegTrajectoryInfo info = wc.getLegTrajectoryInfo(i);
        // phase_progress is a normalized fraction in [0,1].
        assert(info.phase_progress >= -1e-6 && info.phase_progress <= 1.0 + 1e-6);
    }

    // ── updateManual (tip-velocity and tip-position overloads) ──────────────
    Eigen::Vector3d primary_vel(1.0, 0.0, 0.0);
    Eigen::Vector3d secondary_vel(0.0, 1.0, 0.0);
    wc.updateManual(0, primary_vel, 3, secondary_vel);

    Point3D primary_pos = legs[0].getCurrentTipPositionGlobal();
    Point3D secondary_pos = legs[3].getCurrentTipPositionGlobal();
    wc.updateManual(0, primary_pos, 3, secondary_pos);

    std::cout << "[B] OK" << std::endl;
    std::cout << "ALL OK" << std::endl;
    return 0;
}
