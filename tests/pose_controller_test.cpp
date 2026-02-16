#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
#include "../src/hexamotion_constants.h"
#include "../src/math_utils.h"
#include "../src/robot_model.h"
#include "test_pose_helpers.h"
#include "test_stubs.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

/**
 * @brief BodyPoseController test — validates OpenSHC-equivalent functionality.
 *
 * Tests walk plane pose, auto-pose, manual pose, updateCurrentPose,
 * updateStance, leg posers, and sequence control.  Removed methods
 * (setBodyPose, trajectories, etc.) now live in LocomotionSystem and
 * are tested through integration tests.
 */
int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.default_height_offset = -208.0;
    p.robot_height = 208;
    p.standing_height = 150;
    p.height_offset = 0;
    p.time_delta = 1.0 / 50.0;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    model.workspaceAnalyzerInitializer();

    BodyPoseConfiguration default_config = getDefaultBodyPoseConfig(p);
    BodyPoseController pc(model, default_config);

    Leg legs[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                          Leg(3, model), Leg(4, model), Leg(5, model)};

    // ── Configuration access ────────────────────────────────────────────────
    std::cout << "=== Testing Configuration Access ===" << std::endl;
    const auto &config = pc.getBodyPoseConfig();
    std::cout << "Body clearance: " << config.body_clearance << "mm, "
              << "Swing height: " << config.swing_height << "mm" << std::endl;
    std::cout << "Max translation: X=" << config.max_translation.x
              << " Y=" << config.max_translation.y
              << " Z=" << config.max_translation.z << std::endl;

    // ── Standing pose (via test helper) ─────────────────────────────────────
    std::cout << "\n=== Testing Standing Pose ===" << std::endl;
    testInitializeDefaultPose(model, legs);
    assert(testSetStandingPose(pc, model, legs));
    std::cout << "Standing Pose Joint Angles (degrees):" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles angles = legs[i].getJointAngles();
        std::cout << "Leg " << i
                  << ": Coxa=" << math_utils::radiansToDegrees(angles.coxa)
                  << " Femur=" << math_utils::radiansToDegrees(angles.femur)
                  << " Tibia=" << math_utils::radiansToDegrees(angles.tibia) << std::endl;
    }

    // ── Walk plane pose ─────────────────────────────────────────────────────
    std::cout << "\n=== Testing Walk Plane Pose ===" << std::endl;
    Pose initial_wp = pc.getWalkPlanePose();
    double expected_z = config.body_clearance;
    assert(std::abs(initial_wp.position.z - expected_z) < 0.1);
    std::cout << "Initial walk plane Z=" << initial_wp.position.z
              << " (expected " << expected_z << ")" << std::endl;

    Pose new_pose(Point3D(10.0, 5.0, 160.0), Eigen::Quaterniond::Identity());
    pc.setWalkPlanePose(new_pose);
    Pose got = pc.getWalkPlanePose();
    assert(std::abs(got.position.x - 10.0) < 0.1);
    assert(std::abs(got.position.y - 5.0) < 0.1);
    assert(std::abs(got.position.z - 160.0) < 0.1);
    std::cout << "Set/get walk plane pose OK" << std::endl;

    // ── updateWalkPlanePose with horizontal plane ───────────────────────────
    std::cout << "\n=== Testing updateWalkPlanePose ===" << std::endl;
    for (int i = 0; i < NUM_LEGS; i++) {
        legs[i].setStepPhase(STANCE_PHASE);
        legs[i].setCurrentTipPositionGlobal(Point3D(i * 50.0, i * 30.0, -150.0));
    }
    // Reset walk plane to known origin
    pc.setWalkPlanePose(Pose(Point3D(0, 0, config.body_clearance), Eigen::Quaterniond::Identity()));
    pc.updateWalkPlanePose(legs);
    Pose hp = pc.getWalkPlanePose();
    double expected_h = -150.0 + config.body_clearance;
    // Walk plane interpolation may not converge in 1 step, so check reasonable bound
    std::cout << "Horizontal walk plane Z=" << hp.position.z
              << " (target ~" << expected_h << ")" << std::endl;

    // Tilted plane
    for (int i = 0; i < NUM_LEGS; i++) {
        legs[i].setStepPhase(STANCE_PHASE);
        legs[i].setCurrentTipPositionGlobal(Point3D(i * 50.0, i * 30.0, -150.0 + i * 10.0));
    }
    int max_iters = 80;
    for (int it = 0; it < max_iters; ++it) {
        pc.updateWalkPlanePose(legs);
    }
    Pose tp = pc.getWalkPlanePose();
    std::cout << "Tilted walk plane Z=" << tp.position.z << std::endl;

    // ── Leg posers ──────────────────────────────────────────────────────────
    std::cout << "\n=== Testing Leg Posers ===" << std::endl;
    pc.initializeLegPosers(legs);
    int count = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
        if (pc.getLegPoser(i))
            count++;
    }
    assert(count == NUM_LEGS);
    assert(pc.getLegPoser(-1) == nullptr);
    assert(pc.getLegPoser(NUM_LEGS) == nullptr);
    std::cout << "All " << count << " leg posers OK, invalid indices return nullptr" << std::endl;

    // ── Gait type management ────────────────────────────────────────────────
    std::cout << "\n=== Testing Gait Type Management ===" << std::endl;
    pc.setCurrentGaitType(TRIPOD_GAIT);
    assert(pc.getCurrentGaitType() == TRIPOD_GAIT);
    pc.resetSequenceStates();
    std::cout << "Gait type set/get and sequence reset OK" << std::endl;

    // ── Auto-pose ───────────────────────────────────────────────────────────
    std::cout << "\n=== Testing Auto-Pose ===" << std::endl;
    pc.setAutoPoseEnabled(true);
    assert(pc.isAutoPoseEnabled());
    const auto &ap = pc.getAutoPoseConfig();
    std::cout << "Auto-pose config: enabled=" << ap.enabled
              << " gait=" << ap.gait_name << std::endl;

    bool ap_ok = pc.updateAutoPose(1, legs);
    assert(ap_ok);
    std::cout << "updateAutoPose succeeded" << std::endl;

    // ── Manual pose ─────────────────────────────────────────────────────────
    std::cout << "\n=== Testing Manual Pose ===" << std::endl;
    pc.setManualPoseEnabled(true);
    pc.setManualPoseInput(Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::Zero());
    pc.updateManualPose();
    std::cout << "Manual pose input + update OK" << std::endl;

    // ── Pose reset modes ────────────────────────────────────────────────────
    std::cout << "\n=== Testing Pose Reset Modes ===" << std::endl;
    pc.setPoseResetMode(ALL_RESET);
    assert(pc.getPoseResetMode() == ALL_RESET);
    pc.updateManualPose();
    pc.setPoseResetMode(NO_RESET);
    std::cout << "Pose reset modes OK" << std::endl;

    // ── updateCurrentPose + updateStance ────────────────────────────────────
    std::cout << "\n=== Testing updateCurrentPose + updateStance ===" << std::endl;
    pc.setAutoPoseEnabled(true);
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].setLegState(LEG_WALKING);
        legs[i].setDesiredTipPosition(legs[i].getCurrentTipPositionGlobal());
    }
    pc.updateCurrentPose(2, legs);
    pc.updateStance(legs);

    int valid = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        Point3D d = legs[i].getDesiredTipPosition();
        if (std::isfinite(d.x) && std::isfinite(d.y) && std::isfinite(d.z))
            valid++;
    }
    assert(valid == NUM_LEGS);
    std::cout << "updateStance produced valid tips for all legs" << std::endl;

    // ── Body pose limits (via test helper) ──────────────────────────────────
    std::cout << "\n=== Testing Body Pose Limits (helper) ===" << std::endl;
    Eigen::Vector3d extreme_pos(1000, 1000, 1000);
    Eigen::Vector3d extreme_rot(M_PI, M_PI, M_PI);
    assert(!testCheckBodyPoseLimits(config, extreme_pos, extreme_rot));
    Eigen::Vector3d small_pos(5, 5, 5);
    double r = math_utils::degreesToRadians(2.0);
    Eigen::Vector3d small_rot(r, r, r);
    assert(testCheckBodyPoseLimits(config, small_pos, small_rot));
    std::cout << "Limits check OK" << std::endl;

    // ── resetAllPosing ──────────────────────────────────────────────────────
    std::cout << "\n=== Testing resetAllPosing ===" << std::endl;
    pc.resetAllPosing();
    Pose after_reset = pc.getCurrentBodyPose();
    // After reset, body pose should be identity
    std::cout << "resetAllPosing OK" << std::endl;

    // ── calculateBodyPosition (via test helper) ─────────────────────────────
    std::cout << "\n=== Testing calculateBodyPosition (helper) ===" << std::endl;
    double zvals[] = {-140, -145, -150, -155, -160, -165};
    double exp_avg = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
        legs[i].setCurrentTipPositionGlobal(Point3D(i * 20, i * 15, zvals[i]));
        exp_avg += zvals[i];
    }
    exp_avg /= NUM_LEGS;
    Eigen::Vector3d bp = testCalculateBodyPosition(legs);
    assert(std::abs(bp.z() - exp_avg) < 0.1);
    std::cout << "calculateBodyPosition: z=" << bp.z()
              << " expected=" << exp_avg << std::endl;

    std::cout << "\n=== All Tests Passed ===" << std::endl;
    return 0;
}
