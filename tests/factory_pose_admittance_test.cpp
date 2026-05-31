#include "../src/admittance_controller.h"
#include "../src/auto_poser.h"
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
 * @file factory_pose_admittance_test.cpp
 * @brief Coverage test for the residual gaps documented in TODO_TEST_COVERAGE.md:
 *        [G] gait_config_factory, [I] body_pose_controller residual methods,
 *        [L] admittance_controller::updateStiffness and [M] auto_poser.h.
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

/** Validate a single gait configuration has consistent phase timing. */
static void assertValidGait(const GaitConfiguration &cfg, GaitType expected_type) {
    assert(!cfg.gait_name.empty());
    assert(cfg.gait_type == expected_type);
    assert(cfg.phase_config.stance_phase > 0);
    assert(cfg.phase_config.swing_phase > 0);
    assert(cfg.offsets.multipliers.size() == NUM_LEGS);
    StepCycle sc = cfg.generateStepCycle();
    assert(sc.period_ > 0);
    assert(sc.swing_end_ <= sc.period_);
}

/** [G] gait_config_factory: every factory and the GaitType dispatcher. */
static int testGaitConfigFactory(const Parameters &p) {
    std::cout << "=== [G] gait_config_factory ===" << std::endl;

    assertValidGait(createWaveGaitConfig(p), WAVE_GAIT);
    assertValidGait(createTripodGaitConfig(p), TRIPOD_GAIT);
    assertValidGait(createRippleGaitConfig(p), RIPPLE_GAIT);
    assertValidGait(createMetachronalGaitConfig(p), METACHRONAL_GAIT);

    // Dispatcher path for every supported GaitType.
    assertValidGait(createGaitConfig(WAVE_GAIT, p), WAVE_GAIT);
    assertValidGait(createGaitConfig(TRIPOD_GAIT, p), TRIPOD_GAIT);
    assertValidGait(createGaitConfig(RIPPLE_GAIT, p), RIPPLE_GAIT);
    assertValidGait(createGaitConfig(METACHRONAL_GAIT, p), METACHRONAL_GAIT);
    // ADAPTIVE_GAIT falls back to metachronal; NO_GAIT falls back to tripod.
    assertValidGait(createGaitConfig(ADAPTIVE_GAIT, p), METACHRONAL_GAIT);
    assertValidGait(createGaitConfig(NO_GAIT, p), TRIPOD_GAIT);

    // Tripod offsets must form two balanced phase groups (values 0 and 1).
    GaitConfiguration tripod = createTripodGaitConfig(p);
    int group0 = 0, group1 = 0;
    for (const auto &kv : tripod.offsets.multipliers) {
        if (kv.second == 0)
            group0++;
        else if (kv.second == 1)
            group1++;
    }
    assert(group0 == 3 && group1 == 3);

    std::cout << "[G] OK" << std::endl;
    return 0;
}

/** [I] body_pose_controller residual sequence/pose methods. */
static int testBodyPoseControllerResiduals(Parameters p) {
    std::cout << "=== [I] body_pose_controller residuals ===" << std::endl;

    RobotModel model(p);
    model.workspaceAnalyzerInitializer();

    BodyPoseConfiguration cfg = getDefaultBodyPoseConfig(p);
    BodyPoseController pc(model, cfg);

    Leg legs[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                          Leg(3, model), Leg(4, model), Leg(5, model)};
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].initialize(Pose::Identity());
        legs[i].updateTipPosition();
    }
    pc.initializeLegPosers(legs);
    assert(testSetStandingPose(pc, model, legs));

    // Gait-type and gait-phase parameter setters (also exercise resetSequenceStates).
    pc.setCurrentGaitType(TRIPOD_GAIT);
    assert(pc.getCurrentGaitType() == TRIPOD_GAIT);
    pc.setGaitPhaseParams(2, 2, 2);
    pc.refreshAutoPoseParameters();
    pc.resetSequenceStates();
    assert(pc.getStartupPhase() == 0);

    // calculateDefaultPose / getDefaultBodyPose.
    pc.calculateDefaultPose(legs);
    Pose default_pose = pc.getDefaultBodyPose();
    (void)default_pose;

    // updateIKErrorPose must run without throwing.
    pc.setIKErrorPoseEnabled(true);
    pc.updateIKErrorPose(legs);

    // stepToNewStance returns a progress percentage in [0,100].
    int stance_progress = pc.stepToNewStance();
    assert(stance_progress >= 0 && stance_progress <= 100);

    // transitionConfiguration drives joint-space interpolation; progress in [0,100].
    int trans_progress = pc.transitionConfiguration(0.5, legs);
    assert(trans_progress >= 0 && trans_progress <= 100);

    std::cout << "[I] OK" << std::endl;
    return 0;
}

/** [L] admittance_controller::updateStiffness (both overloads). */
static int testAdmittanceUpdateStiffness(Parameters p) {
    std::cout << "=== [L] admittance updateStiffness ===" << std::endl;

    RobotModel model(p);
    model.workspaceAnalyzerInitializer();

    BodyPoseConfiguration cfg = getDefaultBodyPoseConfig(p);
    BodyPoseController pc(model, cfg);

    Leg legs[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                          Leg(3, model), Leg(4, model), Leg(5, model)};
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].initialize(Pose::Identity());
        legs[i].updateTipPosition();
    }
    pc.initializeLegPosers(legs);
    assert(testSetStandingPose(pc, model, legs));

    AdmittanceController admittance(p);

    // Single-leg transition overload: scale 0->1 should set a valid stiffness.
    admittance.updateStiffness(legs, 0, 0.5);
    for (int i = 0; i < NUM_LEGS; ++i) {
        assert(legs[i].getVirtualStiffness() >= 0.0);
    }

    // Walker overload: drive a few walk cycles so at least one leg enters swing.
    WalkController wc(model, legs, cfg);
    wc.setBodyPoseController(&pc);
    GaitConfiguration tripod = createTripodGaitConfig(p);
    wc.setGaitConfiguration(tripod);
    wc.generateWalkspace();

    Eigen::Vector3d body_pos(0.0, 0.0, -p.standing_height);
    Eigen::Vector3d body_orient(0.0, 0.0, 0.0);
    bool saw_swing = false;
    for (int step = 0; step < 20; ++step) {
        wc.updateWalk(Point3D(30.0, 0.0, 0.0), 0.0, body_pos, body_orient);
        admittance.updateStiffness(legs, &wc);
        for (int i = 0; i < NUM_LEGS; ++i) {
            auto ls = wc.getLegStepper(i);
            if (ls && ls->getStepState() == STEP_SWING)
                saw_swing = true;
        }
    }
    // The swing branch (dynamic stiffness scaling) was exercised at least once.
    assert(saw_swing);

    std::cout << "[L] OK" << std::endl;
    return 0;
}

/** [M] auto_poser.h: AutoPoser lifecycle and Bezier pose generation. */
static int testAutoPoser() {
    std::cout << "=== [M] auto_poser ===" << std::endl;

    AutoPoser poser(7);
    assert(poser.getIDNumber() == 7);

    poser.setStartPhase(0);
    poser.setEndPhase(4);
    poser.setXAmplitude(5.0);
    poser.setYAmplitude(3.0);
    poser.setZAmplitude(2.0);
    poser.setRollAmplitude(0.05);
    poser.setPitchAmplitude(0.03);
    poser.setYawAmplitude(0.02);
    poser.setGravityAmplitude(0.0);
    poser.resetChecks();
    assert(!poser.isPosing());

    const int phase_length = 8;
    const int normaliser = 1;
    const double pose_frequency = -1.0; // gait-synced lifecycle
    Eigen::Vector3d gravity(0.0, 0.0, -1.0);

    // Drive through a full posing window starting in POSING (state 0).
    bool produced_nonidentity = false;
    for (int phase = 0; phase < phase_length; ++phase) {
        Pose pose = poser.updatePose(phase, phase_length, normaliser, /*POSING*/ 0,
                                     pose_frequency, gravity);
        if (std::abs(pose.position.x) > 1e-9 || std::abs(pose.position.y) > 1e-9 ||
            std::abs(pose.position.z) > 1e-9)
            produced_nonidentity = true;
    }
    assert(poser.isPosing());
    assert(produced_nonidentity);

    // Gravity-aligned amplitude branch (mutually exclusive with xyz).
    AutoPoser grav_poser(9);
    grav_poser.setStartPhase(0);
    grav_poser.setEndPhase(4);
    grav_poser.setGravityAmplitude(4.0);
    grav_poser.resetChecks();
    bool grav_nonidentity = false;
    for (int phase = 0; phase < phase_length; ++phase) {
        Pose pose = grav_poser.updatePose(phase, phase_length, normaliser, 0,
                                          pose_frequency, gravity);
        if (std::abs(pose.position.z) > 1e-9)
            grav_nonidentity = true;
    }
    assert(grav_nonidentity);

    // Free-running frequency path (sync_with_step_cycle == false).
    AutoPoser free_poser(11);
    free_poser.setStartPhase(0);
    free_poser.setEndPhase(4);
    free_poser.setZAmplitude(1.0);
    free_poser.resetChecks();
    free_poser.updatePose(1, phase_length, normaliser, 0, /*pose_frequency*/ 1.0, gravity);
    assert(free_poser.isPosing());

    std::cout << "[M] OK" << std::endl;
    return 0;
}

int main() {
    Parameters p = makeParams();
    int rc = 0;
    rc |= testGaitConfigFactory(p);
    rc |= testBodyPoseControllerResiduals(p);
    rc |= testAdmittanceUpdateStiffness(p);
    rc |= testAutoPoser();
    std::cout << (rc == 0 ? "ALL OK" : "FAILURES") << std::endl;
    return rc;
}
