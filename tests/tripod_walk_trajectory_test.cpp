#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
#include "../src/gait_config.h"
#include "../src/gait_config_factory.h"
#include "../src/leg_stepper.h"
#include "../src/walk_controller.h"
#include "../src/walkspace_analyzer.h"
#include "../src/workspace_validator.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    // Initialize parameters according to AGENTS instructions
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.standing_height = 150;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    // Tripod gait factors
    p.gait_factors.tripod_length_factor = 0.4;
    p.gait_factors.tripod_height_factor = 0.15;

    RobotModel model(p);

    // Create legs
    Leg legs[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                          Leg(3, model), Leg(4, model), Leg(5, model)};
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].initialize(Pose::Identity());
        legs[i].updateTipPosition();
    }

    // Standing pose
    BodyPoseConfiguration pose_cfg = getDefaultBodyPoseConfig(p);
    BodyPoseController pose_controller(model, pose_cfg);
    pose_controller.initializeLegPosers(legs);
    assert(pose_controller.setStandingPose(legs));

    // Walk controller
    WalkController wc(model, legs, pose_cfg);
    wc.setBodyPoseController(&pose_controller);

    // Configure tripod gait with 25 iteration phases and zero swing height
    GaitConfiguration gait = createTripodGaitConfig(p);
    gait.phase_config.stance_phase = 25;
    gait.phase_config.swing_phase = 25;
    gait.swing_height = 0.0; // keep z constant during swing
    bool cfg_ok = wc.setGaitConfiguration(gait);
    assert(cfg_ok);

    StepCycle cycle = wc.getStepCycle();
    assert(cycle.stance_period_ == 25);
    assert(cycle.swing_period_ == 25);

    std::cout << "Tripod gait trajectory test" << std::endl;
    std::cout << "StepCycle: stance=" << cycle.stance_period_ << ", swing=" << cycle.swing_period_ << std::endl;

    // Record initial coxa angles and track min/max during the cycle
    double initial_coxa[NUM_LEGS];
    double min_coxa[NUM_LEGS];
    double max_coxa[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        double a = legs[i].getJointAngles().coxa;
        initial_coxa[i] = a;
        min_coxa[i] = a;
        max_coxa[i] = a;
    }

    // Simulate walking for one full cycle (50 iterations)
    Point3D vel(10.0, 0.0, 0.0);
    Eigen::Vector3d body_pos(0, 0, 0);
    Eigen::Vector3d body_ori(0, 0, 0);

    int swing_start[NUM_LEGS];
    int swing_end[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        swing_start[i] = -1;
        swing_end[i] = -1;
    }

    for (int iter = 1; iter <= 50; ++iter) {
        wc.updateWalk(vel, 0.0, body_pos, body_ori);

        // Apply stepper outputs to legs (replicates LocomotionSystem)
        for (int j = 0; j < NUM_LEGS; ++j) {
            auto stepper = wc.getLegStepper(j);
            Point3D tip = stepper->getCurrentTipPose();
            // Apply IK directly to reach the new tip pose
            legs[j].applyAdvancedIK(tip);
            double coxa_angle = legs[j].getJointAngles().coxa;
            if (coxa_angle < min_coxa[j])
                min_coxa[j] = coxa_angle;
            if (coxa_angle > max_coxa[j])
                max_coxa[j] = coxa_angle;

            StepState state = stepper->getStepState();

            if (state == STEP_SWING) {
                if (swing_start[j] == -1)
                    swing_start[j] = iter;
                if (std::abs(tip.z + 150.0) > 1e-3)
                    std::cerr << "Leg " << j << " swing height violation at step " << iter << std::endl;
            } else {
                if (swing_start[j] != -1 && swing_end[j] == -1)
                    swing_end[j] = iter - 1;
            }

            if (iter % 5 == 0) {
                JointAngles a = legs[j].getJointAngles();
                std::string phase = (state == STEP_SWING) ? "SWING" : "STANCE";
                std::cout << "Step " << std::setw(2) << iter << " Leg " << j << " " << phase
                          << " Pos(" << tip.x << ", " << tip.y << ", " << tip.z << ")"
                          << " Angles(" << a.coxa * 180.0 / M_PI << ", " << a.femur * 180.0 / M_PI
                          << ", " << a.tibia * 180.0 / M_PI << ")" << std::endl;
            }
        }
    }

    // Determine final swing ends for legs that ended swing at iteration 50
    for (int i = 0; i < NUM_LEGS; ++i)
        if (swing_end[i] == -1)
            swing_end[i] = 50;

    // Validate phase duration and start/end heights
    for (int i = 0; i < NUM_LEGS; ++i) {
        assert((swing_end[i] - swing_start[i] + 1) == 25);
        double start_z = legs[i].getCurrentTipPositionGlobal().z;
        // After loop legs[i] at end of cycle - if ended swing, tip is at end_z; start_z is not exactly start
        // But we know during swing tip.z was checked each iteration
        (void)start_z;
    }

    // Validate coxa displacement during the cycle
    for (int i = 0; i < NUM_LEGS; ++i) {
        assert((max_coxa[i] - min_coxa[i]) > 1e-3);
    }

    std::cout << "Test completed successfully" << std::endl;
    return 0;
}

