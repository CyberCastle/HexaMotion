#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
#include "../src/gait_config.h"
#include "../src/gait_config_factory.h"
#include "../src/leg_stepper.h"
#include "../src/walkspace_analyzer.h"
#include "../src/workspace_validator.h"
#include "test_stubs.h"
#include <iomanip>
#include <iostream>

static double toDegrees(double rad) { return rad * 180.0 / M_PI; }

int main() {
    Parameters p = createDefaultParameters();
    GaitConfiguration gait = createTripodGaitConfig(p);
    StepCycle step_cycle = gait.generateStepCycle();

    RobotModel model(p);
    Leg legs[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                          Leg(3, model), Leg(4, model), Leg(5, model)};

    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].initialize(Pose::Identity());
        legs[i].updateTipPosition();
    }

    BodyPoseConfiguration pose_cfg = getDefaultBodyPoseConfig(p);
    BodyPoseController pose_controller(model, pose_cfg);
    pose_controller.setWalkPlanePoseEnabled(true);
    pose_controller.initializeLegPosers(legs);
    if (!pose_controller.setStandingPose(legs)) {
        std::cerr << "Error al establecer la posición inicial" << std::endl;
        return 1;
    }

    WalkspaceAnalyzer analyzer(model);
    WorkspaceValidator validator(model);
    LegStepper steppers[NUM_LEGS] = {
        LegStepper(0, legs[0].getCurrentTipPositionGlobal(), legs[0], model, &analyzer, &validator),
        LegStepper(1, legs[1].getCurrentTipPositionGlobal(), legs[1], model, &analyzer, &validator),
        LegStepper(2, legs[2].getCurrentTipPositionGlobal(), legs[2], model, &analyzer, &validator),
        LegStepper(3, legs[3].getCurrentTipPositionGlobal(), legs[3], model, &analyzer, &validator),
        LegStepper(4, legs[4].getCurrentTipPositionGlobal(), legs[4], model, &analyzer, &validator),
        LegStepper(5, legs[5].getCurrentTipPositionGlobal(), legs[5], model, &analyzer, &validator)};

    for (int i = 0; i < NUM_LEGS; ++i) {
        steppers[i].setStepCycle(step_cycle);
        steppers[i].setControlFrequency(gait.control_frequency);
        steppers[i].setStepClearanceHeight(gait.swing_height);
        steppers[i].setDefaultTipPose(legs[i].getCurrentTipPositionGlobal());
        steppers[i].setCurrentTipPose(legs[i].getCurrentTipPositionGlobal());
        double offset = static_cast<double>(gait.offsets.getForLegIndex(i) * gait.phase_config.phase_offset) /
                        step_cycle.period_;
        steppers[i].setPhaseOffset(offset);
        steppers[i].setDesiredVelocity(Point3D(20.0, 0.0, 0.0), 0.0);
        steppers[i].updateStride();
        steppers[i].setStepState(STEP_STANCE);
    }

    std::cout << "Iter\tPata\tFase\tPosición (x,y,z)\t\tÁngulos (c,f,t)" << std::endl;

    int total_steps = 20;
    double time_delta = 1.0 / gait.control_frequency;
    for (int step = 0; step < total_steps; ++step) {
        int global_phase = step % step_cycle.period_;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            int offset = gait.offsets.getForLegIndex(leg) * gait.phase_config.phase_offset;
            int leg_phase = (global_phase + offset) % step_cycle.period_;
            bool in_swing = leg_phase >= step_cycle.stance_period_;
            StepState desired_state = in_swing ? STEP_SWING : STEP_STANCE;
            if (steppers[leg].getStepState() != desired_state) {
                steppers[leg].setStepState(desired_state);
                if (desired_state == STEP_SWING)
                    steppers[leg].initializeSwingPeriod(1);
            }
            steppers[leg].updateTipPositionIterative(leg_phase, time_delta, false, false);

            Point3D pos = steppers[leg].getCurrentTipPose();
            JointAngles prev_angles = legs[leg].getJointAngles();
            Point3D prev_pos = legs[leg].getCurrentTipPositionGlobal();
            JointAngles new_angles = model.applyAdvancedIK(leg, prev_pos, pos, prev_angles, time_delta);
            legs[leg].setJointAngles(new_angles);
            legs[leg].setStepPhase(in_swing ? SWING_PHASE : STANCE_PHASE);
        }

        if (step % 5 == 0) {
            for (int leg = 0; leg < NUM_LEGS; ++leg) {
                Point3D pos = legs[leg].getCurrentTipPositionGlobal();
                JointAngles ang = legs[leg].getJointAngles();
                StepPhase ph = legs[leg].getStepPhase();
                std::cout << step << "\t" << leg << "\t" << (ph == SWING_PHASE ? "SW" : "ST")
                          << "\t[" << std::fixed << std::setprecision(1) << pos.x << ", " << pos.y << ", " << pos.z << "]"
                          << "\t[" << std::setprecision(2) << toDegrees(ang.coxa) << ", "
                          << toDegrees(ang.femur) << ", " << toDegrees(ang.tibia) << "]" << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }
    }
    return 0;
}

