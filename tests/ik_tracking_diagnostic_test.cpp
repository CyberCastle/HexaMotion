/**
 * @file ik_tracking_diagnostic_test.cpp
 * @brief Diagnostic test for IK tracking accuracy across all legs.
 *
 * This test isolates the IK pipeline by running the full LocomotionSystem loop and
 * printing, for every leg at each step:
 *   - The IK delta (desired - current) fed to the Jacobian solver
 *   - The cumulative FK error: FK(angles) - desired_tip (open-loop drift)
 *   - The per-step linearization error: (FK_new - FK_old) - (desired_new - desired_old)
 *   - Joint angles (coxa in degrees)
 *
 * The cumulative metric shows open-loop drift inherent to the single-step Jacobian IK
 * (OpenSHC pattern: current = desired, never FK-corrected). The per-step metric isolates
 * the linearization quality of each individual Jacobian step.
 */
#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

static inline double toDeg(double radians) {
    return radians * 180.0 / M_PI;
}

static const char *LEG_NAMES[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"};

int main() {
    std::cout << "=== IK Tracking Diagnostic Test ===" << std::endl;

    // 1. Setup (same as coxa_tripod_symmetry_analytic_test)
    Parameters p = createDefaultParameters();
    p.max_velocity = 1000.0;

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: initialize failed" << std::endl;
        return 1;
    }

    if (!sys.setStandingPose()) {
        std::cerr << "ERROR: setStandingPose failed" << std::endl;
        return 1;
    }

    GaitConfiguration tripod_gait = createGaitConfig(TRIPOD_GAIT, p);
    double leg_reach = RobotModel::computeStandingHorizontalReach(p);
    tripod_gait.step_length = leg_reach * GAIT_TRIPOD_LENGTH_FACTOR;
    tripod_gait.time_to_max_stride = 0.75;

    if (!sys.setGaitConfiguration(tripod_gait)) {
        std::cerr << "ERROR: setGaitConfiguration failed" << std::endl;
        return 1;
    }

    // Disable auto pose
    auto *bpc = sys.getBodyPoseController();
    if (bpc)
        bpc->setAutoPoseEnabled(false);

    sys.walkForward(100.0);
    if (!sys.startWalking()) {
        std::cerr << "ERROR: startWalking failed" << std::endl;
        return 1;
    }

    // Run startup sequence (StateController handles internally via update())
    int startup_attempts = 0;
    while (sys.getSystemState() != SYSTEM_RUNNING && startup_attempts < 500) {
        sys.update();
        startup_attempts++;
    }
    if (sys.getSystemState() != SYSTEM_RUNNING) {
        std::cerr << "ERROR: startup failed" << std::endl;
        return 1;
    }
    std::cout << "Startup complete after " << startup_attempts << " attempts" << std::endl;

    // Print initial state
    std::cout << "\n=== INITIAL STATE (After Startup, Before Walking) ===" << std::endl;
    const RobotModel &model = sys.getRobotModel();
    for (int i = 0; i < NUM_LEGS; i++) {
        const Leg &leg = sys.getLeg(i);
        JointAngles ja = leg.getJointAngles();
        Point3D current_tip = leg.getCurrentTipPositionGlobal();
        Point3D fk_tip = model.forwardKinematicsGlobalCoordinates(i, ja);
        Point3D fk_err = fk_tip - current_tip;
        auto ls = sys.getWalkController()->getLegStepper(i);
        Point3D ls_default = ls ? ls->getDefaultTipPose() : Point3D(0, 0, 0);
        Point3D ls_current = ls ? ls->getCurrentTipPose() : Point3D(0, 0, 0);
        std::cout << LEG_NAMES[i]
                  << "  coxa=" << std::fixed << std::setprecision(2) << toDeg(ja.coxa)
                  << "°  fem=" << toDeg(ja.femur)
                  << "°  tib=" << toDeg(ja.tibia) << "°"
                  << "\n   Leg.currentTip=(" << current_tip.x << ", " << current_tip.y << ", " << current_tip.z << ")"
                  << "  FK=(" << fk_tip.x << ", " << fk_tip.y << ", " << fk_tip.z << ")"
                  << "  |FK_err|=" << fk_err.norm()
                  << "\n   LS.default=(" << ls_default.x << ", " << ls_default.y << ", " << ls_default.z << ")"
                  << "  LS.current=(" << ls_current.x << ", " << ls_current.y << ", " << ls_current.z << ")"
                  << "\n   diff_leg_vs_ls_default=(" << (current_tip.x - ls_default.x) << ", " << (current_tip.y - ls_default.y) << ", " << (current_tip.z - ls_default.z) << ")"
                  << " |diff|=" << (current_tip - ls_default).norm()
                  << std::endl;
    }

    // 2. Run walking loop - instrument all legs
    const int MAX_STEPS = 120; // ~2.4 full gait cycles

    std::cout << "\n=== PER-STEP IK DIAGNOSTIC (all legs) ===" << std::endl;
    std::cout << "step | leg | phase | state | coxa_deg |  delta  | |cumul| | |step_e| | FK_coxa" << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    // Per-leg cumulative FK error tracking (open-loop drift)
    double max_fk_err[NUM_LEGS] = {};
    double accum_fk_err_x[NUM_LEGS] = {};

    // Per-leg incremental (per-step) linearization error tracking
    double max_step_err[NUM_LEGS] = {};
    double sum_step_err[NUM_LEGS] = {};
    int step_err_count[NUM_LEGS] = {};

    // Store previous-step FK positions and joint angles for incremental comparison
    Point3D prev_fk[NUM_LEGS];
    Point3D prev_desired[NUM_LEGS];
    JointAngles prev_angles[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev_angles[i] = sys.getLeg(i).getJointAngles();
        prev_fk[i] = model.forwardKinematicsGlobalCoordinates(i, prev_angles[i]);
        prev_desired[i] = sys.getLeg(i).getDesiredTipPosition();
    }

    for (int step = 0; step < MAX_STEPS; step++) {
        // Capture pre-update state for all legs
        Point3D current_before[NUM_LEGS];
        Point3D desired_before[NUM_LEGS];
        Point3D ls_before[NUM_LEGS];
        for (int i = 0; i < NUM_LEGS; ++i) {
            current_before[i] = sys.getLeg(i).getCurrentTipPositionGlobal();
            desired_before[i] = sys.getLeg(i).getDesiredTipPosition();
            auto ls = sys.getWalkController()->getLegStepper(i);
            ls_before[i] = ls ? ls->getCurrentTipPose() : Point3D(0, 0, 0);
        }

        // Print body pose state before update for first few steps
        if (step <= 2 && bpc) {
            const Pose &bp = bpc->getCurrentBodyPose();
            auto aa = Eigen::AngleAxisd(bp.rotation);
            std::cout << "  [step " << step << " body_pose_BEFORE_update] pos=("
                      << bp.position.x << "," << bp.position.y << "," << bp.position.z
                      << ") rot_angle=" << (aa.angle() * 180.0 / M_PI) << "deg"
                      << " rot_axis=(" << aa.axis().x() << "," << aa.axis().y() << "," << aa.axis().z() << ")"
                      << std::endl;
        }

        // Run one full update cycle (LegStepper → body pose → IK → servos)
        sys.update();

        // Print body pose state after update for first few steps
        if (step <= 2 && bpc) {
            const Pose &bp = bpc->getCurrentBodyPose();
            auto aa = Eigen::AngleAxisd(bp.rotation);
            std::cout << "  [step " << step << " body_pose_AFTER_update] pos=("
                      << bp.position.x << "," << bp.position.y << "," << bp.position.z
                      << ") rot_angle=" << (aa.angle() * 180.0 / M_PI) << "deg"
                      << " rot_axis=(" << aa.axis().x() << "," << aa.axis().y() << "," << aa.axis().z() << ")"
                      << std::endl;
        }

        // Capture post-update state for all legs
        for (int leg_idx = 0; leg_idx < NUM_LEGS; ++leg_idx) {
            const Leg &leg = sys.getLeg(leg_idx);
            JointAngles ja = leg.getJointAngles();
            Point3D desired = leg.getDesiredTipPosition();
            Point3D pre_current = current_before[leg_idx];
            auto ls = sys.getWalkController()->getLegStepper(leg_idx);
            Point3D ls_tip_after = ls ? ls->getCurrentTipPose() : Point3D(0, 0, 0);

            // Delta that was fed to the IK (desired - authoritative current)
            Point3D delta = desired - pre_current;

            // Cumulative FK error: FK(new_angles) vs desired (open-loop drift)
            Point3D fk_result = model.forwardKinematicsGlobalCoordinates(leg_idx, ja);
            Point3D fk_err = fk_result - desired;

            // Per-step linearization error:
            //   actual_fk_delta = FK(new_angles) - FK(old_angles)
            //   desired_delta   = desired_new - desired_old
            //   step_error      = actual_fk_delta - desired_delta
            // This isolates single-step Jacobian quality from accumulated drift.
            Point3D fk_delta = fk_result - prev_fk[leg_idx];
            Point3D desired_delta = desired - prev_desired[leg_idx];
            Point3D step_error = fk_delta - desired_delta;
            double step_err_norm = step_error.norm();

            // Track cumulative error
            max_fk_err[leg_idx] = std::max(max_fk_err[leg_idx], fk_err.norm());
            accum_fk_err_x[leg_idx] += fk_err.x;

            // Track per-step error (skip step 0: startup transient from
            // standstill → walking pollutes the single-step metric)
            if (step > 0) {
                max_step_err[leg_idx] = std::max(max_step_err[leg_idx], step_err_norm);
                sum_step_err[leg_idx] += step_err_norm;
                step_err_count[leg_idx]++;
            }

            // Update previous-step state for next iteration
            prev_fk[leg_idx] = fk_result;
            prev_desired[leg_idx] = desired;
            prev_angles[leg_idx] = ja;

            int phase = ls ? ls->getPhase() : -1;
            const char *state_str = (leg.getStepPhase() == STANCE_PHASE) ? "ST" : "SW";

            // Print extra info for first few steps
            if (step <= 2) {
                std::cout << "  [" << LEG_NAMES[leg_idx] << " step " << step << "]"
                          << " LS_before=(" << std::fixed << std::setprecision(2)
                          << ls_before[leg_idx].x << "," << ls_before[leg_idx].y << "," << ls_before[leg_idx].z << ")"
                          << " LS_after=(" << ls_tip_after.x << "," << ls_tip_after.y << "," << ls_tip_after.z << ")"
                          << " desired_BEFORE=(" << desired_before[leg_idx].x << "," << desired_before[leg_idx].y << "," << desired_before[leg_idx].z << ")"
                          << " desired_AFTER=(" << desired.x << "," << desired.y << "," << desired.z << ")"
                          << " pre_current=(" << pre_current.x << "," << pre_current.y << "," << pre_current.z << ")"
                          << " IK_delta=(" << delta.x << "," << delta.y << "," << delta.z << ")"
                          << " step_err=(" << step_error.x << "," << step_error.y << "," << step_error.z << ")"
                          << std::endl;
            }

            std::cout << std::setw(4) << step << " | "
                      << std::setw(3) << LEG_NAMES[leg_idx] << " | "
                      << std::setw(5) << phase << " | "
                      << std::setw(5) << state_str << " | "
                      << std::fixed << std::setprecision(3)
                      << std::setw(9) << toDeg(ja.coxa) << " | "
                      << std::setw(7) << delta.norm() << " | "
                      << std::setw(7) << fk_err.norm() << " | "
                      << std::setw(7) << step_err_norm << " | "
                      << std::setw(7) << toDeg(ja.coxa)
                      << std::endl;
        }
    }

    // Summary: two metrics per leg
    //   Cumulative: FK(angles) - desired  (open-loop drift, grows because current = desired)
    //   Per-step:   (FK_new - FK_old) - (desired_new - desired_old) (single Jacobian step quality)
    std::cout << "\n=== SUMMARY ===" << std::endl;
    std::cout << std::setw(4) << "Leg"
              << std::setw(18) << "Cumul max (mm)"
              << std::setw(20) << "Cumul X acc (mm)"
              << std::setw(18) << "Step max (mm)"
              << std::setw(18) << "Step mean (mm)" << std::endl;
    std::cout << std::string(78, '-') << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        double mean_step = step_err_count[i] > 0 ? sum_step_err[i] / step_err_count[i] : 0.0;
        std::cout << std::setw(4) << LEG_NAMES[i]
                  << std::setw(18) << std::fixed << std::setprecision(3) << max_fk_err[i]
                  << std::setw(20) << accum_fk_err_x[i]
                  << std::setw(18) << max_step_err[i]
                  << std::setw(18) << mean_step << std::endl;
    }

    // Final joint angles
    std::cout << "\n=== FINAL JOINT ANGLES ===" << std::endl;
    for (int i = 0; i < NUM_LEGS; i++) {
        JointAngles ja = sys.getLeg(i).getJointAngles();
        std::cout << LEG_NAMES[i]
                  << "  coxa=" << std::fixed << std::setprecision(2) << toDeg(ja.coxa)
                  << "°  fem=" << toDeg(ja.femur)
                  << "°  tib=" << toDeg(ja.tibia) << "°"
                  << std::endl;
    }

    return 0;
}
