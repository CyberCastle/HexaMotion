#include "../src/body_pose_config_factory.h"
#include "../src/locomotion_system.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>

/**
 * @brief Test: Validate non-blocking S-curve initial standing pose establishment.
 *
 * Generates random initial joint angles within limits to simulate arbitrary powered-on pose,
 * assigns them to legs, begins the S-curve transition, iteratively steps the locomotion system
 * (calling establishInitialStandingPose()/update()), and logs per-iteration joint angle, velocity
 * and acceleration samples until completion. Test passes when all legs match standing pose within
 * tolerance and controller reports inactive.
 */

static double randRange(double a, double b) {
    double r = (double)std::rand() / (double)RAND_MAX;
    return a + (b - a) * r;
}

int main() {
    std::srand((unsigned)std::time(nullptr));

    Parameters params = createDefaultParameters();
    // Use deterministic time step
    params.time_delta = 0.02; // 50 Hz
    // Segment masses from AGENTS.md (convert g -> kg)
    params.coxa_mass = 54.0 / 1000.0;   // 54 g
    params.femur_mass = 150.0 / 1000.0; // 150 g
    params.tibia_mass = 200.0 / 1000.0; // 200 g

    BodyPoseConfiguration pose_cfg = getDefaultBodyPoseConfig(params);

    // Create locomotion system with dummy interfaces
    DummyIMU imu; // from test_stubs.h (simple inert IMU)
    DummyFSR fsr; // simple FSR stub
    // Custom servo capturing acceleration
    class CaptureServo : public IServoInterface {
      public:
        struct BatchCmd {
            double angles_deg[NUM_LEGS][DOF_PER_LEG];
            double speeds[NUM_LEGS][DOF_PER_LEG];
            double accels[NUM_LEGS][DOF_PER_LEG];
            int iter;
        };
        std::vector<BatchCmd> batch_cmds;
        double last_angles[NUM_LEGS][DOF_PER_LEG]{}; // degrees
        bool initialize() override { return true; }
        bool hasBlockingStatusFlags(int, int) override { return false; }
        bool setJointAngleAndSpeed(int leg, int joint, double angle, double speed) override {
            // Fallback path (should be rare if batch supported). Accel unknown -> 0.
            last_angles[leg][joint] = angle;
            return true;
        }
        bool setJointAngleSpeedAccel(int leg, int joint, double angle, double speed, double accel) override {
            // Per-joint fallback capture (legacy). Update last angle only.
            (void)speed;
            (void)accel;
            last_angles[leg][joint] = angle;
            return true;
        }
        bool syncSetAllJointAnglesSpeedsAccels(const double a[NUM_LEGS][DOF_PER_LEG],
                                               const double s[NUM_LEGS][DOF_PER_LEG],
                                               const double ac[NUM_LEGS][DOF_PER_LEG]) override {
            BatchCmd bc{};
            bc.iter = current_iter_;
            for (int l = 0; l < NUM_LEGS; ++l) {
                for (int j = 0; j < DOF_PER_LEG; ++j) {
                    bc.angles_deg[l][j] = a[l][j];
                    bc.speeds[l][j] = s[l][j];
                    bc.accels[l][j] = ac[l][j];
                    last_angles[l][j] = a[l][j];
                }
            }
            batch_cmds.push_back(bc);
            return true;
        }
        double getJointAngle(int leg, int joint) override { return last_angles[leg][joint]; }
        bool isJointMoving(int, int) override { return true; }
        bool enableTorque(int, int, bool) override { return true; }
        void nextIter(int i) { current_iter_ = i; }

      private:
        int current_iter_ = 0;
    };

    CaptureServo servo;
    LocomotionSystem sys(params);
    bool ok = sys.initialize(&imu, &fsr, &servo, pose_cfg);
    if (!ok) {
        std::cerr << "Init failed" << std::endl;
        return 1;
    }

    // Randomize initial joint angles (radians) within limits and assign to legs directly
    JointAngles initial_angles[NUM_LEGS];
    for (int l = 0; l < NUM_LEGS; ++l) {
        initial_angles[l].coxa = randRange(params.coxa_angle_limits[0], params.coxa_angle_limits[1]) * DEGREES_TO_RADIANS_FACTOR;
        initial_angles[l].femur = randRange(params.femur_angle_limits[0], params.femur_angle_limits[1]) * DEGREES_TO_RADIANS_FACTOR;
        initial_angles[l].tibia = randRange(params.tibia_angle_limits[0], params.tibia_angle_limits[1]) * DEGREES_TO_RADIANS_FACTOR;
        sys.getLeg(l).setJointAngles(initial_angles[l]);
    }

    // Begin transition via locomotion API
    // Begin transition via locomotion API (first call may return false if it only initialized profiles without stepping yet)
    bool start_ret = sys.establishInitialStandingPose();
    if (!start_ret && !sys.isInitialStandingPoseActive()) {
        std::cerr << "Failed to start initial standing pose (controller inactive)" << std::endl;
        return 1;
    }

    // Standing pose target (radians)
    JointAngles target_rad[NUM_LEGS];
    for (int l = 0; l < NUM_LEGS; ++l) {
        auto sj = pose_cfg.standing_pose_joints[l];
        target_rad[l] = JointAngles(sj.coxa, sj.femur, sj.tibia);
    }

    const double angle_tol = 0.5 * DEGREES_TO_RADIANS_FACTOR; // 0.5° tolerance

    int iter = 0;
    const int MAX_ITERS = 500; // safety cap

    // Print test header
    std::cout << "======================================" << std::endl;
    std::cout << "S-Curve Initial Standing Pose Test" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Target pose tolerance: 0.5 degrees" << std::endl;
    std::cout << "Segment masses: coxa=" << params.coxa_mass * 1000 << "g, femur=" << params.femur_mass * 1000 << "g, tibia=" << params.tibia_mass * 1000 << "g" << std::endl;
    std::cout << "Torque balancing: " << (params.startup_norm.enable_torque_balanced ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << std::endl;

    // Display initial random pose
    std::cout << "INITIAL RANDOM JOINT POSITIONS:" << std::endl;
    std::cout << "Leg | Coxa   | Femur  | Tibia  " << std::endl;
    std::cout << "----|--------|--------|--------" << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    for (int l = 0; l < NUM_LEGS; ++l) {
        double coxa_deg = initial_angles[l].coxa * RADIANS_TO_DEGREES_FACTOR;
        double femur_deg = initial_angles[l].femur * RADIANS_TO_DEGREES_FACTOR;
        double tibia_deg = initial_angles[l].tibia * RADIANS_TO_DEGREES_FACTOR;
        std::cout << " " << l << "  |" << std::setw(7) << coxa_deg
                  << " |" << std::setw(7) << femur_deg << " |" << std::setw(7) << tibia_deg << std::endl;
    }
    std::cout << std::endl;
    bool reported_alignment = false;
    BodyPoseController *ctrl = sys.getBodyPoseController();

    std::cout << std::fixed << std::setprecision(2);
    while (sys.isInitialStandingPoseActive() && iter < MAX_ITERS) {
        servo.nextIter(iter);
        // Update triggers stepping (establishInitialStandingPose() not required each loop)
        sys.update();

        // Show progress every 25 iterations
        if (iter % 25 == 0) {
            std::cout << "Progress: Iteration " << iter;
            if (ctrl && ctrl->isInitialStandingPoseActive()) {
                double progress = ctrl->getInitialStandingPoseProgress() * 100.0;
                std::cout << " (S-curve " << std::setprecision(1) << progress << "% complete)";
                if (ctrl->isInitialStandingAlignmentPhase()) {
                    std::cout << " [ALIGN Phase]";
                } else {
                    std::cout << " [LIFT Phase]";
                }
            }
            std::cout << std::endl;
        }

        // Consolidated per-iteration joint angles (degrees) from internal leg state (radians -> deg)
        if (iter % 10 == 0 || iter < 5) { // Show first 5 and every 10th iteration
            std::cout << std::setprecision(1);
            std::cout << "Iter " << std::setw(3) << iter << " | ";
            for (int l = 0; l < NUM_LEGS; ++l) {
                JointAngles q = sys.getLeg(l).getJointAngles();
                double coxa_deg = q.coxa * RADIANS_TO_DEGREES_FACTOR;
                double femur_deg = q.femur * RADIANS_TO_DEGREES_FACTOR;
                double tibia_deg = q.tibia * RADIANS_TO_DEGREES_FACTOR;
                std::cout << "L" << l << "(" << std::setw(4) << coxa_deg << ","
                          << std::setw(4) << femur_deg << "," << std::setw(4) << tibia_deg << ") ";
            }
            std::cout << std::endl;
        }
        // Alignment detection using controller phase + joint angle comparison
        if (ctrl && ctrl->isInitialStandingPoseActive() && ctrl->isInitialStandingAlignmentPhase()) {
            bool aligned = true;
            for (int l = 0; l < NUM_LEGS; ++l) {
                StandingPoseJoints sj = ctrl->getStandingPoseJoints(l);
                double cur = sys.getLeg(l).getJointAngles().coxa;
                if (std::fabs(cur - sj.coxa) > 1.0 * DEGREES_TO_RADIANS_FACTOR) {
                    aligned = false;
                    break;
                }
            }
            if (!reported_alignment && aligned) {
                std::cout << ">>> ALIGNMENT COMPLETE at iteration " << iter << " <<<" << std::endl;
                reported_alignment = true;
            }
        } else if (reported_alignment && ctrl && ctrl->isInitialStandingPoseActive()) {
            // First iteration after leaving ALIGN phase
            std::cout << ">>> LIFT PHASE STARTED at iteration " << iter << " <<<" << std::endl;
            // Only once
            reported_alignment = false; // prevent re-use; or keep if need? we'll keep false to avoid duplicate
        }

        // Log servo commands for detailed analysis (only first few and last few iterations)
        if (iter < 3 || iter > MAX_ITERS - 5) {
            for (const auto &bc : servo.batch_cmds) {
                if (bc.iter == iter) {
                    std::cout << "  BatchCmd iter=" << bc.iter << " :";
                    // Show first leg summary
                    std::cout << " L0 (" << std::setprecision(2) << bc.angles_deg[0][0] << "° v=" << bc.speeds[0][0]
                              << " a=" << bc.accels[0][0] << ", " << bc.angles_deg[0][1] << "° v=" << bc.speeds[0][1]
                              << " a=" << bc.accels[0][1] << ", " << bc.angles_deg[0][2] << "° v=" << bc.speeds[0][2]
                              << " a=" << bc.accels[0][2] << ")" << std::endl;
                }
            }
        }
        ++iter;
    }

    if (iter >= MAX_ITERS) {
        std::cerr << "ERROR: Did not finish within iteration cap (" << MAX_ITERS << ")" << std::endl;
        return 1;
    }

    // Print final results summary
    std::cout << std::endl
              << "======================================" << std::endl;
    std::cout << "TRANSITION COMPLETED SUCCESSFULLY" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Total iterations: " << iter << std::endl;

    // Calculate and show servo command statistics
    double total_speed = 0.0, max_speed = 0.0, total_accel = 0.0, max_accel = 0.0;
    int cmd_count = 0;
    for (const auto &bc : servo.batch_cmds) {
        for (int l = 0; l < NUM_LEGS; ++l) {
            for (int j = 0; j < DOF_PER_LEG; ++j) {
                double sp = bc.speeds[l][j];
                double ac = bc.accels[l][j];
                total_speed += sp;
                total_accel += ac;
                if (sp > max_speed)
                    max_speed = sp;
                if (ac > max_accel)
                    max_accel = ac;
                cmd_count++;
            }
        }
    }

    if (cmd_count > 0) {
        std::cout << "Servo commands issued: " << cmd_count << std::endl;
        std::cout << "Average speed: " << std::setprecision(3) << (total_speed / cmd_count) << std::endl;
        std::cout << "Maximum speed: " << std::setprecision(3) << max_speed << std::endl;
        std::cout << "Average accel: " << std::setprecision(3) << (total_accel / cmd_count) << std::endl;
        std::cout << "Maximum accel: " << std::setprecision(3) << max_accel << std::endl;
    }

    std::cout << std::endl
              << "FINAL JOINT POSITIONS:" << std::endl;
    std::cout << "Leg | Coxa   | Femur  | Tibia  | Target Coxa | Target Femur | Target Tibia" << std::endl;
    std::cout << "----|--------|--------|--------|-------------|--------------|-------------" << std::endl;

    // Verify all joints near target
    bool all_ok = true;
    for (int l = 0; l < NUM_LEGS; ++l) {
        JointAngles q = sys.getLeg(l).getJointAngles();
        double coxa_deg = q.coxa * RADIANS_TO_DEGREES_FACTOR;
        double femur_deg = q.femur * RADIANS_TO_DEGREES_FACTOR;
        double tibia_deg = q.tibia * RADIANS_TO_DEGREES_FACTOR;
        double target_coxa_deg = target_rad[l].coxa * RADIANS_TO_DEGREES_FACTOR;
        double target_femur_deg = target_rad[l].femur * RADIANS_TO_DEGREES_FACTOR;
        double target_tibia_deg = target_rad[l].tibia * RADIANS_TO_DEGREES_FACTOR;

        std::cout << " " << l << "  |" << std::setw(7) << std::setprecision(1) << coxa_deg
                  << " |" << std::setw(7) << femur_deg << " |" << std::setw(7) << tibia_deg
                  << " |" << std::setw(12) << target_coxa_deg << " |" << std::setw(13) << target_femur_deg
                  << " |" << std::setw(12) << target_tibia_deg << std::endl;

        if (std::fabs(q.coxa - target_rad[l].coxa) > angle_tol ||
            std::fabs(q.femur - target_rad[l].femur) > angle_tol ||
            std::fabs(q.tibia - target_rad[l].tibia) > angle_tol) {
            std::cerr << "ERROR: Leg " << l << " not at target (tolerance exceeded)" << std::endl;
            all_ok = false;
        }
    }

    if (!all_ok) {
        std::cout << std::endl
                  << "TEST FAILED: Some legs did not reach target pose" << std::endl;
        return 1;
    }

    std::cout << std::endl
              << "✓ TEST PASSED: All legs reached target pose within tolerance" << std::endl;
    std::cout << "S-curve initial standing pose transition completed in " << iter << " iterations." << std::endl;
    return 0;
}
