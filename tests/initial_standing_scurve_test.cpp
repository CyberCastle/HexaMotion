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

    BodyPoseConfiguration pose_cfg = getDefaultBodyPoseConfig(params);

    // Create locomotion system with dummy interfaces
    DummyIMU imu; // from test_stubs.h (simple inert IMU)
    DummyFSR fsr; // simple FSR stub
    // Custom servo capturing acceleration
    class CaptureServo : public IServoInterface {
      public:
        struct Cmd {
            int leg;
            int joint;
            double angle;
            double speed;
            double accel;
            int iter;
        };
        std::vector<Cmd> cmds;
        double last_angles[NUM_LEGS][3]{};
        bool initialize() override { return true; }
        bool hasBlockingStatusFlags(int, int) override { return false; }
        bool setJointAngleAndSpeed(int leg, int joint, double angle, double speed) override {
            return setJointAngleSpeedAccel(leg, joint, angle, speed, 0.0);
        }
        bool setJointAngleSpeedAccel(int leg, int joint, double angle, double speed, double accel) override {
            Cmd c{leg, joint, angle, speed, accel, current_iter_};
            cmds.push_back(c);
            last_angles[leg][joint] = angle;
            return true;
        }
        double getJointAngle(int leg, int joint) override { return last_angles[leg][joint]; }
        bool isJointMoving(int, int) override { return true; }
        bool enableTorque(int, int, bool) override { return true; }
        bool syncSetAllJointAnglesAndSpeeds(const double a[NUM_LEGS][DOF_PER_LEG], const double s[NUM_LEGS][DOF_PER_LEG]) override {
            // emulate batch path (no accel)
            for (int l = 0; l < NUM_LEGS; ++l)
                for (int j = 0; j < DOF_PER_LEG; ++j)
                    setJointAngleSpeedAccel(l, j, a[l][j], s[l][j], 0.0);
            return true;
        }
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
    for (int l = 0; l < NUM_LEGS; ++l) {
        JointAngles qa;
        qa.coxa = randRange(params.coxa_angle_limits[0], params.coxa_angle_limits[1]) * DEGREES_TO_RADIANS_FACTOR;
        qa.femur = randRange(params.femur_angle_limits[0], params.femur_angle_limits[1]) * DEGREES_TO_RADIANS_FACTOR;
        qa.tibia = randRange(params.tibia_angle_limits[0], params.tibia_angle_limits[1]) * DEGREES_TO_RADIANS_FACTOR;
        sys.getLeg(l).setJointAngles(qa);
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
    std::cout << "Iter,Leg,Joint,PosDeg,Speed,Accel" << std::endl;

    bool reported_alignment = false;
    BodyPoseController *ctrl = sys.getBodyPoseController();

    std::cout << std::fixed << std::setprecision(4);
    while (sys.isInitialStandingPoseActive() && iter < MAX_ITERS) {
        servo.nextIter(iter);
        // Update triggers stepping (establishInitialStandingPose() not required each loop)
        sys.update();
        // Consolidated per-iteration joint angles (degrees) from internal leg state (radians -> deg)
        std::cout << "AnglesIter," << iter;
        for (int l = 0; l < NUM_LEGS; ++l) {
            JointAngles q = sys.getLeg(l).getJointAngles();
            double degs[3] = {q.coxa * RADIANS_TO_DEGREES_FACTOR,
                              q.femur * RADIANS_TO_DEGREES_FACTOR,
                              q.tibia * RADIANS_TO_DEGREES_FACTOR};
            for (int j = 0; j < DOF_PER_LEG; ++j) {
                std::cout << ",L" << l << "J" << j << "=" << degs[j];
            }
        }
        std::cout << std::endl;
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
                std::cout << "AlignmentCompleteAtIter," << iter << std::endl;
                reported_alignment = true;
            }
        } else if (reported_alignment && ctrl && ctrl->isInitialStandingPoseActive()) {
            // First iteration after leaving ALIGN phase
            std::cout << "LiftPhaseStartIter," << iter << std::endl;
            // Only once
            reported_alignment = false; // prevent re-use; or keep if need? we'll keep false to avoid duplicate
        }

        // Dump latest commands issued this iteration (those with Cmd.iter==iter)
        for (const auto &c : servo.cmds) {
            if (c.iter == iter) {
                std::cout << iter << "," << c.leg << "," << c.joint << "," << c.angle << "," << c.speed << "," << c.accel << std::endl;
            }
        }
        ++iter;
    }

    if (iter >= MAX_ITERS) {
        std::cerr << "Did not finish within iteration cap" << std::endl;
        return 1;
    }

    // Verify all joints near target
    bool all_ok = true;
    for (int l = 0; l < NUM_LEGS; ++l) {
        JointAngles q = sys.getLeg(l).getJointAngles();
        if (std::fabs(q.coxa - target_rad[l].coxa) > angle_tol ||
            std::fabs(q.femur - target_rad[l].femur) > angle_tol ||
            std::fabs(q.tibia - target_rad[l].tibia) > angle_tol) {
            std::cerr << "Leg " << l << " not at target." << std::endl;
            all_ok = false;
        }
    }

    if (!all_ok) {
        return 1;
    }

    std::cout << "S-curve initial standing pose transition completed in " << iter << " iterations." << std::endl;
    return 0;
}
