#include "../src/body_pose_config_factory.h"
#include "../src/locomotion_system.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

namespace {
class CapturingServo : public IServoInterface {
  public:
    CapturingServo() {
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
                last_angle_deg_[leg][joint] = 0.0;
                last_speed_[leg][joint] = 0.0;
            }
        }
    }

    bool initialize() override { return true; }

    bool hasBlockingStatusFlags(int, int) override {
        return false;
    }

    bool setJointAngleAndSpeed(int leg_index, int joint_index, double angle, double speed) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS || joint_index < 0 || joint_index >= DOF_PER_LEG) {
            return false;
        }
        last_angle_deg_[leg_index][joint_index] = angle;
        last_speed_[leg_index][joint_index] = speed;
        return true;
    }

    double getJointAngle(int, int) override {
        return 0.0;
    }

    bool isJointMoving(int, int) override { return false; }
    bool enableTorque(int, int, bool) override { return true; }

    double getLastAngleDeg(int leg_index, int joint_index) const {
        return last_angle_deg_[leg_index][joint_index];
    }

  private:
    double last_angle_deg_[NUM_LEGS][DOF_PER_LEG];
    double last_speed_[NUM_LEGS][DOF_PER_LEG];
};

bool near(double actual, double expected, double eps = 1e-6) {
    return std::abs(actual - expected) <= eps;
}
} // namespace

int main() {
    Parameters params = createDefaultParameters();

    // Per-joint calibration offsets (degrees) for leg 0.
    params.joint_angle_offset_deg[0][0] = 10.0;
    params.joint_angle_offset_deg[0][1] = -5.0;
    params.joint_angle_offset_deg[0][2] = 2.0;

    // Per-joint max angular speed (deg/s) for leg 0.
    // With dt = 0.02 s => max delta per update: coxa=1 deg, femur=4 deg, tibia=disabled.
    params.joint_max_angular_speed_deg_s[0][0] = 50.0;
    params.joint_max_angular_speed_deg_s[0][1] = 200.0;
    params.joint_max_angular_speed_deg_s[0][2] = 0.0;

    DummyIMU imu;
    DummyFSR fsr;
    CapturingServo servo;

    LocomotionSystem system(params);
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

    if (!system.initialize(&imu, &fsr, &servo, pose_config)) {
        std::cout << "FAIL: initialize()" << std::endl;
        return 1;
    }

    JointAngles cmd1[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        cmd1[i] = JointAngles(0.0, 0.0, 0.0);
    }

    if (!system.setRobotJointAngles(cmd1)) {
        std::cout << "FAIL: setRobotJointAngles(cmd1)" << std::endl;
        return 1;
    }

    bool ok = true;

    // First command should include offsets directly (no previous command for limiting).
    ok = ok && near(servo.getLastAngleDeg(0, 0), 10.0);
    ok = ok && near(servo.getLastAngleDeg(0, 1), -5.0);
    ok = ok && near(servo.getLastAngleDeg(0, 2), 2.0);

    JointAngles cmd2[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        cmd2[i] = JointAngles(0.0, 0.0, 0.0);
    }

    cmd2[0].coxa = math_utils::degreesToRadians(20.0);
    cmd2[0].femur = math_utils::degreesToRadians(20.0);
    cmd2[0].tibia = math_utils::degreesToRadians(-20.0);

    if (!system.setRobotJointAngles(cmd2)) {
        std::cout << "FAIL: setRobotJointAngles(cmd2)" << std::endl;
        return 1;
    }

    // Target after sign+offset would be coxa=30, femur=15, tibia=-18.
    // Speed limits from previous command:
    // coxa: 10 -> 11 (max +1 deg)
    // femur: -5 -> -1 (max +4 deg)
    // tibia: no limit => -18
    ok = ok && near(servo.getLastAngleDeg(0, 0), 11.0);
    ok = ok && near(servo.getLastAngleDeg(0, 1), -1.0);
    ok = ok && near(servo.getLastAngleDeg(0, 2), -18.0);

    // External desired/prev desired command API validation.
    system.beginDesiredJointCommandCycle();
    bool state_set_ok = system.setDesiredJointCommandState(0, 1,
                                                           math_utils::degreesToRadians(15.0),
                                                           math_utils::degreesToRadians(25.0),
                                                           1.25);
    state_set_ok = state_set_ok && system.setDesiredJointEffort(0, 1, 2.5);

    LocomotionSystem::DesiredJointCommandState state{};
    bool state_get_ok = system.getDesiredJointCommandState(0, 1, state);

    ok = ok && state_set_ok && state_get_ok;
    ok = ok && near(state.desired_position_rad, math_utils::degreesToRadians(15.0));
    ok = ok && near(state.desired_velocity_rad_s, math_utils::degreesToRadians(25.0));
    ok = ok && near(state.desired_effort, 2.5);

    system.beginDesiredJointCommandCycle();
    LocomotionSystem::DesiredJointCommandState state_after_cycle{};
    state_get_ok = system.getDesiredJointCommandState(0, 1, state_after_cycle);
    ok = ok && state_get_ok;
    ok = ok && near(state_after_cycle.prev_desired_position_rad, state.desired_position_rad);
    ok = ok && near(state_after_cycle.prev_desired_velocity_rad_s, state.desired_velocity_rad_s);
    ok = ok && near(state_after_cycle.prev_desired_effort, state.desired_effort);

    if (!ok) {
        std::cout << "FAIL: per-joint offset/max_angular_speed calibration mismatch" << std::endl;
        std::cout << "Observed leg0 commands: coxa=" << servo.getLastAngleDeg(0, 0)
                  << " femur=" << servo.getLastAngleDeg(0, 1)
                  << " tibia=" << servo.getLastAngleDeg(0, 2) << std::endl;
        return 1;
    }

    std::cout << "PASS: joint_output_calibration_test" << std::endl;
    return 0;
}