#ifndef TRIPOD_GAIT_CONTROLLER_H
#define TRIPOD_GAIT_CONTROLLER_H

#include "analytic_robot_model.h"
#include "math_utils.h"
#include "robot_model.h"
#include <array>

/**
 * @class TripodGaitController
 * @brief Simple analytic tripod gait implementation.
 *
 * This controller generates tripod gait trajectories for the hexapod
 * robot described in AGENTS.md. It keeps the body at a fixed height of
 * 150 mm and provides basic commands for directional movement and
 * rotation.
 */
class TripodGaitController {
  public:
    /** Construct controller with robot parameters. */
    explicit TripodGaitController(const Parameters &params);

    /** Begin gait motion. */
    void start();
    /** Stop gait motion. */
    void stop();

    /** Command forward walking with given step length. */
    void moveForward(double step);
    /** Command backward walking with given step length. */
    void moveBackward(double step);
    /** Rotate left by setting angular step. */
    void rotateLeft(double angle_step);
    /** Rotate right by setting angular step. */
    void rotateRight(double angle_step);

    /**
     * @brief Update joint targets for the current time step.
     * @param dt Time delta in seconds.
     * @param angles Output array with joint angles for each leg.
     */
    void update(double dt, std::array<JointAngles, NUM_LEGS> &angles);

  private:
    Point3D computeFootPosition(int leg, double x_off, double z_off) const;
    JointAngles solveIK(int leg, const Point3D &target) const;

    Parameters params_;
    AnalyticRobotModel model_;

    double phase_;
    double frequency_;
    double step_length_;
    double step_height_;
    double body_yaw_;
    double linear_sign_;
    double angular_step_;
    bool walking_;
};

#endif // TRIPOD_GAIT_CONTROLLER_H
