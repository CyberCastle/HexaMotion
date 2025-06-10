#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include "model.h"

/**
 * @brief Basic admittance controller for body stabilization.
 */
class AdmittanceController {
  public:
    /**
     * @brief Construct a new controller.
     * @param model Reference to the robot model.
     * @param imu   IMU sensor interface pointer.
     * @param fsr   FSR sensor interface pointer.
     */
    AdmittanceController(RobotModel &model, IIMUInterface *imu, IFSRInterface *fsr);

    /**
     * @brief Compute orientation error with respect to a target pose.
     * @param target Desired roll, pitch and yaw in degrees.
     * @return Difference between target and current orientation.
     */
    Eigen::Vector3f orientationError(const Eigen::Vector3f &target);

    /**
     * @brief Maintain body orientation using a simple admittance filter.
     * @param target Desired orientation in degrees.
     * @param current Current orientation, updated in place.
     * @param dt      Time step in seconds.
     * @return True if the operation succeeded.
     */
    bool maintainOrientation(const Eigen::Vector3f &target, Eigen::Vector3f &current, float dt);

    /**
     * @brief Check static stability from FSR readings.
     * @param leg_pos  Current foot positions.
     * @param leg_states Leg state for each leg.
     * @return True if the robot is considered stable.
     */
    bool checkStability(const Point3D leg_pos[NUM_LEGS], const LegState leg_states[NUM_LEGS]);

  private:
    RobotModel &model;
    IIMUInterface *imu;
    IFSRInterface *fsr;
};

#endif // ADMITTANCE_CONTROLLER_H
