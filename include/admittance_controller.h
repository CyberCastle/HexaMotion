#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include "model.h"

class AdmittanceController {
  public:
    AdmittanceController(RobotModel &model, IIMUInterface *imu, IFSRInterface *fsr);

    Eigen::Vector3f orientationError(const Eigen::Vector3f &target);
    bool maintainOrientation(const Eigen::Vector3f &target, Eigen::Vector3f &current, float dt);
    bool checkStability(const Point3D leg_pos[NUM_LEGS], const LegState leg_states[NUM_LEGS]);

  private:
    RobotModel &model;
    IIMUInterface *imu;
    IFSRInterface *fsr;
};

#endif // ADMITTANCE_CONTROLLER_H
