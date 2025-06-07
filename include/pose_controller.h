#ifndef POSE_CONTROLLER_H
#define POSE_CONTROLLER_H

#include "model.h"

class PoseController {
  public:
    PoseController(RobotModel &model, IServoInterface *servos);

    bool setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation,
                     Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);
    bool setLegPosition(int leg_index, const Point3D &position,
                        Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);
    void initializeDefaultPose(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS],
                               float hex_radius, float robot_height);
    bool setStandingPose(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS], float robot_height);
    bool setCrouchPose(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS], float robot_height);

  private:
    RobotModel &model;
    IServoInterface *servos;
};

#endif // POSE_CONTROLLER_H
