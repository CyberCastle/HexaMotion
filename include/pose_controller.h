#ifndef POSE_CONTROLLER_H
#define POSE_CONTROLLER_H

#include "model.h"

/**
 * @brief Kinematic pose controller for body and legs.
 */
class PoseController {
  public:
    /**
     * @brief Construct a pose controller.
     * @param model  Reference to the robot model.
     * @param servos Servo interface used to command joints.
     */
    PoseController(RobotModel &model, IServoInterface *servos);

    /**
     * @brief Set the robot body pose.
     * @param position Desired body position in world frame.
     * @param orientation Desired body orientation (roll, pitch, yaw in degrees).
     * @param leg_positions Array of leg tip positions, updated in place.
     * @param joint_angles Array of joint angles, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation,
                     Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

    /**
     * @brief Move a single leg to a target position.
     * @param leg_index Index of the leg to move (0-5).
     * @param position  Target tip position in world frame.
     * @param leg_positions Array of leg tip positions, updated in place.
     * @param joint_angles Array of joint angles, updated in place.
     * @return True if the command was successful.
     */
    bool setLegPosition(int leg_index, const Point3D &position,
                        Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

    /**
     * @brief Initialize default leg pose around the body.
     * @param leg_positions Array of leg tip positions to fill.
     * @param joint_angles Array of joint angles to fill.
     * @param hex_radius  Body hexagon radius in millimeters.
     * @param robot_height Nominal body height.
     */
    void initializeDefaultPose(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS],
                               float hex_radius, float robot_height);

    /**
     * @brief Set the robot to a standing pose.
     * @param leg_positions Array of leg positions to update.
     * @param joint_angles Array of joint angles to update.
     * @param robot_height Target body height.
     * @return True on success.
     */
    bool setStandingPose(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS], float robot_height);

    /**
     * @brief Set the robot to a crouch pose.
     * @param leg_positions Array of leg positions to update.
     * @param joint_angles Array of joint angles to update.
     * @param robot_height Target body height for crouching.
     * @return True on success.
     */
    bool setCrouchPose(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS], float robot_height);

  private:
    RobotModel &model;
    IServoInterface *servos;
};

#endif // POSE_CONTROLLER_H
