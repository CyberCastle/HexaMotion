#ifndef POSE_CONTROLLER_H
#define POSE_CONTROLLER_H

#include "HexaModel.h"

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
     * @brief Set the robot body pose using quaternion for orientation.
     * @param position Desired body position in world frame.
     * @param quaternion Desired body orientation as quaternion [w, x, y, z].
     * @param leg_positions Array of leg tip positions, updated in place.
     * @param joint_angles Array of joint angles, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPoseQuaternion(const Eigen::Vector3f &position, const Eigen::Vector4f &quaternion,
                               Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

    /**
     * @brief Interpolate between two poses using quaternions for smooth orientation.
     * @param start_pos Start position.
     * @param start_quat Start orientation as quaternion [w, x, y, z].
     * @param end_pos End position.
     * @param end_quat End orientation as quaternion [w, x, y, z].
     * @param t Interpolation parameter (0.0 to 1.0).
     * @param leg_positions Array of leg tip positions, updated in place.
     * @param joint_angles Array of joint angles, updated in place.
     * @return True if interpolated pose is achievable and applied.
     */
    bool interpolatePose(const Eigen::Vector3f &start_pos, const Eigen::Vector4f &start_quat,
                         const Eigen::Vector3f &end_pos, const Eigen::Vector4f &end_quat,
                         float t, Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

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

    /**
     * @brief Set body pose with smooth trajectory interpolation from current servo positions.
     * This is the default method that uses current servo positions as starting point,
     * equivalent to OpenSHC's smooth movement approach.
     * @param position Desired body position in world frame.
     * @param orientation Desired body orientation (roll, pitch, yaw in degrees).
     * @param leg_positions Array of leg tip positions, updated in place.
     * @param joint_angles Array of joint angles, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPoseSmooth(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation,
                           Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

    /**
     * @brief Set body pose with smooth trajectory interpolation using quaternions.
     * @param position Desired body position in world frame.
     * @param quaternion Desired body orientation as quaternion [w, x, y, z].
     * @param leg_positions Array of leg tip positions, updated in place.
     * @param joint_angles Array of joint angles, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPoseSmoothQuaternion(const Eigen::Vector3f &position, const Eigen::Vector4f &quaternion,
                                     Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

    /**
     * @brief Get current servo positions and calculate corresponding leg positions.
     * @param leg_positions Array to store current leg positions.
     * @param joint_angles Array to store current joint angles.
     * @return True if current positions were successfully retrieved.
     */
    bool getCurrentServoPositions(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

    /**
     * @brief Set body pose using original non-smooth method (for compatibility).
     * This method bypasses smooth trajectory interpolation.
     * @param position Desired body position in world frame.
     * @param orientation Desired body orientation (roll, pitch, yaw in degrees).
     * @param leg_positions Array of leg tip positions, updated in place.
     * @param joint_angles Array of joint angles, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPoseImmediate(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation,
                              Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

    /**
     * @brief Configure smooth trajectory behavior.
     * @param use_current_positions Enable using current servo positions as starting point.
     * @param interpolation_speed Speed of interpolation (0.01-1.0).
     * @param max_steps Maximum interpolation steps.
     */
    void configureSmoothTrajectory(bool use_current_positions, float interpolation_speed = 0.1f, uint8_t max_steps = 20);

    /**
     * @brief Check if a trajectory interpolation is currently in progress.
     * @return True if trajectory is being interpolated.
     */
    bool isTrajectoryInProgress() const { return trajectory_in_progress; }

    /**
     * @brief Reset/stop current trajectory interpolation.
     */
    void resetTrajectory() {
        trajectory_in_progress = false;
        trajectory_progress = 0.0f;
        trajectory_step_count = 0;
    }

  private:
    // Helper function to compute fixed poses using analytical planar geometry
    // Consolidates common pose calculation logic for standing, default, and crouch poses
    void computePose(float height, float radius, Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);

    RobotModel &model;
    IServoInterface *servos;

    // Smooth trajectory state variables
    bool trajectory_in_progress;
    Point3D trajectory_start_positions[NUM_LEGS];
    JointAngles trajectory_start_angles[NUM_LEGS];
    Point3D trajectory_target_positions[NUM_LEGS];
    JointAngles trajectory_target_angles[NUM_LEGS];
    float trajectory_progress;
    uint8_t trajectory_step_count;

    // Internal smooth trajectory methods
    bool initializeTrajectoryFromCurrent(const Eigen::Vector3f &target_position,
                                         const Eigen::Vector3f &target_orientation,
                                         Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);
    bool updateTrajectoryStep(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]);
    bool isTrajectoryComplete() const;

    // Quaternion utility functions
    Eigen::Vector4f quaternionSlerp(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2, float t);
};

#endif // POSE_CONTROLLER_H
