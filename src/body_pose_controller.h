#ifndef BODY_POSE_CONTROLLER_H
#define BODY_POSE_CONTROLLER_H

#include "HexaModel.h"
#include "body_pose_config.h"
#include "leg.h"

// Forward declarations
class IServoInterface;
class LegPoser;

/**
 * @brief Kinematic body pose controller for robot body pose management.
 */
class BodyPoseController {
  public:
    /**
     * @brief Construct a body pose controller.
     * @param model  Reference to the robot model.
     * @param config Initial body pose configuration (use body_pose_config_factory.h to create)
     */
    BodyPoseController(RobotModel &model, const BodyPoseConfiguration &config);
    ~BodyPoseController();

    /**
     * @brief Set the robot body pose.
     * @param position Desired body position in world frame.
     * @param orientation Desired body orientation (roll, pitch, yaw in degrees).
     * @param legs Array of Leg objects, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                     Leg legs[NUM_LEGS]);

    /**
     * @brief Set the robot body pose using quaternion for orientation.
     * @param position Desired body position in world frame.
     * @param quaternion Desired body orientation as quaternion [w, x, y, z].
     * @param legs Array of Leg objects, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPoseQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                               Leg legs[NUM_LEGS]);

    /**
     * @brief Interpolate between two poses using quaternions for smooth orientation.
     * @param start_pos Start position.
     * @param start_quat Start orientation as quaternion [w, x, y, z].
     * @param end_pos End position.
     * @param end_quat End orientation as quaternion [w, x, y, z].
     * @param t Interpolation parameter (0.0 to 1.0).
     * @param legs Array of Leg objects, updated in place.
     * @return True if interpolated pose is achievable and applied.
     */
    bool interpolatePose(const Eigen::Vector3d &start_pos, const Eigen::Vector4d &start_quat,
                         const Eigen::Vector3d &end_pos, const Eigen::Vector4d &end_quat,
                         double t, Leg legs[NUM_LEGS]);

    /**
     * @brief Move a single leg to a target position using LegPoser.
     * @param leg_index Index of the leg to move (0-5).
     * @param position  Target tip position in world frame.
     * @param legs Array of Leg objects, updated in place.
     * @return True if the command was successful.
     */
    bool setLegPosition(int leg_index, const Point3D &position, Leg legs[NUM_LEGS]);

    /**
     * @brief Initialize default leg pose around the body using LegPoser.
     * @param legs Array of Leg objects to initialize.
     */
    void initializeDefaultPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Set the robot to a standing pose using LegPoser.
     * @param legs Array of Leg objects to update.
     * @return True on success.
     */
    bool setStandingPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Set body pose with smooth trajectory interpolation from current servo positions.
     * This is the default method that uses current servo positions as starting point,
     * equivalent to OpenSHC's smooth movement approach.
     * @param position Desired body position in world frame.
     * @param orientation Desired body orientation (roll, pitch, yaw in degrees).
     * @param legs Array of Leg objects, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                           Leg legs[NUM_LEGS], IServoInterface *servos = nullptr);

    /**
     * @brief Set body pose with smooth trajectory interpolation using quaternions.
     * @param position Desired body position in world frame.
     * @param quaternion Desired body orientation as quaternion [w, x, y, z].
     * @param legs Array of Leg objects, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPoseSmoothQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                     Leg legs[NUM_LEGS]);

    /**
     * @brief Get current servo positions and calculate corresponding leg positions.
     * Note: This method now requires the servo interface to be passed as a parameter
     * since the PoseController no longer stores a reference to it.
     * @param servos Servo interface to read current positions from.
     * @param legs Array of Leg objects to update with current positions.
     * @return True if current positions were successfully retrieved.
     */
    bool getCurrentServoPositions(IServoInterface *servos, Leg legs[NUM_LEGS]);

    /**
     * @brief Set body pose using original non-smooth method (for compatibility).
     * This method bypasses smooth trajectory interpolation.
     * @param position Desired body position in world frame.
     * @param orientation Desired body orientation (roll, pitch, yaw in degrees).
     * @param legs Array of Leg objects, updated in place.
     * @return True if pose is achievable and applied.
     */
    bool setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                              Leg legs[NUM_LEGS]);

    /**
     * @brief Configure smooth trajectory behavior.
     * @param use_current_positions Enable using current servo positions as starting point.
     * @param interpolation_speed Speed of interpolation (0.01-1.0).
     * @param max_steps Maximum interpolation steps.
     */
    void configureSmoothTrajectory(bool use_current_positions, double interpolation_speed = 0.1f, uint8_t max_steps = 20);

    /**
     * @brief Get current body pose configuration
     * @return Reference to current body pose configuration
     */
    const BodyPoseConfiguration &getBodyPoseConfig() const { return body_pose_config; }

    /**
     * @brief Set body pose configuration directly (decoupled approach)
     * @param config Pre-configured body pose configuration structure
     */
    void setBodyPoseConfig(const BodyPoseConfiguration &config) {
        body_pose_config = config;
    }

    /**
     * @brief Get configured standing pose joint angles (OpenSHC equivalent)
     * @return Array of standing pose joint configurations
     */
    const std::array<StandingPoseJoints, NUM_LEGS> &getStandingPoseJoints() const {
        return body_pose_config.standing_pose_joints;
    }

    /**
     * @brief Set custom standing pose joint angles (OpenSHC equivalent)
     * @param joints Array of standing pose joint configurations
     */
    void setStandingPoseJoints(const std::array<StandingPoseJoints, NUM_LEGS> &joints) {
        body_pose_config.standing_pose_joints = joints;
    }

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

    /**
     * @brief Get LegPoser for a specific leg
     * @param leg_index Index of the leg (0-5)
     * @return Pointer to LegPoser, or nullptr if not available
     */
    LegPoser* getLegPoser(int leg_index) const;

    /**
     * @brief Initialize LegPoser instances for all legs
     * @param legs Array of Leg objects to associate with LegPosers
     */
    void initializeLegPosers(Leg legs[NUM_LEGS]);

  private:
    // OpenSHC-style body pose calculation using dynamic configuration
    bool calculateBodyPoseFromConfig(double height_offset, Leg legs[NUM_LEGS]);

    // OpenSHC-style body pose limit validation
    bool checkBodyPoseLimits(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    // Quaternion spherical linear interpolation (SLERP)
    Eigen::Vector4d quaternionSlerp(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, double t);

    RobotModel &model;

    // OpenSHC-style body pose configuration
    BodyPoseConfiguration body_pose_config;

    // LegPoser instances for each leg (forward declaration)
    class LegPoserImpl;
    std::array<LegPoserImpl*, NUM_LEGS> leg_posers_;

    // Smooth trajectory state variables
    bool trajectory_in_progress;
    Point3D trajectory_start_positions[NUM_LEGS];
    JointAngles trajectory_start_angles[NUM_LEGS];
    Point3D trajectory_target_positions[NUM_LEGS];
    JointAngles trajectory_target_angles[NUM_LEGS];
    double trajectory_progress;
    uint8_t trajectory_step_count;

    // Internal smooth trajectory methods
    bool initializeTrajectoryFromCurrent(const Eigen::Vector3d &target_position,
                                         const Eigen::Vector3d &target_orientation,
                                         Leg legs[NUM_LEGS], IServoInterface *servos = nullptr);
    bool updateTrajectoryStep(Leg legs[NUM_LEGS]);
};

#endif // BODY_POSE_CONTROLLER_H
