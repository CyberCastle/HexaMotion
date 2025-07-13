#ifndef BODY_POSE_CONTROLLER_H
#define BODY_POSE_CONTROLLER_H

#include "body_pose_config.h"
#include "imu_auto_pose.h"
#include "leg.h"
#include "leg_poser.h"
#include "robot_model.h"
#include <Eigen/Dense>
#include <memory>

// Forward declarations
class IServoInterface;

/**
 * @brief BodyPoseController class for HexaMotion
 *
 * This class handles body pose control and leg coordination for the hexapod robot.
 * It provides functionality for setting body poses, managing leg positions,
 * and coordinating stance transitions.
 *
 * Adapted from OpenSHC's PoseController but simplified for HexaMotion architecture.
 * No progress tracking - that's handled by LocomotionSystem.
 */
class BodyPoseController {
  public:
    /**
     * @brief Constructor
     * @param m Reference to the robot model
     * @param config Body pose configuration
     */
    BodyPoseController(RobotModel &m, const BodyPoseConfiguration &config);

    /**
     * @brief Destructor
     */
    ~BodyPoseController();

    /**
     * @brief Initialize leg posers for all legs
     * @param legs Array of Leg objects
     */
    void initializeLegPosers(Leg legs[NUM_LEGS]);

    /**
     * @brief Get leg poser for a specific leg
     * @param leg_index Index of the leg
     * @return Pointer to LegPoser, or nullptr if invalid
     */
    LegPoser *getLegPoser(int leg_index) const;

    /**
     * @brief Set body pose with position and orientation
     * @param position Body position in world coordinates
     * @param orientation Body orientation in euler angles (degrees)
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                     Leg legs[NUM_LEGS]);

    /**
     * @brief Set body pose with quaternion orientation
     * @param position Body position in world coordinates
     * @param quaternion Body orientation as quaternion
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool setBodyPoseQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                               Leg legs[NUM_LEGS]);

    /**
     * @brief Set individual leg position
     * @param leg_index Index of the leg to move
     * @param position Target position in world coordinates
     * @param legs Array of Leg objects
     * @return true if successful, false otherwise
     */
    bool setLegPosition(int leg_index, const Point3D &position, Leg legs[NUM_LEGS]);

    /**
     * @brief Calculate body pose from configuration
     * @param height_offset Height offset from default pose
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool calculateBodyPoseFromConfig(double height_offset, Leg legs[NUM_LEGS]);

    /**
     * @brief Initialize default pose for all legs
     * @param legs Array of Leg objects to initialize
     */
    void initializeDefaultPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Set standing pose for all legs
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool setStandingPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Interpolate between two poses
     * @param start_pos Starting position
     * @param start_quat Starting quaternion
     * @param end_pos Ending position
     * @param end_quat Ending quaternion
     * @param t Interpolation parameter (0.0 to 1.0)
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool interpolatePose(const Eigen::Vector3d &start_pos, const Eigen::Vector4d &start_quat,
                         const Eigen::Vector3d &end_pos, const Eigen::Vector4d &end_quat,
                         double t, Leg legs[NUM_LEGS]);

    /**
     * @brief Calculate body position based on current leg positions
     * @param legs Array of Leg objects
     * @return Current body position as Vector3d
     */
    Eigen::Vector3d calculateBodyPosition(Leg legs[NUM_LEGS]) const;

    /**
     * @brief Check if body pose is within configured limits
     * @param position Body position to check
     * @param orientation Body orientation to check
     * @return true if within limits, false otherwise
     */
    bool checkBodyPoseLimits(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    /**
     * @brief Execute startup sequence (READY -> RUNNING transition)
     * @param legs Array of Leg objects to update
     * @return true if sequence is complete, false if still in progress
     */
    bool executeStartupSequence(Leg legs[NUM_LEGS]);

    /**
     * @brief Execute direct startup sequence with simultaneous leg coordination (OpenSHC equivalent)
     * Uses time_to_start parameter directly as total sequence time
     * @param legs Array of Leg objects to update
     * @return true if sequence is complete, false if still in progress
     */
    bool executeDirectStartup(Leg legs[NUM_LEGS]);

    /**
     * @brief Execute shutdown sequence (RUNNING -> READY transition)
     * @param legs Array of Leg objects to update
     * @return true if sequence is complete, false if still in progress
     */
    bool executeShutdownSequence(Leg legs[NUM_LEGS]);

    /**
     * @brief Update auto-pose during gait execution (OpenSHC equivalent)
     * @param gait_phase Current gait phase (0.0 to 1.0)
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool updateAutoPose(double gait_phase, Leg legs[NUM_LEGS]);

    /**
     * @brief Tripod leg coordination for stance transition (tripod gait only)
     * @param legs Array of Leg objects to update
     * @param step_height Height for leg lifting during transition
     * @param step_time Time for the step transition
     * @return true if transition is complete, false if still in progress
     */
    bool stepToNewStance(Leg legs[NUM_LEGS], double step_height, double step_time);

    /**
     * @brief Set current gait type for startup sequence selection
     * @param gait_type The GaitType enum value
     */
    void setCurrentGaitType(GaitType gait_type) { current_gait_type_ = gait_type; }

    /**
     * @brief Get current gait type
     * @return Current gait type as GaitType enum
     */
    GaitType getCurrentGaitType() const { return current_gait_type_; }

    /**
     * @brief Reset all startup/shutdown sequence states
     */
    void resetSequenceStates() {
        step_to_new_stance_current_group = 0;
        step_to_new_stance_sequence_generated = false;
        direct_startup_sequence_initialized = false;
        shutdown_sequence_initialized = false;
    }

    // Compatibility methods for existing tests
    const BodyPoseConfiguration &getBodyPoseConfig() const { return body_pose_config; }
    void setBodyPoseConfig(const BodyPoseConfiguration &config) { body_pose_config = config; }
    void configureSmoothTrajectory(bool use_current_positions, double interpolation_speed = 0.1, uint8_t max_steps = 20);

    // Auto-pose configuration accessors
    const AutoPoseConfiguration &getAutoPoseConfig() const { return auto_pose_config; }
    void setAutoPoseConfig(const AutoPoseConfiguration &config) { auto_pose_config = config; }

    // Auto-pose state control
    bool isAutoPoseEnabled() const { return auto_pose_enabled; }
    void setAutoPoseEnabled(bool enabled) { auto_pose_enabled = enabled; }

    // Smooth trajectory methods
    bool setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                           Leg legs[NUM_LEGS], IServoInterface *servos = nullptr);
    bool setBodyPoseSmoothQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                     Leg legs[NUM_LEGS]);
    bool setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                              Leg legs[NUM_LEGS]);
    bool getCurrentServoPositions(IServoInterface *servos, Leg legs[NUM_LEGS]);
    bool initializeTrajectoryFromCurrent(const Eigen::Vector3d &target_position,
                                         const Eigen::Vector3d &target_orientation,
                                         Leg legs[NUM_LEGS], IServoInterface *servos);
    bool updateTrajectoryStep(Leg legs[NUM_LEGS]);
    void resetTrajectory();
    bool isTrajectoryInProgress() const { return trajectory_in_progress; }

  private:
    RobotModel &model;                      //< Reference to robot model
    BodyPoseConfiguration body_pose_config; //< Body pose configuration
    AutoPoseConfiguration auto_pose_config; //< Auto-pose configuration

    // Leg posers for each leg
    class LegPoserImpl;
    LegPoserImpl *leg_posers_[NUM_LEGS];

    // Auto-pose state
    bool auto_pose_enabled;

    // Current gait type for startup sequence selection (OpenSHC compatibility)
    GaitType current_gait_type_;

    // Smooth trajectory support
    bool trajectory_in_progress;
    double trajectory_progress;
    int trajectory_step_count;
    Point3D trajectory_start_positions[NUM_LEGS];
    JointAngles trajectory_start_angles[NUM_LEGS];
    Point3D trajectory_target_positions[NUM_LEGS];
    JointAngles trajectory_target_angles[NUM_LEGS];

    // stepToNewStance state variables (moved from static to class level)
    int step_to_new_stance_current_group;
    bool step_to_new_stance_sequence_generated;

    // executeDirectStartup state variables (moved from static to class level)
    bool direct_startup_sequence_initialized;

    // executeShutdownSequence state variables (moved from static to class level)
    bool shutdown_sequence_initialized;

    // Tripod gait leg groupings (OpenSHC compatible)
    // Group A: AR (0), CR (2), BL (4) - Anterior Right, Center Right, Back Left
    // Group B: BR (1), CL (3), AL (5) - Back Right, Center Left, Anterior Left
    static constexpr int tripod_leg_groups[2][3] = {{0, 2, 4}, {1, 3, 5}};

    // Constants
    static constexpr double ANGULAR_SCALING = 1.0;
};

#endif // BODY_POSE_CONTROLLER_H
