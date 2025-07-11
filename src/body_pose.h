#ifndef BODY_POSE_H
#define BODY_POSE_H

#include "robot_model.h"
#include <Eigen/Dense>

/**
 * @brief Body pose structure containing only robot body pose information
 *
 * This class encapsulates all information related to the robot body pose,
 * separating body pose logic from leg pose logic. It includes position,
 * orientation, and various body-specific pose parameters.
 */
class BodyPose {
public:
    /**
     * @brief Body pose state containing position and orientation
     */
    struct BodyPoseState {
        Point3D position;                    //< Body position in world frame (meters)
        Eigen::Vector3d euler_angles;        //< Body orientation as Euler angles (roll, pitch, yaw in radians)
        Eigen::Vector4d quaternion;          //< Body orientation as quaternion [w, x, y, z]
        double height;                       //< Body height above ground (meters)
        bool use_quaternion;                 //< Whether to use quaternion or Euler angles
        double blend_factor;                 //< Blending factor for pose transitions
        bool pose_active;                    //< Whether pose is actively applied

        BodyPoseState()
            : position(0, 0, 0), euler_angles(0, 0, 0), quaternion(1, 0, 0, 0),
              height(0.0), use_quaternion(false), blend_factor(1.0), pose_active(false) {}
    };

    /**
     * @brief Body pose limits for safety constraints
     */
    struct BodyPoseLimits {
        Point3D translation_limits;          //< Maximum translation limits (±X, ±Y, ±Z in meters)
        Eigen::Vector3d rotation_limits;     //< Maximum rotation limits (±Roll, ±Pitch, ±Yaw in radians)
        double height_min, height_max;       //< Height limits (meters)
        double max_translation_velocity;     //< Maximum translation velocity (m/s)
        double max_rotation_velocity;        //< Maximum rotation velocity (rad/s)

        BodyPoseLimits()
            : translation_limits(0.1, 0.1, 0.05), rotation_limits(0.3, 0.3, 0.5),
              height_min(80.0), height_max(150.0), max_translation_velocity(0.1), max_rotation_velocity(0.5) {}
    };

    /**
     * @brief Body pose configuration parameters
     */
    struct BodyPoseConfig {
        double body_clearance;               //< Requested height of robot body above ground (m)
        double swing_height;                 //< Vertical displacement of swing trajectory above default (m)
        bool gravity_aligned_tips;           //< Flag denoting if tip should align with gravity direction
        bool force_symmetric_pose;           //< Force hexagonal symmetry if true
        std::string auto_pose_type;          //< String denoting the default auto posing cycle type
        bool start_up_sequence;              //< Flag allowing execution of start up and shutdown sequences
        double time_to_start;                //< The time to complete a direct start up

        BodyPoseConfig()
            : body_clearance(208.0), swing_height(0.02), gravity_aligned_tips(false),
              force_symmetric_pose(false), auto_pose_type("default"), start_up_sequence(false), time_to_start(2.0) {}
    };

    /**
     * @brief Available body pose control modes
     */
    enum BodyPoseMode {
        BODY_POSE_TRANSLATION,    //< Move body in X,Y,Z
        BODY_POSE_ROTATION,       //< Rotate body around X,Y,Z axes
        BODY_POSE_HEIGHT,         //< Adjust overall height
        BODY_POSE_COMBINED,       //< Combined translation and rotation
        BODY_POSE_MANUAL,         //< Manual body pose mode
        BODY_POSE_AUTO,           //< Automatic body pose mode
        BODY_POSE_CUSTOM          //< Custom pose sequences
    };

    /**
     * @brief Body pose reset modes
     */
    enum BodyPoseResetMode {
        BODY_POSE_RESET_NONE,     //< Don't reset any pose components
        BODY_POSE_RESET_TRANSLATION, //< Reset only translation components
        BODY_POSE_RESET_ROTATION,    //< Reset only rotation components
        BODY_POSE_RESET_ALL       //< Reset all pose components
    };

    /**
     * @brief Body pose state for auto posing
     */
    enum BodyPosingState {
        BODY_POSING_COMPLETE,     //< Auto posing is complete
        BODY_POSING_IN_PROGRESS,  //< Auto posing is in progress
        BODY_POSING_PAUSED        //< Auto posing is paused
    };

private:
    BodyPoseState current_pose_;
    BodyPoseState target_pose_;
    BodyPoseLimits pose_limits_;
    BodyPoseConfig pose_config_;
    BodyPoseMode current_mode_;
    BodyPoseResetMode reset_mode_;
    BodyPosingState auto_posing_state_;

    // Pose interpolation
    double interpolation_speed_;
    bool smooth_transitions_;
    bool trajectory_in_progress_;
    double trajectory_progress_;
    uint8_t trajectory_step_count_;

    // Auto posing parameters
    int pose_phase_;
    double pose_frequency_;
    int pose_phase_length_;
    int normaliser_;

    // IMU-based pose correction
    Eigen::Vector3d rotation_absement_error_;
    Eigen::Vector3d rotation_position_error_;
    Eigen::Vector3d rotation_velocity_error_;

    // Velocity inputs for manual control
    Eigen::Vector3d translation_velocity_input_;
    Eigen::Vector3d rotation_velocity_input_;

public:
    /**
     * @brief Default constructor
     */
    BodyPose();

    /**
     * @brief Constructor with configuration
     * @param config Body pose configuration
     */
    explicit BodyPose(const BodyPoseConfig& config);

    /**
     * @brief Initialize body pose with default settings
     */
    void initialize();

    /**
     * @brief Set current body pose state
     * @param pose New body pose state
     */
    void setCurrentPose(const BodyPoseState& pose);

    /**
     * @brief Get current body pose state
     * @return Current body pose state
     */
    const BodyPoseState& getCurrentPose() const { return current_pose_; }

    /**
     * @brief Set target body pose state
     * @param pose Target body pose state
     */
    void setTargetPose(const BodyPoseState& pose);

    /**
     * @brief Get target body pose state
     * @return Target body pose state
     */
    const BodyPoseState& getTargetPose() const { return target_pose_; }

    /**
     * @brief Set body pose using position and Euler angles
     * @param position Body position
     * @param euler_angles Body orientation as Euler angles (roll, pitch, yaw in radians)
     */
    void setPose(const Point3D& position, const Eigen::Vector3d& euler_angles);

    /**
     * @brief Set body pose using position and quaternion
     * @param position Body position
     * @param quaternion Body orientation as quaternion [w, x, y, z]
     */
    void setPoseQuaternion(const Point3D& position, const Eigen::Vector4d& quaternion);

    /**
     * @brief Set body pose mode
     * @param mode New body pose mode
     */
    void setPoseMode(BodyPoseMode mode);

    /**
     * @brief Get current body pose mode
     * @return Current body pose mode
     */
    BodyPoseMode getPoseMode() const { return current_mode_; }

    /**
     * @brief Set body pose reset mode
     * @param mode New reset mode
     */
    void setResetMode(BodyPoseResetMode mode);

    /**
     * @brief Get current body pose reset mode
     * @return Current reset mode
     */
    BodyPoseResetMode getResetMode() const { return reset_mode_; }

    /**
     * @brief Set manual pose velocity input
     * @param translation Translation velocity input
     * @param rotation Rotation velocity input
     */
    void setManualPoseInput(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation);

    /**
     * @brief Get manual pose velocity input
     * @param translation Output translation velocity input
     * @param rotation Output rotation velocity input
     */
    void getManualPoseInput(Eigen::Vector3d& translation, Eigen::Vector3d& rotation) const;

    /**
     * @brief Reset all pose components to identity
     */
    void resetAllPosing();

    /**
     * @brief Update pose interpolation (call at regular intervals)
     * @param dt Delta time in seconds
     */
    void updatePoseInterpolation(double dt);

    /**
     * @brief Check if trajectory interpolation is in progress
     * @return True if trajectory is being interpolated
     */
    bool isTrajectoryInProgress() const { return trajectory_in_progress_; }

    /**
     * @brief Reset/stop current trajectory interpolation
     */
    void resetTrajectory();

    /**
     * @brief Enable/disable smooth pose transitions
     * @param enable Whether to enable smooth transitions
     * @param speed Interpolation speed (0-1)
     */
    void setSmoothTransitions(bool enable, double speed = 0.1);

    /**
     * @brief Get body pose configuration
     * @return Body pose configuration
     */
    const BodyPoseConfig& getPoseConfig() const { return pose_config_; }

    /**
     * @brief Set body pose configuration
     * @param config New body pose configuration
     */
    void setPoseConfig(const BodyPoseConfig& config);

    /**
     * @brief Get body pose limits
     * @return Body pose limits
     */
    const BodyPoseLimits& getPoseLimits() const { return pose_limits_; }

    /**
     * @brief Set body pose limits
     * @param limits New body pose limits
     */
    void setPoseLimits(const BodyPoseLimits& limits);

    /**
     * @brief Convert current pose to OpenSHC-style Pose structure
     * @return Pose structure equivalent to OpenSHC's Pose
     */
    Pose toPose() const;

    /**
     * @brief Create BodyPose from OpenSHC-style Pose structure
     * @param pose OpenSHC-style Pose structure
     * @return BodyPose object
     */
    static BodyPose fromPose(const Pose& pose);

    /**
     * @brief Check if pose is within configured limits
     * @param pose Pose to check
     * @return True if pose is within limits
     */
    bool checkPoseLimits(const BodyPoseState& pose) const;

    /**
     * @brief Constrain pose to limits
     * @param pose Pose to constrain (modified in place)
     */
    void constrainPose(BodyPoseState& pose) const;

    /**
     * @brief Get auto posing state
     * @return Current auto posing state
     */
    BodyPosingState getAutoPosingState() const { return auto_posing_state_; }

    /**
     * @brief Set auto posing state
     * @param state New auto posing state
     */
    void setAutoPosingState(BodyPosingState state);

    /**
     * @brief Get pose phase for auto posing
     * @return Current pose phase
     */
    int getPosePhase() const { return pose_phase_; }

    /**
     * @brief Set pose phase for auto posing
     * @param phase New pose phase
     */
    void setPosePhase(int phase);

    /**
     * @brief Get pose frequency
     * @return Current pose frequency
     */
    double getPoseFrequency() const { return pose_frequency_; }

    /**
     * @brief Set pose frequency
     * @param frequency New pose frequency
     */
    void setPoseFrequency(double frequency);

    /**
     * @brief Get pose phase length
     * @return Current pose phase length
     */
    int getPosePhaseLength() const { return pose_phase_length_; }

    /**
     * @brief Set pose phase length
     * @param length New pose phase length
     */
    void setPosePhaseLength(int length);

    /**
     * @brief Get normaliser value
     * @return Current normaliser value
     */
    int getNormaliser() const { return normaliser_; }

    /**
     * @brief Set normaliser value
     * @param normaliser New normaliser value
     */
    void setNormaliser(int normaliser);

    /**
     * @brief Get rotation absement error (for IMU posing PID)
     * @return Rotation absement error
     */
    const Eigen::Vector3d& getRotationAbsementError() const { return rotation_absement_error_; }

    /**
     * @brief Set rotation absement error
     * @param error New rotation absement error
     */
    void setRotationAbsementError(const Eigen::Vector3d& error);

    /**
     * @brief Get rotation position error (for IMU posing PID)
     * @return Rotation position error
     */
    const Eigen::Vector3d& getRotationPositionError() const { return rotation_position_error_; }

    /**
     * @brief Set rotation position error
     * @param error New rotation position error
     */
    void setRotationPositionError(const Eigen::Vector3d& error);

    /**
     * @brief Get rotation velocity error (for IMU posing PID)
     * @return Rotation velocity error
     */
    const Eigen::Vector3d& getRotationVelocityError() const { return rotation_velocity_error_; }

    /**
     * @brief Set rotation velocity error
     * @param error New rotation velocity error
     */
    void setRotationVelocityError(const Eigen::Vector3d& error);

private:
    /**
     * @brief Initialize pose limits with default values
     */
    void initializePoseLimits();

    /**
     * @brief Validate pose state
     * @param pose Pose to validate
     * @return True if pose is valid
     */
    bool validatePoseState(const BodyPoseState& pose) const;
};

#endif // BODY_POSE_H