#ifndef WALK_CONTROLLER_H
#define WALK_CONTROLLER_H

#include "body_pose_config.h"
#include "body_pose_controller.h"
#include "gait_config.h"
#include "gait_config_factory.h"
/** Include for GaitType, LegState, WalkState definitions. */
#include "gait_types.h"
/** Include for LegStepper definition. */
#include "leg_stepper.h"
#include "math_utils.h"
#include "robot_model.h"
#include "terrain_adaptation.h"
#include "velocity_limits.h"
#include <map>
#include <memory>

/**
 * @brief Complete walking controller with OpenSHC architecture
 */
class WalkController {
  public:
    /**
     * @brief Constructor with robot model and leg references
     * @param m Robot model for kinematics and parameters
     * @param legs Array of references to the actual Leg objects from LocomotionSystem
     * @param pose_config Body pose configuration containing standing pose joints
     */
    WalkController(RobotModel &m, Leg legs[NUM_LEGS], const BodyPoseConfiguration &pose_config);

    /**
     * @brief Destructor
     */
    ~WalkController() = default;

    /**
     * @brief Initialize the walk controller with default parameters and current robot pose
     * @param current_body_position Current position of the robot body
     * @param current_body_orientation Current orientation of the robot body
     */
    void init(const Eigen::Vector3d &current_body_position, const Eigen::Vector3d &current_body_orientation);

    /**
     * @brief Generate walkspace for the robot
     */
    void generateWalkspace();

    /**
     * @brief Generate velocity limits for the current step cycle
     */
    void generateLimits(StepCycle step);

    /**
     * @brief Generate velocity limits for current gait step cycle
     */
    void generateLimits();

    /**
     * @brief Update walking with velocity commands and current robot pose (OpenSHC equivalent).
     *
     * Applies velocity limiting (via VelocityLimits::getLimit), acceleration
     * ramping, gait state machine transitions and per-leg stepper updates.
     *
     * @param linear_velocity_input   Desired linear velocity (mm/s).
     * @param angular_velocity_input   Desired angular velocity (rad/s).
     * @param current_body_position    Current robot body position.
     * @param current_body_orientation Current robot body orientation (roll, pitch, yaw).
     */
    void updateWalk(const Point3D &linear_velocity_input, double angular_velocity_input,
                    const Eigen::Vector3d &current_body_position, const Eigen::Vector3d &current_body_orientation);

    /**
     * @brief Calculate odometry for the given time period
     */
    Pose calculateOdometry(double time_period);

    /**
     * @brief Update manual leg control with tip velocity inputs (OpenSHC equivalent).
     *
     * Two modes are available via Parameters::manual_leg:
     * - joint_control: maps velocity inputs to coxa/tibia joint positions directly (3DOF only)
     * - tip_control: moves tip in cartesian space in the robot frame
     *
     * @param primary_leg_index Index of the primary selected leg (-1 if none)
     * @param primary_tip_velocity Velocity input for the primary leg tip
     * @param secondary_leg_index Index of the secondary selected leg (-1 if none)
     * @param secondary_tip_velocity Velocity input for the secondary leg tip
     */
    void updateManual(int primary_leg_index, const Eigen::Vector3d &primary_tip_velocity,
                      int secondary_leg_index, const Eigen::Vector3d &secondary_tip_velocity);

    /**
     * @brief Update manual leg control with direct cartesian tip positions (OpenSHC equivalent).
     *
     * @param primary_leg_index Index of the primary selected leg (-1 if none)
     * @param primary_tip_position Desired tip position for the primary leg
     * @param secondary_leg_index Index of the secondary selected leg (-1 if none)
     * @param secondary_tip_position Desired tip position for the secondary leg
     */
    void updateManual(int primary_leg_index, const Point3D &primary_tip_position,
                      int secondary_leg_index, const Point3D &secondary_tip_position);

    /** Modifiers for velocity/acceleration limit maps (OpenSHC equivalent). */
    void setLinearSpeedLimitMap(const std::map<int, double> &limit_map) { max_linear_speed_ = limit_map; }
    void setAngularSpeedLimitMap(const std::map<int, double> &limit_map) { max_angular_speed_ = limit_map; }
    void setLinearAccelerationLimitMap(const std::map<int, double> &limit_map) { max_linear_acceleration_ = limit_map; }
    void setAngularAccelerationLimitMap(const std::map<int, double> &limit_map) { max_angular_acceleration_ = limit_map; }

    /**
     * @brief Set body pose controller reference for walk plane functionality
     * @param controller Pointer to BodyPoseController instance
     */
    void setBodyPoseController(BodyPoseController *controller);

    /**
     * @brief Compute velocity/acceleration limits for a hypothetical gait configuration.
     *
     * This does not modify the current active gait configuration or leg steppers.
     */
    void computeLimitsForConfig(const GaitConfiguration &gait_config,
                                std::map<int, double> &max_linear_speed,
                                std::map<int, double> &max_angular_speed,
                                std::map<int, double> &max_linear_acceleration,
                                std::map<int, double> &max_angular_acceleration);

    /**
     * @brief Get interpolated limit for a command from a provided bearing limit map.
     */
    double getLimit(const Eigen::Vector2d &linear_velocity_input,
                    double angular_velocity_input,
                    const std::map<int, double> &limit_map) const;

    /**
     * @brief Estimate gravity vector
     */
    Point3D estimateGravity() const;

    /** Accessors. */
    /** Moved implementation to cpp. */
    StepCycle getStepCycle() const;
    /** Moved implementation to cpp. */
    double getTimeDelta() const;
    /** Moved implementation to cpp. */
    double getStepClearance() const;
    /** Moved implementation to cpp. */
    double getStepDepth() const;

    Point3D getDesiredLinearVelocity() const { return desired_linear_velocity_; }
    double getDesiredAngularVelocity() const { return desired_angular_velocity_; }
    WalkState getWalkState() const { return walk_state_; }
    std::map<int, double> getWalkspace() const { return walkspace_; }
    /** Walk plane functionality moved to BodyPoseController. */
    /** Moved implementation to cpp. */
    Point3D getWalkPlane() const;
    /** Moved implementation to cpp. */
    Point3D getWalkPlaneNormal() const;
    Pose getOdometryIdeal() const { return odometry_ideal_; }
    std::shared_ptr<LegStepper> getLegStepper(int leg_index) const;

    /** Modifiers. */
    void setPoseState(int state) { pose_state_ = state; }
    void setRegenerateWalkspace() { regenerate_walkspace_ = true; }

    /** Terrain adaptation methods. */
    void enableRoughTerrainMode(bool enabled, bool force_normal_touchdown = true, bool proactive_adaptation = true);
    void enableForceNormalTouchdown(bool enabled);
    void enableGravityAlignedTips(bool enabled);
    const TerrainAdaptation::WalkPlane &getTerrainWalkPlane() const;
    const TerrainAdaptation::StepPlane &getStepPlane(int leg_index) const;
    bool hasTouchdownDetection(int leg_index) const;
    /** Terrain adaptation accessors for LegStepper. */
    const TerrainAdaptation &getTerrainAdaptation() const { return terrain_adaptation_; }
    RobotModel &getModel() { return model; }

    /**
     * @brief Update terrain adaptation state with latest sensor data
     * @param fsr_interface FSR interface (may be nullptr)
     * @param imu_interface IMU interface (may be nullptr)
     */
    void updateTerrainAdaptation(IFSRInterface *fsr_interface, IIMUInterface *imu_interface);

    /** Gait configuration management methods (OpenSHC equivalent). */
    /**
     * @brief Set gait configuration and apply to all leg steppers
     * @param gait_config The gait configuration to apply
     * @return true if successful, false otherwise
     */
    bool setGaitConfiguration(const GaitConfiguration &gait_config);

    /**
     * @brief Get current gait configuration
     * @return Current gait configuration
     */
    const GaitConfiguration &getCurrentGaitConfig() const { return current_gait_config_; }

    /**
     * @brief Set gait using gait configuration
     * @param gait_config The gait configuration to set
     * @return true if successful, false otherwise
     */
    bool setGait(const GaitConfiguration &gait_config);

    /**
     * @brief Set gait using gait type (convenience method)
     * @param gait_type The gait type enum to set
     * @return true if successful, false otherwise
     */
    bool setGait(GaitType gait_type);

    /**
     * @brief Get current gait name
     * @return Current gait name
     */
    std::string getCurrentGaitName() const { return current_gait_config_.gait_name; }

    /**
     * @brief Apply gait configuration to leg steppers
     * @param gait_config The gait configuration to apply
     */
    void applyGaitConfigToLegSteppers(const GaitConfiguration &gait_config);

    /** Step parameter control. */
    /**
     * @brief Get current step height from gait configuration
     * @return Step height in mm
     */
    double getStepHeight() const { return current_gait_config_.swing_height; }

    /**
     * @brief Get current step length from gait configuration
     * @return Step length in mm
     */
    double getStepLength() const { return current_gait_config_.step_length; }

    /**
     * @brief Get current stance duration from gait configuration
     * @return Stance duration (0-1)
     */
    /** Moved implementation to cpp. */
    double getStanceDuration() const;

    /**
     * @brief Get current swing duration from gait configuration
     * @return Swing duration (0-1)
     */
    /** Moved implementation to cpp. */
    double getSwingDuration() const;

    /**
     * @brief Get current cycle frequency from gait configuration
     * @return Cycle frequency in Hz
     */
    /** Moved implementation to cpp. */
    double getCycleFrequency() const;

    /**
     * @brief Get calculated leg trajectory information for locomotion system
     * @param leg_index Index of the leg (0-5)
     * @return Calculated tip position and trajectory information
     */
    struct LegTrajectoryInfo {
        Point3D target_position;
        StepPhase step_phase;
        double phase_progress;
        bool is_stance;
        Point3D velocity;
    };

    LegTrajectoryInfo getLegTrajectoryInfo(int leg_index) const;

  private:
    RobotModel &model;
    double standing_horizontal_reach_;

    /** OpenSHC architecture components. */
    double time_delta_;
    double step_clearance_;
    double step_depth_;
    Point3D desired_linear_velocity_;
    double desired_angular_velocity_;
    WalkState walk_state_;
    std::map<int, double> walkspace_;
    Pose odometry_ideal_;
    int pose_state_;

    /** Current robot pose (provided by BodyPoseController). */
    Eigen::Vector3d current_body_position_;
    Eigen::Vector3d current_body_orientation_;

    /** State tracking. */
    bool regenerate_walkspace_;
    int legs_at_correct_phase_;
    int legs_completed_first_step_;
    bool return_to_default_attempted_;

    /** Leg steppers. */
    std::vector<std::shared_ptr<LegStepper>> leg_steppers_;

    /** Gait configuration system (OpenSHC equivalent). */
    GaitConfiguration current_gait_config_;

    /** Terrain adaptation system. */
    TerrainAdaptation terrain_adaptation_;

    /** Body pose controller reference for walk plane functionality. */
    BodyPoseController *body_pose_controller_;

    /** Velocity/acceleration limits (OpenSHC equivalent maps keyed by bearing). */
    VelocityLimits velocity_limits_;
    std::map<int, double> max_linear_speed_;
    std::map<int, double> max_angular_speed_;
    std::map<int, double> max_linear_acceleration_;
    std::map<int, double> max_angular_acceleration_;

    /** Collision avoidance: track current leg positions. */
    Point3D current_leg_positions_[NUM_LEGS];

    /** Reference to legs array for body pose controller updates. */
    Leg *legs_array_;

    /** Global phase counter for gait coordination (OpenSHC equivalent). */
    int global_phase_;

    /** Helper methods. */
};

#endif /**< WALK_CONTROLLER_H */
