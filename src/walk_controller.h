#ifndef WALK_CONTROLLER_H
#define WALK_CONTROLLER_H

#include "HexaModel.h"
#include "terrain_adaptation.h"
#include "velocity_limits.h"
#include "workspace_validator.h"
#include "math_utils.h"
#include "leg_stepper.h"
#include <memory>
#include <map>

/**
 * @brief Leg states for state machine control (OpenSHC equivalent)
 */
enum LegState {
    LEG_WALKING,                ///< The leg is in a 'walking' state - participates in walking cycle
    LEG_MANUAL,                 ///< The leg is in a 'manual' state - able to move via manual manipulation inputs
    LEG_STATE_COUNT,            ///< Misc enum defining number of LegStates
    LEG_WALKING_TO_MANUAL = -1, ///< The leg is in 'walking to manual' state - transitioning from 'walking' to 'manual' state
    LEG_MANUAL_TO_WALKING = -2, ///< The leg is in 'manual to walking' state - transitioning from 'manual' to 'walking' state
};

/**
 * @brief Step cycle timing parameters (OpenSHC equivalent)
 */
struct StepCycle {
    double frequency_;      ///< Step frequency in Hz
    int period_;           ///< Total step cycle length in iterations
    int swing_period_;     ///< Swing period length in iterations
    int stance_period_;    ///< Stance period length in iterations
    int stance_end_;       ///< Iteration when stance period ends
    int swing_start_;      ///< Iteration when swing period starts
    int swing_end_;        ///< Iteration when swing period ends
    int stance_start_;     ///< Iteration when stance period starts
};

// WalkState and StepState are defined above

/**
 * @brief Complete walking controller with OpenSHC architecture
 */
class WalkController {
public:
    /**
     * @brief Construct a walk controller.
     * @param model Reference to the robot model used for kinematics.
     */
    explicit WalkController(RobotModel &model);

    /**
     * @brief Initialize the walk controller with default parameters
     */
    void init();

    /**
     * @brief Generate walkspace for the robot
     */
    void generateWalkspace();

    /**
     * @brief Generate velocity limits for the current step cycle
     */
    void generateLimits(StepCycle step);

    /**
     * @brief Generate step cycle timing parameters
     */
    StepCycle generateStepCycle(bool set_step_cycle = true);

    /**
     * @brief Get velocity limit for a given bearing
     */
    double getLimit(const Point3D& linear_velocity_input, double angular_velocity_input,
                   const std::map<int, double>& limit);

    /**
     * @brief Update walking with velocity commands (OpenSHC equivalent)
     */
    void updateWalk(const Point3D& linear_velocity_input, double angular_velocity_input);

    /**
     * @brief Update walk plane estimation
     */
    void updateWalkPlane();

    /**
     * @brief Calculate odometry for the given time period
     */
    Point3D calculateOdometry(double time_period);

    /**
     * @brief Estimate gravity vector
     */
    Point3D estimateGravity() const;

    // Accessors
    StepCycle getStepCycle() const { return step_; }
    double getTimeDelta() const { return time_delta_; }
    double getStepClearance() const { return step_clearance_; }
    double getStepDepth() const { return step_depth_; }
    double getBodyClearance() const { return body_clearance_; }
    Point3D getDesiredLinearVelocity() const { return desired_linear_velocity_; }
    double getDesiredAngularVelocity() const { return desired_angular_velocity_; }
    WalkState getWalkState() const { return walk_state_; }
    std::map<int, double> getWalkspace() const { return walkspace_; }
    Point3D getWalkPlane() const { return walk_plane_; }
    Point3D getWalkPlaneNormal() const { return walk_plane_normal_; }
    Point3D getOdometryIdeal() const { return odometry_ideal_; }
    Point3D getModelCurrentPose() const;
    std::shared_ptr<LegStepper> getLegStepper(int leg_index) const;

    // Modifiers
    void setPoseState(int state) { pose_state_ = state; }
    void setLinearSpeedLimitMap(const std::map<int, double>& limit_map) { max_linear_speed_ = limit_map; }
    void setAngularSpeedLimitMap(const std::map<int, double>& limit_map) { max_angular_speed_ = limit_map; }
    void setLinearAccelerationLimitMap(const std::map<int, double>& limit_map) { max_linear_acceleration_ = limit_map; }
    void setAngularAccelerationLimitMap(const std::map<int, double>& limit_map) { max_angular_acceleration_ = limit_map; }
    void setRegenerateWalkspace() { regenerate_walkspace_ = true; }

    // Legacy interface compatibility
    bool setGaitType(GaitType gait);
    bool planGaitSequence(double vx, double vy, double omega);
    void updateGaitPhase(double dt);
    double getGaitPhase() const { return gait_phase; }
    Point3D footTrajectory(int leg, double phase, double step_height, double step_length,
                           double stance_duration, double swing_duration, double robot_height,
                           const double leg_phase_offsets[NUM_LEGS], LegState (&leg_states)[NUM_LEGS],
                           IFSRInterface *fsr, IIMUInterface *imu);

    // Velocity limiting methods
    VelocityLimits::LimitValues getVelocityLimits(double bearing_degrees = 0.0f) const;
    VelocityLimits::LimitValues applyVelocityLimits(double vx, double vy, double omega) const;
    bool validateVelocityCommand(double vx, double vy, double omega) const;
    void updateVelocityLimits(double frequency, double stance_ratio, double time_to_max_stride = 2.0f);
    void setVelocitySafetyMargin(double margin);
    void setAngularVelocityScaling(double scaling);
    VelocityLimits::WorkspaceConfig getWorkspaceConfig() const;

    // Terrain adaptation methods
    void enableRoughTerrainMode(bool enabled, bool force_normal_touchdown = true, bool proactive_adaptation = true);
    void enableForceNormalTouchdown(bool enabled);
    void enableGravityAlignedTips(bool enabled);
    void setExternalTarget(int leg_index, const TerrainAdaptation::ExternalTarget &target);
    void setExternalDefault(int leg_index, const TerrainAdaptation::ExternalTarget &default_pos);
    const TerrainAdaptation::WalkPlane &getTerrainWalkPlane() const;
    const TerrainAdaptation::ExternalTarget &getExternalTarget(int leg_index) const;
    const TerrainAdaptation::StepPlane &getStepPlane(int leg_index) const;
    bool hasTouchdownDetection(int leg_index) const;
    const VelocityLimits::LimitValues &getCurrentVelocities() const;

    // Terrain adaptation accessors for LegStepper
    const TerrainAdaptation& getTerrainAdaptation() const { return terrain_adaptation_; }
    RobotModel& getModel() { return model; }

private:
    RobotModel &model;

    // OpenSHC architecture components
    StepCycle step_;
    double time_delta_;
    double step_clearance_;
    double step_depth_;
    double body_clearance_;
    Point3D desired_linear_velocity_;
    double desired_angular_velocity_;
    WalkState walk_state_;
    std::map<int, double> walkspace_;
    Point3D walk_plane_;
    Point3D walk_plane_normal_;
    Point3D odometry_ideal_;
    int pose_state_;

    // Velocity limits
    std::map<int, double> max_linear_speed_;
    std::map<int, double> max_angular_speed_;
    std::map<int, double> max_linear_acceleration_;
    std::map<int, double> max_angular_acceleration_;

    // State tracking
    bool regenerate_walkspace_;
    int legs_at_correct_phase_;
    int legs_completed_first_step_;
    bool return_to_default_attempted_;

    // Leg steppers
    std::vector<std::shared_ptr<LegStepper>> leg_steppers_;

    // Legacy compatibility
    GaitType current_gait;
    double gait_phase;

    // Terrain adaptation system
    TerrainAdaptation terrain_adaptation_;

    // Velocity limits system
    VelocityLimits velocity_limits_;
    VelocityLimits::LimitValues current_velocity_limits_;
    VelocityLimits::LimitValues current_velocities_;

    // Workspace validation
    std::unique_ptr<WorkspaceValidator> workspace_validator_;

    // Collision avoidance: track current leg positions
    Point3D current_leg_positions_[NUM_LEGS];
};

#endif // WALK_CONTROLLER_H
