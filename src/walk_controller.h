#ifndef WALK_CONTROLLER_H
#define WALK_CONTROLLER_H

#include "gait_config.h"
#include "gait_config_factory.h"
#include "leg_stepper.h" // Include for StepCycle definition
#include "math_utils.h"
#include "robot_model.h"
#include "terrain_adaptation.h"
#include "velocity_limits.h"
#include "walkspace_analyzer.h"
#include "workspace_validator.h"
#include <map>
#include <memory>

/**
 * @brief Leg states for state machine control (OpenSHC equivalent)
 */
enum LegState {
    LEG_WALKING,                //< The leg is in a 'walking' state - participates in walking cycle
    LEG_MANUAL,                 //< The leg is in a 'manual' state - able to move via manual manipulation inputs
    LEG_STATE_COUNT,            //< Misc enum defining number of LegStates
    LEG_WALKING_TO_MANUAL = -1, //< The leg is in 'walking to manual' state - transitioning from 'walking' to 'manual' state
    LEG_MANUAL_TO_WALKING = -2, //< The leg is in 'manual to walking' state - transitioning from 'manual' to 'walking' state
};

// StepCycle is now defined in leg_stepper.h to avoid duplication

// WalkState and StepState are defined above

/**
 * @brief Complete walking controller with OpenSHC architecture
 */
class WalkController {
  public:
    /**
     * @brief Constructor with robot model and leg references
     * @param m Robot model for kinematics and parameters
     * @param legs Array of references to the actual Leg objects from LocomotionSystem
     */
    WalkController(RobotModel &m, Leg legs[NUM_LEGS]);

    /**
     * @brief Destructor
     */
    ~WalkController() = default;

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
     * @brief Update walking with velocity commands (OpenSHC equivalent)
     */
    void updateWalk(const Point3D &linear_velocity_input, double angular_velocity_input);

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
    StepCycle getStepCycle() const { return current_gait_config_.step_cycle; }
    double getTimeDelta() const { return time_delta_; }
    double getStepClearance() const { return step_clearance_; }
    double getStepDepth() const { return step_depth_; }

    Point3D getDesiredLinearVelocity() const { return desired_linear_velocity_; }
    double getDesiredAngularVelocity() const { return desired_angular_velocity_; }
    WalkState getWalkState() const { return walk_state_; }
    std::map<int, double> getWalkspace() const { return walkspace_; }
    Point3D getWalkPlane() const { return walk_plane_; }
    Point3D getWalkPlaneNormal() const { return walk_plane_normal_; }
    Point3D getOdometryIdeal() const { return odometry_ideal_; }
    std::shared_ptr<LegStepper> getLegStepper(int leg_index) const;

    // Modifiers
    void setPoseState(int state) { pose_state_ = state; }
    void setLinearSpeedLimitMap(const std::map<int, double> &limit_map) { max_linear_speed_ = limit_map; }
    void setAngularSpeedLimitMap(const std::map<int, double> &limit_map) { max_angular_speed_ = limit_map; }
    void setLinearAccelerationLimitMap(const std::map<int, double> &limit_map) { max_linear_acceleration_ = limit_map; }
    void setAngularAccelerationLimitMap(const std::map<int, double> &limit_map) { max_angular_acceleration_ = limit_map; }
    void setRegenerateWalkspace() { regenerate_walkspace_ = true; }

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
    const TerrainAdaptation &getTerrainAdaptation() const { return terrain_adaptation_; }
    RobotModel &getModel() { return model; }

    // Walkspace analysis control methods (OpenSHC equivalent)
    void enableWalkspaceAnalysis(bool enabled);
    bool isWalkspaceAnalysisEnabled() const;
    const WalkspaceAnalyzer::AnalysisInfo &getWalkspaceAnalysisInfo() const;
    std::string getWalkspaceAnalysisInfoString() const;
    void resetWalkspaceAnalysisStats();
    WalkspaceAnalyzer::WalkspaceResult analyzeCurrentWalkspace();
    bool generateWalkspaceMap();
    double getWalkspaceRadius(double bearing_degrees) const;

    // Enhanced walkspace analysis methods
    const std::map<int, double> &getCurrentWalkspaceMap() const;
    bool isWalkspaceMapGenerated() const;
    double getStabilityMargin() const;
    double getOverallStabilityScore() const;
    std::map<int, double> getLegReachabilityScores() const;
    bool isCurrentlyStable() const;

    // Gait configuration management methods (OpenSHC equivalent)
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
     * @brief Set gait by name using gait factory
     * @param gait_name Name of the gait to set
     * @return true if successful, false otherwise
     */
    bool setGaitByName(const std::string &gait_name);

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

    // Step parameter control
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
    double getStanceDuration() const {
        return (double)current_gait_config_.phase_config.stance_phase /
               (current_gait_config_.phase_config.stance_phase + current_gait_config_.phase_config.swing_phase);
    }

    /**
     * @brief Get current swing duration from gait configuration
     * @return Swing duration (0-1)
     */
    double getSwingDuration() const {
        return (double)current_gait_config_.phase_config.swing_phase /
               (current_gait_config_.phase_config.stance_phase + current_gait_config_.phase_config.swing_phase);
    }

    /**
     * @brief Get current cycle frequency from gait configuration
     * @return Cycle frequency in Hz
     */
    double getCycleFrequency() const { return current_gait_config_.step_frequency; }

  private:
    RobotModel &model;

    // OpenSHC architecture components
    StepCycle step_;
    double time_delta_;
    double step_clearance_;
    double step_depth_;
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

    // Gait configuration system (OpenSHC equivalent)
    GaitConfiguration current_gait_config_;
    GaitSelectionConfig gait_selection_config_;

    // Terrain adaptation system
    TerrainAdaptation terrain_adaptation_;

    // Velocity limits system
    VelocityLimits velocity_limits_;
    VelocityLimits::LimitValues current_velocity_limits_;
    VelocityLimits::LimitValues current_velocities_;

    // Workspace validation
    std::unique_ptr<WorkspaceValidator> workspace_validator_;

    // Walkspace analysis (OpenSHC equivalent)
    std::unique_ptr<WalkspaceAnalyzer> walkspace_analyzer_;

    // Collision avoidance: track current leg positions
    Point3D current_leg_positions_[NUM_LEGS];

    // Helper methods
    // Helper methods
    double calculateStabilityIndex() const;
    bool checkTerrainConditions() const;
    double calculateLegReach() const;

    // Declaración del método auxiliar para la posición de apoyo por defecto
    Point3D calculateDefaultStancePosition(int leg_index);
};

#endif // WALK_CONTROLLER_H
