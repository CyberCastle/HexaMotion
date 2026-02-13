#ifndef WORKSPACE_ANALYZER_H
#define WORKSPACE_ANALYZER_H

#include "hexamotion_constants.h"
#include "precision_config.h"
#include "robot_model.h"
#include <map>

class RobotModel; // Forward declaration

// OpenSHC-compatible type definitions (must be before class declaration)
typedef std::map<int, double> Workplane;       // bearing -> radius
typedef std::map<double, Workplane> Workspace; // height -> Workplane

/**
 * @brief Configuration for workspace validation behavior
 */
struct ValidationConfig {
    bool enable_collision_checking = true;
    bool enable_joint_limit_checking = true;
    bool enable_terrain_adaptation = true;
    double safety_margin = 20.0f;           // Safety margin for collision avoidance (mm)
    double angular_velocity_scaling = 0.8f; // Scaling factor for angular velocities
    double max_velocity_scale = 1.2f;       // Maximum velocity scaling factor
    double workspace_margin_factor = 0.95f; // Factor for workspace boundary margins
    double collision_safety_margin = 25.0f; // Specific margin for collision avoidance (mm)
    double safety_margin_factor = 0.9f;     // Factor for total reach calculations
    double minimum_reach_factor = 0.1f;     // Factor for minimum reach calculations
};

/**
 * @brief Workspace bounds information
 */
struct WorkspaceBounds {
    double min_reach;
    double max_reach;
    double preferred_min_reach;
    double preferred_max_reach;
    bool has_height_restrictions;
    double min_height;
    double max_height;
    Point3D center_position;
};

/**
 * @brief Velocity constraints for a specific direction and configuration
 */
struct VelocityConstraints {
    double max_forward_velocity;
    double max_backward_velocity;
    double max_lateral_velocity;
    double max_angular_velocity;
    bool direction_blocked;
    double scaling_factor;
    double max_linear_velocity;
    double max_acceleration;
    double workspace_radius;
    double stance_radius;
};

/**
 * @brief OpenSHC-style workspace generation and reachability system
 *
 * Equivalent to OpenSHC implementation with generateWorkspace() and getWorkplane() methods.
 */
class WorkspaceAnalyzer {
  private:
    const RobotModel &model_;
    ComputeConfig config_;
    ValidationConfig validation_config_;

    // Physical robot configuration offset
    double reference_height_offset_; // z = getDefaultHeightOffset() offset for physical robot configuration

    // Workspace geometry
    std::map<int, double> walkspace_map_; // Bearing -> radius mapping for analysis

    // Enhanced workspace storage with height layers (OpenSHC-compatible)
    Workspace leg_workspaces_[NUM_LEGS]; // 3D workspace per leg

    // Default and identity tip positions for walkspace generation (OpenSHC parity).
    // In OpenSHC these come from LegStepper::getIdentityTipPose() / getDefaultTipPose().
    // Initially both are set to FK(zero), matching OpenSHC's initial state where default == identity.
    Point3D identity_tip_positions_[NUM_LEGS];
    Point3D default_tip_positions_[NUM_LEGS];

    // Walkspace generation status
    bool walkspace_map_generated_;

    // Workspace cache flags (OpenSHC-style caching)
    bool leg_workspace_generated_[NUM_LEGS]; //< Cache flags for each leg workspace

  public:
    /**
     * @brief Constructor
     * @param model Reference to robot model
     * @param config Computation configuration
     * @param validation_config Configuration for validation behavior
     */
    explicit WorkspaceAnalyzer(const RobotModel &model,
                               ComputeConfig config = ComputeConfig::medium(),
                               const ValidationConfig &validation_config = ValidationConfig());

    /**
     * @brief Initialize workspace analyzer with robot geometry
     */
    void initialize();

    // ========================================================================
    // WORKSPACE GENERATION AND ANALYSIS (OpenSHC equivalent methods)
    // ========================================================================

    /**
     * @brief Generate walkspace map for current robot configuration
     * Equivalent to OpenSHC's generateWorkspace() function
     */
    void generateWorkspace();

    /**
     * @brief Get workplane at specific height with interpolation
     * Equivalent to OpenSHC's getWorkplane() function
     * @param leg_index Leg index (0-5)
     * @param height Height above workspace origin (mm)
     * @return Interpolated workplane at specified height
     */
    Workplane getWorkplane(int leg_index, double height) const;

    /**
     * @brief Get full 3D workspace for specific leg
     * @param leg_index Leg index (0-5)
     * @return Complete workspace with all height layers
     */
    Workspace getLegWorkspace(int leg_index) const;

    /**
     * @brief Constrain a tip target to the leg workspace (OpenSHC parity)
     * @param leg_index Leg index (0-5)
     * @param reference_tip_position Target position in robot frame
     * @return Reachable position constrained within workspace limits
     */
    Point3D makeReachable(int leg_index, const Point3D &reference_tip_position) const;

    /**
     * @brief Set identity and default tip positions for walkspace generation.
     *
     * In OpenSHC, WalkController::generateWalkspace() reads these from each
     * LegStepper.  Call this before generateWorkspace() so that Phase 1 uses
     * the correct default positions and Phase 2 computes the proper default
     * shift.  If never called, both arrays default to FK(zero) (matching
     * OpenSHC's initial state where default == identity).
     *
     * @param identity_tips Identity tip positions (flat walk-plane stance)
     * @param default_tips Default tip positions (runtime-adjusted, initially
     *                     equal to identity)
     */
    void setTipPositions(const Point3D identity_tips[NUM_LEGS],
                         const Point3D default_tips[NUM_LEGS]);

    /**
     * @brief Check if position is reachable by specific leg
     * @param leg_index Leg index (0-5)
     * @param position Target position in robot frame
     * @param use_ik_validation If true, uses IK validation (slower but accurate)
     * @return True if position is reachable
     */
    bool isPositionReachable(int leg_index, const Point3D &position, bool use_ik_validation = false);

    /**
     * @brief Get walkspace radius for specific bearing
     * @param bearing_degrees Bearing in degrees (0-360)
     * @return Maximum walkspace radius at bearing
     */
    double getWalkspaceRadius(double bearing_degrees) const;

    // ========================================================================
    // WORKSPACE BOUNDS AND CONSTRAINTS
    // ========================================================================

    /**
     * @brief Get comprehensive workspace bounds for a leg
     * @param leg_index Index of the leg
     * @return Complete workspace bounds information
     */
    WorkspaceBounds getWorkspaceBounds(int leg_index) const;

    /**
     * @brief Calculate velocity constraints for a specific leg and bearing
     * @param leg_index Index of the leg
     * @param bearing_degrees Direction of movement (0-360)
     * @param gait_frequency Gait frequency in Hz
     * @param stance_ratio Fraction of time leg is on ground
     * @return Velocity constraints for this configuration
     */
    VelocityConstraints calculateVelocityConstraints(int leg_index, double bearing_degrees = 0.0f,
                                                     double gait_frequency = 1.0f, double stance_ratio = 0.6f) const;

    /**
     * @brief Invalidate workspace cache for all legs (force regeneration)
     * Similar to OpenSHC's approach when robot configuration changes
     */
    void invalidateWorkspaceCache();

    /**
     * @brief Invalidate workspace cache for specific leg
     * @param leg_index Index of leg to invalidate (0-5)
     */
    void invalidateWorkspaceCache(int leg_index);

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    /**
     * @brief Set computational precision level
     * @param config New configuration
     */
    void setPrecisionConfig(const ComputeConfig &config) { config_ = config; }

    /**
     * @brief Update configuration
     */
    void updateConfig(const ValidationConfig &new_config) { validation_config_ = new_config; }

    /**
     * @brief Get current configuration
     */
    const ValidationConfig &getConfig() const { return validation_config_; }

    // ========================================================================
    // UTILITY AND TESTING METHODS
    // ========================================================================

    /**
     * @brief Get current walkspace map
     * @return Map of bearing to radius values
     */
    const std::map<int, double> &getWalkspaceMap() const { return walkspace_map_; }

    /**
     * @brief Check if walkspace map has been generated
     * @return True if walkspace map is available
     */
    bool isWalkspaceMapGenerated() const { return walkspace_map_generated_; }

    /**
     * @brief Constrain position to geometric workspace bounds only
     */
    Point3D constrainToGeometricWorkspace(int leg_index, const Point3D &target) const;

  private:
    // ========================================================================
    // PRIVATE ANALYSIS METHODS
    // ========================================================================

    void generateWalkspaceForLeg(int leg_index);
    bool detailedReachabilityCheck(int leg_index, const Point3D &position);
};

#endif // WORKSPACE_ANALYZER_H
