#ifndef WORKSPACE_VALIDATOR_H
#define WORKSPACE_VALIDATOR_H

#include "hexamotion_constants.h"
#include "robot_model.h"
#include <cmath>

class RobotModel; // Forward declaration

/**
 * @file workspace_validator.h
 * @brief Workspace validation and collision avoidance system
 *
 * This is the single source of truth for all workspace validation,
 * collision detection, and spatial constraints in HexaMotion.
 *
 * Replaces all legacy workspace/collision modules including:
 * - LegCollisionAvoidance
 * - Various workspace checks in different modules
 */

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
    double min_radius;       // Added for compatibility
    double max_radius;       // Added for compatibility
    Point3D center_position; // Added for center position tracking
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
    double max_linear_velocity; // Added for compatibility
    double max_acceleration;    // Added for compatibility
    double workspace_radius;    // Added for workspace calculations
    double stance_radius;       // Added for angular calculations
};

/**
 * @brief Scaling factors for different movement types
 */
struct ScalingFactors {
    double linear_scale;
    double angular_scale;
    double workspace_scale;
    double collision_scale;
    double velocity_scale;     // Added for velocity scaling
    double acceleration_scale; // Added for acceleration scaling
    double safety_margin;      // Added for safety margin scaling
};

/**
 * @brief Comprehensive validation result
 */
struct ValidationResult {
    Point3D constrained_position;
    bool is_reachable;
    bool is_collision_free;
    bool is_within_joint_limits;
    double distance_from_base;
    double collision_risk_factor;
    bool is_valid;
    Point3D safe_position;

    bool isValid() const {
        return is_reachable && is_collision_free && is_within_joint_limits;
    }
};

/**
 * @brief Workspace validator and collision avoidance system
 */
class WorkspaceValidator {
  public:
    /**
     * @brief Constructor
     * @param model Reference to robot model
     * @param config Configuration for validation behavior
     */
    WorkspaceValidator(const RobotModel &model, const ValidationConfig &config = ValidationConfig());

    /**
     * @brief Validate a target position with comprehensive checking
     * @param leg_index Index of the leg
     * @param target_position Target position to validate
     * @param current_leg_positions Current positions of all legs
     * @param constrain_if_invalid If true, attempt to constrain invalid positions
     * @return Comprehensive validation result
     */
    ValidationResult validateTarget(int leg_index, Point3D target_position,
                                    const Point3D current_leg_positions[NUM_LEGS],
                                    bool constrain_if_invalid = true);

    /**
     * @brief Check if a position is geometrically reachable
     * @param leg_index Index of the leg
     * @param target Target position to check
     * @return true if reachable
     */
    bool isReachable(int leg_index, const Point3D &target) const;

    /**
     * @brief Calculate collision risk factor between legs
     * @param leg_index Index of the leg
     * @param target_position Target position to check
     * @param current_leg_positions Current positions of all legs
     * @return Risk factor (0.0 = no risk, 1.0 = high risk)
     */
    double checkCollisionRisk(int leg_index, const Point3D &target_position,
                              const Point3D current_leg_positions[NUM_LEGS]) const;

    /**
     * @brief Constrain position to valid workspace
     * @param leg_index Index of the leg
     * @param target Target position to constrain
     * @param current_leg_positions Current positions of all legs
     * @return Constrained position
     */
    Point3D constrainToValidWorkspace(int leg_index, const Point3D &target,
                                      const Point3D current_leg_positions[NUM_LEGS]) const;

    /**
     * @brief Get workspace bounds for a specific leg
     * @param leg_index Index of the leg
     * @param min_reach Output: minimum reachable distance
     * @param max_reach Output: maximum reachable distance
     */
    void getWorkspaceBounds(int leg_index, double &min_reach, double &max_reach) const;

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
     * @brief Check if position is reachable (optimized for WalkspaceAnalyzer)
     * @param leg_index Index of the leg
     * @param position Position to check
     * @param use_ik_validation If true, uses IK validation (slower but accurate)
     * @return true if position is reachable
     */
    bool isPositionReachable(int leg_index, const Point3D &position, bool use_ik_validation = false) const;

    /**
     * @brief Get scaling factors
     * @return Current scaling factors used across all modules
     */
    ScalingFactors getScalingFactors() const;

    /**
     * @brief Update safety margin
     * @param margin New safety margin value
     */
    void updateSafetyMargin(double margin);

    /**
     * @brief Update angular scaling
     * @param scaling New angular scaling value
     */
    void updateAngularScaling(double scaling);

    /**
     * @brief Update configuration
     */
    void updateConfig(const ValidationConfig &new_config) { config_ = new_config; }

    /**
     * @brief Get current configuration
     */
    const ValidationConfig &getConfig() const { return config_; }

    /**
     * @brief Calculate limit proximity for joint angles (OpenSHC-style)
     * @param leg_index Index of the leg
     * @param joint_angles Current joint angles
     * @return Limit proximity (1.0 = furthest from limit, 0.0 = at limit)
     */
    double calculateLimitProximity(int leg_index, const JointAngles &joint_angles) const;

    /**
     * @brief Get base position for a specific leg (for testing)
     */
    Point3D getLegBase(int leg_index) const;

    /**
     * @brief Calculate distance from leg base to target (for testing)
     */
    double getDistanceFromBase(int leg_index, const Point3D &target) const;

    /**
     * @brief Check if position respects joint limits (requires IK) (for testing)
     */
    bool checkJointLimits(int leg_index, const Point3D &target) const;

    /**
     * @brief Constrain position to geometric workspace bounds only (for testing)
     */
    Point3D constrainToGeometricWorkspace(int leg_index, const Point3D &target) const;

    /**
     * @brief Calculate minimum safe hexagon radius to prevent leg collisions
     * @param leg_reach Maximum reach of each leg
     * @param safety_margin Safety margin between adjacent leg workspaces (mm)
     * @return Minimum safe hexagon radius
     */
    static double calculateSafeHexagonRadius(double leg_reach, double safety_margin = 30.0f);

    /**
     * @brief Get distance between two 2D points (ignoring Z coordinate)
     * @param p1 First point
     * @param p2 Second point
     * @return Distance in mm
     */
    static double getDistance2D(const Point3D &p1, const Point3D &p2);

    /**
     * @brief Get the indices of adjacent legs for a given leg
     * @param leg_index Index of the leg (0-5)
     * @param left_adjacent Will be set to left adjacent leg index
     * @param right_adjacent Will be set to right adjacent leg index
     */
    static void getAdjacentLegIndices(int leg_index, int &left_adjacent, int &right_adjacent);

    /**
     * @brief Check if two leg workspaces would overlap
     * @param leg1_base Base position of first leg
     * @param leg1_reach Reach of first leg
     * @param leg2_base Base position of second leg
     * @param leg2_reach Reach of second leg
     * @param safety_margin Safety margin to maintain
     * @return true if workspaces would overlap
     */
    static bool checkWorkspaceOverlap(const Point3D &leg1_base, double leg1_reach,
                                      const Point3D &leg2_base, double leg2_reach,
                                      double safety_margin = 20.0f);

    /**
     * @brief Check if a target would collide with adjacent legs
     * @param leg_index Index of the leg
     * @param target_position Target position to check
     * @param hexagon_radius Radius of the hexagon base
     * @param leg_reach Maximum leg reach
     * @param adjacent_positions Current positions of all legs
     * @return true if collision would occur
     */
    bool wouldCollideWithAdjacent(int leg_index, const Point3D &target_position,
                                  double hexagon_radius, double leg_reach,
                                  const Point3D adjacent_positions[NUM_LEGS]) const;

    /**
     * @brief Adjust position to avoid collisions with adjacent legs
     * @param leg_index Index of the leg
     * @param target_position Position to adjust (modified in place)
     * @param hexagon_radius Radius of the hexagon base
     * @param leg_reach Maximum leg reach
     * @param adjacent_positions Current positions of all legs
     * @return true if adjustment was successful
     */
    bool adjustForCollisionAvoidance(int leg_index, Point3D &target_position,
                                     double hexagon_radius, double leg_reach,
                                     const Point3D adjacent_positions[NUM_LEGS]) const;

  private:
    const RobotModel &model_;
    ValidationConfig config_;

    /**
     * @brief Transform target from global to leg-local coordinates
     */
    Point3D globalToLegLocal(int leg_index, const Point3D &global_pos) const;

    /**
     * @brief Transform target from leg-local to global coordinates
     */
    Point3D legLocalToGlobal(int leg_index, const Point3D &local_pos) const;

    /**
     * @brief Apply collision avoidance adjustment (internal implementation)
     */
    Point3D adjustForCollisionAvoidanceInternal(int leg_index, const Point3D &target,
                                                const Point3D current_leg_positions[NUM_LEGS]) const;

    // Constants
    static constexpr double MIN_LEG_SEPARATION = 50.0f; // Minimum distance between leg tips (mm)
};

#endif // WORKSPACE_VALIDATOR_H
