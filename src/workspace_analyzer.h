#ifndef WORKSPACE_ANALYZER_H
#define WORKSPACE_ANALYZER_H

#include "hexamotion_constants.h"
#include "precision_config.h"
#include "robot_model.h"
#include <map>
#include <vector>

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
 * @brief Scaling factors for different movement types
 */
struct ScalingFactors {
    double linear_scale;
    double angular_scale;
    double workspace_scale;
    double collision_scale;
    double velocity_scale;
    double acceleration_scale;
    double safety_margin;
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
 * @brief Unified workspace analysis and validation system
 *
 * Combines the functionality of WalkspaceAnalyzer and WorkspaceValidator
 * to provide comprehensive workspace analysis, validation, and collision
 * avoidance for the HexaMotion robot system.
 *
 * Equivalent to OpenSHC implementation with generateWorkspace() and getWorkplane() methods.
 */
class WorkspaceAnalyzer {
  public:
    /**
     * @brief Walkspace analysis result structure
     */
    struct WalkspaceResult {
        Point3D center_of_mass;                //< Calculated center of mass
        double stability_margin;               //< Stability margin (mm)
        std::vector<Point3D> support_polygon;  //< Support polygon vertices
        std::map<int, double> walkspace_radii; //< Walkspace radii by bearing
        bool is_stable;                        //< Overall stability flag
        double reachable_area;                 //< Total reachable area (mm²)
    };

    /**
     * @brief Comprehensive analysis information for external systems
     */
    struct AnalysisInfo {
        WalkspaceResult current_result;            //< Current walkspace analysis result
        std::map<int, WorkspaceBounds> leg_bounds; //< Workspace bounds for each leg
        std::map<int, double> leg_reachability;    //< Reachability scores (0-1) for each leg
        double overall_stability_score;            //< Overall stability score (0-1)
        bool analysis_enabled;                     //< Whether analysis is currently enabled
        unsigned long last_analysis_time;          //< Timestamp of last analysis
        int analysis_count;                        //< Number of analyses performed
        double average_analysis_time_ms;           //< Average analysis time in milliseconds
        double total_analysis_time_ms;             //< Total time spent in analysis
        double min_analysis_time_ms;               //< Minimum analysis time
        double max_analysis_time_ms;               //< Maximum analysis time
        std::map<int, double> walkspace_radii;     //< Current walkspace radii by bearing
        bool walkspace_map_generated;              //< Whether walkspace map has been generated
    };

  private:
    const RobotModel &model_;
    ComputeConfig config_;
    ValidationConfig validation_config_;

    // Physical robot configuration offset
    double reference_height_offset_; // z = getDefaultHeightOffset() offset for physical robot configuration

    // Workspace geometry
    WorkspaceBounds leg_workspace_[NUM_LEGS];
    std::map<int, double> walkspace_map_; // Bearing -> radius mapping for analysis

    // Enhanced workspace storage with height layers (OpenSHC-compatible)
    Workspace leg_workspaces_[NUM_LEGS]; // 3D workspace per leg

    // Analysis parameters (constants now defined in hexamotion_constants.h)
    static constexpr double STABILITY_THRESHOLD = 10.0f; // mm
    static constexpr double MIN_LEG_SEPARATION = 50.0f;  // Minimum distance between leg tips (mm)

    // Runtime control flags
    bool analysis_enabled_;                 //< Flag to enable/disable real-time analysis
    AnalysisInfo analysis_info_;            //< Comprehensive analysis information
    unsigned long last_analysis_timestamp_; //< Timestamp of last analysis
    double total_analysis_time_;            //< Total time spent in analysis

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

    // ========================================================================
    // POSITION VALIDATION AND REACHABILITY
    // ========================================================================

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
     * @brief Check if position is reachable by specific leg
     * @param leg_index Leg index (0-5)
     * @param position Target position in robot frame
     * @param use_ik_validation If true, uses IK validation (slower but accurate)
     * @return True if position is reachable
     */
    bool isPositionReachable(int leg_index, const Point3D &position, bool use_ik_validation = false);

    // ========================================================================
    // COLLISION DETECTION AND AVOIDANCE
    // ========================================================================

    /**
     * @brief Calculate collision risk factor between legs
     * @param leg_index Index of the leg
     * @param target_position Target position to check
     * @param current_leg_positions Current positions of all legs
     * @return Risk factor (0.0 = no risk, 1.0 = high risk)
     */
    double checkCollisionRisk(int leg_index, const Point3D &target,
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

    // ========================================================================
    // WALKSPACE ANALYSIS
    // ========================================================================

    /**
     * @brief Analyze current walkspace given leg positions
     * @param leg_positions Current leg tip positions
     * @return Walkspace analysis result
     */
    WalkspaceResult analyzeWalkspace(const Point3D leg_positions[NUM_LEGS]);

    /**
     * @brief Get walkspace radius for specific bearing
     * @param bearing_degrees Bearing in degrees (0-360)
     * @return Maximum walkspace radius at bearing
     */
    double getWalkspaceRadius(double bearing_degrees) const;

    /**
     * @brief Get optimal step positions for body movement
     * @param body_movement Desired body movement vector
     * @param current_positions Current leg positions
     * @param optimal_positions Output optimal leg positions
     * @return True if optimization succeeded
     */
    bool getOptimalStepPositions(const Point3D &body_movement,
                                 const Point3D current_positions[NUM_LEGS],
                                 Point3D optimal_positions[NUM_LEGS]);

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

    // ========================================================================
    // ANALYSIS CONTROL AND INFORMATION
    // ========================================================================

    /**
     * @brief Get physical reference height offset (z = getDefaultHeightOffset())
     * @return Reference height offset where robot body is positioned when all angles are 0°
     */
    double getPhysicalReferenceHeight() const { return reference_height_offset_; }

    /**
     * @brief Enable or disable real-time walkspace analysis
     * @param enabled True to enable analysis, false to disable
     */
    void enableAnalysis(bool enabled) {
        analysis_enabled_ = enabled;
        analysis_info_.analysis_enabled = enabled;
    }

    /**
     * @brief Check if analysis is currently enabled
     * @return True if analysis is enabled
     */
    bool isAnalysisEnabled() const { return analysis_enabled_; }

    /**
     * @brief Get comprehensive analysis information for external systems
     * @return Complete analysis information structure
     */
    const AnalysisInfo &getAnalysisInfo() const { return analysis_info_; }

    /**
     * @brief Get analysis information as a formatted string for debugging
     * @return Formatted string with analysis information
     */
    std::string getAnalysisInfoString() const;

    /**
     * @brief Reset analysis statistics
     */
    void resetAnalysisStats();

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
    // CONFIGURATION AND SCALING
    // ========================================================================

    /**
     * @brief Set computational precision level
     * @param config New configuration
     */
    void setPrecisionConfig(const ComputeConfig &config) { config_ = config; }

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
    bool isWalkspaceMapGenerated() const { return analysis_info_.walkspace_map_generated; }

    /**
     * @brief Check if workspace for specific leg has been cached
     * @param leg_index Index of leg to check (0-5)
     * @return True if workspace is cached for this leg
     */
    bool isLegWorkspaceCached(int leg_index) const {
        return (leg_index >= 0 && leg_index < NUM_LEGS) ? leg_workspace_generated_[leg_index] : false;
    }

    /**
     * @brief Calculate limit proximity for joint angles (OpenSHC-style)
     * @param leg_index Index of the leg
     * @param joint_angles Current joint angles
     * @return Limit proximity (1.0 = furthest from limit, 0.0 = at limit)
     */
    double calculateLimitProximity(int leg_index, const JointAngles &joint_angles) const;

    /**
     * @brief Get base position for a specific leg
     */
    Point3D getLegBase(int leg_index) const;

    /**
     * @brief Calculate distance from leg base to target
     */
    double getDistanceFromBase(int leg_index, const Point3D &target) const;

    /**
     * @brief Check if position respects joint limits (requires IK)
     */
    bool checkJointLimits(int leg_index, const Point3D &target) const;

    /**
     * @brief Constrain position to geometric workspace bounds only
     */
    Point3D constrainToGeometricWorkspace(int leg_index, const Point3D &target) const;

    // ========================================================================
    // STATIC UTILITY METHODS
    // ========================================================================

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
    // ========================================================================
    // PRIVATE ANALYSIS METHODS
    // ========================================================================

    void calculateLegWorkspaceBounds(int leg_index);
    void generateWalkspaceForLeg(int leg_index);
    bool detailedReachabilityCheck(int leg_index, const Point3D &position);
    Point3D calculateCenterOfMass(const Point3D leg_positions[NUM_LEGS]);
    double calculateStabilityMargin(const Point3D leg_positions[NUM_LEGS]);
    void calculateSupportPolygon(const Point3D leg_positions[NUM_LEGS],
                                 std::vector<Point3D> &polygon);

    // Precision-dependent implementations
    bool simpleStepOptimization(const Point3D &movement,
                                const Point3D current[NUM_LEGS],
                                Point3D optimal[NUM_LEGS]);
    bool balancedStepOptimization(const Point3D &movement,
                                  const Point3D current[NUM_LEGS],
                                  Point3D optimal[NUM_LEGS]);
    bool advancedStepOptimization(const Point3D &movement,
                                  const Point3D current[NUM_LEGS],
                                  Point3D optimal[NUM_LEGS]);

    // Analysis tracking methods
    void updateAnalysisInfo(const WalkspaceResult &result, unsigned long analysis_time_ms);
    double calculateLegReachability(int leg_index, const Point3D leg_positions[NUM_LEGS]) const;
    double calculateOverallStabilityScore(const WalkspaceResult &result) const;
};

#endif // WORKSPACE_ANALYZER_H
