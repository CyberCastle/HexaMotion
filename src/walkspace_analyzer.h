#ifndef WALKSPACE_ANALYZER_H
#define WALKSPACE_ANALYZER_H

#include "robot_model.h"
#include "precision_config.h"
#include <map>
#include <vector>

// OpenSHC-compatible type definitions (must be before class declaration)
typedef std::map<int, double> Workplane;       // bearing -> radius
typedef std::map<double, Workplane> Workspace; // height -> Workplane

/**
 * @brief Walkspace analysis system equivalent to OpenSHC implementation
 *
 * Provides configurable precision vs computational complexity for:
 * - Leg workspace boundary calculation
 * - Walking stability analysis
 * - Reachable area computation
 * - Step position optimization
 */
class WalkspaceAnalyzer {
  public:
    /**
     * @brief Walkspace analysis result structure
     */
    struct WalkspaceResult {
        Point3D center_of_mass;               ///< Calculated center of mass
        double stability_margin;               ///< Stability margin (mm)
        std::vector<Point3D> support_polygon; ///< Support polygon vertices
        std::map<int, double> walkspace_radii; ///< Walkspace radii by bearing
        bool is_stable;                       ///< Overall stability flag
        double reachable_area;                 ///< Total reachable area (mm²)
    };

    /**
     * @brief Individual leg workspace bounds
     */
    struct WorkspaceBounds {
        double min_radius; ///< Minimum reachable radius (mm)
        double max_radius; ///< Maximum reachable radius (mm)
        double min_height; ///< Minimum reachable height (mm)
        double max_height; ///< Maximum reachable height (mm)
        double min_angle;  ///< Minimum angular range (degrees)
        double max_angle;  ///< Maximum angular range (degrees)
    };

    /**
     * @brief Comprehensive analysis information for external systems
     */
    struct AnalysisInfo {
        WalkspaceResult current_result;        ///< Current walkspace analysis result
        std::map<int, WorkspaceBounds> leg_bounds; ///< Workspace bounds for each leg
        std::map<int, double> leg_reachability;    ///< Reachability scores (0-1) for each leg
        double overall_stability_score;            ///< Overall stability score (0-1)
        bool analysis_enabled;                     ///< Whether analysis is currently enabled
        unsigned long last_analysis_time;          ///< Timestamp of last analysis
        int analysis_count;                        ///< Number of analyses performed
        double average_analysis_time_ms;           ///< Average analysis time in milliseconds
        double total_analysis_time_ms;             ///< Total time spent in analysis
        double min_analysis_time_ms;               ///< Minimum analysis time
        double max_analysis_time_ms;               ///< Maximum analysis time
        std::map<int, double> walkspace_radii;     ///< Current walkspace radii by bearing
        bool walkspace_map_generated;              ///< Whether walkspace map has been generated
    };

  private:
    RobotModel &model_;
    ComputeConfig config_;

    // Workspace geometry
    WorkspaceBounds leg_workspace_[NUM_LEGS];
    std::map<int, double> walkspace_map_; // Bearing -> radius mapping (legacy)

    // Enhanced workspace storage with height layers (OpenSHC-compatible)
    Workspace leg_workspaces_[NUM_LEGS]; // 3D workspace per leg

    // Analysis parameters
    static constexpr int BEARING_STEP = 5;                // Degrees
    static constexpr double MAX_WORKSPACE_RADIUS = 500.0f; // mm
    static constexpr double STABILITY_THRESHOLD = 10.0f;   // mm

    // Runtime control flags
    bool analysis_enabled_;                    ///< Flag to enable/disable real-time analysis
    AnalysisInfo analysis_info_;               ///< Comprehensive analysis information
    unsigned long last_analysis_timestamp_;    ///< Timestamp of last analysis
    double total_analysis_time_;               ///< Total time spent in analysis

  public:
    explicit WalkspaceAnalyzer(RobotModel &model, ComputeConfig config = ComputeConfig::medium());

    /**
     * @brief Initialize walkspace analyzer with robot geometry
     */
    void initialize();

    /**
     * @brief Generate walkspace map for current robot configuration
     * Equivalent to OpenSHC's generateWalkspace() function
     */
    void generateWalkspace();

    /**
     * @brief Check if position is reachable by specific leg
     * @param leg_index Leg index (0-5)
     * @param position Target position in robot frame
     * @return True if position is reachable
     */
    bool isPositionReachable(int leg_index, const Point3D &position);

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

    /**
     * @brief Set computational precision level
     * @param config New configuration
     */
    void setPrecisionConfig(const ComputeConfig &config) { config_ = config; }

    /**
     * @brief Get current walkspace map
     * @return Map of bearing to radius values
     */
    const std::map<int, double> &getWalkspaceMap() const { return walkspace_map_; }

    /**
     * @brief Get workplane at specific height with interpolation
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
     * @brief Check if position is reachable using workplane interpolation
     * @param leg_index Leg index (0-5)
     * @param position Target position in robot frame
     * @return True if position is reachable according to workplane data
     */
    bool isPositionReachableWithWorkplane(int leg_index, const Point3D &position) const;

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
    const AnalysisInfo& getAnalysisInfo() const { return analysis_info_; }

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
     * @brief Get current walkspace map for external use
     * @return Map of bearing to radius values
     */
    const std::map<int, double>& getCurrentWalkspaceMap() const { return walkspace_map_; }

    /**
     * @brief Check if walkspace map has been generated
     * @return True if walkspace map is available
     */
    bool isWalkspaceMapGenerated() const { return analysis_info_.walkspace_map_generated; }

  private:
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

#endif // WALKSPACE_ANALYZER_H
