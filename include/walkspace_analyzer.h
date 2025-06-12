#ifndef WALKSPACE_ANALYZER_H
#define WALKSPACE_ANALYZER_H

#include "HexaModel.h"
#include "precision_config.h"
#include <map>
#include <vector>

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
        float stability_margin;               ///< Stability margin (mm)
        std::vector<Point3D> support_polygon; ///< Support polygon vertices
        std::map<int, float> walkspace_radii; ///< Walkspace radii by bearing
        bool is_stable;                       ///< Overall stability flag
        float reachable_area;                 ///< Total reachable area (mm²)
    };

    /**
     * @brief Individual leg workspace bounds
     */
    struct WorkspaceBounds {
        float min_radius; ///< Minimum reachable radius (mm)
        float max_radius; ///< Maximum reachable radius (mm)
        float min_height; ///< Minimum reachable height (mm)
        float max_height; ///< Maximum reachable height (mm)
        float min_angle;  ///< Minimum angular range (degrees)
        float max_angle;  ///< Maximum angular range (degrees)
    };

  private:
    RobotModel &model_;
    ComputeConfig config_;

    // Workspace geometry
    WorkspaceBounds leg_workspace_[NUM_LEGS];
    std::map<int, float> walkspace_map_; // Bearing -> radius mapping

    // Analysis parameters
    static constexpr int BEARING_STEP = 5;                // Degrees
    static constexpr float MAX_WORKSPACE_RADIUS = 500.0f; // mm
    static constexpr float STABILITY_THRESHOLD = 10.0f;   // mm

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
    float getWalkspaceRadius(float bearing_degrees) const;

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
    const std::map<int, float> &getWalkspaceMap() const { return walkspace_map_; }

  private:
    void calculateLegWorkspaceBounds(int leg_index);
    void generateWalkspaceForLeg(int leg_index);
    bool detailedReachabilityCheck(int leg_index, const Point3D &position);
    Point3D calculateCenterOfMass(const Point3D leg_positions[NUM_LEGS]);
    float calculateStabilityMargin(const Point3D leg_positions[NUM_LEGS]);
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
};

#endif // WALKSPACE_ANALYZER_H
