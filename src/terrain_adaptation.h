#ifndef TERRAIN_ADAPTATION_H
#define TERRAIN_ADAPTATION_H

#include "HexaModel.h"
#include <ArduinoEigen.h>
#include <memory>
#include <vector>

// Forward declaration to avoid circular dependency
class WorkspaceValidator;

/**
 * @brief Terrain adaptation system for hexapod locomotion
 *
 * This system implements dynamic terrain adaptation equivalent to OpenSHC's
 * rough terrain handling system, including:
 * - Walk plane estimation using least squares fitting
 * - External target pose handling for proactive/reactive terrain adaptation
 * - Step surface detection and adjustment
 * - Rough terrain mode with touchdown detection
 * - Dynamic walkspace generation based on terrain geometry
 */
class TerrainAdaptation {
  public:
    /**
     * @brief External target structure for terrain-aware stepping
     */
    struct ExternalTarget {
        Point3D position;        ///< Target tip position
        double swing_clearance;   ///< Height clearance during swing
        std::string frame_id;    ///< Reference frame ID
        unsigned long timestamp; ///< Request timestamp
        bool defined;            ///< Whether target is valid

        ExternalTarget() : position(0, 0, 0), swing_clearance(0),
                           frame_id(""), timestamp(0), defined(false) {}
    };

    /**
     * @brief Step plane detection structure
     */
    struct StepPlane {
        Point3D position; ///< Step surface position
        Point3D normal;   ///< Step surface normal vector
        bool valid;       ///< Whether detection is valid
        double confidence; ///< Detection confidence (0-1)

        StepPlane() : position(0, 0, 0), normal(0, 0, 1), valid(false), confidence(0) {}
    };

    /**
     * @brief Walk plane estimation structure
     */
    struct WalkPlane {
        Eigen::Vector3d coeffs; ///< Plane coefficients [a,b,c] for ax+by+c=z
        Eigen::Vector3d normal; ///< Plane normal vector
        bool valid;             ///< Whether estimation is valid
        double confidence;       ///< Estimation confidence (0-1)

        WalkPlane() : coeffs(0, 0, 0), normal(0, 0, 1), valid(false), confidence(0) {}
    };

  private:
    RobotModel &model_;
    std::unique_ptr<WorkspaceValidator> workspace_validator_; // Workspace validation
    bool rough_terrain_mode_;
    bool force_normal_touchdown_;
    bool gravity_aligned_tips_;

    // Terrain detection parameters
    double touchdown_threshold_; ///< FSR threshold for touchdown detection
    double liftoff_threshold_;   ///< FSR threshold for liftoff detection
    double step_depth_;          ///< Depth to probe for reactive terrain detection

    // Walk plane estimation
    WalkPlane current_walk_plane_;
    std::vector<Point3D> foot_contact_history_;
    static const size_t MAX_CONTACT_HISTORY = 20;

    // Per-leg terrain data
    ExternalTarget external_targets_[NUM_LEGS];
    ExternalTarget external_defaults_[NUM_LEGS];
    StepPlane step_planes_[NUM_LEGS];
    bool touchdown_detection_[NUM_LEGS];

    // IMU integration for gravity estimation
    Eigen::Vector3d gravity_estimate_;

  public:
    explicit TerrainAdaptation(RobotModel &model);
    ~TerrainAdaptation(); // Needed for unique_ptr with forward declaration

    /**
     * @brief Initialize terrain adaptation system
     */
    void initialize();

    /**
     * @brief Update terrain adaptation for current cycle
     * @param fsr_interface FSR sensor interface
     * @param imu_interface IMU sensor interface
     */
    void update(IFSRInterface *fsr_interface, IIMUInterface *imu_interface);

    /**
     * @brief Enable/disable rough terrain mode
     * @param enabled Whether to enable rough terrain adaptation
     */
    void setRoughTerrainMode(bool enabled) { rough_terrain_mode_ = enabled; }

    /**
     * @brief Enable/disable force normal touchdown
     * @param enabled Whether to force normal touchdown to walk plane
     */
    void setForceNormalTouchdown(bool enabled) { force_normal_touchdown_ = enabled; }

    /**
     * @brief Enable/disable gravity-aligned tips
     * @param enabled Whether tips should align with gravity
     */
    void setGravityAlignedTips(bool enabled) { gravity_aligned_tips_ = enabled; }

    /**
     * @brief Set FSR touchdown threshold
     * @param threshold FSR threshold for touchdown detection (N or ADC units)
     */
    void setTouchdownThreshold(double threshold) { touchdown_threshold_ = threshold; }

    /**
     * @brief Set FSR liftoff threshold
     * @param threshold FSR threshold for liftoff detection (N or ADC units)
     */
    void setLiftoffThreshold(double threshold) { liftoff_threshold_ = threshold; }

    /**
     * @brief Get current touchdown threshold
     * @return Touchdown threshold value
     */
    double getTouchdownThreshold() const { return touchdown_threshold_; }

    /**
     * @brief Get current liftoff threshold
     * @return Liftoff threshold value
     */
    double getLiftoffThreshold() const { return liftoff_threshold_; }

    /**
     * @brief Update FSR thresholds from model parameters
     * Call this method when model parameters change to sync threshold values
     */
    void updateThresholdsFromModel();

    /**
     * @brief Set external target for specific leg
     * @param leg_index Leg index (0-5)
     * @param target External target specification
     */
    void setExternalTarget(int leg_index, const ExternalTarget &target);

    /**
     * @brief Set external default position for specific leg
     * @param leg_index Leg index (0-5)
     * @param default_pos External default specification
     */
    void setExternalDefault(int leg_index, const ExternalTarget &default_pos);

    /**
     * @brief Get current walk plane estimation
     * @return Current walk plane
     */
    const WalkPlane &getWalkPlane() const { return current_walk_plane_; }

    /**
     * @brief Get external target for specific leg
     * @param leg_index Leg index (0-5)
     * @return External target if defined
     */
    const ExternalTarget &getExternalTarget(int leg_index) const;

    /**
     * @brief Get step plane detection for specific leg
     * @param leg_index Leg index (0-5)
     * @return Step plane if detected
     */
    const StepPlane &getStepPlane(int leg_index) const;

    /**
     * @brief Check if leg has touchdown detection enabled
     * @param leg_index Leg index (0-5)
     * @return True if touchdown detection is active
     */
    bool hasTouchdownDetection(int leg_index) const;

    /**
     * @brief Estimate gravity vector from IMU data
     * @return Estimated gravity vector
     */
    Eigen::Vector3d estimateGravity() const { return gravity_estimate_; }

    /**
     * @brief Get current gravity vector estimate
     * @return Gravity vector estimate
     */
    Eigen::Vector3d getGravityVector() const { return gravity_estimate_; }

    /**
     * @brief Adapt foot trajectory for terrain-aware stepping
     * @param leg_index Leg index (0-5)
     * @param trajectory Base trajectory from gait controller
     * @param leg_state Current leg state
     * @param swing_progress Swing phase progress (0-1)
     * @return Terrain-adapted trajectory
     */
    Point3D adaptTrajectoryForTerrain(int leg_index, const Point3D &trajectory,
                                     StepPhase leg_state, double swing_progress);

    /**
     * @brief Check if target position is reachable within terrain constraints
     * @param leg_index Leg index (0-5)
     * @param target Target position to check
     * @return True if reachable considering terrain
     */
    bool isTargetReachableOnTerrain(int leg_index, const Point3D &target);

  private:
    /**
     * @brief Update walk plane estimation using least squares fitting
     */
    void updateWalkPlaneEstimation();

    /**
     * @brief Detect touchdown/liftoff events using FSR data
     * @param leg_index Leg index (0-5)
     * @param fsr_data FSR sensor data
     */
    void detectTouchdownEvents(int leg_index, const FSRData &fsr_data);

    /**
     * @brief Update step plane detection for specific leg
     * @param leg_index Leg index (0-5)
     * @param fsr_data FSR sensor data
     */
    void updateStepPlaneDetection(int leg_index, const FSRData &fsr_data);

    /**
     * @brief Update gravity estimation from IMU
     * @param imu_data IMU sensor data
     */
    void updateGravityEstimation(const IMUData &imu_data);

    /**
     * @brief Enhanced terrain analysis using absolute positioning data
     * @param imu_data IMU sensor data with absolute positioning capability
     */
    void updateAdvancedTerrainAnalysis(const IMUData &imu_data);

    /**
     * @brief Apply proactive terrain adaptation using step plane data
     * @param leg_index Leg index (0-5)
     * @param base_trajectory Base trajectory
     * @return Proactively adapted trajectory
     */
    Point3D applyProactiveAdaptation(int leg_index, const Point3D &base_trajectory);

    /**
     * @brief Apply reactive terrain adaptation using step depth probing
     * @param leg_index Leg index (0-5)
     * @param base_trajectory Base trajectory
     * @return Reactively adapted trajectory
     */
    Point3D applyReactiveAdaptation(int leg_index, const Point3D &base_trajectory);

    /**
     * @brief Force trajectory to touchdown normal to walk plane
     * @param leg_index Leg index (0-5)
     * @param trajectory Current trajectory
     * @return Normal touchdown trajectory
     */
    Point3D forceNormalTouchdown(int leg_index, const Point3D &trajectory);

    /**
     * @brief Calculate projection of point onto walk plane
     * @param point Input point
     * @return Projected point on walk plane
     */
    Point3D projectOntoWalkPlane(const Point3D &point);

    /**
     * @brief Check if sufficient foot contacts exist for plane estimation
     * @return True if plane estimation is possible
     */
    bool hasValidFootContactData() const;
};

#endif // TERRAIN_ADAPTATION_H
