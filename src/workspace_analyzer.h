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
 * @brief OpenSHC-style workspace generation and reachability system
 *
 * Equivalent to OpenSHC implementation with generateWorkspace() and getWorkplane() methods.
 */
class WorkspaceAnalyzer {
  private:
    const RobotModel &model_;
    ComputeConfig config_;

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
     */
    explicit WorkspaceAnalyzer(const RobotModel &model,
                               ComputeConfig config = ComputeConfig::medium());

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

    // ========================================================================
    // UTILITY AND TESTING METHODS
    // ========================================================================

    /**
     * @brief Get current walkspace map
     * @return Map of bearing to radius values
     */
    const std::map<int, double> &getWalkspaceMap() const { return walkspace_map_; }

  private:
    // ========================================================================
    // PRIVATE ANALYSIS METHODS
    // ========================================================================

    void generateWalkspaceForLeg(int leg_index);
    bool detailedReachabilityCheck(int leg_index,
                                   const Point3D &position,
                                   const JointAngles &initial_guess,
                                   JointAngles *solution = nullptr);
};

#endif // WORKSPACE_ANALYZER_H
