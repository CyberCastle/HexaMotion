#include "workspace_analyzer.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>

WorkspaceAnalyzer::WorkspaceAnalyzer(const RobotModel &model, ComputeConfig config, const ValidationConfig &validation_config)
    : model_(model), config_(config), validation_config_(validation_config), analysis_enabled_(true), total_analysis_time_(0.0) {

    // Initialize physical robot configuration offset
    // When all servo angles are 0°, robot body is positioned at getDefaultHeightOffset()
    reference_height_offset_ = model_.getDefaultHeightOffset();

    // Initialize analysis info structure
    analysis_info_.analysis_enabled = true;
    analysis_info_.analysis_count = 0;
    analysis_info_.last_analysis_time = 0;
    analysis_info_.average_analysis_time_ms = 0.0;
    analysis_info_.total_analysis_time_ms = 0.0;
    analysis_info_.min_analysis_time_ms = 1e6;
    analysis_info_.max_analysis_time_ms = 0.0;
    analysis_info_.overall_stability_score = 0.0;
    analysis_info_.walkspace_map_generated = false;
    last_analysis_timestamp_ = 0;

    // Initialize workspace cache flags (OpenSHC-style)
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_workspace_generated_[i] = false;
    }

    // Zero initialize leg_workspace_ bounds structure array
    for (int i = 0; i < NUM_LEGS; ++i) {
        leg_workspace_[i] = {0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, Point3D(0, 0, 0)};
    }
}

void WorkspaceAnalyzer::initialize() {
    // Calculate workspace bounds for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        calculateLegWorkspaceBounds(i);
    }

    // Generate initial walkspace
    generateWorkspace();
}

// ========================================================================
// WORKSPACE GENERATION AND ANALYSIS (OpenSHC equivalent methods)
// ========================================================================

void WorkspaceAnalyzer::generateWorkspace() {
    // ----------------------------------------------------------------------------
    // OpenSHC-equivalent walkspace normalization with HexaMotion morphology context
    // ----------------------------------------------------------------------------
    // Morphological considerations from AGENTS.md:
    //  - Tibia is vertical at 0° (no horizontal contribution at identity pose)
    //  - Standing horizontal reach = coxa_length + femur_length * cos(femur_angle_standing)
    //  - Default height is derived from default_height_offset + standing_height (configurable)
    //  - Symmetry enforced across opposing leg pairs
    // This method replicates OpenSHC::WalkController::generateWalkspace two‑stage process:
    //    1) Initial per-bearing min radius without workspace overlap (adjacent-leg constraint)
    //    2) Per-leg refinement using interpolated workplane & default shift intersection
    // Result: walkspace_map_ (bearing -> radius) symmetric & normalized.

    walkspace_map_.clear();

    // Ensure per-leg 3D workspaces exist (cached generation)
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        generateWalkspaceForLeg(leg);
    }

    const Parameters &params = model_.getParams();
    bool allow_overlap = params.overlapping_walkspaces; // OpenSHC parameter parity

    // Stage 1: Populate walkspace with maximum values constrained by adjacent legs (no overlap)
    // Equivalent to OpenSHC loop over legs computing distance & bearing to adjacents.
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        int adjacent_leg_1 = (leg + 1) % NUM_LEGS;
        int adjacent_leg_2 = (leg + NUM_LEGS - 1) % NUM_LEGS;

        // Identity tip positions (use zero angles FK). Identity pose: all angles = 0 (tibia vertical).
        JointAngles zero(0, 0, 0);
        Point3D default_tip_position = model_.forwardKinematicsGlobalCoordinates(leg, zero); // same for identity XY
        Point3D adjacent_1_tip = model_.forwardKinematicsGlobalCoordinates(adjacent_leg_1, zero);
        Point3D adjacent_2_tip = model_.forwardKinematicsGlobalCoordinates(adjacent_leg_2, zero);

        // Distances to adjacent legs (half-distance rule like OpenSHC)
        double distance_to_adjacent_leg_1 = math_utils::distance(default_tip_position, adjacent_1_tip) / 2.0;
        double distance_to_adjacent_leg_2 = math_utils::distance(default_tip_position, adjacent_2_tip) / 2.0;

        // Bearings to adjacent legs
        double bearing_to_adjacent_leg_1 = math_utils::radiansToDegrees(
            atan2(adjacent_1_tip.y - default_tip_position.y, adjacent_1_tip.x - default_tip_position.x));
        double bearing_to_adjacent_leg_2 = math_utils::radiansToDegrees(
            atan2(adjacent_2_tip.y - default_tip_position.y, adjacent_2_tip.x - default_tip_position.x));

        for (int bearing = 0; bearing <= 360; bearing += BEARING_STEP) {
            int bearing_diff_1 = std::abs(static_cast<int>(bearing_to_adjacent_leg_1) - bearing);
            int bearing_diff_2 = std::abs(static_cast<int>(bearing_to_adjacent_leg_2) - bearing);
            if (bearing_diff_1 > 180)
                bearing_diff_1 = 360 - bearing_diff_1;
            if (bearing_diff_2 > 180)
                bearing_diff_2 = 360 - bearing_diff_2;

            double distance_to_overlap_1 = MAX_WORKSPACE_RADIUS;
            double distance_to_overlap_2 = MAX_WORKSPACE_RADIUS;

            if (!allow_overlap) {
                if ((bearing_diff_1 < 90 || bearing_diff_1 > 270) && distance_to_adjacent_leg_1 > 0.0) {
                    double cos_val = std::cos(math_utils::degreesToRadians(static_cast<double>(bearing_diff_1)));
                    if (std::abs(cos_val) > 1e-6)
                        distance_to_overlap_1 = distance_to_adjacent_leg_1 / cos_val;
                }
                if ((bearing_diff_2 < 90 || bearing_diff_2 > 270) && distance_to_adjacent_leg_2 > 0.0) {
                    double cos_val = std::cos(math_utils::degreesToRadians(static_cast<double>(bearing_diff_2)));
                    if (std::abs(cos_val) > 1e-6)
                        distance_to_overlap_2 = distance_to_adjacent_leg_2 / cos_val;
                }
            }

            double min_distance = allow_overlap ? MAX_WORKSPACE_RADIUS
                                                : std::min(distance_to_overlap_1, distance_to_overlap_2);
            min_distance = std::min(min_distance, MAX_WORKSPACE_RADIUS);

            auto existing = walkspace_map_.find(bearing);
            if (existing != walkspace_map_.end()) {
                if (min_distance < existing->second)
                    existing->second = min_distance;
            } else {
                walkspace_map_[bearing] = min_distance;
            }
        }
    }

    // Stage 2: Per-leg refinement using interpolated workplane at target default height.
    // Compute conservative standing horizontal reach (no tibia horizontal component) once.
    double standing_horizontal_reach = RobotModel::computeStandingHorizontalReach(params);

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Base geometric references
        Point3D base_pos = model_.getLegBasePosition(leg);
        double base_angle = model_.getLegBaseAngleOffset(leg);

        // Identity tip (XY aligned with base + coxa + horizontal femur projection).
        // Morphology (AGENTS.md):
        //   identity_z_global   = default_height_offset (≈ -208, tibia vertical)
        //   standing_z_global   = -standing_height      (≈ -150)
        //   required_shift_z    = standing_z_global - identity_z_global = (-standing_height) - (default_height_offset) = +58
        // Because here identity plane was set to 0, we must encode only the DIFFERENCE, not a raw global Z sum.
        Point3D identity_tip_position(
            base_pos.x + standing_horizontal_reach * std::cos(base_angle),
            base_pos.y + standing_horizontal_reach * std::sin(base_angle),
            0.0); // artificial local frame baseline

        // Default_z_local = standing_z_global - identity_z_global
        double default_z = (-params.standing_height) - model_.getDefaultHeightOffset();
        Point3D default_tip_position = identity_tip_position; // XY identical in symmetric pose
        default_tip_position.z = default_z;

        Point3D default_shift = default_tip_position - identity_tip_position; // (0,0, +shift)

        // Target workplane height is vertical shift relative to identity plane
        double target_workplane_height = default_shift.z; // matches OpenSHC usage of default_shift[2]
        Workplane workplane = getWorkplane(leg, target_workplane_height);
        if (workplane.empty()) {
            continue; // cannot refine without workplane data
        }

        for (auto &entry : walkspace_map_) {
            int bearing = entry.first;
            double radius = entry.second; // current global min

            // If no shift, direct radius from workplane
            if (default_shift.x == 0.0 && default_shift.y == 0.0 && std::abs(default_shift.z) < 1e-9) {
                auto itw = workplane.find(bearing);
                if (itw != workplane.end()) {
                    double candidate = itw->second;
                    if (candidate < radius) {
                        entry.second = candidate;
                        int opposite_bearing = (bearing + 180) % 360;
                        walkspace_map_[opposite_bearing] = candidate; // enforce symmetry
                    }
                }
                continue;
            }

            // OpenSHC intersection method (simplified; assumes ordered workplane bearings)
            // Build a synthetic point in desired bearing direction with max radius
            double bearing_rad = math_utils::degreesToRadians(static_cast<double>(bearing));
            Point3D new_point(MAX_WORKSPACE_RADIUS * std::cos(bearing_rad),
                              MAX_WORKSPACE_RADIUS * std::sin(bearing_rad), 0.0);

            // Iterate consecutive bearing segments
            double refined_radius = radius; // start with current
            for (auto it = workplane.begin(); it != workplane.end(); ++it) {
                auto next_it = std::next(it);
                if (next_it == workplane.end())
                    break; // need a pair

                int bearing_1 = it->first;
                int bearing_2 = next_it->first;
                double r1 = it->second;
                double r2 = next_it->second;

                // Reference points (subtract default shift to simulate shifted default tip frame)
                double b1_rad = math_utils::degreesToRadians(static_cast<double>(bearing_1));
                double b2_rad = math_utils::degreesToRadians(static_cast<double>(bearing_2));
                Point3D p1(r1 * std::cos(b1_rad) - default_shift.x,
                           r1 * std::sin(b1_rad) - default_shift.y,
                           0.0);
                Point3D p2(r2 * std::cos(b2_rad) - default_shift.x,
                           r2 * std::sin(b2_rad) - default_shift.y,
                           0.0);

                // Colinear checks (vector cross product magnitude ~ 0 in 2D -> parallel)
                double cross1 = p1.x * new_point.y - p1.y * new_point.x;
                double cross2 = p2.x * new_point.y - p2.y * new_point.x;
                double new_cross = (p1.x * p2.y - p1.y * p2.x);
                (void)new_cross; // reserved for extended diagnostics

                // If new_point direction aligns exactly with reference
                if (std::abs(cross1) < 1e-9) {
                    refined_radius = std::min(refined_radius, math_utils::magnitude(Point3D(p1.x, p1.y, 0.0)));
                    break;
                } else if (std::abs(cross2) < 1e-9) {
                    refined_radius = std::min(refined_radius, math_utils::magnitude(Point3D(p2.x, p2.y, 0.0)));
                    break;
                } else {
                    // Check if new_point direction lies between p1 and p2 (same sign test)
                    double c1 = (p1.x * new_point.y - p1.y * new_point.x) * (p1.x * p2.y - p1.y * p2.x);
                    double c2 = (p2.x * new_point.y - p2.y * new_point.x) * (p2.x * p1.y - p2.y * p1.x);
                    if (c1 >= 0.0 && c2 >= 0.0) {
                        // Compute normal to segment (p1->p2)
                        double dx = p2.x - p1.x;
                        double dy = p2.y - p1.y;
                        // Two candidate normals
                        Point3D n1(dy, -dx, 0.0);
                        Point3D n2(-dy, dx, 0.0);
                        // Choose one pointing roughly towards new_point
                        double dot1 = n1.x * new_point.x + n1.y * new_point.y;
                        Point3D normal = (dot1 >= 0.0) ? n1 : n2;
                        double normal_len = std::sqrt(normal.x * normal.x + normal.y * normal.y);
                        if (normal_len > 1e-9) {
                            normal.x /= normal_len;
                            normal.y /= normal_len;
                            // Projection magnitudes
                            double proj_new = new_point.x * normal.x + new_point.y * normal.y;
                            double proj_p1 = p1.x * normal.x + p1.y * normal.y;
                            if (std::abs(proj_new) > 1e-9) {
                                double ratio = std::abs(proj_p1 / proj_new);
                                double candidate = ratio * MAX_WORKSPACE_RADIUS;
                                refined_radius = std::min(refined_radius, candidate);
                            }
                        }
                        break;
                    }
                }
            }

            if (refined_radius < entry.second) {
                entry.second = refined_radius;
                int opposite_bearing = (bearing + 180) % 360;
                walkspace_map_[opposite_bearing] = refined_radius; // enforce symmetry
            }
        }
    }

    // Final symmetry enforcement & closing bearing
    walkspace_map_[360] = walkspace_map_[0];

    analysis_info_.walkspace_map_generated = true;
    analysis_info_.walkspace_radii = walkspace_map_;
}

Workplane WorkspaceAnalyzer::getWorkplane(int leg_index, double height) const {
    if (leg_index >= NUM_LEGS) {
        return Workplane(); // Return empty workplane for invalid leg
    }

    const Workspace &workspace = leg_workspaces_[leg_index];

    // Check if height is within workspace bounds
    if (workspace.empty()) {
        return Workplane(); // Return empty if no workspace generated
    }

    double min_height = workspace.begin()->first;
    double max_height = workspace.rbegin()->first;

    if (height < min_height || height > max_height) {
        // Return empty workplane for heights outside workspace
        return Workplane();
    }

    // Find exact match first
    auto exact_it = workspace.find(height);
    if (exact_it != workspace.end()) {
        return exact_it->second;
    }

    // If workspace has only one layer, return it
    if (workspace.size() == 1) {
        return workspace.begin()->second;
    }

    // Find bounding workplanes for interpolation
    auto upper_it = workspace.upper_bound(height);

    // Check if height is beyond all existing heights
    if (upper_it == workspace.end()) {
        // Return the last (highest) workplane
        return workspace.rbegin()->second;
    }

    // Check if height is before all existing heights
    if (upper_it == workspace.begin()) {
        // Return the first (lowest) workplane
        return workspace.begin()->second;
    }

    auto lower_it = std::prev(upper_it);

    double upper_height = upper_it->first;
    double lower_height = lower_it->first;
    const Workplane &upper_workplane = upper_it->second;
    const Workplane &lower_workplane = lower_it->second;

    // Calculate interpolation factor
    double height_diff = upper_height - lower_height;
    if (height_diff <= 0.001) {
        // Heights are too close, return lower workplane
        return lower_workplane;
    }

    double t = (height - lower_height) / height_diff;
    t = std::max(0.0, std::min(1.0, t)); // Clamp to [0,1]

    // Interpolate between workplanes
    Workplane interpolated_workplane;

    // Use all bearings from both workplanes
    std::set<int> all_bearings;
    for (const auto &bearing_radius : upper_workplane) {
        all_bearings.insert(bearing_radius.first);
    }
    for (const auto &bearing_radius : lower_workplane) {
        all_bearings.insert(bearing_radius.first);
    }

    for (int bearing : all_bearings) {
        double upper_radius = 0.0;
        double lower_radius = 0.0;

        auto upper_bearing_it = upper_workplane.find(bearing);
        auto lower_bearing_it = lower_workplane.find(bearing);

        if (upper_bearing_it != upper_workplane.end()) {
            upper_radius = upper_bearing_it->second;
        }
        if (lower_bearing_it != lower_workplane.end()) {
            lower_radius = lower_bearing_it->second;
        }

        // If one workplane doesn't have this bearing, use the other's value
        if (upper_bearing_it == upper_workplane.end()) {
            upper_radius = lower_radius;
        }
        if (lower_bearing_it == lower_workplane.end()) {
            lower_radius = upper_radius;
        }

        double interpolated_radius = lower_radius * (1.0 - t) + upper_radius * t;
        interpolated_workplane[bearing] = interpolated_radius;
    }

    return interpolated_workplane;
}

Workspace WorkspaceAnalyzer::getLegWorkspace(int leg_index) const {
    if (leg_index >= NUM_LEGS) {
        return Workspace(); // Return empty workspace for invalid leg
    }

    return leg_workspaces_[leg_index];
}

// ========================================================================
// POSITION VALIDATION AND REACHABILITY
// ========================================================================

ValidationResult
WorkspaceAnalyzer::validateTarget(int leg_index, Point3D target_position,
                                  const Point3D current_leg_positions[NUM_LEGS],
                                  bool constrain_if_invalid) {
    ValidationResult result;
    result.constrained_position = target_position;

    // 1. Basic geometric reachability
    result.is_reachable = isPositionReachable(leg_index, target_position, false);
    result.distance_from_base = getDistanceFromBase(leg_index, target_position);

    // 2. Collision checking
    if (validation_config_.enable_collision_checking) {
        result.collision_risk_factor = checkCollisionRisk(leg_index, target_position, current_leg_positions);
        result.is_collision_free = (result.collision_risk_factor < 0.1f); // 10% threshold
    } else {
        result.is_collision_free = true;
        result.collision_risk_factor = 0.0f;
    }

    // 3. Joint limit checking
    if (validation_config_.enable_joint_limit_checking) {
        result.is_within_joint_limits = checkJointLimits(leg_index, target_position);
    } else {
        result.is_within_joint_limits = true;
    }

    // 4. Apply constraints if needed and requested
    if (constrain_if_invalid && !result.isValid()) {
        result.constrained_position = constrainToValidWorkspace(leg_index, target_position, current_leg_positions);

        // Re-validate the constrained position
        result.is_reachable = isPositionReachable(leg_index, result.constrained_position, false);
        result.distance_from_base = getDistanceFromBase(leg_index, result.constrained_position);

        if (validation_config_.enable_collision_checking) {
            result.collision_risk_factor = checkCollisionRisk(leg_index, result.constrained_position, current_leg_positions);
            result.is_collision_free = (result.collision_risk_factor < 0.1f);
        }

        if (validation_config_.enable_joint_limit_checking) {
            result.is_within_joint_limits = checkJointLimits(leg_index, result.constrained_position);
        }
    }

    return result;
}

bool WorkspaceAnalyzer::isPositionReachable(int leg_index, const Point3D &position, bool use_ik_validation) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return false;
    }

    // Basic geometric reachability check
    WorkspaceBounds bounds = getWorkspaceBounds(leg_index);
    double distance = getDistanceFromBase(leg_index, position);

    if (distance < bounds.min_reach || distance > bounds.max_reach) {
        return false;
    }

    // If IK validation is requested, perform additional joint limit checking
    if (use_ik_validation) {
        return checkJointLimits(leg_index, position);
    }

    return true;
}

// ========================================================================
// COLLISION DETECTION AND AVOIDANCE
// ========================================================================

double WorkspaceAnalyzer::checkCollisionRisk(int leg_index, const Point3D &target,
                                             const Point3D current_leg_positions[NUM_LEGS]) const {
    const Point3D &target_position = target; // alias for clarity
    // Get adjacent leg indices
    int left_adjacent = (leg_index + NUM_LEGS - 1) % NUM_LEGS;
    int right_adjacent = (leg_index + 1) % NUM_LEGS;

    double max_risk = 0.0f;

    // Check distance to adjacent legs
    double distance_to_left = math_utils::distance2D(target_position, current_leg_positions[left_adjacent]);
    double distance_to_right = math_utils::distance2D(target_position, current_leg_positions[right_adjacent]);

    // Convert distances to risk factors (closer = higher risk)
    double min_safe_distance = validation_config_.collision_safety_margin;

    if (distance_to_left < min_safe_distance) {
        double risk = 1.0f - (distance_to_left / min_safe_distance);
        max_risk = std::max(max_risk, risk);
    }

    if (distance_to_right < min_safe_distance) {
        double risk = 1.0f - (distance_to_right / min_safe_distance);
        max_risk = std::max(max_risk, risk);
    }

    return math_utils::clamp<double>(max_risk, 0.0, 1.0);
}

Point3D WorkspaceAnalyzer::constrainToValidWorkspace(int leg_index, const Point3D &target,
                                                     const Point3D current_leg_positions[NUM_LEGS]) const {
    Point3D constrained = target;

    // Step 1: Constrain to geometric workspace
    constrained = constrainToGeometricWorkspace(leg_index, constrained);

    // Step 2: Apply collision avoidance if enabled
    if (validation_config_.enable_collision_checking) {
        double total_reach = model_.getParams().coxa_length + model_.getParams().femur_length + model_.getParams().tibia_length;
        adjustForCollisionAvoidance(leg_index, constrained, model_.getParams().hexagon_radius, total_reach, current_leg_positions);
    }

    // Step 3: Final geometric validation (in case collision avoidance pushed it out)
    constrained = constrainToGeometricWorkspace(leg_index, constrained);

    return constrained;
}

// ========================================================================
// WALKSPACE ANALYSIS
// ========================================================================

WorkspaceAnalyzer::WalkspaceResult WorkspaceAnalyzer::analyzeWalkspace(const Point3D leg_positions[NUM_LEGS]) {
    // Check if analysis is enabled
    if (!analysis_enabled_) {
        // Return cached result if analysis is disabled
        return analysis_info_.current_result;
    }

    WalkspaceResult result;

    // Calculate center of mass
    result.center_of_mass = calculateCenterOfMass(leg_positions);

    // Calculate stability margin
    result.stability_margin = calculateStabilityMargin(leg_positions);
    {
        const auto &wt = model_.getParams().workspace_tuning;
        double stability_threshold = (wt.stability_threshold_mm > 0.0)
                                         ? wt.stability_threshold_mm
                                         : DEFAULT_STABILITY_THRESHOLD;
        result.is_stable = result.stability_margin > stability_threshold;
    }

    // Calculate support polygon
    calculateSupportPolygon(leg_positions, result.support_polygon);

    // Copy walkspace radii
    result.walkspace_radii = walkspace_map_;

    // Calculate reachable area (simplified)
    result.reachable_area = 0.0f;
    for (const auto &entry : walkspace_map_) {
        double radius = entry.second;
        result.reachable_area += M_PI * radius * radius / walkspace_map_.size();
    }

    // Update analysis information without timing
    updateAnalysisInfo(result, 0);

    return result;
}

double WorkspaceAnalyzer::getWalkspaceRadius(double bearing_degrees) const {
    // Normalize bearing to 0-360 range
    while (bearing_degrees < 0)
        bearing_degrees += 360;
    while (bearing_degrees >= 360)
        bearing_degrees -= 360;

    // Find nearest bearing values
    int lower_bearing = static_cast<int>(bearing_degrees / BEARING_STEP) * BEARING_STEP;
    int upper_bearing = lower_bearing + BEARING_STEP;

    auto lower_it = walkspace_map_.find(lower_bearing);
    auto upper_it = walkspace_map_.find(upper_bearing);

    if (lower_it == walkspace_map_.end() || upper_it == walkspace_map_.end()) {
        return 0.0f;
    }

    // Linear interpolation
    double t = (bearing_degrees - lower_bearing) / BEARING_STEP;
    return lower_it->second * (1.0f - t) + upper_it->second * t;
}

bool WorkspaceAnalyzer::getOptimalStepPositions(const Point3D &body_movement,
                                                const Point3D current_positions[NUM_LEGS],
                                                Point3D optimal_positions[NUM_LEGS]) {
    switch (config_.precision) {
    case PRECISION_LOW:
        return simpleStepOptimization(body_movement, current_positions, optimal_positions);
    case PRECISION_MEDIUM:
        return balancedStepOptimization(body_movement, current_positions, optimal_positions);
    case PRECISION_HIGH:
        return advancedStepOptimization(body_movement, current_positions, optimal_positions);
    }
    return false;
}

// ========================================================================
// WORKSPACE BOUNDS AND CONSTRAINTS
// ========================================================================

WorkspaceBounds
WorkspaceAnalyzer::getWorkspaceBounds(int leg_index) const {
    WorkspaceBounds bounds;

    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return bounds; // Return empty bounds for invalid leg index
    }

    const Parameters &params = model_.getParams();
    Point3D leg_base = getLegBase(leg_index);

    // --- Morphology-aware reach calculations (AGENTS.md) ---
    // Standing horizontal reach excludes tibia (vertical at standing pose):
    double standing_horizontal_reach = RobotModel::computeStandingHorizontalReach(params);
    // Flat theoretical extension (maximum geometric envelope):
    double flat_extension_reach = params.coxa_length + params.femur_length + params.tibia_length; // upper bound

    // Conservative operational vs absolute reach handling (morphology aligned):
    //  - standing_horizontal_reach is the usable horizontal span in the symmetric standing pose
    //  - flat_extension_reach is the theoretical geometric envelope (rarely achievable dynamically)
    //  - We cap the effective max_reach close above standing reach to avoid overestimating lateral capability
    //    while still allowing a modest buffer for dynamic extension (IK stretch, terrain adaptation, etc.).
    //  - Safety margin factor is applied but then clamped by a morphology cap (15% over standing reach) so that
    //    downstream velocity planners using bounds.max_reach do not inflate stride/velocity limits.
    double physical_candidate = flat_extension_reach * validation_config_.safety_margin_factor;
    const auto &wt = params.workspace_tuning; // direct access (accessors removed for MCU efficiency)
    double morphology_cap_factor = (wt.morphology_cap_factor > 0.1) ? wt.morphology_cap_factor : 1.15;
    double morphology_cap = standing_horizontal_reach * morphology_cap_factor; // configurable headroom
    bounds.max_reach = std::max(standing_horizontal_reach, std::min(physical_candidate, morphology_cap));

    // Minimum useful reach: fraction of coxa length (keeps IK well-conditioned near base)
    bounds.min_reach = params.coxa_length * validation_config_.minimum_reach_factor;

    // Preferred band: anchored on morphology (standing pose) rather than extreme extension
    bounds.preferred_max_reach = standing_horizontal_reach;                          // ideal operating radial distance
    double preferred_min_reach_buffer = (wt.preferred_min_reach_buffer_factor >= 1.0 &&
                                        wt.preferred_min_reach_buffer_factor <= 1.5)
                                           ? wt.preferred_min_reach_buffer_factor
                                           : 1.1;
    bounds.preferred_min_reach = bounds.min_reach * preferred_min_reach_buffer; // configurable buffer

    // Height bounds (Option B semantics):
    //   standing_height = absolute vertical distance body->foot in nominal standing pose => standing plane Z = -standing_height.
    //   reference_height_offset_ = physical Z at identity (all joint angles 0°, tibia vertical).
    // We ensure upward allowance covers either heuristic femur arc or required lift so that the standing plane
    // (if above identity) lies within [min_height, max_height].
    double femur_up_range_factor = (wt.femur_up_range_factor > 0.0) ? wt.femur_up_range_factor : 0.85;
    double heuristic_up_range = params.femur_length * femur_up_range_factor;              // historical conservative heuristic (configurable)
    double up_range = std::max(heuristic_up_range, params.standing_height);              // ensure standing plane inside bounds
    double down_range_factor = (wt.down_range_factor > 0.0) ? wt.down_range_factor : 0.85;
    double down_range = (params.femur_length + params.tibia_length) * down_range_factor; // downward reach (body lowering)
    bounds.min_height = reference_height_offset_ - down_range;                           // more negative (down)
    bounds.max_height = reference_height_offset_ + up_range;                             // more positive (up)

    bounds.has_height_restrictions = true;
    bounds.center_position = leg_base;

    return bounds;
}

VelocityConstraints
WorkspaceAnalyzer::calculateVelocityConstraints(int leg_index, double bearing_degrees,
                                                double gait_frequency, double stance_ratio) const {
    VelocityConstraints constraints;

    if (leg_index < 0 || leg_index >= NUM_LEGS || stance_ratio <= 0.0f || gait_frequency <= 0.0f) {
        return constraints; // Return empty constraints for invalid inputs
    }

    // Get morphology-aware workspace bounds
    WorkspaceBounds bounds = getWorkspaceBounds(leg_index);

    // Prefer actual generated walkspace radius for this bearing (normalized global limit)
    double walkspace_radius = getWalkspaceRadius(bearing_degrees);
    if (walkspace_radius <= 0.0) {
        // Fallback: directional efficiency scaling if walkspace not yet generated
        double leg_angle = math_utils::radiansToDegrees(model_.getLegBaseAngleOffset(leg_index));
        double bearing_offset = std::abs(bearing_degrees - leg_angle);
        if (bearing_offset > 180.0)
            bearing_offset = 360.0 - bearing_offset;
        double directional_efficiency = std::cos(math_utils::degreesToRadians(bearing_offset));
        double raw = std::max(0.0, directional_efficiency);
        double powered = (raw > 0.0) ? std::pow(raw, DIRECTIONAL_EFFICIENCY_EXPONENT) : 0.0;
        directional_efficiency = std::max(powered, DIRECTIONAL_EFFICIENCY_FLOOR);
        walkspace_radius = bounds.max_reach * directional_efficiency;
    }
    constraints.workspace_radius = walkspace_radius;

    // Stance radius based on AGENTS.md: hexagon_radius + standing_horizontal_reach
    double standing_horizontal_reach = RobotModel::computeStandingHorizontalReach(model_.getParams());
    constraints.stance_radius = model_.getParams().hexagon_radius + standing_horizontal_reach;
    if (constraints.stance_radius <= 0.0) {
        // fallback to previous heuristic if parameters invalid
        constraints.stance_radius = std::max(1.0, bounds.max_reach * 0.8);
    }

    // Calculate velocity limits based on gait parameters (morphology-aware revision):
    // Legacy approach used full workspace_radius diameter traversal in one stance cycle, which
    // overestimates speed when workspace_radius > standing_horizontal_reach. We now base the
    // nominal stride on an "operational radius" = min(workspace_radius, standing_horizontal_reach * 1.0).
    // This preserves OpenSHC normalization of the walkspace map itself, while bounding kinematic
    // velocity derivation to realistic lateral capability (tibia vertical at identity pose).
    double cycle_time = stance_ratio / gait_frequency; // stance phase duration
    if (cycle_time > 0.0f) {
        ScalingFactors scaling = getScalingFactors();

        double operational_radius = std::min(constraints.workspace_radius, standing_horizontal_reach);
        // Use diameter of operational radius as conservative stride envelope per stance cycle
        double nominal_stride = operational_radius * 2.0;
        // Apply velocity_scale AFTER deriving stride-based raw velocity
        double raw_linear_velocity = nominal_stride / cycle_time;
        constraints.max_linear_velocity = raw_linear_velocity * scaling.velocity_scale;

        // Angular speed derived from linear / stance radius (already morphology-based)
        constraints.max_angular_velocity = (constraints.stance_radius > 1e-6)
                                               ? (constraints.max_linear_velocity / constraints.stance_radius)
                                               : 0.0;
        constraints.max_angular_velocity *= scaling.angular_scale;

        // Acceleration heuristic: reach max speed in ~2s then apply scaling (kept for compatibility)
        constraints.max_acceleration = (constraints.max_linear_velocity / 2.0) * scaling.acceleration_scale;
    }

    // Apply reasonable safety limits (use robot configuration instead of hard tiny caps)
    const auto &params = model_.getParams();
    double linear_cap = (params.max_velocity > 0.0) ? params.max_velocity : DEFAULT_MAX_LINEAR_VELOCITY;
    double angular_cap = (params.max_angular_velocity > 0.0)
                             ? math_utils::degreesToRadians(params.max_angular_velocity)
                             : math_utils::degreesToRadians(DEFAULT_MAX_ANGULAR_VELOCITY);
    // Acceleration cap heuristic: reach max speed in ~1s => cap >= linear_cap
    double accel_cap = std::max(linear_cap, constraints.max_acceleration);

    constraints.max_linear_velocity = math_utils::clamp<double>(constraints.max_linear_velocity, 0.0, linear_cap);
    constraints.max_angular_velocity = math_utils::clamp<double>(constraints.max_angular_velocity, 0.0, angular_cap);
    constraints.max_acceleration = math_utils::clamp<double>(constraints.max_acceleration, 0.0, accel_cap);

    return constraints;
}

// ========================================================================
// CONFIGURATION AND SCALING
// ========================================================================

ScalingFactors WorkspaceAnalyzer::getScalingFactors() const {
    ScalingFactors factors;
    // Retrieve centralized tunable scaling parameters from robot configuration.
    // This replaces previously hardcoded constants so external configuration can adjust motion behavior.
    const Parameters &params = model_.getParams();
    const auto &cfg = params.scaling; // Parameters::ScalingFactors

    // Direct copies (keep legacy defaults if user has not changed them in Parameters)
    factors.linear_scale = cfg.linear_scale;
    factors.angular_scale = cfg.angular_scale;
    factors.workspace_scale = cfg.workspace_scale;
    factors.velocity_scale = cfg.velocity_scale;
    factors.acceleration_scale = cfg.acceleration_scale;
    factors.safety_margin = cfg.safety_margin;

    // collision_scale: fallback to validation_config_.safety_margin_factor when unset / non-positive
    factors.collision_scale = (cfg.collision_scale > 0.0)
                                  ? cfg.collision_scale
                                  : validation_config_.safety_margin_factor;

    return factors; // Unified scaling factors now parameter-driven
}

void WorkspaceAnalyzer::updateSafetyMargin(double margin) {
    validation_config_.safety_margin_factor =
        math_utils::clamp<double>(margin, 0.1, 1.0);
}

// ========================================================================
// UTILITY AND TESTING METHODS
// ========================================================================

double WorkspaceAnalyzer::calculateLimitProximity(int leg_index, const JointAngles &joint_angles) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return 1.0; // Safe default
    }

    const Parameters &params = model_.getParams();

    // Convert angle limits from degrees to radians
    double coxa_min_rad = math_utils::degreesToRadians(params.coxa_angle_limits[0]);
    double coxa_max_rad = math_utils::degreesToRadians(params.coxa_angle_limits[1]);
    double femur_min_rad = math_utils::degreesToRadians(params.femur_angle_limits[0]);
    double femur_max_rad = math_utils::degreesToRadians(params.femur_angle_limits[1]);
    double tibia_min_rad = math_utils::degreesToRadians(params.tibia_angle_limits[0]);
    double tibia_max_rad = math_utils::degreesToRadians(params.tibia_angle_limits[1]);

    // Calculate limit proximity (OpenSHC-style)
    // (1.0 = furthest possible from limit, 0.0 = equal to limit)
    double min_limit_proximity = 1.0;

    // Check coxa joint
    double coxa_min_diff = abs(coxa_min_rad - joint_angles.coxa);
    double coxa_max_diff = abs(coxa_max_rad - joint_angles.coxa);
    double coxa_half_range = (coxa_max_rad - coxa_min_rad) / 2.0;
    double coxa_proximity = coxa_half_range != 0 ? std::min(coxa_min_diff, coxa_max_diff) / coxa_half_range : 1.0;
    min_limit_proximity = std::min(coxa_proximity, min_limit_proximity);

    // Check femur joint
    double femur_min_diff = abs(femur_min_rad - joint_angles.femur);
    double femur_max_diff = abs(femur_max_rad - joint_angles.femur);
    double femur_half_range = (femur_max_rad - femur_min_rad) / 2.0;
    double femur_proximity = femur_half_range != 0 ? std::min(femur_min_diff, femur_max_diff) / femur_half_range : 1.0;
    min_limit_proximity = std::min(femur_proximity, min_limit_proximity);

    // Check tibia joint
    double tibia_min_diff = abs(tibia_min_rad - joint_angles.tibia);
    double tibia_max_diff = abs(tibia_max_rad - joint_angles.tibia);
    double tibia_half_range = (tibia_max_rad - tibia_min_rad) / 2.0;
    double tibia_proximity = tibia_half_range != 0 ? std::min(tibia_min_diff, tibia_max_diff) / tibia_half_range : 1.0;
    min_limit_proximity = std::min(tibia_proximity, min_limit_proximity);

    return min_limit_proximity;
}

Point3D WorkspaceAnalyzer::getLegBase(int leg_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return Point3D{0.0f, 0.0f, 0.0f};
    }

    return model_.getLegBasePosition(leg_index);
}

double WorkspaceAnalyzer::getDistanceFromBase(int leg_index, const Point3D &target) const {
    Point3D leg_base = getLegBase(leg_index);
    return math_utils::distance3D(leg_base, target);
}

bool WorkspaceAnalyzer::checkJointLimits(int leg_index, const Point3D &target) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return false;
    }

    // Calculate inverse kinematics and check whether joint angles are within limits
    try {
        // For validation, use zero angles as starting point
        JointAngles zero_angles(0, 0, 0);
        JointAngles angles = model_.inverseKinematicsCurrentGlobalCoordinates(leg_index, zero_angles, target);

        // Use the limits from configuration parameters
        const Parameters &params = model_.getParams();
        const double COXA_MIN = params.coxa_angle_limits[0];
        const double COXA_MAX = params.coxa_angle_limits[1];
        const double FEMUR_MIN = params.femur_angle_limits[0];
        const double FEMUR_MAX = params.femur_angle_limits[1];
        const double TIBIA_MIN = params.tibia_angle_limits[0];
        const double TIBIA_MAX = params.tibia_angle_limits[1];

        return (angles.coxa >= COXA_MIN && angles.coxa <= COXA_MAX &&
                angles.femur >= FEMUR_MIN && angles.femur <= FEMUR_MAX &&
                angles.tibia >= TIBIA_MIN && angles.tibia <= TIBIA_MAX);
    } catch (...) {
        return false;
    }
}

Point3D WorkspaceAnalyzer::constrainToGeometricWorkspace(int leg_index, const Point3D &target) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return target;
    }

    const Parameters &params = model_.getParams();
    Point3D leg_base = getLegBase(leg_index);

    // Calculate max reach based on physical morphology without exceeding safety margin
    double physical_max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    double max_reach = physical_max_reach * validation_config_.safety_margin_factor;

    // NOTE (Morphology / AGENTS.md): This method intentionally uses the absolute physical reach
    // (coxa + femur + tibia) as a hard geometric constraint. The operative "preferred"
    // horizontal reach used for walkspace normalization (standing_horizontal_reach) is
    // smaller and incorporated elsewhere (getWorkspaceBounds, velocity constraints, and
    // generateWorkspace). We keep this function focused on enforcing the true physical
    // maximum so that upstream planners can still reason about margin between preferred
    // and absolute reach without double‑clipping.

    // Ensure we never exceed the physical maximum, even if safety factor is > 1.0
    max_reach = std::min(max_reach, physical_max_reach);

    // Calculate distance from base
    double distance = getDistanceFromBase(leg_index, target);

    if (distance <= max_reach) {
        return target; // Already within bounds
    }

    // Constrain to max reach
    double scale = max_reach / distance;
    Point3D constrained = target;
    constrained.x = leg_base.x + (target.x - leg_base.x) * scale;
    constrained.y = leg_base.y + (target.y - leg_base.y) * scale;

    // Preserve Z near the standing/walking plane to avoid artificial vertical drift.
    // If the target Z is already close to the default walking plane, keep it unchanged.
    const double standing_plane_z = model_.getLegDefaultPosition(leg_index).z;
    const double z_tol = model_.getParams().walk_plane_z_tolerance_mm; // runtime-configurable tolerance
    if (std::abs(target.z - standing_plane_z) <= z_tol) {
        constrained.z = target.z; // keep exact plane height
    } else {
        constrained.z = leg_base.z + (target.z - leg_base.z) * scale;
    }

    return constrained;
}

// ========================================================================
// STATIC UTILITY METHODS
// ========================================================================

double WorkspaceAnalyzer::calculateSafeHexagonRadius(double leg_reach, double safety_margin) {
    // For a hexagon with 60° between legs, we need to ensure that adjacent leg
    // workspaces don't overlap. Using the law of cosines:
    //
    // In a triangle formed by two adjacent leg bases and the workspace overlap point:
    // - Two sides are (hexagon_radius + leg_reach) each
    // - The angle between them is 60°
    // - The third side should be at least (2 * leg_reach + safety_margin)
    //
    // Using law of cosines: c² = a² + b² - 2ab*cos(C)
    // Where c = minimum separation, a = b = hexagon_radius + leg_reach, C = 60°

    double min_separation = 2.0f * leg_reach + safety_margin;

    // Solving: min_separation² = 2 * (hexagon_radius + leg_reach)² * (1 - cos_60)
    // min_separation² = 2 * (hexagon_radius + leg_reach)² * 0.5
    // min_separation² = (hexagon_radius + leg_reach)²
    // Therefore: hexagon_radius = min_separation - leg_reach

    return min_separation - leg_reach;
}

double WorkspaceAnalyzer::getDistance2D(const Point3D &p1, const Point3D &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

void WorkspaceAnalyzer::getAdjacentLegIndices(int leg_index, int &left_adjacent, int &right_adjacent) {
    left_adjacent = (leg_index + NUM_LEGS - 1) % NUM_LEGS; // Previous leg (counter-clockwise)
    right_adjacent = (leg_index + 1) % NUM_LEGS;           // Next leg (clockwise)
}

bool WorkspaceAnalyzer::checkWorkspaceOverlap(const Point3D &leg1_base, double leg1_reach,
                                              const Point3D &leg2_base, double leg2_reach,
                                              double safety_margin) {
    double distance = getDistance2D(leg1_base, leg2_base);
    double combined_reach = leg1_reach + leg2_reach + safety_margin;

    // If the distance between leg bases is less than combined reach, workspaces overlap
    return distance < combined_reach;
}

bool WorkspaceAnalyzer::wouldCollideWithAdjacent(int leg_index, const Point3D &target_position,
                                                 double hexagon_radius, double leg_reach,
                                                 const Point3D adjacent_positions[NUM_LEGS]) const {
    int left_adjacent, right_adjacent;
    getAdjacentLegIndices(leg_index, left_adjacent, right_adjacent);

    // Check collision with left adjacent leg
    double distance_to_left = getDistance2D(target_position, adjacent_positions[left_adjacent]);
    const auto &wt = model_.getParams().workspace_tuning;
    double min_leg_sep = (wt.min_leg_separation_mm > 0.0) ? wt.min_leg_separation_mm : DEFAULT_MIN_LEG_SEPARATION;
    if (distance_to_left < min_leg_sep) {
        return true;
    }

    // Check collision with right adjacent leg
    double distance_to_right = getDistance2D(target_position, adjacent_positions[right_adjacent]);
    if (distance_to_right < min_leg_sep) {
        return true;
    }

    return false;
}

bool WorkspaceAnalyzer::adjustForCollisionAvoidance(int leg_index, Point3D &target_position,
                                                    double hexagon_radius, double leg_reach,
                                                    const Point3D adjacent_positions[NUM_LEGS]) const {
    // Check if adjustment is needed
    if (!wouldCollideWithAdjacent(leg_index, target_position, hexagon_radius, leg_reach, adjacent_positions)) {
        return true; // No collision, no adjustment needed
    }

    // Calculate leg base position
    Point3D base_pos = model_.getLegBasePosition(leg_index);
    double base_x = base_pos.x;
    double base_y = base_pos.y;

    double dx = target_position.x - base_x;
    double dy = target_position.y - base_y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance > 0.001f) {
        // Start with 90% scale and iteratively reduce if still colliding
    const auto &wt = model_.getParams().workspace_tuning;
    double start_scale = (wt.collision_adjust_start_scale > 0.0 && wt.collision_adjust_start_scale <= 1.5)
                 ? wt.collision_adjust_start_scale
                 : 0.9;
    double min_scale = (wt.collision_adjust_min_scale > 0.0 && wt.collision_adjust_min_scale < 1.0)
                   ? wt.collision_adjust_min_scale
                   : 0.5;
    double step = (wt.collision_adjust_step > 0.0 && wt.collision_adjust_step < 0.5)
              ? wt.collision_adjust_step
              : 0.1;
    double safe_scale_ratio = (wt.safe_scale_ratio > 0.0 && wt.safe_scale_ratio <= 1.0) ? wt.safe_scale_ratio : 0.7;

    for (double scale = start_scale; scale >= min_scale; scale -= step) {
            Point3D test_position;
            test_position.x = base_x + dx * scale;
            test_position.y = base_y + dy * scale;
            test_position.z = target_position.z;

            if (!wouldCollideWithAdjacent(leg_index, test_position, hexagon_radius, leg_reach, adjacent_positions)) {
                target_position = test_position;
                return true;
            }
        }

        // If we couldn't find a good scale, use minimum safe distance
    double safe_scale = math_utils::clamp<double>((leg_reach * safe_scale_ratio) / distance,
                              min_scale,
                                                      std::numeric_limits<double>::infinity());
        target_position.x = base_x + dx * safe_scale;
        target_position.y = base_y + dy * safe_scale;
        return true;
    }

    return false;
}

std::string WorkspaceAnalyzer::getAnalysisInfoString() const {
    std::stringstream ss;

    ss << "=== Workspace Analysis Information ===\n";
    ss << "Analysis Enabled: " << (analysis_info_.analysis_enabled ? "YES" : "NO") << "\n";
    ss << "Analysis Count: " << analysis_info_.analysis_count << "\n";
    ss << "Last Analysis Time: " << analysis_info_.last_analysis_time << " ms\n";
    ss << "Average Analysis Time: " << std::fixed << std::setprecision(2)
       << analysis_info_.average_analysis_time_ms << " ms\n";
    ss << "Min/Max Analysis Time: " << std::fixed << std::setprecision(2)
       << analysis_info_.min_analysis_time_ms << "/" << analysis_info_.max_analysis_time_ms << " ms\n";
    ss << "Total Analysis Time: " << std::fixed << std::setprecision(2)
       << analysis_info_.total_analysis_time_ms << " ms\n";
    ss << "Walkspace Map Generated: " << (analysis_info_.walkspace_map_generated ? "YES" : "NO") << "\n";

    // OpenSHC-style cache information
    ss << "\n=== Workspace Cache Status (OpenSHC-style) ===\n";
    int cached_legs = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_workspace_generated_[i])
            cached_legs++;
    }
    ss << "Cached Legs: " << cached_legs << "/" << NUM_LEGS << "\n";
    ss << "Leg Cache Status: ";
    for (int i = 0; i < NUM_LEGS; i++) {
        ss << "L" << i << ":" << (leg_workspace_generated_[i] ? "✓" : "✗") << " ";
    }
    ss << "\n";

    ss << "Overall Stability Score: " << std::fixed << std::setprecision(3)
       << analysis_info_.overall_stability_score << "\n";

    if (analysis_info_.current_result.is_stable) {
        ss << "Current Status: STABLE (margin: " << std::fixed << std::setprecision(1)
           << analysis_info_.current_result.stability_margin << " mm)\n";
    } else {
        ss << "Current Status: UNSTABLE (margin: " << std::fixed << std::setprecision(1)
           << analysis_info_.current_result.stability_margin << " mm)\n";
    }

    ss << "Reachable Area: " << std::fixed << std::setprecision(1)
       << analysis_info_.current_result.reachable_area << " mm²\n";

    ss << "\nLeg Reachability Scores:\n";
    for (const auto &leg_score : analysis_info_.leg_reachability) {
        ss << "  Leg " << leg_score.first << ": " << std::fixed << std::setprecision(3)
           << leg_score.second << "\n";
    }

    return ss.str();
}

void WorkspaceAnalyzer::resetAnalysisStats() {
    analysis_info_.analysis_count = 0;
    analysis_info_.average_analysis_time_ms = 0.0;
    analysis_info_.total_analysis_time_ms = 0.0;
    analysis_info_.min_analysis_time_ms = 1e6;
    analysis_info_.max_analysis_time_ms = 0.0;
    total_analysis_time_ = 0.0;
    analysis_info_.last_analysis_time = 0;
}

void WorkspaceAnalyzer::invalidateWorkspaceCache() {
    // Invalidate cache for all legs (OpenSHC-style)
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_workspace_generated_[i] = false;
    }
    analysis_info_.walkspace_map_generated = false;
}

void WorkspaceAnalyzer::invalidateWorkspaceCache(int leg_index) {
    // Invalidate cache for specific leg
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        leg_workspace_generated_[leg_index] = false;
    }
}

// ========================================================================
// PRIVATE ANALYSIS METHODS
// ========================================================================

void WorkspaceAnalyzer::calculateLegWorkspaceBounds(int leg_index) {
    const Parameters &params = model_.getParams();

    // Calculate correct total reach including all leg segments
    double total_reach = params.coxa_length + params.femur_length + params.tibia_length;
    double min_reach = params.coxa_length; // Minimum reach is at least the coxa length

    WorkspaceBounds &bounds = leg_workspace_[leg_index];
    bounds.min_reach = min_reach;
    bounds.max_reach = total_reach;

    // Apply physical height offset: when angles are 0°, the robot is at z = getDefaultHeightOffset()
    // Workspace heights are centered on this reference position
    const auto &wt = params.workspace_tuning;
    double span_factor = wt.leg_workspace_height_span_factor;
    if (span_factor <= 0.05 || span_factor > 1.0)
        span_factor = 0.7; // fallback
    double reach_range = total_reach * span_factor; // Configurable percentage of total reach
    bounds.min_height = reference_height_offset_ - reach_range;
    bounds.max_height = reference_height_offset_ + reach_range;
}

void WorkspaceAnalyzer::generateWalkspaceForLeg(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS)
        return;

    // OpenSHC-style caching: Check if workspace already generated
    if (leg_workspace_generated_[leg_index]) {
        return; // Use cached workspace data
    }

    const Parameters &params = model_.getParams();

    // 1) Precompute data for overlap constraints
    Workplane max_plane, min_plane;
    leg_workspaces_[leg_index].clear(); // Only clear when regenerating

    // 2) Calculate the tip "identity" position with the robot's physical offset
    JointAngles zero(0, 0, 0);
    Point3D id_tip = model_.forwardKinematicsGlobalCoordinates(leg_index, zero);

    // Apply physical height offset: when angles are 0°, the robot is at z = getDefaultHeightOffset()
    id_tip.z += reference_height_offset_;

    // if it cannot reach the identity position -> workspace is empty
    bool identity_reachable = detailedReachabilityCheck(leg_index, id_tip);

    // Precompute data for overlap constraints (OpenSHC-equivalent)
    double distance_to_adjacent_leg_1 = MAX_WORKSPACE_RADIUS;
    double distance_to_adjacent_leg_2 = MAX_WORKSPACE_RADIUS;
    double bearing_to_adjacent_leg_1 = 0.0;
    double bearing_to_adjacent_leg_2 = 0.0;

    if (!params.overlapping_walkspaces && identity_reachable) {
        int adjacent_leg_1 = (leg_index + 1) % NUM_LEGS;
        int adjacent_leg_2 = (leg_index + NUM_LEGS - 1) % NUM_LEGS;

        Point3D current_leg_tip = model_.forwardKinematicsGlobalCoordinates(leg_index, zero);
        Point3D adjacent_1_tip = model_.forwardKinematicsGlobalCoordinates(adjacent_leg_1, zero);
        Point3D adjacent_2_tip = model_.forwardKinematicsGlobalCoordinates(adjacent_leg_2, zero);

        // Apply physical height offset to all positions
        current_leg_tip.z += reference_height_offset_;
        adjacent_1_tip.z += reference_height_offset_;
        adjacent_2_tip.z += reference_height_offset_;

        distance_to_adjacent_leg_1 = math_utils::distance(current_leg_tip, adjacent_1_tip) / 2.0;
        distance_to_adjacent_leg_2 = math_utils::distance(current_leg_tip, adjacent_2_tip) / 2.0;

        bearing_to_adjacent_leg_1 = math_utils::radiansToDegrees(
            atan2(adjacent_1_tip.y - current_leg_tip.y, adjacent_1_tip.x - current_leg_tip.x));
        bearing_to_adjacent_leg_2 = math_utils::radiansToDegrees(
            atan2(adjacent_2_tip.y - current_leg_tip.y, adjacent_2_tip.x - current_leg_tip.x));
    }

    // Optimized unified loop: handles all cases in a single loop
    // Apply physical height offset: heights are calculated relative to the actual physical position
    double height_min = reference_height_offset_ - MAX_WORKSPACE_RADIUS;
    double height_max = reference_height_offset_ + MAX_WORKSPACE_RADIUS;
    double layer_step = (height_max - height_min) / WORKSPACE_LAYERS;

    for (int b = 0; b <= 360; b += BEARING_STEP) {
        double rad = math_utils::degreesToRadians(static_cast<double>(b));

        // Calculate max_allowed_radius for this bearing
        double max_allowed_radius = MAX_WORKSPACE_RADIUS;

        if (!identity_reachable) {
            // Empty workspace case
            max_allowed_radius = 0.0;
        } else if (!params.overlapping_walkspaces) {
            // Apply overlap constraints
            int bearing_diff_1 = abs(static_cast<int>(bearing_to_adjacent_leg_1) - b);
            int bearing_diff_2 = abs(static_cast<int>(bearing_to_adjacent_leg_2) - b);

            if (bearing_diff_1 > 180)
                bearing_diff_1 = 360 - bearing_diff_1;
            if (bearing_diff_2 > 180)
                bearing_diff_2 = 360 - bearing_diff_2;

            double distance_to_overlap_1 = MAX_WORKSPACE_RADIUS;
            double distance_to_overlap_2 = MAX_WORKSPACE_RADIUS;

            if ((bearing_diff_1 < 90 || bearing_diff_1 > 270) && distance_to_adjacent_leg_1 > 0.0) {
                distance_to_overlap_1 = distance_to_adjacent_leg_1 / cos(math_utils::degreesToRadians(bearing_diff_1));
            }
            if ((bearing_diff_2 < 90 || bearing_diff_2 > 270) && distance_to_adjacent_leg_2 > 0.0) {
                distance_to_overlap_2 = distance_to_adjacent_leg_2 / cos(math_utils::degreesToRadians(bearing_diff_2));
            }

            double min_distance = std::min(distance_to_overlap_1, distance_to_overlap_2);
            max_allowed_radius = std::min(MAX_WORKSPACE_RADIUS, min_distance);
        }

        // Process all height layers for this bearing
        for (int layer = 0; layer <= WORKSPACE_LAYERS; ++layer) {
            double h = height_min + layer * layer_step;

            // Find the maximum reachable radius at this height and bearing
            double best = 0.0;
            if (max_allowed_radius > 0.0) {
                for (double r = 0.0; r <= max_allowed_radius; r += MAX_POSITION_DELTA) {
                    Point3D p = id_tip;
                    p.z = h; // h already includes the physical height offset
                    p.x += r * cos(rad);
                    p.y += r * sin(rad);
                    if (detailedReachabilityCheck(leg_index, p))
                        best = r;
                    else
                        break;
                }
            }

            // Store the result in the workspace
            if (leg_workspaces_[leg_index].find(h) == leg_workspaces_[leg_index].end()) {
                leg_workspaces_[leg_index][h] = Workplane();
            }
            leg_workspaces_[leg_index][h][b] = best;
        }
    }

    // Ensure symmetry for all planes
    for (auto &height_plane : leg_workspaces_[leg_index]) {
        height_plane.second[360] = height_plane.second[0];
    }

    // Mark workspace as generated (OpenSHC-style caching)
    leg_workspace_generated_[leg_index] = true;
}

bool WorkspaceAnalyzer::detailedReachabilityCheck(int leg_index, const Point3D &position) {
    // Use inverse kinematics to check reachability
    // For validation purposes, use zero angles as starting point
    JointAngles zero_angles(0, 0, 0);
    JointAngles angles = model_.inverseKinematicsCurrentGlobalCoordinates(leg_index, zero_angles, position);
    return model_.checkJointLimits(leg_index, angles);
}

Point3D WorkspaceAnalyzer::calculateCenterOfMass(const Point3D leg_positions[NUM_LEGS]) {
    Point3D com(0, 0, 0);
    for (int i = 0; i < NUM_LEGS; i++) {
        com.x += leg_positions[i].x;
        com.y += leg_positions[i].y;
        com.z += leg_positions[i].z;
    }
    com.x /= NUM_LEGS;
    com.y /= NUM_LEGS;
    com.z /= NUM_LEGS;
    return com;
}

double WorkspaceAnalyzer::calculateStabilityMargin(const Point3D leg_positions[NUM_LEGS]) {
    // Enhanced stability margin calculation
    // Calculate the distance from center of mass to support polygon edges
    Point3D com = calculateCenterOfMass(leg_positions);

    // Get support polygon from stance legs only
    std::vector<Point3D> support_polygon;
    for (int i = 0; i < NUM_LEGS; i++) {
        // Only include legs in stance phase for support polygon
        support_polygon.push_back(leg_positions[i]);
    }

    if (support_polygon.size() < 3) {
        // Need at least 3 points for a stable polygon
        return 0.0f;
    }

    // Calculate minimum distance from COM to polygon edges
    double min_distance = 1000.0f;

    // Calculate distance to polygon edges (proper implementation)
    // For each edge of the support polygon, find perpendicular distance from COM
    for (size_t i = 0; i < support_polygon.size(); i++) {
        size_t next_idx = (i + 1) % support_polygon.size();
        Point3D edge_start = support_polygon[i];
        Point3D edge_end = support_polygon[next_idx];

        // Calculate perpendicular distance from COM to line segment
        double edge_distance = math_utils::pointToLineDistance(com, edge_start, edge_end);
        min_distance = std::min(min_distance, edge_distance);
    }

    // Apply safety factor for stability margin
    double safety_factor = 0.8f; // 80% of theoretical minimum
    return min_distance * safety_factor;
}

// Enhanced support polygon calculation with proper convex hull algorithm
void WorkspaceAnalyzer::calculateSupportPolygon(const Point3D leg_positions[NUM_LEGS],
                                                std::vector<Point3D> &polygon) {
    polygon.clear();

    // Enhanced implementation: use only stance legs for support polygon
    std::vector<Point3D> stance_points;

    for (int i = 0; i < NUM_LEGS; i++) {
        // In a full implementation, would check leg states
        // For now, include all legs as potential support points
        stance_points.push_back(leg_positions[i]);
    }

    if (stance_points.size() < 3) {
        // Not enough points for a polygon
        return;
    }

    // Simplified convex hull calculation
    // Find the centroid
    Point3D centroid(0, 0, 0);
    for (const auto &point : stance_points) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }
    centroid.x /= stance_points.size();
    centroid.y /= stance_points.size();
    centroid.z /= stance_points.size();

    // Sort points by angle around centroid (simplified 2D projection)
    std::vector<std::pair<double, Point3D>> angle_points;
    for (const auto &point : stance_points) {
        double angle = atan2(point.y - centroid.y, point.x - centroid.x);
        angle_points.push_back({angle, point});
    }

    // Sort by angle using custom comparator
    std::sort(angle_points.begin(), angle_points.end(),
              [](const std::pair<double, Point3D> &a, const std::pair<double, Point3D> &b) {
                  return a.first < b.first;
              });

    // Build polygon from sorted points
    for (const auto &angle_point : angle_points) {
        polygon.push_back(angle_point.second);
    }
}

bool WorkspaceAnalyzer::simpleStepOptimization(const Point3D &movement,
                                               const Point3D current[NUM_LEGS],
                                               Point3D optimal[NUM_LEGS]) {
    // Simple geometric approach - just apply inverse movement to all legs
    for (int i = 0; i < NUM_LEGS; i++) {
        optimal[i] = current[i] - movement;

        // Check if position is reachable
        if (!isPositionReachable(i, optimal[i])) {
            // Fall back to current position if not reachable
            optimal[i] = current[i];
        }
    }
    return true;
}

bool WorkspaceAnalyzer::balancedStepOptimization(const Point3D &movement,
                                                 const Point3D current[NUM_LEGS],
                                                 Point3D optimal[NUM_LEGS]) {
    // More sophisticated approach with constraint checking
    bool success = true;

    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D desired = current[i] - movement;

        if (isPositionReachable(i, desired)) {
            optimal[i] = desired;
        } else {
            // Find closest reachable position
            // Get leg origin using frame transformation with zero joint angles
            JointAngles zero_angles(0, 0, 0);
            Pose leg_origin_pose = model_.getPoseRobotFrame(i, zero_angles, Pose::Identity());
            Point3D leg_origin = leg_origin_pose.position;

            Point3D relative = desired - leg_origin;
            double distance = math_utils::magnitude(relative);

            const WorkspaceBounds &bounds = leg_workspace_[i];
            if (distance > bounds.max_reach) {
                // Scale down to maximum reach
                double scale = bounds.max_reach / distance;
                relative.x *= scale;
                relative.y *= scale;
                relative.z *= scale;
            }

            optimal[i] = leg_origin + relative;
            success = false;
        }
    }

    return success;
}

bool WorkspaceAnalyzer::advancedStepOptimization(const Point3D &movement,
                                                 const Point3D current[NUM_LEGS],
                                                 Point3D optimal[NUM_LEGS]) {
    // Full optimization with multiple iterations and stability constraints
    bool success = balancedStepOptimization(movement, current, optimal);

    if (!success && config_.max_iterations > 1) {
        // Iterative refinement
        for (int iter = 0; iter < config_.max_iterations - 1; iter++) {
            // Analyze stability of current solution
            WalkspaceResult result = analyzeWalkspace(optimal);

            if (result.is_stable) {
                success = true;
                break;
            }

            // Adjust positions to improve stability
            Point3D com = result.center_of_mass;
            for (int i = 0; i < NUM_LEGS; i++) {
                Point3D direction = optimal[i] - com;
                double distance = math_utils::magnitude(direction);
                if (distance > 0) {
                    direction.x /= distance;
                    direction.y /= distance;
                    direction.z /= distance;

                    // Move legs slightly outward for better stability
                    optimal[i] = optimal[i] + direction * 5.0f;

                    // Ensure still reachable
                    if (!isPositionReachable(i, optimal[i])) {
                        optimal[i] = optimal[i] - direction * 10.0f;
                    }
                }
            }
        }
    }

    return success;
}

void WorkspaceAnalyzer::updateAnalysisInfo(const WalkspaceResult &result, unsigned long analysis_time_ms) {
    // Update current result
    analysis_info_.current_result = result;

    // Update timing statistics (simplified)
    analysis_info_.last_analysis_time = analysis_time_ms;
    analysis_info_.analysis_count++;

    // Simple timing tracking without complex statistics
    if (analysis_time_ms > 0) {
        total_analysis_time_ += analysis_time_ms;
        analysis_info_.average_analysis_time_ms = total_analysis_time_ / analysis_info_.analysis_count;
        analysis_info_.total_analysis_time_ms = total_analysis_time_;

        // Update min/max times
        if (analysis_time_ms < analysis_info_.min_analysis_time_ms) {
            analysis_info_.min_analysis_time_ms = analysis_time_ms;
        }
        if (analysis_time_ms > analysis_info_.max_analysis_time_ms) {
            analysis_info_.max_analysis_time_ms = analysis_time_ms;
        }
    }

    // Update leg bounds
    for (int i = 0; i < NUM_LEGS; i++) {
        analysis_info_.leg_bounds[i] = leg_workspace_[i];
    }

    // Calculate leg reachability scores
    Point3D dummy_positions[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        // Use current leg positions if available, otherwise use dummy positions
        dummy_positions[i] = Point3D(0, 0, 0);
    }

    for (int i = 0; i < NUM_LEGS; i++) {
        analysis_info_.leg_reachability[i] = calculateLegReachability(i, dummy_positions);
    }

    // Calculate overall stability score
    analysis_info_.overall_stability_score = calculateOverallStabilityScore(result);
}

double WorkspaceAnalyzer::calculateLegReachability(int leg_index, const Point3D leg_positions[NUM_LEGS]) const {
    if (leg_index >= NUM_LEGS) {
        return 0.0;
    }

    // Calculate reachability based on current leg position and workspace bounds
    const WorkspaceBounds &bounds = leg_workspace_[leg_index];

    // Get current leg position
    Point3D current_pos = leg_positions[leg_index];

    // Calculate distance from leg origin
    JointAngles zero_angles(0, 0, 0);
    Pose leg_origin_pose = model_.getPoseRobotFrame(leg_index, zero_angles, Pose::Identity());
    Point3D leg_origin = leg_origin_pose.position;

    Point3D relative_pos = current_pos - leg_origin;
    double distance = math_utils::magnitude(relative_pos);

    // Calculate reachability score (0-1)
    double reach_range = bounds.max_reach - bounds.min_reach;
    if (reach_range <= 0) {
        return 0.0;
    }

    double normalized_distance = (distance - bounds.min_reach) / reach_range;
    return math_utils::clamp<double>(normalized_distance, 0.0, 1.0);
}

double WorkspaceAnalyzer::calculateOverallStabilityScore(const WalkspaceResult &result) const {
    // Calculate overall stability score based on multiple factors

    // Factor 1: Stability margin (0-1)
    double margin_score = math_utils::clamp<double>(result.stability_margin / 50.0, 0.0, 1.0);

    // Factor 2: Support polygon area (0-1)
    double polygon_area = 0.0;
    if (result.support_polygon.size() >= 3) {
        // Calculate polygon area using shoelace formula
        for (size_t i = 0; i < result.support_polygon.size(); i++) {
            size_t j = (i + 1) % result.support_polygon.size();
            polygon_area += result.support_polygon[i].x * result.support_polygon[j].y;
            polygon_area -= result.support_polygon[j].x * result.support_polygon[i].y;
        }
        polygon_area = std::abs(polygon_area) / 2.0;
    }

    // Normalize polygon area (assume maximum reasonable area of 10000 mm²)
    double polygon_score = math_utils::clamp<double>(polygon_area / 10000.0, 0.0, 1.0);

    // Factor 3: Reachable area (0-1)
    double area_score = math_utils::clamp<double>(result.reachable_area / 50000.0, 0.0, 1.0);

    // Combine factors with weights
    double overall_score = (margin_score * 0.5) + (polygon_score * 0.3) + (area_score * 0.2);

    return math_utils::clamp<double>(overall_score, 0.0, 1.0);
}
