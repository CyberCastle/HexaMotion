#include "workspace_analyzer.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <cmath>
#include <limits>
#include <set>

WorkspaceAnalyzer::WorkspaceAnalyzer(const RobotModel &model, ComputeConfig config, const ValidationConfig &validation_config)
    : model_(model), config_(config), validation_config_(validation_config), walkspace_map_generated_(false) {

    // Initialize physical robot configuration offset
    // When all servo angles are 0°, robot body is positioned at getDefaultHeightOffset()
    reference_height_offset_ = model_.getDefaultHeightOffset(); // Initialize reference height offset

    // Initialize workspace cache flags (OpenSHC-style)
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_workspace_generated_[i] = false;
    }

    // Initialize identity/default tips using standing neutral pose equivalent to
    // BodyPoseConfiguration defaults (same baseline used by WalkController LegStepper setup).
    // This keeps standalone WorkspaceAnalyzer generation numerically aligned with
    // WalkController::generateWalkspace() when setTipPositions() is not explicitly provided.
    CalculatedServoAngles standing_calc =
        RobotModel::calculateServoAnglesForHeight(model_.getParams().standing_height, model_.getParams());
    JointAngles neutral_angles = standing_calc.valid
                                     ? JointAngles(0.0, standing_calc.femur, standing_calc.tibia)
                                     : JointAngles(0.0, 0.0, 0.0);

    for (int i = 0; i < NUM_LEGS; i++) {
        identity_tip_positions_[i] = model_.forwardKinematicsGlobalCoordinates(i, neutral_angles);
        default_tip_positions_[i] = identity_tip_positions_[i];
    }
}

void WorkspaceAnalyzer::initialize() {
    // Generate per-leg 3D workspaces and global walkspace (OpenSHC equivalent)
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
    // OpenSHC uses getDefaultTipPose().position_ which initially equals identity_tip_pose_.
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        int adjacent_leg_1 = (leg + 1) % NUM_LEGS;
        int adjacent_leg_2 = (leg + NUM_LEGS - 1) % NUM_LEGS;

        // OpenSHC parity: use stored default tip positions (set via setTipPositions or
        // initialized to FK(zero) matching OpenSHC's initial default == identity state).
        Point3D default_tip_position = default_tip_positions_[leg];
        Point3D adjacent_1_tip = default_tip_positions_[adjacent_leg_1];
        Point3D adjacent_2_tip = default_tip_positions_[adjacent_leg_2];

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
    // OpenSHC equivalent: uses identity and default tip positions from LegStepper.
    // At initial startup default == identity, so default_shift == (0,0,0) and the simple
    // radius-from-workplane path is taken.  After WalkController updates positions via
    // setTipPositions(), the shift-based intersection path activates.

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // OpenSHC parity: identity and default come from stored tip positions.
        Point3D identity_tip_position = identity_tip_positions_[leg];
        Point3D default_tip_position = default_tip_positions_[leg];
        Point3D default_shift = default_tip_position - identity_tip_position;

        // Target workplane height is vertical shift relative to identity plane
        double target_workplane_height = default_shift.z; // matches OpenSHC usage of default_shift[2]
        Workplane workplane = getWorkplane(leg, target_workplane_height);
        if (workplane.empty()) {
            continue; // cannot refine without workplane data
        }

        for (auto &entry : walkspace_map_) {
            int bearing = entry.first;
            double radius = entry.second; // current global min

            // OpenSHC parity: if no shift, use workplane radius directly
            if (std::abs(default_shift.x) < 1e-9 && std::abs(default_shift.y) < 1e-9 && std::abs(default_shift.z) < 1e-9) {
                auto itw = workplane.find(bearing);
                if (itw != workplane.end()) {
                    double candidate = itw->second;
                    if (candidate < radius) {
                        entry.second = candidate;
                        int opposite_bearing = (bearing + 180) % 360;
                        walkspace_map_[opposite_bearing] = candidate;
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

    walkspace_map_generated_ = true;
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

Point3D WorkspaceAnalyzer::makeReachable(int leg_index, const Point3D &reference_tip_position) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return reference_tip_position;
    }

    JointAngles zero(0, 0, 0);
    Point3D identity_tip_position = model_.forwardKinematicsGlobalCoordinates(leg_index, zero);

    Point3D identity_to_test = reference_tip_position - identity_tip_position;
    double distance_to_test = std::hypot(identity_to_test.x, identity_to_test.y);

    Workplane workplane = getWorkplane(leg_index, identity_to_test.z);
    if (workplane.empty()) {
        return reference_tip_position;
    }

    double raw_bearing = atan2(identity_to_test.y, identity_to_test.x);
    int bearing = static_cast<int>(math_utils::radiansToDegrees(raw_bearing));
    while (bearing < 0)
        bearing += 360;
    while (bearing >= 360)
        bearing -= 360;

    auto upper_it = workplane.lower_bound(bearing);
    if (upper_it == workplane.end()) {
        upper_it = workplane.begin();
    }
    int upper_bound = upper_it->first;
    int lower_bound = (upper_bound - BEARING_STEP + 360) % 360;

    int adjusted_bearing = bearing;
    int adjusted_upper = upper_bound;
    if (adjusted_bearing < lower_bound)
        adjusted_bearing += 360;
    if (adjusted_upper < lower_bound)
        adjusted_upper += 360;

    double interpolation_progress = 0.0;
    double denominator = static_cast<double>(adjusted_upper - lower_bound);
    if (std::abs(denominator) > 1e-9) {
        interpolation_progress = static_cast<double>(adjusted_bearing - lower_bound) / denominator;
    }

    double lower_radius = workplane.at(lower_bound);
    double upper_radius = workplane.at(upper_bound % 360);
    double distance_to_limit = lower_radius * (1.0 - interpolation_progress) + upper_radius * interpolation_progress;

    if (distance_to_test <= distance_to_limit) {
        return reference_tip_position;
    }

    Point3D new_tip_position(distance_to_limit * std::cos(raw_bearing),
                             distance_to_limit * std::sin(raw_bearing),
                             identity_to_test.z);

    return identity_tip_position + new_tip_position;
}

void WorkspaceAnalyzer::setTipPositions(const Point3D identity_tips[NUM_LEGS],
                                        const Point3D default_tips[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; i++) {
        identity_tip_positions_[i] = identity_tips[i];
        default_tip_positions_[i] = default_tips[i];
    }
}

void WorkspaceAnalyzer::generateWalkspaceForLeg(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS)
        return;

    // OpenSHC-style caching: Check if workspace already generated
    if (leg_workspace_generated_[leg_index]) {
        return; // Use cached workspace data
    }

    // 1) Initialize workspace storage
    Workplane max_plane, min_plane;
    leg_workspaces_[leg_index].clear();

    // 2) Calculate the tip identity position (OpenSHC-style reference origin)
    JointAngles zero(0, 0, 0);
    Point3D id_tip = model_.forwardKinematicsGlobalCoordinates(leg_index, zero);

    // If it cannot reach the identity position -> workspace is empty
    bool identity_reachable = detailedReachabilityCheck(leg_index, id_tip);

    // OpenSHC parity: workspace layers are relative to the identity tip workplane (height 0).
    // Per-leg workspace is bounded only by IK reachability (no adjacent-leg overlap constraints).
    // Overlap constraints are applied later in generateWorkspace() (walkspace Phase 1).
    double height_min = -MAX_WORKSPACE_RADIUS;
    double height_max = MAX_WORKSPACE_RADIUS;
    double layer_step = (height_max - height_min) / WORKSPACE_LAYERS;

    for (int b = 0; b <= 360; b += BEARING_STEP) {
        double rad = math_utils::degreesToRadians(static_cast<double>(b));

        // OpenSHC parity: max radius is bounded only by MAX_WORKSPACE_RADIUS.
        // Adjacent-leg overlap constraints are applied in generateWorkspace(), not here.
        double max_allowed_radius = identity_reachable ? MAX_WORKSPACE_RADIUS : 0.0;

        // Process all height layers for this bearing
        for (int layer = 0; layer <= WORKSPACE_LAYERS; ++layer) {
            double h = height_min + layer * layer_step;

            // Find the maximum reachable radius at this height and bearing
            double best = 0.0;
            if (max_allowed_radius > 0.0) {
                for (double r = 0.0; r <= max_allowed_radius; r += MAX_POSITION_DELTA) {
                    Point3D p = id_tip;
                    p.z = id_tip.z + h;
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
    try {
        JointAngles zero_angles(0, 0, 0);
        JointAngles angles = model_.inverseKinematicsCurrentGlobalCoordinates(leg_index, zero_angles, position);
        return model_.checkJointLimits(leg_index, angles);
    } catch (...) {
        return false;
    }
}

bool WorkspaceAnalyzer::isPositionReachable(int leg_index, const Point3D &position, bool use_ik_validation) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return false;
    }

    if (use_ik_validation) {
        return detailedReachabilityCheck(leg_index, position);
    }

    JointAngles zero(0, 0, 0);
    Point3D identity_tip_position = model_.forwardKinematicsGlobalCoordinates(leg_index, zero);
    Point3D relative = position - identity_tip_position;
    double planar_distance = std::hypot(relative.x, relative.y);

    Workplane workplane = getWorkplane(leg_index, relative.z);
    if (workplane.empty()) {
        return false;
    }

    double raw_bearing = std::atan2(relative.y, relative.x);
    int bearing = static_cast<int>(math_utils::radiansToDegrees(raw_bearing));
    while (bearing < 0)
        bearing += 360;
    while (bearing >= 360)
        bearing -= 360;

    auto upper_it = workplane.lower_bound(bearing);
    if (upper_it == workplane.end()) {
        upper_it = workplane.begin();
    }

    int upper_bound = upper_it->first;
    int lower_bound = (upper_bound - BEARING_STEP + 360) % 360;

    int adjusted_bearing = bearing;
    int adjusted_upper = upper_bound;
    if (adjusted_bearing < lower_bound)
        adjusted_bearing += 360;
    if (adjusted_upper < lower_bound)
        adjusted_upper += 360;

    double interpolation_progress = 0.0;
    double denominator = static_cast<double>(adjusted_upper - lower_bound);
    if (std::abs(denominator) > 1e-9) {
        interpolation_progress = static_cast<double>(adjusted_bearing - lower_bound) / denominator;
    }

    double lower_radius = workplane.at(lower_bound);
    double upper_radius = workplane.at(upper_bound % 360);
    double reachable_radius = lower_radius * (1.0 - interpolation_progress) + upper_radius * interpolation_progress;

    return planar_distance <= (reachable_radius + 1e-6);
}

double WorkspaceAnalyzer::getWalkspaceRadius(double bearing_degrees) const {
    while (bearing_degrees < 0)
        bearing_degrees += 360;
    while (bearing_degrees >= 360)
        bearing_degrees -= 360;

    int lower_bearing = static_cast<int>(bearing_degrees / BEARING_STEP) * BEARING_STEP;
    int upper_bearing = lower_bearing + BEARING_STEP;

    auto lower_it = walkspace_map_.find(lower_bearing);
    auto upper_it = walkspace_map_.find(upper_bearing);
    if (upper_it == walkspace_map_.end() && upper_bearing == 360) {
        upper_it = walkspace_map_.find(0);
    }

    if (lower_it == walkspace_map_.end() || upper_it == walkspace_map_.end()) {
        return 0.0;
    }

    double t = (bearing_degrees - lower_bearing) / BEARING_STEP;
    return lower_it->second * (1.0 - t) + upper_it->second * t;
}

WorkspaceBounds
WorkspaceAnalyzer::getWorkspaceBounds(int leg_index) const {
    WorkspaceBounds bounds;
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return bounds;
    }

    const Workspace &workspace = leg_workspaces_[leg_index];
    if (workspace.empty()) {
        return bounds;
    }

    bounds.min_height = workspace.begin()->first;
    bounds.max_height = workspace.rbegin()->first;
    bounds.has_height_restrictions = true;
    bounds.center_position = model_.getLegBasePosition(leg_index);

    bounds.min_reach = std::numeric_limits<double>::max();
    bounds.max_reach = 0.0;

    for (const auto &layer : workspace) {
        const Workplane &plane = layer.second;
        for (const auto &entry : plane) {
            bounds.min_reach = std::min(bounds.min_reach, entry.second);
            bounds.max_reach = std::max(bounds.max_reach, entry.second);
        }
    }

    if (!std::isfinite(bounds.min_reach)) {
        bounds.min_reach = 0.0;
    }

    bounds.preferred_min_reach = bounds.min_reach;
    bounds.preferred_max_reach = bounds.max_reach;
    return bounds;
}

VelocityConstraints
WorkspaceAnalyzer::calculateVelocityConstraints(int leg_index, double bearing_degrees,
                                                double gait_frequency, double stance_ratio) const {
    VelocityConstraints constraints;

    if (leg_index < 0 || leg_index >= NUM_LEGS || gait_frequency <= 0.0 || stance_ratio <= 0.0) {
        return constraints;
    }

    double walkspace_radius = getWalkspaceRadius(bearing_degrees);
    if (walkspace_radius <= 0.0) {
        walkspace_radius = getWorkspaceBounds(leg_index).max_reach;
    }

    constraints.workspace_radius = std::max(0.0, walkspace_radius);
    constraints.stance_radius = std::hypot(default_tip_positions_[leg_index].x, default_tip_positions_[leg_index].y);
    if (constraints.stance_radius <= 1e-6) {
        constraints.stance_radius = std::max(1.0, model_.getParams().hexagon_radius);
    }

    double cycle_time = stance_ratio / gait_frequency;
    if (cycle_time <= 0.0) {
        return constraints;
    }

    constraints.max_linear_velocity = (2.0 * constraints.workspace_radius) / cycle_time;
    constraints.max_angular_velocity = constraints.max_linear_velocity / constraints.stance_radius;
    constraints.max_acceleration = constraints.max_linear_velocity / std::max(1e-6, cycle_time);

    const Parameters &params = model_.getParams();
    double linear_cap = (params.max_velocity > 0.0) ? params.max_velocity : DEFAULT_MAX_LINEAR_VELOCITY;
    double angular_cap = (params.max_angular_velocity > 0.0)
                             ? math_utils::degreesToRadians(params.max_angular_velocity)
                             : math_utils::degreesToRadians(DEFAULT_MAX_ANGULAR_VELOCITY);

    constraints.max_linear_velocity = math_utils::clamp<double>(constraints.max_linear_velocity, 0.0, linear_cap);
    constraints.max_angular_velocity = math_utils::clamp<double>(constraints.max_angular_velocity, 0.0, angular_cap);
    constraints.max_acceleration = std::max(0.0, constraints.max_acceleration);

    return constraints;
}

Point3D WorkspaceAnalyzer::constrainToGeometricWorkspace(int leg_index, const Point3D &target) const {
    return makeReachable(leg_index, target);
}

void WorkspaceAnalyzer::invalidateWorkspaceCache() {
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_workspace_generated_[i] = false;
    }
    walkspace_map_generated_ = false;
}

void WorkspaceAnalyzer::invalidateWorkspaceCache(int leg_index) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        leg_workspace_generated_[leg_index] = false;
    }
}
