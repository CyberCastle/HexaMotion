#include "workspace_analyzer.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

WorkspaceAnalyzer::WorkspaceAnalyzer(const RobotModel &model, ComputeConfig config, const ValidationConfig &validation_config)
    : model_(model), config_(config), validation_config_(validation_config), analysis_enabled_(true), total_analysis_time_(0.0) {
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

    initialize();
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
    walkspace_map_.clear();

    // Generate walkspace for all legs first (only once per leg)
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        generateWalkspaceForLeg(leg);
    }

    // Pre-calculate leg base positions and adjacent distances to avoid redundant calculations
    Point3D leg_origins[NUM_LEGS];
    double adjacent_distances[NUM_LEGS];
    
    JointAngles zero_angles(0, 0, 0);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Pose leg_origin_pose = model_.getPoseRobotFrame(leg, zero_angles, Pose::Identity());
        leg_origins[leg] = leg_origin_pose.position;
        
        // Calculate minimum distance to adjacent legs
        int adjacent1 = (leg + 1) % NUM_LEGS;
        int adjacent2 = (leg + NUM_LEGS - 1) % NUM_LEGS;
        
        Pose adj1_pose = model_.getPoseRobotFrame(adjacent1, zero_angles, Pose::Identity());
        Pose adj2_pose = model_.getPoseRobotFrame(adjacent2, zero_angles, Pose::Identity());
        
        double dist_to_adj1 = math_utils::distance(leg_origins[leg], adj1_pose.position) / 2.0f;
        double dist_to_adj2 = math_utils::distance(leg_origins[leg], adj2_pose.position) / 2.0f;
        
        adjacent_distances[leg] = std::min(dist_to_adj1, dist_to_adj2);
    }

    // Generate walkspace for each bearing
    for (int bearing = 0; bearing <= 360; bearing += BEARING_STEP) {
        double min_radius = MAX_WORKSPACE_RADIUS;
        double bearing_rad = math_utils::degreesToRadians(bearing);

        // Find minimum radius across all legs for this bearing
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            // Use pre-generated workspace data
            Workplane workplane = getWorkplane(leg, 0.0); // Use ground level workplane

            if (!workplane.empty()) {
                auto bearing_it = workplane.find(bearing);
                if (bearing_it != workplane.end()) {
                    min_radius = std::min(min_radius, bearing_it->second);
                }
            }

            // Apply bearing direction constraint using pre-calculated adjacent distances
            double projected_reach = adjacent_distances[leg];
            
            // Only apply cosine correction if cosine is significant (not near 90°, 270°)
            if (std::abs(cos(bearing_rad)) > 0.1) {
                projected_reach = adjacent_distances[leg] / std::abs(cos(bearing_rad));
            }

            min_radius = std::min(min_radius, projected_reach);
        }

        walkspace_map_[bearing] = std::max(min_radius, 0.0);
    }

    // Ensure symmetry (OpenSHC equivalent)
    walkspace_map_[360] = walkspace_map_[0];

    // Update analysis info
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
    auto lower_it = std::prev(upper_it);

    double upper_height = upper_it->first;
    double lower_height = lower_it->first;
    const Workplane &upper_workplane = upper_it->second;
    const Workplane &lower_workplane = lower_it->second;

    // Calculate interpolation factor
    double t = (height - lower_height) / (upper_height - lower_height);

    // Interpolate between workplanes
    Workplane interpolated_workplane;
    for (const auto &bearing_radius : upper_workplane) {
        int bearing = bearing_radius.first;
        double upper_radius = bearing_radius.second;

        auto lower_bearing_it = lower_workplane.find(bearing);
        if (lower_bearing_it != lower_workplane.end()) {
            double lower_radius = lower_bearing_it->second;
            double interpolated_radius = lower_radius * (1.0f - t) + upper_radius * t;
            interpolated_workplane[bearing] = interpolated_radius;
        }
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

double WorkspaceAnalyzer::checkCollisionRisk(int leg_index, const Point3D &target_position,
                                             const Point3D current_leg_positions[NUM_LEGS]) const {
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

Point3D WorkspaceAnalyzer::constrainToValidWorkspace(int leg_index, const Point3D &target_position,
                                                     const Point3D current_leg_positions[NUM_LEGS]) const {
    Point3D constrained = target_position;

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

    // Record start time for performance tracking
    unsigned long start_time = millis();

    WalkspaceResult result;

    // Calculate center of mass
    result.center_of_mass = calculateCenterOfMass(leg_positions);

    // Calculate stability margin
    result.stability_margin = calculateStabilityMargin(leg_positions);
    result.is_stable = result.stability_margin > STABILITY_THRESHOLD;

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

    // Calculate analysis time
    unsigned long analysis_time = millis() - start_time;

    // Update analysis information
    updateAnalysisInfo(result, analysis_time);

    // Update timestamp
    last_analysis_timestamp_ = millis();

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

    // Calculate theoretical reach limits
    double total_leg_length = params.coxa_length + params.femur_length + params.tibia_length;
    double theoretical_min = std::abs(params.femur_length - params.tibia_length);

    // Apply safety factors from configuration
    bounds.max_reach = total_leg_length * validation_config_.safety_margin_factor;
    bounds.min_reach = params.coxa_length * validation_config_.minimum_reach_factor;

    // Set preferred reach values (using the theoretical minimum calculation)
    bounds.preferred_max_reach = total_leg_length * validation_config_.safety_margin_factor;
    bounds.preferred_min_reach = theoretical_min * validation_config_.minimum_reach_factor;

    // Height bounds (assuming ground level operation)
    bounds.min_height = leg_base.z - total_leg_length;
    bounds.max_height = leg_base.z + total_leg_length * 0.5f; // Limited upward reach

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

    // Get workspace bounds for this leg
    WorkspaceBounds bounds = getWorkspaceBounds(leg_index);

    // Calculate effective workspace radius based on bearing
    double leg_angle = model_.getLegBaseAngleOffset(leg_index) * RADIANS_TO_DEGREES_FACTOR;
    double bearing_offset = std::abs(bearing_degrees - leg_angle);
    if (bearing_offset > 180.0f) {
        bearing_offset = 360.0f - bearing_offset;
    }

    // Efficiency decreases as we move away from leg's natural direction
    double directional_efficiency = std::cos(math_utils::degreesToRadians(bearing_offset));
    directional_efficiency = math_utils::clamp<double>(directional_efficiency, 0.3, 1.0); // Minimum 30% efficiency

    constraints.workspace_radius = bounds.max_reach * directional_efficiency;
    constraints.stance_radius = bounds.max_reach * 0.8f; // For angular calculations

    // Calculate velocity limits based on gait parameters
    double cycle_time = stance_ratio / gait_frequency;
    if (cycle_time > 0.0f) {
        // Maximum linear speed: can traverse workspace diameter in one cycle
        constraints.max_linear_velocity = (constraints.workspace_radius * 2.0f) / cycle_time;

        // Apply scaling factors
        ScalingFactors scaling = getScalingFactors();
        constraints.max_linear_velocity *= scaling.velocity_scale;

        // Maximum angular speed
        constraints.max_angular_velocity = constraints.max_linear_velocity / constraints.stance_radius;
        constraints.max_angular_velocity *= scaling.angular_scale;

        // Maximum acceleration (reach max speed in 2 seconds)
        constraints.max_acceleration = constraints.max_linear_velocity / 2.0f;
        constraints.max_acceleration *= scaling.acceleration_scale;
    }

    // Apply reasonable safety limits
    constraints.max_linear_velocity =
        math_utils::clamp<double>(constraints.max_linear_velocity, 0.0, 5.0);
    constraints.max_angular_velocity =
        math_utils::clamp<double>(constraints.max_angular_velocity, 0.0, 10.0);
    constraints.max_acceleration =
        math_utils::clamp<double>(constraints.max_acceleration, 0.0, 10.0);

    return constraints;
}

// ========================================================================
// CONFIGURATION AND SCALING
// ========================================================================

ScalingFactors WorkspaceAnalyzer::getScalingFactors() const {
    ScalingFactors factors;

    // These scaling factors replace the scattered constants across the codebase:
    // - WORKSPACE_SCALING_FACTOR (0.8f) -> workspace_scale (0.65f) - more conservative
    // - WALKSPACE_SCALING_FACTOR (0.65f) -> workspace_scale (0.65f) - consistent
    // - DEFAULT_ANGULAR_SCALING (1.0f) -> angular_scale (1.0f) - maintained
    // - Various safety margins -> unified safety_margin (0.9f)

    factors.workspace_scale = 0.65f;                                   // Conservative workspace scaling
    factors.linear_scale = 0.65f;                                      // Consistent with existing walkspace analysis
    factors.velocity_scale = 0.9f;                                     // 10% safety margin for velocity calculations
    factors.angular_scale = 1.0f;                                      // No scaling for angular velocities by default
    factors.acceleration_scale = 1.0f;                                 // No scaling for acceleration by default
    factors.collision_scale = validation_config_.safety_margin_factor; // Use configured safety margin

    return factors;
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
    double coxa_min_rad = params.coxa_angle_limits[0] * M_PI / 180.0;
    double coxa_max_rad = params.coxa_angle_limits[1] * M_PI / 180.0;
    double femur_min_rad = params.femur_angle_limits[0] * M_PI / 180.0;
    double femur_max_rad = params.femur_angle_limits[1] * M_PI / 180.0;
    double tibia_min_rad = params.tibia_angle_limits[0] * M_PI / 180.0;
    double tibia_max_rad = params.tibia_angle_limits[1] * M_PI / 180.0;

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

double WorkspaceAnalyzer::getDistanceFromBase(int leg_index, const Point3D &target_position) const {
    Point3D leg_base = getLegBase(leg_index);
    return math_utils::distance3D(leg_base, target_position);
}

bool WorkspaceAnalyzer::checkJointLimits(int leg_index, const Point3D &target_position) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return false;
    }

    // Calcular la cinemática inversa y comprobar si los ángulos articulares están dentro de los límites
    try {
        // For validation, use zero angles as starting point
        JointAngles zero_angles(0, 0, 0);
        JointAngles angles = model_.inverseKinematicsCurrentGlobalCoordinates(leg_index, zero_angles, target_position);

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

Point3D WorkspaceAnalyzer::constrainToGeometricWorkspace(int leg_index, const Point3D &target_position) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return target_position;
    }

    const Parameters &params = model_.getParams();
    Point3D leg_base = getLegBase(leg_index);

    // Calculate max reach
    double max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    max_reach *= validation_config_.safety_margin_factor;

    // Calculate distance from base
    double distance = getDistanceFromBase(leg_index, target_position);

    if (distance <= max_reach) {
        return target_position; // Already within bounds
    }

    // Constrain to max reach
    double scale = max_reach / distance;
    Point3D constrained = target_position;
    constrained.x = leg_base.x + (target_position.x - leg_base.x) * scale;
    constrained.y = leg_base.y + (target_position.y - leg_base.y) * scale;
    constrained.z = leg_base.z + (target_position.z - leg_base.z) * scale;

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
    if (distance_to_left < MIN_LEG_SEPARATION) {
        return true;
    }

    // Check collision with right adjacent leg
    double distance_to_right = getDistance2D(target_position, adjacent_positions[right_adjacent]);
    if (distance_to_right < MIN_LEG_SEPARATION) {
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
        for (double scale = 0.9f; scale >= 0.5f; scale -= 0.1f) {
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
        double safe_scale = math_utils::clamp<double>((leg_reach * 0.7) / distance, 0.5, std::numeric_limits<double>::infinity());
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

// ========================================================================
// PRIVATE ANALYSIS METHODS
// ========================================================================

void WorkspaceAnalyzer::calculateLegWorkspaceBounds(int leg_index) {
    const Parameters &params = model_.getParams();

    double total_reach = params.femur_length + params.tibia_length;
    double min_reach = 30.0f; // Minimum practical reach when leg is folded

    WorkspaceBounds &bounds = leg_workspace_[leg_index];
    bounds.min_reach = min_reach;
    bounds.max_reach = total_reach;
    bounds.min_height = -total_reach;
    bounds.max_height = total_reach;
}

void WorkspaceAnalyzer::generateWalkspaceForLeg(int leg_index) {
    // OpenSHC-equivalent workspace generation adapted for HexaMotion
    // Based on OpenSHC Leg::generateWorkspace() algorithm

    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return;
    }

    // Get robot parameters to determine realistic limits
    const Parameters &params = model_.getParams();
    double total_leg_length = params.coxa_length + params.femur_length + params.tibia_length;
    double max_realistic_radius = std::min(static_cast<double>(MAX_WORKSPACE_RADIUS), total_leg_length);

    // Initialize max/min workplanes (OpenSHC style)
    Workplane max_workplane;
    Workplane min_workplane;
    for (int bearing = 0; bearing <= 360; bearing += BEARING_STEP) {
        max_workplane[bearing] = max_realistic_radius;
        min_workplane[bearing] = 0.0;
    }

    // Clear existing workspace for this leg
    leg_workspaces_[leg_index].clear();

    // Calculate identity tip pose (equivalent to leg at default 0° angles)
    JointAngles identity_angles(0.0, 0.0, 0.0);
    Point3D identity_tip_position;

    try {
        identity_tip_position = model_.forwardKinematicsGlobalCoordinates(leg_index, identity_angles);
    } catch (...) {
        // If can't reach identity pose, set zero workspace
        leg_workspaces_[leg_index][0.0] = min_workplane;
        return;
    }

    // For precision levels, use simplified approach for better performance
    if (config_.precision == PRECISION_LOW) {
        // Simple mode: just insert max workplane at ground level
        leg_workspaces_[leg_index][0.0] = max_workplane;
        return;
    }

    // Full OpenSHC-style workspace generation for MEDIUM and HIGH precision
    bool found_lower_limit = false;
    bool found_upper_limit = false;
    double max_plane_height = 0.0;
    double min_plane_height = 0.0;
    double search_height_delta = max_realistic_radius / WORKSPACE_LAYERS;

    double search_height = 0.0;
    int search_bearing = 0;
    bool within_limits = true;
    int iteration = 1;
    Point3D origin_tip_position, target_tip_position;
    double distance_from_origin = 0.0;
    int number_iterations = 0;
    bool workspace_generation_complete = false;

    // Main OpenSHC algorithm loop
    while (!workspace_generation_complete) {
        // Calculate current identity position with height offset
        Point3D current_identity = identity_tip_position;
        current_identity.z += search_height;

        // Set origin and target for linear interpolation (OpenSHC style)
        if (iteration == 1) {
            within_limits = true;

            // Search for upper and lower vertical limit of workspace
            if (!found_lower_limit || !found_upper_limit) {
                number_iterations = static_cast<int>(max_realistic_radius / MAX_POSITION_DELTA);
                origin_tip_position = current_identity;

                Point3D search_limit = current_identity;
                search_limit.z += (found_lower_limit ? max_realistic_radius : -max_realistic_radius);
                target_tip_position = search_limit;
            }
            // Track to new workplane origin at search height
            else if (search_bearing == 0) {
                number_iterations = std::max(1, static_cast<int>(search_height_delta / MAX_POSITION_DELTA));
                origin_tip_position = identity_tip_position; // Current position
                target_tip_position = current_identity;
            }
            // Search along search bearing for limits
            else {
                number_iterations = static_cast<int>(max_realistic_radius / MAX_POSITION_DELTA);
                origin_tip_position = current_identity;
                target_tip_position = current_identity;

                double bearing_rad = search_bearing * M_PI / 180.0;
                target_tip_position.x += max_realistic_radius * cos(bearing_rad);
                target_tip_position.y += max_realistic_radius * sin(bearing_rad);
            }
        }

        // Move tip position linearly along search bearing (OpenSHC interpolation)
        double i = double(iteration) / number_iterations;
        Point3D desired_tip_position;
        desired_tip_position.x = origin_tip_position.x * (1.0 - i) + target_tip_position.x * i;
        desired_tip_position.y = origin_tip_position.y * (1.0 - i) + target_tip_position.y * i;
        desired_tip_position.z = origin_tip_position.z * (1.0 - i) + target_tip_position.z * i;

        // Test reachability using inverse kinematics
        bool ik_result = detailedReachabilityCheck(leg_index, desired_tip_position);

        // Calculate distance from origin (OpenSHC style)
        Point3D diff = desired_tip_position - current_identity;
        distance_from_origin = sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

        // Check if leg is still within limits
        within_limits = within_limits && ik_result;

        // Search not complete -> iterate along current search bearing
        if (within_limits && iteration < number_iterations) {
            iteration++;
        }
        // Current search along bearing complete -> save result and iterate
        else {
            iteration = 1;

            // Lower vertical limit found
            if (!found_lower_limit) {
                found_lower_limit = true;
                min_plane_height = -distance_from_origin;
                leg_workspaces_[leg_index][min_plane_height] = min_workplane;
                continue;
            }
            // Upper vertical limit found
            else if (!found_upper_limit) {
                found_upper_limit = true;
                max_plane_height = distance_from_origin;
                search_height_delta = (max_plane_height - min_plane_height) / WORKSPACE_LAYERS;

                int upper_levels = static_cast<int>(abs(max_plane_height) / search_height_delta);
                search_height = upper_levels * search_height_delta;

                leg_workspaces_[leg_index][max_plane_height] = min_workplane;
                leg_workspaces_[leg_index][search_height] = max_workplane;
                continue;
            }
            // Tracked to origin of new workplane (search_bearing == 0)
            else if (search_bearing == 0) {
                // No special action needed - ready for bearing search
            }
            // Search along bearing complete - save in workspace
            else {
                leg_workspaces_[leg_index][search_height][search_bearing] = distance_from_origin;
            }

            // Iterate search bearing (0 -> 360 anti-clockwise, OpenSHC style)
            if (search_bearing + BEARING_STEP <= 360) {
                search_bearing += BEARING_STEP;
            }
            // Iterate search height (top to bottom, OpenSHC style)
            else {
                search_bearing = 0;
                // Ensure symmetry: bearing 0° = bearing 360°
                if (leg_workspaces_[leg_index].find(search_height) != leg_workspaces_[leg_index].end()) {
                    leg_workspaces_[leg_index][search_height][0] = leg_workspaces_[leg_index][search_height][360];
                }

                search_height -= search_height_delta;
                if (search_height >= min_plane_height) {
                    leg_workspaces_[leg_index][search_height] = max_workplane;
                }
                // All searches complete
                else {
                    workspace_generation_complete = true;
                }
            }
        }
    }

    // Ensure all workplanes have proper symmetry (bearing 0° = bearing 360°)
    for (auto &workspace_entry : leg_workspaces_[leg_index]) {
        Workplane &workplane = workspace_entry.second;
        if (workplane.find(0) != workplane.end() && workplane.find(360) != workplane.end()) {
            workplane[360] = workplane[0];
        }
    }
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

    // Update timing statistics
    analysis_info_.last_analysis_time = analysis_time_ms;
    analysis_info_.analysis_count++;
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

    // Update timestamp
    analysis_info_.last_analysis_time = millis();
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
