#include "workspace_validator.h"
#include "HexaModel.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>
#include <limits>

WorkspaceValidator::WorkspaceValidator(const RobotModel &model, const ValidationConfig &config)
    : model_(model), config_(config) {
}

ValidationResult
WorkspaceValidator::validateTarget(int leg_index, Point3D target_position,
                                   const Point3D current_leg_positions[NUM_LEGS],
                                   bool constrain_if_invalid) {
    ValidationResult result;
    result.constrained_position = target_position;

    // 1. Basic geometric reachability
    result.is_reachable = isReachable(leg_index, target_position);
    result.distance_from_base = getDistanceFromBase(leg_index, target_position);

    // 2. Collision checking
    if (config_.enable_collision_checking) {
        result.collision_risk_factor = checkCollisionRisk(leg_index, target_position, current_leg_positions);
        result.is_collision_free = (result.collision_risk_factor < 0.1f); // 10% threshold
    } else {
        result.is_collision_free = true;
        result.collision_risk_factor = 0.0f;
    }

    // 3. Joint limit checking
    if (config_.enable_joint_limit_checking) {
        result.is_within_joint_limits = checkJointLimits(leg_index, target_position);
    } else {
        result.is_within_joint_limits = true;
    }

    // 4. Apply constraints if needed and requested
    if (constrain_if_invalid && !result.isValid()) {
        result.constrained_position = constrainToValidWorkspace(leg_index, target_position, current_leg_positions);

        // Re-validate the constrained position
        result.is_reachable = isReachable(leg_index, result.constrained_position);
        result.distance_from_base = getDistanceFromBase(leg_index, result.constrained_position);

        if (config_.enable_collision_checking) {
            result.collision_risk_factor = checkCollisionRisk(leg_index, result.constrained_position, current_leg_positions);
            result.is_collision_free = (result.collision_risk_factor < 0.1f);
        }

        if (config_.enable_joint_limit_checking) {
            result.is_within_joint_limits = checkJointLimits(leg_index, result.constrained_position);
        }
    }

    return result;
}

bool WorkspaceValidator::isReachable(int leg_index, const Point3D &target_position) const {
    double min_reach, max_reach;
    getWorkspaceBounds(leg_index, min_reach, max_reach);

    double distance = getDistanceFromBase(leg_index, target_position);
    return (distance >= min_reach && distance <= max_reach);
}

double WorkspaceValidator::checkCollisionRisk(int leg_index, const Point3D &target_position,
                                             const Point3D current_leg_positions[NUM_LEGS]) const {
    // Get adjacent leg indices
    int left_adjacent = (leg_index + NUM_LEGS - 1) % NUM_LEGS;
    int right_adjacent = (leg_index + 1) % NUM_LEGS;

    double max_risk = 0.0f;

    // Check distance to adjacent legs
    double distance_to_left = math_utils::distance2D(target_position, current_leg_positions[left_adjacent]);
    double distance_to_right = math_utils::distance2D(target_position, current_leg_positions[right_adjacent]);

    // Convert distances to risk factors (closer = higher risk)
    double min_safe_distance = config_.collision_safety_margin;

    if (distance_to_left < min_safe_distance) {
        double risk = 1.0f - (distance_to_left / min_safe_distance);
        max_risk = std::max(max_risk, risk);
    }

    if (distance_to_right < min_safe_distance) {
        double risk = 1.0f - (distance_to_right / min_safe_distance);
        max_risk = std::max(max_risk, risk);
    }

    return std::clamp<double>(max_risk, 0.0, 1.0);
}

Point3D WorkspaceValidator::constrainToValidWorkspace(int leg_index, const Point3D &target_position,
                                                      const Point3D current_leg_positions[NUM_LEGS]) const {
    Point3D constrained = target_position;

    // Step 1: Constrain to geometric workspace
    constrained = constrainToGeometricWorkspace(leg_index, constrained);

    // Step 2: Apply collision avoidance if enabled
    if (config_.enable_collision_checking) {
        double total_reach = model_.getParams().coxa_length + model_.getParams().femur_length + model_.getParams().tibia_length;
        adjustForCollisionAvoidance(leg_index, constrained, model_.getParams().hexagon_radius, total_reach, current_leg_positions);
    }

    // Step 3: Final geometric validation (in case collision avoidance pushed it out)
    constrained = constrainToGeometricWorkspace(leg_index, constrained);

    return constrained;
}

void WorkspaceValidator::getWorkspaceBounds(int leg_index, double &min_reach, double &max_reach) const {
    const Parameters &params = model_.getParams();

    double total_reach = params.coxa_length + params.femur_length + params.tibia_length;
    double theoretical_min = std::abs(params.femur_length - params.tibia_length);

    max_reach = total_reach * config_.safety_margin_factor;
    min_reach = theoretical_min * config_.minimum_reach_factor;
}

WorkspaceBounds
WorkspaceValidator::getWorkspaceBounds(int leg_index) const {
    WorkspaceBounds bounds;

    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return bounds; // Return empty bounds for invalid leg index
    }

    const Parameters &params = model_.getParams();
    Point3D leg_base = getLegBase(leg_index);

    // Calculate theoretical reach limits
    double total_leg_length = params.coxa_length + params.femur_length + params.tibia_length;

    // Apply safety factors from configuration
    bounds.max_radius = total_leg_length * config_.safety_margin_factor;
    bounds.min_radius = params.coxa_length * config_.minimum_reach_factor;

    // Height bounds (assuming ground level operation)
    bounds.min_height = leg_base.z - total_leg_length;
    bounds.max_height = leg_base.z + total_leg_length * 0.5f; // Limited upward reach

    bounds.center_position = leg_base;

    return bounds;
}

VelocityConstraints
WorkspaceValidator::calculateVelocityConstraints(int leg_index, double bearing_degrees,
                                                 double gait_frequency, double stance_ratio) const {
    VelocityConstraints constraints;

    if (leg_index < 0 || leg_index >= NUM_LEGS || stance_ratio <= 0.0f || gait_frequency <= 0.0f) {
        return constraints; // Return empty constraints for invalid inputs
    }

    // Get workspace bounds for this leg
    WorkspaceBounds bounds = getWorkspaceBounds(leg_index);

    // Calculate effective workspace radius based on bearing
    Point3D leg_base = getLegBase(leg_index);
    double leg_angle = model_.getParams().dh_parameters[leg_index][0][3];
    double bearing_offset = std::abs(bearing_degrees - leg_angle);
    if (bearing_offset > 180.0f) {
        bearing_offset = 360.0f - bearing_offset;
    }

    // Efficiency decreases as we move away from leg's natural direction
    double directional_efficiency = std::cos(math_utils::degreesToRadians(bearing_offset));
    directional_efficiency = std::clamp<double>(directional_efficiency, 0.3, 1.0); // Minimum 30% efficiency

    constraints.workspace_radius = bounds.max_radius * directional_efficiency;
    constraints.stance_radius = bounds.max_radius * 0.8f; // For angular calculations

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
        std::clamp<double>(constraints.max_linear_velocity, 0.0, 5.0);
    constraints.max_angular_velocity =
        std::clamp<double>(constraints.max_angular_velocity, 0.0, 10.0);
    constraints.max_acceleration =
        std::clamp<double>(constraints.max_acceleration, 0.0, 10.0);

    return constraints;
}

bool WorkspaceValidator::isPositionReachable(int leg_index, const Point3D &position,
                                             bool use_ik_validation) const {
    if (use_ik_validation) {
        // Use full IK validation for accuracy
        return checkJointLimits(leg_index, position) && isReachable(leg_index, position);
    } else {
        // Use fast geometric check only
        return isReachable(leg_index, position);
    }
}

ScalingFactors WorkspaceValidator::getScalingFactors() const {
    ScalingFactors factors;

    // These scaling factors replace the scattered constants across the codebase:
    // - WORKSPACE_SCALING_FACTOR (0.8f) -> workspace_scale (0.65f) - more conservative
    // - WALKSPACE_SCALING_FACTOR (0.65f) -> workspace_scale (0.65f) - consistent
    // - DEFAULT_ANGULAR_SCALING (1.0f) -> angular_scale (1.0f) - maintained
    // - Various safety margins -> unified safety_margin (0.9f)

    factors.workspace_scale = 0.65f;                        // Conservative workspace scaling
    factors.linear_scale = 0.65f;                           // Consistent with existing walkspace analysis
    factors.velocity_scale = 0.9f;                          // 10% safety margin for velocity calculations
    factors.angular_scale = 1.0f;                           // No scaling for angular velocities by default
    factors.acceleration_scale = 1.0f;                      // No scaling for acceleration by default
    factors.collision_scale = config_.safety_margin_factor; // Use configured safety margin

    return factors;
}

void WorkspaceValidator::updateSafetyMargin(double margin) {
    config_.safety_margin_factor =
        std::clamp<double>(margin, 0.1, 1.0);
}

void WorkspaceValidator::updateAngularScaling(double scaling) {
    // Store angular scaling in configuration for future use
    // Note: This could be extended to modify a stored scaling factor if needed
    // For now, angular scaling is handled in getScalingFactors()
}

// ===== MIGRATED FROM LegCollisionAvoidance =====

double WorkspaceValidator::calculateSafeHexagonRadius(double leg_reach, double safety_margin) {
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
    double cos_60 = 0.5f; // cos(60°) = 0.5

    // Solving: min_separation² = 2 * (hexagon_radius + leg_reach)² * (1 - cos_60)
    // min_separation² = 2 * (hexagon_radius + leg_reach)² * 0.5
    // min_separation² = (hexagon_radius + leg_reach)²
    // Therefore: hexagon_radius = min_separation - leg_reach

    return min_separation - leg_reach;
}

double WorkspaceValidator::getDistance2D(const Point3D &p1, const Point3D &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

void WorkspaceValidator::getAdjacentLegIndices(int leg_index, int &left_adjacent, int &right_adjacent) {
    left_adjacent = (leg_index + NUM_LEGS - 1) % NUM_LEGS; // Previous leg (counter-clockwise)
    right_adjacent = (leg_index + 1) % NUM_LEGS;           // Next leg (clockwise)
}

bool WorkspaceValidator::checkWorkspaceOverlap(const Point3D &leg1_base, double leg1_reach,
                                               const Point3D &leg2_base, double leg2_reach,
                                               double safety_margin) {
    double distance = getDistance2D(leg1_base, leg2_base);
    double combined_reach = leg1_reach + leg2_reach + safety_margin;

    // If the distance between leg bases is less than combined reach, workspaces overlap
    return distance < combined_reach;
}

bool WorkspaceValidator::wouldCollideWithAdjacent(int leg_index, const Point3D &target_position,
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

bool WorkspaceValidator::adjustForCollisionAvoidance(int leg_index, Point3D &target_position,
                                                     double hexagon_radius, double leg_reach,
                                                     const Point3D adjacent_positions[NUM_LEGS]) const {
    // Check if adjustment is needed
    if (!wouldCollideWithAdjacent(leg_index, target_position, hexagon_radius, leg_reach, adjacent_positions)) {
        return true; // No collision, no adjustment needed
    }

    // Calculate leg base position
    const Parameters &params = model_.getParams();
    double base_angle = params.dh_parameters[leg_index][0][3];
    double base_x = hexagon_radius * cos(base_angle * M_PI / 180.0f);
    double base_y = hexagon_radius * sin(base_angle * M_PI / 180.0f);

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
        double safe_scale = std::clamp<double>((leg_reach * 0.7) / distance, 0.5, std::numeric_limits<double>::infinity());
        target_position.x = base_x + dx * safe_scale;
        target_position.y = base_y + dy * safe_scale;
        return true;
    }

    return false;
}

// Implementation of missing methods

Point3D WorkspaceValidator::getLegBase(int leg_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return Point3D{0.0f, 0.0f, 0.0f};
    }

    const Parameters &params = model_.getParams();
    double angle_rad = math_utils::degreesToRadians(params.dh_parameters[leg_index][0][3]);

    return Point3D{
        params.hexagon_radius * cos(angle_rad),
        params.hexagon_radius * sin(angle_rad),
        0.0f};
}

bool WorkspaceValidator::checkJointLimits(int leg_index, const Point3D &target_position) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return false;
    }

    // Calculate inverse kinematics and check if joint angles are within limits
    try {
        JointAngles angles = model_.inverseKinematics(leg_index, target_position);

        // Basic joint limit checks (these would be robot-specific)
        const double COXA_MIN = -45.0f, COXA_MAX = 45.0f;
        const double FEMUR_MIN = -90.0f, FEMUR_MAX = 90.0f;
        const double TIBIA_MIN = -135.0f, TIBIA_MAX = 45.0f;

        return (angles.coxa >= COXA_MIN && angles.coxa <= COXA_MAX &&
                angles.femur >= FEMUR_MIN && angles.femur <= FEMUR_MAX &&
                angles.tibia >= TIBIA_MIN && angles.tibia <= TIBIA_MAX);
    } catch (...) {
        return false;
    }
}

double WorkspaceValidator::getDistanceFromBase(int leg_index, const Point3D &target_position) const {
    Point3D leg_base = getLegBase(leg_index);
    return math_utils::distance3D(leg_base, target_position);
}

Point3D WorkspaceValidator::constrainToGeometricWorkspace(int leg_index, const Point3D &target_position) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return target_position;
    }

    const Parameters &params = model_.getParams();
    Point3D leg_base = getLegBase(leg_index);

    // Calculate max reach
    double max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    max_reach *= config_.safety_margin_factor;

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
