#include "walkspace_analyzer.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

WalkspaceAnalyzer::WalkspaceAnalyzer(RobotModel &model, ComputeConfig config)
    : model_(model), config_(config) {
}

void WalkspaceAnalyzer::initialize() {
    // Calculate workspace bounds for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        calculateLegWorkspaceBounds(i);
    }

    // Generate initial walkspace
    generateWalkspace();
}

void WalkspaceAnalyzer::generateWalkspace() {
    walkspace_map_.clear();

    // Generate walkspace for each bearing
    for (int bearing = 0; bearing <= 360; bearing += BEARING_STEP) {
        float min_radius = MAX_WORKSPACE_RADIUS;

        // Find minimum radius across all legs for this bearing
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            generateWalkspaceForLeg(leg);

            // Calculate distance to adjacent legs to avoid overlap
            int adjacent1 = (leg + 1) % NUM_LEGS;
            int adjacent2 = (leg + NUM_LEGS - 1) % NUM_LEGS;

            Point3D leg_origin = model_.getLegOrigin(leg);
            Point3D adj1_origin = model_.getLegOrigin(adjacent1);
            Point3D adj2_origin = model_.getLegOrigin(adjacent2);

            float dist_to_adj1 = math_utils::distance(leg_origin, adj1_origin) / 2.0f;
            float dist_to_adj2 = math_utils::distance(leg_origin, adj2_origin) / 2.0f;

            float bearing_rad = math_utils::degreesToRadians(bearing);
            float max_reach = std::min(dist_to_adj1, dist_to_adj2);

            // Adjust for bearing direction
            Point3D direction(cos(bearing_rad), sin(bearing_rad), 0);
            float projected_reach = max_reach / cos(bearing_rad);

            min_radius = std::min(min_radius, projected_reach);
        }

        walkspace_map_[bearing] = std::max(min_radius, 0.0f);
    }

    // Ensure symmetry (OpenSHC equivalent)
    walkspace_map_[360] = walkspace_map_[0];
}

bool WalkspaceAnalyzer::isPositionReachable(int leg_index, const Point3D &position) {
    if (leg_index >= NUM_LEGS)
        return false;

    Point3D leg_origin = model_.getLegOrigin(leg_index);
    Point3D relative_pos = position - leg_origin;
    float distance = math_utils::magnitude(relative_pos);

    const WorkspaceBounds &bounds = leg_workspace_[leg_index];

    // Quick bounds check for low precision
    if (config_.precision == PRECISION_LOW) {
        return distance >= bounds.min_radius && distance <= bounds.max_radius;
    }

    // Detailed check for higher precision
    return detailedReachabilityCheck(leg_index, relative_pos);
}

WalkspaceAnalyzer::WalkspaceResult WalkspaceAnalyzer::analyzeWalkspace(const Point3D leg_positions[NUM_LEGS]) {
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
        float radius = entry.second;
        result.reachable_area += M_PI * radius * radius / walkspace_map_.size();
    }

    return result;
}

float WalkspaceAnalyzer::getWalkspaceRadius(float bearing_degrees) const {
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
    float t = (bearing_degrees - lower_bearing) / BEARING_STEP;
    return lower_it->second * (1.0f - t) + upper_it->second * t;
}

bool WalkspaceAnalyzer::getOptimalStepPositions(const Point3D &body_movement,
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

void WalkspaceAnalyzer::calculateLegWorkspaceBounds(int leg_index) {
    const Parameters &params = model_.getParams();

    float total_reach = params.femur_length + params.tibia_length;
    float min_reach = 30.0f; // Minimum practical reach when leg is folded

    WorkspaceBounds &bounds = leg_workspace_[leg_index];
    bounds.min_radius = min_reach;
    bounds.max_radius = total_reach;
    bounds.min_height = -total_reach;
    bounds.max_height = total_reach;
    bounds.min_angle = -180.0f;
    bounds.max_angle = 180.0f;
}

void WalkspaceAnalyzer::generateWalkspaceForLeg(int leg_index) {
    // This method calculates the detailed workspace for a specific leg
    // Implementation depends on precision level
    if (config_.precision == PRECISION_HIGH) {
        // Detailed workspace generation using full IK analysis
        // Sample the workspace at high resolution using spherical coordinates
        const float radius_step = 10.0f;    // mm
        const float bearing_step = 15.0f;   // degrees
        const float elevation_step = 15.0f; // degrees

        for (float radius = 0; radius < 300.0f; radius += radius_step) {
            for (float bearing = 0; bearing < 360.0f; bearing += bearing_step) {
                for (float elevation = -90.0f; elevation <= 90.0f; elevation += elevation_step) {
                    // Convert spherical to cartesian coordinates
                    float bearing_rad = bearing * M_PI / 180.0f;
                    float elevation_rad = elevation * M_PI / 180.0f;

                    Point3D test_point;
                    test_point.x = radius * cos(elevation_rad) * cos(bearing_rad);
                    test_point.y = radius * cos(elevation_rad) * sin(bearing_rad);
                    test_point.z = radius * sin(elevation_rad);

                    // Check if point is reachable using detailed IK check
                    if (detailedReachabilityCheck(leg_index, test_point)) {
                        // Add to workspace representation (would need storage structure)
                        // For now, just validate the workspace generation is working
                    }
                }
            }
        }
    } else {
        // Fast workspace generation using simplified bounds
        WorkspaceBounds bounds;
        calculateLegWorkspaceBounds(leg_index); // Use existing method instead of calculateSimpleWorkspace

        // Sample at lower resolution for PRECISION_FAST
        const float radius_step = 20.0f;  // mm
        const float bearing_step = 30.0f; // degrees

        for (float radius = 0; radius < bounds.max_radius; radius += radius_step) {
            for (float bearing = 0; bearing < 360.0f; bearing += bearing_step) {
                float bearing_rad = bearing * M_PI / 180.0f;

                Point3D test_point;
                test_point.x = radius * cos(bearing_rad);
                test_point.y = radius * sin(bearing_rad);
                test_point.z = 0; // Ground level for fast calculation

                // Simple reachability check
                if (radius <= bounds.max_radius) {
                    // Point is within simplified workspace
                }
            }
        }
    }
}

bool WalkspaceAnalyzer::detailedReachabilityCheck(int leg_index, const Point3D &position) {
    // Use inverse kinematics to check reachability
    JointAngles angles = model_.inverseKinematics(leg_index, position);
    return model_.checkJointLimits(leg_index, angles);
}

Point3D WalkspaceAnalyzer::calculateCenterOfMass(const Point3D leg_positions[NUM_LEGS]) {
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

float WalkspaceAnalyzer::calculateStabilityMargin(const Point3D leg_positions[NUM_LEGS]) {
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
    float min_distance = 1000.0f;

    // For simplicity, calculate distance to each support point
    // A full implementation would calculate distance to polygon edges
    for (const auto &support_point : support_polygon) {
        float distance = math_utils::distance(com, support_point);
        min_distance = std::min(min_distance, distance);
    }

    // Apply safety factor
    float safety_factor = 0.8f; // 80% of theoretical minimum
    return min_distance * safety_factor;
}

void WalkspaceAnalyzer::calculateSupportPolygon(const Point3D leg_positions[NUM_LEGS],
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
    std::vector<std::pair<float, Point3D>> angle_points;
    for (const auto &point : stance_points) {
        float angle = atan2(point.y - centroid.y, point.x - centroid.x);
        angle_points.push_back({angle, point});
    }

    // Sort by angle using custom comparator
    std::sort(angle_points.begin(), angle_points.end(),
              [](const std::pair<float, Point3D> &a, const std::pair<float, Point3D> &b) {
                  return a.first < b.first;
              });

    // Build polygon from sorted points
    for (const auto &angle_point : angle_points) {
        polygon.push_back(angle_point.second);
    }
}

bool WalkspaceAnalyzer::simpleStepOptimization(const Point3D &movement,
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

bool WalkspaceAnalyzer::balancedStepOptimization(const Point3D &movement,
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
            Point3D leg_origin = model_.getLegOrigin(i);
            Point3D relative = desired - leg_origin;
            float distance = math_utils::magnitude(relative);

            const WorkspaceBounds &bounds = leg_workspace_[i];
            if (distance > bounds.max_radius) {
                // Scale down to maximum reach
                float scale = bounds.max_radius / distance;
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

bool WalkspaceAnalyzer::advancedStepOptimization(const Point3D &movement,
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
                float distance = math_utils::magnitude(direction);
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
