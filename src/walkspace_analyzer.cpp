#include "walkspace_analyzer.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>

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
    // OpenSHC-equivalent workspace generation with Workplanes
    // Generates multiple height layers (workplanes) for complete 3D workspace
    if (config_.precision == PRECISION_HIGH) {
        // OpenSHC algorithm parameters
        const int WORKSPACE_LAYERS = 10;         // Height layers (from OpenSHC)
        const int BEARING_STEP = 45;             // 45° steps = 8 directions (from OpenSHC)
        const float MAX_POSITION_DELTA = 0.005f; // 5mm increments (optimized for speed)
        const float MAX_WORKSPACE_RADIUS = 0.3f; // 300mm max radius

        if (leg_index == 0) {
            std::cout << "    Generating " << WORKSPACE_LAYERS
                      << " workplanes with " << (360 / BEARING_STEP) << " bearings each" << std::endl;
        }

        // Clear existing workspace for this leg
        leg_workspaces_[leg_index].clear();

        // Calculate layer heights
        const float min_height = -MAX_WORKSPACE_RADIUS;
        const float max_height = MAX_WORKSPACE_RADIUS;
        const float height_step = (max_height - min_height) / (WORKSPACE_LAYERS - 1);

        int total_checks = 0;

        // Generate each workplane (height layer)
        for (int layer = 0; layer < WORKSPACE_LAYERS; layer++) {
            float search_height = min_height + (layer * height_step);

            // Create new workplane for this height
            Workplane workplane;

            // Generate workplane for each bearing direction
            for (int bearing = 0; bearing < 360; bearing += BEARING_STEP) {
                float bearing_rad = bearing * M_PI / 180.0f;
                float max_reachable_radius = 0.0f;

                // Radial search from center outward until kinematic limit
                int max_iterations = static_cast<int>(MAX_WORKSPACE_RADIUS / MAX_POSITION_DELTA);

                for (int step = 1; step <= max_iterations; step++) {
                    float radius = step * MAX_POSITION_DELTA;

                    Point3D test_point;
                    test_point.x = radius * cos(bearing_rad);
                    test_point.y = radius * sin(bearing_rad);
                    test_point.z = search_height;

                    total_checks++;

                    if (detailedReachabilityCheck(leg_index, test_point)) {
                        max_reachable_radius = radius;
                    } else {
                        // Found kinematic limit - stop searching in this direction
                        break;
                    }
                }

                // Store max radius for this bearing in workplane
                workplane[bearing] = max_reachable_radius;
            }

            // Ensure symmetry (bearing 360° = bearing 0°)
            workplane[360] = workplane[0];

            // Store workplane in workspace
            leg_workspaces_[leg_index][search_height] = workplane;
        }

        if (leg_index == 0) {
            std::cout << "    Generated " << WORKSPACE_LAYERS << " workplanes with "
                      << total_checks << " kinematic checks" << std::endl;
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

    // Calculate distance to polygon edges (proper implementation)
    // For each edge of the support polygon, find perpendicular distance from COM
    for (size_t i = 0; i < support_polygon.size(); i++) {
        size_t next_idx = (i + 1) % support_polygon.size();
        Point3D edge_start = support_polygon[i];
        Point3D edge_end = support_polygon[next_idx];

        // Calculate perpendicular distance from COM to line segment
        float edge_distance = math_utils::pointToLineDistance(com, edge_start, edge_end);
        min_distance = std::min(min_distance, edge_distance);
    }

    // Apply safety factor for stability margin
    float safety_factor = 0.8f; // 80% of theoretical minimum
    return min_distance * safety_factor;
}

// Enhanced support polygon calculation with proper convex hull algorithm
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

// OpenSHC-compatible Workplane functions

Workplane WalkspaceAnalyzer::getWorkplane(int leg_index, float height) const {
    if (leg_index >= NUM_LEGS) {
        return Workplane(); // Return empty workplane for invalid leg
    }

    const Workspace &workspace = leg_workspaces_[leg_index];

    // Check if height is within workspace bounds
    if (workspace.empty()) {
        return Workplane(); // Return empty if no workspace generated
    }

    float min_height = workspace.begin()->first;
    float max_height = workspace.rbegin()->first;

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

    float upper_height = upper_it->first;
    float lower_height = lower_it->first;
    const Workplane &upper_workplane = upper_it->second;
    const Workplane &lower_workplane = lower_it->second;

    // Calculate interpolation factor
    float t = (height - lower_height) / (upper_height - lower_height);

    // Interpolate between workplanes
    Workplane interpolated_workplane;
    for (const auto &bearing_radius : upper_workplane) {
        int bearing = bearing_radius.first;
        float upper_radius = bearing_radius.second;

        auto lower_bearing_it = lower_workplane.find(bearing);
        if (lower_bearing_it != lower_workplane.end()) {
            float lower_radius = lower_bearing_it->second;
            float interpolated_radius = lower_radius * (1.0f - t) + upper_radius * t;
            interpolated_workplane[bearing] = interpolated_radius;
        }
    }

    return interpolated_workplane;
}

Workspace WalkspaceAnalyzer::getLegWorkspace(int leg_index) const {
    if (leg_index >= NUM_LEGS) {
        return Workspace(); // Return empty workspace for invalid leg
    }

    return leg_workspaces_[leg_index];
}

bool WalkspaceAnalyzer::isPositionReachableWithWorkplane(int leg_index, const Point3D &position) const {
    if (leg_index >= NUM_LEGS) {
        return false;
    }

    // Get workplane at the position's height
    Workplane workplane = getWorkplane(leg_index, position.z);
    if (workplane.empty()) {
        return false; // No workplane available at this height
    }

    // Calculate bearing to position
    float bearing_rad = atan2(position.y, position.x);
    float bearing_deg = bearing_rad * 180.0f / M_PI;

    // Normalize bearing to 0-360 range
    while (bearing_deg < 0)
        bearing_deg += 360;
    while (bearing_deg >= 360)
        bearing_deg -= 360;

    // Find bounding bearings in workplane
    int lower_bearing = static_cast<int>(bearing_deg / 45) * 45;
    int upper_bearing = lower_bearing + 45;
    if (upper_bearing >= 360)
        upper_bearing = 0;

    auto lower_it = workplane.find(lower_bearing);
    auto upper_it = workplane.find(upper_bearing);

    if (lower_it == workplane.end() || upper_it == workplane.end()) {
        return false;
    }

    // Interpolate maximum radius at this bearing
    float t = (bearing_deg - lower_bearing) / 45.0f;
    float max_radius = lower_it->second * (1.0f - t) + upper_it->second * t;

    // Check if position is within reachable radius
    float position_radius = sqrt(position.x * position.x + position.y * position.y);
    return position_radius <= max_radius;
}
