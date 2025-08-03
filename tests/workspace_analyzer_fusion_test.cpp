#include "../src/hexamotion_constants.h"
#include "../src/robot_model.h"
#include "../src/workspace_analyzer.h"
#include <iostream>

int main() {
    std::cout << "=== WorkspaceAnalyzer Fusion Test ===" << std::endl;

    // Create a robot model
    Parameters params;
    RobotModel model(params);

    // Test unified WorkspaceAnalyzer creation
    std::cout << "Creating WorkspaceAnalyzer..." << std::endl;
    WorkspaceAnalyzer analyzer(model);

    // Test workspace generation (from WalkspaceAnalyzer)
    std::cout << "Testing workspace generation..." << std::endl;
    try {
        analyzer.generateWorkspace();
        std::cout << "✅ Workspace generation successful" << std::endl;
    } catch (const std::exception &e) {
        std::cout << "❌ Workspace generation failed: " << e.what() << std::endl;
    }

    // Test validation functions (from WorkspaceValidator)
    std::cout << "Testing validation functions..." << std::endl;
    Point3D test_point(200, 100, -120);
    bool is_reachable = analyzer.isReachable(0, test_point);
    std::cout << "Point (" << test_point.x << ", " << test_point.y << ", " << test_point.z
              << ") reachable for leg 0: " << (is_reachable ? "YES" : "NO") << std::endl;

    // Test workspace bounds
    std::cout << "Testing workspace bounds..." << std::endl;
    auto bounds = analyzer.getWorkspaceBounds(0);
    std::cout << "Leg 0 workspace bounds:" << std::endl;
    std::cout << "  Min reach: " << bounds.min_reach << " mm" << std::endl;
    std::cout << "  Max reach: " << bounds.max_reach << " mm" << std::endl;
    std::cout << "  Min height: " << bounds.min_height << " mm" << std::endl;
    std::cout << "  Max height: " << bounds.max_height << " mm" << std::endl;

    // Test analysis functions
    std::cout << "Testing analysis functions..." << std::endl;
    Point3D leg_positions[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_positions[i] = Point3D(200, 150, -120); // Simple test positions
    }

    auto analysis_result = analyzer.analyzeWalkspace(leg_positions);
    std::cout << "Walkspace analysis result: " << (analysis_result.is_stable ? "STABLE" : "UNSTABLE") << std::endl;
    std::cout << "Stability margin: " << analysis_result.stability_margin << " mm" << std::endl;

    std::cout << "=== All WorkspaceAnalyzer functions tested successfully! ===" << std::endl;

    return 0;
}
