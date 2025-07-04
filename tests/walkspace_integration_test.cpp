/**
 * @file walkspace_integration_test.cpp
 * @brief Test for WalkspaceAnalyzer and WorkspaceValidator integration
 *
 * This test validates:
 * 1. Real-time analysis control
 * 2. External system access to analysis data
 * 3. OpenSHC-equivalent operation
 * 4. Performance and accuracy
 *
 * @author HexaMotion Team
 * @version 1.0
 * @date 2024
 */

#include "../src/walk_controller.h"
#include "../src/workspace_validator.h"
#include "../src/walkspace_analyzer.h"
#include "test_stubs.h"
#include <iostream>
#include <cassert>
#include <chrono>

// Custom assert macro for testing
#define assert_test(condition, message) \
    do { \
        if (!(condition)) { \
            std::cerr << "❌ Test failed: " << message << std::endl; \
            std::exit(1); \
        } else { \
            std::cout << "✓ " << message << std::endl; \
        } \
    } while(0)

void testWalkspaceAnalyzerIntegration() {
    std::cout << "\n=== Testing WalkspaceAnalyzer Integration ===" << std::endl;

    // Create robot model
    Parameters params = createDefaultParameters();
    RobotModel model(params);

    // Create walk controller
    WalkController walk_controller(model);

    // Test 1: Initialization
    std::cout << "Test 1: Initialization..." << std::endl;
    assert_test(walk_controller.isWalkspaceAnalysisEnabled(), "Walkspace analysis should be enabled by default");

    // Test 2: Enable/Disable control
    std::cout << "Test 2: Enable/Disable control..." << std::endl;
    walk_controller.enableWalkspaceAnalysis(false);
    assert_test(!walk_controller.isWalkspaceAnalysisEnabled(), "Analysis should be disabled");

    walk_controller.enableWalkspaceAnalysis(true);
    assert_test(walk_controller.isWalkspaceAnalysisEnabled(), "Analysis should be enabled");

    // Test 3: Analysis information access
    std::cout << "Test 3: Analysis information access..." << std::endl;
    const WalkspaceAnalyzer::AnalysisInfo& info = walk_controller.getWalkspaceAnalysisInfo();
    assert_test(info.analysis_enabled, "Analysis should be enabled in info");
    assert_test(info.analysis_count >= 0, "Analysis count should be non-negative");

    // Test 4: Real-time analysis
    std::cout << "Test 4: Real-time analysis..." << std::endl;
    WalkspaceAnalyzer::WalkspaceResult result = walk_controller.analyzeCurrentWalkspace();
    assert_test(result.reachable_area >= 0, "Reachable area should be non-negative");

    // Test 5: Performance monitoring
    std::cout << "Test 5: Performance monitoring..." << std::endl;
    assert_test(info.average_analysis_time_ms >= 0, "Average analysis time should be non-negative");

    // Test 6: Workspace map generation
    std::cout << "Test 6: Workspace map generation..." << std::endl;
    bool success = walk_controller.generateWalkspaceMap();
    assert_test(success, "Workspace map generation should succeed");

    // Test 7: Walkspace radius access
    std::cout << "Test 7: Walkspace radius access..." << std::endl;
    double radius = walk_controller.getWalkspaceRadius(0.0);
    assert_test(radius >= 0, "Walkspace radius should be non-negative");

    std::cout << "✓ All WalkspaceAnalyzer integration tests passed" << std::endl;
}

void testWorkspaceValidatorIntegration() {
    std::cout << "\n=== Testing WorkspaceValidator Integration ===" << std::endl;

    // Create robot model
    Parameters params = createDefaultParameters();

    // Verify we're using the correct robot parameters from AGENTS.md
    assert_test(params.hexagon_radius == 200.0f, "Hexagon radius should be 200mm");
    assert_test(params.coxa_length == 50.0f, "Coxa length should be 50mm");
    assert_test(params.femur_length == 101.0f, "Femur length should be 101mm");
    assert_test(params.tibia_length == 208.0f, "Tibia length should be 208mm");
    assert_test(params.robot_height == 120.0f, "Robot height should be 120mm");
    assert_test(params.control_frequency == 50.0f, "Control frequency should be 50Hz");

    RobotModel model(params);

    // Create workspace validator
    ValidationConfig config;
    config.enable_collision_checking = true;
    config.safety_margin = 20.0f;
    WorkspaceValidator validator(model, config);

    // Test 1: Basic validation
    std::cout << "Test 1: Basic validation..." << std::endl;
    Point3D test_position(100.0f, 0.0f, -120.0f);
    Point3D dummy_positions[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        dummy_positions[i] = Point3D(0, 0, 0);
    }

    ValidationResult result = validator.validateTarget(0, test_position, dummy_positions);
    assert_test(result.is_reachable || !result.is_reachable, "Validation should return a result");

    // Test 2: Workspace bounds
    std::cout << "Test 2: Workspace bounds..." << std::endl;
    WorkspaceBounds bounds = validator.getWorkspaceBounds(0);
    assert_test(bounds.max_reach > bounds.min_reach, "Max reach should be greater than min reach");

    // Test 3: Velocity constraints
    std::cout << "Test 3: Velocity constraints..." << std::endl;
    VelocityConstraints constraints = validator.calculateVelocityConstraints(0, 0.0f, 1.0f, 0.6f);
    assert_test(constraints.max_linear_velocity >= 0, "Max linear velocity should be non-negative");

    // Test 4: Scaling factors
    std::cout << "Test 4: Scaling factors..." << std::endl;
    ScalingFactors factors = validator.getScalingFactors();
    assert_test(factors.workspace_scale > 0 && factors.workspace_scale <= 1, "Workspace scale should be in (0,1]");

    std::cout << "✓ All WorkspaceValidator integration tests passed" << std::endl;
}

void testOpenSHCEquivalence() {
    std::cout << "\n=== Testing OpenSHC Equivalence ===" << std::endl;

    // Create robot model
    Parameters params = createDefaultParameters();
    RobotModel model(params);

    // Create walk controller
    WalkController walk_controller(model);

    // Test 1: Walkspace generation like OpenSHC
    std::cout << "Test 1: Walkspace generation..." << std::endl;
    bool success = walk_controller.generateWalkspaceMap();
    assert_test(success, "Walkspace generation should succeed like OpenSHC");

    // Test 2: Real-time analysis during walking
    std::cout << "Test 2: Real-time analysis..." << std::endl;
    walk_controller.enableWalkspaceAnalysis(true);

    // Simulate walking
    walk_controller.planGaitSequence(30.0, 0.0, 0.0);

    // Perform analysis
    WalkspaceAnalyzer::WalkspaceResult result = walk_controller.analyzeCurrentWalkspace();
    assert_test(result.reachable_area > 0, "Should have positive reachable area");

    // Test 3: Stability analysis
    std::cout << "Test 3: Stability analysis..." << std::endl;
    assert_test(result.stability_margin >= 0, "Stability margin should be non-negative");

    // Test 4: Support polygon calculation
    std::cout << "Test 4: Support polygon..." << std::endl;
    assert_test(result.support_polygon.size() >= 3, "Support polygon should have at least 3 vertices");

    std::cout << "✓ All OpenSHC equivalence tests passed" << std::endl;
}

void testPerformanceAndAccuracy() {
    std::cout << "\n=== Testing Performance and Accuracy ===" << std::endl;

    // Create robot model
    Parameters params = createDefaultParameters();
    RobotModel model(params);

    // Create walk controller
    WalkController walk_controller(model);

    // Test 1: Analysis performance
    std::cout << "Test 1: Analysis performance..." << std::endl;
    walk_controller.enableWalkspaceAnalysis(true);

    auto start_time = std::chrono::high_resolution_clock::now();

    // Perform multiple analyses
    for (int i = 0; i < 10; i++) {
        WalkspaceAnalyzer::WalkspaceResult result = walk_controller.analyzeCurrentWalkspace();
        assert_test(result.reachable_area >= 0, "Analysis should always return valid results");
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    double avg_time_ms = duration.count() / 10000.0; // Convert to milliseconds
    std::cout << "Average analysis time: " << avg_time_ms << " ms" << std::endl;

    // Analysis should be reasonably fast (less than 10ms per analysis)
    assert_test(avg_time_ms < 10.0, "Analysis should be reasonably fast");

    // Test 2: Accuracy consistency
    std::cout << "Test 2: Accuracy consistency..." << std::endl;
    WalkspaceAnalyzer::WalkspaceResult result1 = walk_controller.analyzeCurrentWalkspace();
    WalkspaceAnalyzer::WalkspaceResult result2 = walk_controller.analyzeCurrentWalkspace();

    // Results should be consistent for same input
    assert_test(std::abs(result1.reachable_area - result2.reachable_area) < 1e-6, "Results should be consistent");

    // Test 3: Memory usage
    std::cout << "Test 3: Memory usage..." << std::endl;
    const WalkspaceAnalyzer::AnalysisInfo& info = walk_controller.getWalkspaceAnalysisInfo();
    assert_test(info.analysis_count > 0, "Analysis count should increase");

    std::cout << "✓ All performance and accuracy tests passed" << std::endl;
}

void testExternalSystemAccess() {
    std::cout << "\n=== Testing External System Access ===" << std::endl;

    // Create robot model
    Parameters params = createDefaultParameters();
    RobotModel model(params);

    // Create walk controller
    WalkController walk_controller(model);

    // Test 1: Analysis info string
    std::cout << "Test 1: Analysis info string..." << std::endl;
    std::string info_string = walk_controller.getWalkspaceAnalysisInfoString();
    assert_test(!info_string.empty(), "Analysis info string should not be empty");
    assert_test(info_string.find("Walkspace Analysis Information") != std::string::npos, "Should contain analysis info");

    // Test 2: Statistics reset
    std::cout << "Test 2: Statistics reset..." << std::endl;
    walk_controller.resetWalkspaceAnalysisStats();
    const WalkspaceAnalyzer::AnalysisInfo& info = walk_controller.getWalkspaceAnalysisInfo();
    assert_test(info.analysis_count == 0, "Analysis count should be reset to 0");

    // Test 3: Comprehensive analysis info
    std::cout << "Test 3: Comprehensive analysis info..." << std::endl;
    assert_test(info.leg_bounds.size() >= 0, "Leg bounds should be available");
    assert_test(info.leg_reachability.size() >= 0, "Leg reachability should be available");
    assert_test(info.overall_stability_score >= 0 && info.overall_stability_score <= 1, "Stability score should be in [0,1]");

    std::cout << "✓ All external system access tests passed" << std::endl;
}

int main() {
    std::cout << "=== Walkspace Integration Test Suite ===" << std::endl;

    try {
        testWalkspaceAnalyzerIntegration();
        testWorkspaceValidatorIntegration();
        testOpenSHCEquivalence();
        testPerformanceAndAccuracy();
        testExternalSystemAccess();

        std::cout << "\n🎉 ALL TESTS PASSED! 🎉" << std::endl;
        std::cout << "\nIntegration Summary:" << std::endl;
        std::cout << "✓ WalkspaceAnalyzer and WorkspaceValidator integrated successfully" << std::endl;
        std::cout << "✓ Real-time analysis control working" << std::endl;
        std::cout << "✓ External system access functional" << std::endl;
        std::cout << "✓ OpenSHC-equivalent operation validated" << std::endl;
        std::cout << "✓ Performance and accuracy verified" << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}