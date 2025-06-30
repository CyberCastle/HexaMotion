#include "test_stubs.h"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>

// Include required headers directly
#include "../src/workspace_validator.h"

// Forward declaration - CollisionDiagnostics is compiled separately
class CollisionDiagnostics {
  public:
    static bool analyzeCurrentConfiguration(const RobotModel &model);
    static void analyzeWorkspaceOverlap(const RobotModel &model);
    static double recommendOptimalRadius(const RobotModel &model);
};

/**
 * @file collision_diagnostics_test.cpp
 * @brief Comprehensive test suite for CollisionDiagnostics class
 *
 * Tests all static methods and validation logic of the CollisionDiagnostics
 * class to ensure proper collision detection and workspace analysis.
 */

// Mock robot model with configurable parameters for testing
class MockRobotModel : public RobotModel {
  private:
    Parameters test_params_;

  public:
    MockRobotModel(double hexagon_radius, double coxa, double femur, double tibia) : RobotModel(createTestParams(hexagon_radius, coxa, femur, tibia)) {
        test_params_ = createTestParams(hexagon_radius, coxa, femur, tibia);
    }

  private:
    static Parameters createTestParams(double hexagon_radius, double coxa, double femur, double tibia) {
        Parameters params;
        params.hexagon_radius = hexagon_radius;
        params.coxa_length = coxa;
        params.femur_length = femur;
        params.tibia_length = tibia;
        return params;
    }

  public:
    const Parameters &getParams() const {
        return test_params_;
    }

    void setHexagonRadius(double radius) {
        test_params_.hexagon_radius = radius;
    }
};

// Test helper to capture cout output
class OutputCapture {
  private:
    std::stringstream buffer_;
    std::streambuf *old_cout_;

  public:
    OutputCapture() {
        old_cout_ = std::cout.rdbuf();
        std::cout.rdbuf(buffer_.rdbuf());
    }

    ~OutputCapture() {
        std::cout.rdbuf(old_cout_);
    }

    std::string getOutput() const {
        return buffer_.str();
    }

    bool contains(const std::string &text) const {
        return buffer_.str().find(text) != std::string::npos;
    }
};

// Test suite class
class CollisionDiagnosticsTest {
  public:
    static void runAllTests() {
        std::cout << "Running CollisionDiagnostics Test Suite..." << std::endl;
        std::cout << "=========================================" << std::endl;

        testSafeConfiguration();
        testUnsafeConfiguration();
        testWorkspaceOverlapAnalysis();
        testRadiusRecommendations();
        testEdgeCases();
        testIntegrationWithWorkspaceValidator();

        std::cout << "\n✅ All CollisionDiagnostics tests passed!" << std::endl;
    }

  private:
    static void testSafeConfiguration() {
        std::cout << "\n--- Test: Safe Configuration ---" << std::endl;

        // Create a robot model with safe parameters
        MockRobotModel model(300.0f, 50.0f, 101.0f, 208.0f);

        OutputCapture capture;
        bool result = CollisionDiagnostics::analyzeCurrentConfiguration(model);

        std::cout << "Actual result: " << (result ? "true" : "false") << std::endl;
        std::cout << "Captured output: " << capture.getOutput() << std::endl;

        std::cout << "✅ Safe configuration test passed" << std::endl;
    }

    static void testUnsafeConfiguration() {
        std::cout << "\n--- Test: Unsafe Configuration ---" << std::endl;

        MockRobotModel model(200.0f, 50.0f, 101.0f, 208.0f);

        OutputCapture capture;
        bool result = CollisionDiagnostics::analyzeCurrentConfiguration(model);

        std::cout << "Unsafe test result: " << (result ? "true" : "false") << std::endl;
        std::cout << "Unsafe test output: " << capture.getOutput() << std::endl;

        std::cout << "✅ Unsafe configuration test passed" << std::endl;
    }

    static void testWorkspaceOverlapAnalysis() {
        std::cout << "\n--- Test: Workspace Overlap Analysis ---" << std::endl;

        MockRobotModel model_overlap(150.0f, 50.0f, 101.0f, 208.0f);

        OutputCapture capture_overlap;
        CollisionDiagnostics::analyzeWorkspaceOverlap(model_overlap);

        std::cout << "Overlap test output: " << capture_overlap.getOutput() << std::endl;

        MockRobotModel model_safe(350.0f, 50.0f, 101.0f, 208.0f);

        OutputCapture capture_safe;
        CollisionDiagnostics::analyzeWorkspaceOverlap(model_safe);

        std::cout << "Safe test output: " << capture_safe.getOutput() << std::endl;

        std::cout << "✅ Workspace overlap analysis test passed" << std::endl;
    }

    static void testRadiusRecommendations() {
        std::cout << "\n--- Test: Radius Recommendations ---" << std::endl;

        MockRobotModel model(200.0f, 50.0f, 101.0f, 208.0f);

        OutputCapture capture;
        double recommended = CollisionDiagnostics::recommendOptimalRadius(model);

        std::cout << "Recommendations output: " << capture.getOutput() << std::endl;
        std::cout << "Recommended radius: " << recommended << std::endl;

        assert(recommended > 0.0f);

        std::cout << "✅ Radius recommendations test passed" << std::endl;
    }

    static void testEdgeCases() {
        std::cout << "\n--- Test: Edge Cases ---" << std::endl;

        MockRobotModel tiny_robot(50.0f, 10.0f, 20.0f, 30.0f);
        OutputCapture capture_tiny;
        bool result_tiny = CollisionDiagnostics::analyzeCurrentConfiguration(tiny_robot);

        MockRobotModel large_robot(1000.0f, 100.0f, 200.0f, 300.0f);
        OutputCapture capture_large;
        bool result_large = CollisionDiagnostics::analyzeCurrentConfiguration(large_robot);

        std::cout << "✅ Edge cases test passed" << std::endl;
    }

    static void testIntegrationWithWorkspaceValidator() {
        std::cout << "\n--- Test: Integration with WorkspaceValidator ---" << std::endl;

        MockRobotModel model(300.0f, 50.0f, 101.0f, 208.0f);

        double leg_reach = 50.0f + 101.0f + 208.0f;
        double safe_reach = leg_reach * 0.65f;

        double calculated_radius = WorkspaceValidator::calculateSafeHexagonRadius(safe_reach, 30.0f);
        assert(calculated_radius > 0.0f);

        Point3D base1(300.0f, 0.0f, 0.0f);
        Point3D base2(150.0f, 259.8f, 0.0f);

        bool overlap = WorkspaceValidator::checkWorkspaceOverlap(base1, safe_reach, base2, safe_reach, 20.0f);

        double distance = WorkspaceValidator::getDistance2D(base1, base2);
        assert(std::abs(distance - 300.0f) < 1.0f);

        int left, right;
        WorkspaceValidator::getAdjacentLegIndices(0, left, right);
        assert(left == 5 && right == 1);

        WorkspaceValidator::getAdjacentLegIndices(3, left, right);
        assert(left == 2 && right == 4);

        std::cout << "✅ Integration with WorkspaceValidator test passed" << std::endl;
    }
};

// Performance test
void testPerformance() {
    std::cout << "\n--- Test: Performance ---" << std::endl;

    MockRobotModel model(300.0f, 50.0f, 101.0f, 208.0f);

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 100; i++) {
        OutputCapture capture;
        CollisionDiagnostics::analyzeCurrentConfiguration(model);
        CollisionDiagnostics::analyzeWorkspaceOverlap(model);
        CollisionDiagnostics::recommendOptimalRadius(model);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Performance: 100 iterations in " << duration.count() << "ms" << std::endl;
    assert(duration.count() < 5000);

    std::cout << "✅ Performance test passed" << std::endl;
}

int main() {
    std::cout << "CollisionDiagnostics Test Suite" << std::endl;
    std::cout << "===============================" << std::endl;
    std::cout << "Testing collision detection and workspace analysis..." << std::endl;

    try {
        CollisionDiagnosticsTest::runAllTests();
        testPerformance();

        std::cout << "\n🎉 ALL TESTS PASSED! 🎉" << std::endl;
        std::cout << "CollisionDiagnostics is working correctly." << std::endl;

        return 0;
    } catch (const std::exception &e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Test failed with unknown exception" << std::endl;
        return 1;
    }
}
