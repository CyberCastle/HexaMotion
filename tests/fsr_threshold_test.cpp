/**
 * @file fsr_threshold_test.cpp
 * @brief Test for FSR threshold configuration validation
 */

#include "../src/terrain_adaptation.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>

int main() {
    std::cout << "FSR THRESHOLD CONFIGURATION TEST" << std::endl;
    std::cout << "=================================" << std::endl;

    // Test 1: Default thresholds when not configured
    std::cout << "\n=== Testing Default Thresholds ===" << std::endl;
    {
        Parameters params = createDefaultParameters();
        params.fsr_touchdown_threshold = 0.0f; // Not configured
        params.fsr_liftoff_threshold = 0.0f;   // Not configured

        RobotModel model(params);
        TerrainAdaptation terrain_adaptation(model);
        terrain_adaptation.initialize();

        // Should use default values from constants
        assert(terrain_adaptation.getTouchdownThreshold() == 10.0f); // DEFAULT_FSR_TOUCHDOWN_THRESHOLD
        assert(terrain_adaptation.getLiftoffThreshold() == 5.0f);    // DEFAULT_FSR_LIFTOFF_THRESHOLD

        std::cout << "✓ Default thresholds applied correctly" << std::endl;
        std::cout << "  Touchdown: " << terrain_adaptation.getTouchdownThreshold() << std::endl;
        std::cout << "  Liftoff: " << terrain_adaptation.getLiftoffThreshold() << std::endl;
    }

    // Test 2: Custom thresholds from model parameters
    std::cout << "\n=== Testing Custom Model Thresholds ===" << std::endl;
    {
        Parameters params = createDefaultParameters();
        params.fsr_touchdown_threshold = 25.0f; // Custom touchdown threshold
        params.fsr_liftoff_threshold = 12.0f;   // Custom liftoff threshold

        RobotModel model(params);
        TerrainAdaptation terrain_adaptation(model);
        terrain_adaptation.initialize();

        // Should use configured values
        assert(terrain_adaptation.getTouchdownThreshold() == 25.0f);
        assert(terrain_adaptation.getLiftoffThreshold() == 12.0f);

        std::cout << "✓ Custom thresholds from model applied correctly" << std::endl;
        std::cout << "  Touchdown: " << terrain_adaptation.getTouchdownThreshold() << std::endl;
        std::cout << "  Liftoff: " << terrain_adaptation.getLiftoffThreshold() << std::endl;
    }

    // Test 3: Dynamic threshold updates
    std::cout << "\n=== Testing Dynamic Threshold Updates ===" << std::endl;
    {
        Parameters params = createDefaultParameters();
        RobotModel model(params);
        TerrainAdaptation terrain_adaptation(model);
        terrain_adaptation.initialize();

        // Update thresholds dynamically
        terrain_adaptation.setTouchdownThreshold(30.0f);
        terrain_adaptation.setLiftoffThreshold(15.0f);

        assert(terrain_adaptation.getTouchdownThreshold() == 30.0f);
        assert(terrain_adaptation.getLiftoffThreshold() == 15.0f);

        std::cout << "✓ Dynamic threshold updates working correctly" << std::endl;
        std::cout << "  Touchdown: " << terrain_adaptation.getTouchdownThreshold() << std::endl;
        std::cout << "  Liftoff: " << terrain_adaptation.getLiftoffThreshold() << std::endl;
    }

    // Test 4: Partial configuration (only touchdown configured)
    std::cout << "\n=== Testing Partial Configuration ===" << std::endl;
    {
        Parameters params = createDefaultParameters();
        params.fsr_touchdown_threshold = 20.0f; // Only touchdown configured
        params.fsr_liftoff_threshold = 0.0f;    // Liftoff not configured

        RobotModel model(params);
        TerrainAdaptation terrain_adaptation(model);
        terrain_adaptation.initialize();

        // Should use configured touchdown and default liftoff
        assert(terrain_adaptation.getTouchdownThreshold() == 20.0f);
        assert(terrain_adaptation.getLiftoffThreshold() == 5.0f);

        std::cout << "✓ Partial configuration working correctly" << std::endl;
        std::cout << "  Touchdown: " << terrain_adaptation.getTouchdownThreshold() << " (configured)" << std::endl;
        std::cout << "  Liftoff: " << terrain_adaptation.getLiftoffThreshold() << " (default)" << std::endl;
    }

    // Test 5: Manual threshold update (equivalent to model sync)
    std::cout << "\n=== Testing Manual Threshold Updates ===" << std::endl;
    {
        Parameters params = createDefaultParameters();
        params.fsr_touchdown_threshold = 40.0f;
        params.fsr_liftoff_threshold = 20.0f;

        RobotModel model(params);
        TerrainAdaptation terrain_adaptation(model);
        terrain_adaptation.initialize();

        // Manually update thresholds to simulate model sync
        terrain_adaptation.setTouchdownThreshold(50.0f);
        terrain_adaptation.setLiftoffThreshold(25.0f);

        assert(terrain_adaptation.getTouchdownThreshold() == 50.0f);
        assert(terrain_adaptation.getLiftoffThreshold() == 25.0f);

        std::cout << "✓ Manual threshold updates working correctly" << std::endl;
        std::cout << "  Touchdown: " << terrain_adaptation.getTouchdownThreshold() << std::endl;
        std::cout << "  Liftoff: " << terrain_adaptation.getLiftoffThreshold() << std::endl;
    }

    std::cout << "\n=================================" << std::endl;
    std::cout << "✓ ALL FSR THRESHOLD TESTS PASSED" << std::endl;
    std::cout << "=================================" << std::endl;

    return 0;
}
