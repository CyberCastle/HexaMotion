#ifndef PRECISION_CONFIG_H
#define PRECISION_CONFIG_H

#include <Arduino.h>

/**
 * @brief Configurable precision levels for computational complexity trade-offs
 * Equivalent to OpenSHC's adaptive computation system
 */
enum PrecisionLevel {
    PRECISION_LOW = 0,    ///< Fast, basic calculations (50Hz update, 5 iterations)
    PRECISION_MEDIUM = 1, ///< Balanced performance (100Hz update, 10 iterations)
    PRECISION_HIGH = 2    ///< Maximum accuracy (200Hz update, 20 iterations)
};

/**
 * @brief Computational complexity configuration
 */
struct ComputeConfig {
    PrecisionLevel precision; ///< Computation precision level
    uint8_t max_iterations;   ///< Maximum iterations for iterative algorithms
    double tolerance;          ///< Convergence tolerance
    uint16_t update_freq_hz;  ///< Update frequency in Hz
    bool use_approximations;  ///< Enable fast approximations

    static ComputeConfig low() {
        return {PRECISION_LOW, 5, 0.01f, 50, true};
    }

    static ComputeConfig medium() {
        return {PRECISION_MEDIUM, 10, 0.005f, 100, false};
    }

    static ComputeConfig high() {
        return {PRECISION_HIGH, 20, 0.001f, 200, false};
    }

    double getDeltaTime() const {
        return 1.0 / static_cast<double>(update_freq_hz);
    }
};

#endif // PRECISION_CONFIG_H
