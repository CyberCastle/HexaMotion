#ifndef GAIT_CONFIG_H
#define GAIT_CONFIG_H

#include "gait_types.h" // Centralized gait enum
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <array>
#include <cassert>
#include <cmath>
#include <map>
#include <string>
#include <vector>

/**
 * @file gait_config.h
 * @brief OpenSHC-equivalent gait configuration data structures
 *
 * This implementation follows OpenSHC's gait parameter system where:
 * - Each gait type has specific stance_phase, swing_phase, and phase_offset parameters
 * - Leg offset multipliers define the phase timing for each leg
 * - Configuration parameters match OpenSHC's gait.yaml structure
 */

/**
 * @brief Step cycle timing parameters (OpenSHC equivalent)
 */
struct StepCycle {
    double frequency_;  //< Step frequency in Hz
    int period_;        //< Total step cycle length in iterations
    int swing_period_;  //< Swing period length in iterations
    int stance_period_; //< Stance period length in iterations
    int stance_end_;    //< Iteration when stance period ends
    int swing_start_;   //< Iteration when swing period starts
    int swing_end_;     //< Iteration when swing period ends
    int stance_start_;  //< Iteration when stance period starts
};

/**
 * @brief Gait phase configuration (OpenSHC equivalent)
 * Defines the timing parameters for stance and swing phases
 */
struct GaitPhaseConfig {
    int stance_phase; //< Length of stance phase in iterations
    int swing_phase;  //< Length of swing phase in iterations
    int phase_offset; //< Phase offset between legs
};

/**
 * @brief Leg offset multipliers for phase timing (OpenSHC equivalent)
 * Maps each leg to its phase offset multiplier for gait coordination
 */
struct LegOffsetMultipliers {
    std::map<std::string, int> multipliers; //< Leg name to offset multiplier mapping

    // Get multiplier for leg index (0-5)
    int getForLegIndex(int leg_index) const {
        const std::string leg_names[] = {"AR", "BR", "CR", "CL", "BL", "AL"};
        if (leg_index >= 0 && leg_index < NUM_LEGS) {
            return multipliers.count(leg_names[leg_index]) ? multipliers.at(leg_names[leg_index]) : 0;
        }
        return 0;
    }
};

/**
 * @brief Complete gait configuration for a specific gait type
 * Equivalent to OpenSHC's gait.yaml configuration structure
 */
struct GaitConfiguration {
    std::string gait_name;        //< Name of the gait (e.g., "tripod_gait", "wave_gait")
    GaitType gait_type;           //< Type of gait from GaitType enum
    GaitPhaseConfig phase_config; //< Phase timing configuration
    LegOffsetMultipliers offsets; //< Leg offset multipliers

    // Gait-specific parameters
    double step_length;                //< Default step length in mm
    double swing_height;               //< Swing trajectory height in mm
    double step_depth;                 //< Reactive step depth in mm
    double body_clearance;             //< Body clearance above ground in mm
    double stance_span_modifier = 0.0; // Modificador de span lateral de apoyo (OpenSHC compatible)

    // OpenSHC trajectory parameters
    double swing_width;    //< Lateral shift at mid-swing position in mm (OpenSHC mid_lateral_shift)
    double step_frequency; //< Step frequency in Hz (OpenSHC default: 1.0 Hz)
    double time_delta;     //< Control loop timestep in seconds (copied from Parameters)

    // Gait performance parameters
    double stability_factor;     //< Stability factor (0.0-1.0, higher = more stable)
    bool supports_rough_terrain; //< Whether gait supports rough terrain adaptation

    // Generate StepCycle using stored configuration (OpenSHC-style normalization)
    StepCycle generateStepCycle() const {
        StepCycle step_cycle{};
        int base_step_period = phase_config.stance_phase + phase_config.swing_phase;
        if (time_delta <= 0.0 || base_step_period <= 0 || step_frequency <= 0.0) {
            step_cycle.period_ = 0;
            step_cycle.frequency_ = 0.0;
            return step_cycle;
        }

        step_cycle.stance_end_ = static_cast<int>(phase_config.stance_phase * 0.5);
        step_cycle.swing_start_ = step_cycle.stance_end_;
        step_cycle.swing_end_ = step_cycle.swing_start_ + phase_config.swing_phase;
        step_cycle.stance_start_ = step_cycle.swing_end_;

        double swing_ratio = double(phase_config.swing_phase) / double(base_step_period);
        double raw_step_period = ((1.0 / step_frequency) / time_delta) / swing_ratio;
        int even_normaliser = math_utils::roundToEvenInt(raw_step_period / base_step_period);
        if (even_normaliser < 1) {
            even_normaliser = 1;
        }

        step_cycle.period_ = even_normaliser * base_step_period;
        step_cycle.frequency_ = 1.0 / (step_cycle.period_ * time_delta);

        int normaliser = step_cycle.period_ / base_step_period;
        step_cycle.stance_end_ *= normaliser;
        step_cycle.swing_start_ *= normaliser;
        step_cycle.swing_end_ *= normaliser;
        step_cycle.stance_start_ *= normaliser;

        step_cycle.stance_period_ = math_utils::mod(step_cycle.stance_end_ - step_cycle.stance_start_, step_cycle.period_);
        step_cycle.swing_period_ = step_cycle.swing_end_ - step_cycle.swing_start_;

        /** OpenSHC parity: assert stance and swing periods are even. */
        assert(step_cycle.stance_period_ % 2 == 0);
        assert(step_cycle.swing_period_ % 2 == 0);

        return step_cycle;
    }

    // Helper methods for velocity limits compatibility
    double getStanceRatio() const {
        return (double)phase_config.stance_phase /
               (phase_config.stance_phase + phase_config.swing_phase);
    }

    double getSwingRatio() const {
        return (double)phase_config.swing_phase /
               (phase_config.stance_phase + phase_config.swing_phase);
    }

    double getStepFrequency() const {
        return step_frequency; // Configurable OpenSHC step frequency
    }
};

/**
 * @brief Gait execution parameters
 * Runtime parameters for gait execution
 */
struct GaitExecutionParams {
    double current_velocity;     //< Current walking velocity
    double current_step_length;  //< Current step length
    double current_swing_height; //< Current swing height
    bool rough_terrain_mode;     //< Whether rough terrain mode is active
    bool force_normal_touchdown; //< Whether to force normal touchdown

    // Timing parameters
    double time_delta;      //< Time delta for current iteration
    double gait_cycle_time; //< Total gait cycle time
    double stance_time;     //< Stance phase time
    double swing_time;      //< Swing phase time
};

#endif // GAIT_CONFIG_H