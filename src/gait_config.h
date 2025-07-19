#ifndef GAIT_CONFIG_H
#define GAIT_CONFIG_H

#include "hexamotion_constants.h"
#include <array>
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

    // Convenience accessors for each leg
    int getAR() const { return multipliers.count("AR") ? multipliers.at("AR") : 0; }
    int getBR() const { return multipliers.count("BR") ? multipliers.at("BR") : 0; }
    int getCR() const { return multipliers.count("CR") ? multipliers.at("CR") : 0; }
    int getCL() const { return multipliers.count("CL") ? multipliers.at("CL") : 0; }
    int getBL() const { return multipliers.count("BL") ? multipliers.at("BL") : 0; }
    int getAL() const { return multipliers.count("AL") ? multipliers.at("AL") : 0; }

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
    GaitPhaseConfig phase_config; //< Phase timing configuration
    LegOffsetMultipliers offsets; //< Leg offset multipliers
    StepCycle step_cycle;         //< StepCycle calculado y listo para usar

    // Gait-specific parameters
    double step_frequency;             //< Step frequency in Hz (calculated from phase config)
    double step_cycle_time;            //< Complete step cycle time in seconds (1.0 / step_frequency)
    double step_length;                //< Default step length in mm
    double swing_height;               //< Swing trajectory height in mm
    double body_clearance;             //< Body clearance above ground in mm
    double stance_span_modifier = 0.0; // Modificador de span lateral de apoyo (OpenSHC compatible)

    // Gait performance parameters
    double max_velocity;         //< Maximum walking velocity in mm/s
    double stability_factor;     //< Stability factor (0.0-1.0, higher = more stable)
    bool supports_rough_terrain; //< Whether gait supports rough terrain adaptation

    // Velocity limits parameters (unified configuration)
    double stance_ratio;       //< Ratio of stance phase (0.0 - 1.0)
    double swing_ratio;        //< Ratio of swing phase (0.0 - 1.0)
    double time_to_max_stride; //< Time to reach maximum stride (s)

    // Gait description
    std::string description;             //< Human-readable description of the gait
    std::vector<std::string> step_order; //< Order of leg movements in the gait

    // Helper methods for velocity limits compatibility
    double getStanceRatio() const {
        return (double)phase_config.stance_phase /
               (phase_config.stance_phase + phase_config.swing_phase);
    }

    double getSwingRatio() const {
        return (double)phase_config.swing_phase /
               (phase_config.stance_phase + phase_config.swing_phase);
    }
};

/**
 * @brief Gait selection configuration
 * Contains all available gaits and selection parameters
 */
struct GaitSelectionConfig {
    std::map<std::string, GaitConfiguration> available_gaits; //< All available gait configurations
    std::string default_gait;                                 //< Default gait to use
    std::string current_gait;                                 //< Currently selected gait

    // Gait transition parameters
    double transition_time;  //< Time to transition between gaits in seconds
    bool smooth_transitions; //< Whether to use smooth gait transitions

    // Gait selection criteria
    double min_stability_threshold; //< Minimum stability threshold for gait selection
    double max_velocity_threshold;  //< Maximum velocity threshold for gait selection
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