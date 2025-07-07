#ifndef GAIT_CONFIG_FACTORY_H
#define GAIT_CONFIG_FACTORY_H

#include "gait_config.h"
#include "leg_stepper.h"
#include <string>
#include <vector>

/**
 * @file gait_config_factory.h
 * @brief OpenSHC-equivalent gait configuration factory declarations
 *
 * This header provides factory functions to create gait configurations
 * that match OpenSHC's gait.yaml structure and parameters.
 */

// Gait configuration creation functions
GaitConfiguration createWaveGaitConfig(const Parameters& params);
GaitConfiguration createTripodGaitConfig(const Parameters& params);
GaitConfiguration createRippleGaitConfig(const Parameters& params);
GaitConfiguration createAmbleGaitConfig(const Parameters& params);
GaitConfiguration createMetachronalGaitConfig(const Parameters& params);

// Gait selection configuration
GaitSelectionConfig createGaitSelectionConfig();

// Convenience getter functions
GaitConfiguration getDefaultGaitConfig();
GaitConfiguration getConservativeGaitConfig();
GaitConfiguration getHighSpeedGaitConfig();
GaitConfiguration getAdaptiveGaitConfig();
GaitConfiguration getGaitConfigByName(const std::string& gait_name);

// Utility functions
StepCycle calculateStepCycleFromGait(const GaitConfiguration& gait_config, double time_delta = 0.01);
std::vector<std::string> getAvailableGaitNames();

#endif // GAIT_CONFIG_FACTORY_H