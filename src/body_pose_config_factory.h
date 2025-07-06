#ifndef BODY_POSE_CONFIG_FACTORY_H
#define BODY_POSE_CONFIG_FACTORY_H

#include "robot_model.h"
#include "body_pose_config.h"

/**
 * @file body_pose_config_factory.h
 * @brief Factory functions for creating body pose configurations
 *
 * This module separates the pose configuration creation logic from the data structures,
 * allowing pose_controller to only depend on data structures defined in pose_config.h
 */

/**
 * @brief Calculate hexagonal leg stance positions based on robot parameters
 * @param params Robot parameters containing dimensions and joint limits
 * @return Array of calculated stance positions in millimeters
 */
std::array<LegStancePosition, NUM_LEGS> calculateHexagonalStancePositions(const Parameters &params);

/**
 * @brief Get default standing pose joint angles (OpenSHC equivalent)
 * For a hexagonal robot in static equilibrium, this should return:
 * - Coxa ≈ 0° (radially aligned with mounting)
 * - Femur and Tibia calculated using inverse kinematics for target height
 * @param params Robot parameters containing dimensions and joint limits
 * @return Array of standing pose joint configurations
 */
std::array<StandingPoseJoints, NUM_LEGS> getDefaultStandingPoseJoints(const Parameters &params);

/**
 * @brief Get default body pose configuration using robot parameters (OpenSHC equivalent)
 * @param params Robot parameters from HexaModel
 * @return Default body pose configuration calculated from robot specifications
 */
BodyPoseConfiguration getDefaultBodyPoseConfig(const Parameters &params);

/**
 * @brief Get conservative body pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return Conservative body pose configuration with reduced limits for safety
 */
BodyPoseConfiguration getConservativeBodyPoseConfig(const Parameters &params);

/**
 * @brief Get high-speed body pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return High-speed body pose configuration optimized for faster locomotion
 */
BodyPoseConfiguration getHighSpeedBodyPoseConfig(const Parameters &params);

/**
 * @brief Create auto-pose configuration for tripod gait (OpenSHC equivalent)
 * Based on OpenSHC's auto_pose.yaml configuration
 * @param params Robot parameters
 * @return Auto-pose configuration structure
 */
AutoPoseConfiguration createAutoPoseConfiguration(const Parameters &params);

/**
 * @brief Create conservative auto-pose configuration
 * @param params Robot parameters
 * @return Conservative auto-pose configuration with reduced amplitudes
 */
AutoPoseConfiguration createConservativeAutoPoseConfiguration(const Parameters &params);

/**
 * @brief Create high-speed auto-pose configuration
 * @param params Robot parameters
 * @return High-speed auto-pose configuration with increased amplitudes
 */
AutoPoseConfiguration createHighSpeedAutoPoseConfiguration(const Parameters &params);

#endif // BODY_POSE_CONFIG_FACTORY_H
