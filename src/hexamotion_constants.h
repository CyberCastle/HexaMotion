#ifndef HEXAMOTION_CONSTANTS_H
#define HEXAMOTION_CONSTANTS_H

/**
 * @file hexamotion_constants.h
 * @brief Global constants for HexaMotion robot system
 *
 * This file contains all the commonly used constants throughout the HexaMotion
 * system, including velocity control, angular limits, and robot geometry values.
 * Centralizing these constants improves maintainability and consistency.
 */

#define NUM_LEGS 6
// Degrees of freedom per leg and total DOF
#define DOF_PER_LEG 3
#define TOTAL_DOF (NUM_LEGS * DOF_PER_LEG)

// Numerical differentiation step for Jacobians
#define JACOBIAN_DELTA 0.001f

#include "math_utils.h"

// ========================================================================
// VELOCITY CONTROL CONSTANTS
// ========================================================================

// Servo speed limits and defaults
#define SERVO_SPEED_MIN 0.1     // Minimum servo speed multiplier
#define SERVO_SPEED_MAX 3.0     // Maximum servo speed multiplier
#define SERVO_SPEED_DEFAULT 1.0 // Default servo speed multiplier

// Velocity scaling limits
#define VELOCITY_SCALE_MIN 0.1 // Minimum velocity scaling factor
#define VELOCITY_SCALE_MAX 5.0 // Maximum velocity scaling factor

// Speed ratio limits
#define SPEED_RATIO_MIN 0.05            // Minimum speed ratio (5%)
#define SPEED_RATIO_MAX_VALIDATION 0.95 // Maximum for minimum_speed_ratio validation
#define SPEED_RATIO_MIN_VALIDATION 1.05 // Minimum for maximum_speed_ratio validation
#define SPEED_RATIO_MAX 3.0             // Maximum speed ratio (300%)

// Gait modifier limits
#define GAIT_MODIFIER_MIN 0.1 // Minimum gait speed modifier
#define GAIT_MODIFIER_MAX 2.0 // Maximum gait speed modifier

// Leg compensation limits
#define LEG_COMPENSATION_MIN 0.5 // Minimum leg speed compensation
#define LEG_COMPENSATION_MAX 2.0 // Maximum leg speed compensation

// Default velocity scaling factors
#define DEFAULT_LINEAR_VELOCITY_SCALE 2.0  // Default linear velocity scaling factor
#define DEFAULT_ANGULAR_VELOCITY_SCALE 1.5 // Default angular velocity scaling factor
#define DEFAULT_MIN_SPEED_RATIO 0.2        // Default minimum speed ratio (20%)
#define DEFAULT_MAX_SPEED_RATIO 1.8        // Default maximum speed ratio (180%)

// Default gait speed factors
#define DEFAULT_TRIPOD_SPEED_FACTOR 1.2      // Default tripod gait speed factor
#define DEFAULT_WAVE_SPEED_FACTOR 0.8        // Default wave gait speed factor
#define DEFAULT_RIPPLE_SPEED_FACTOR 0.9      // Default ripple gait speed factor
#define DEFAULT_METACHRONAL_SPEED_FACTOR 1.0 // Default metachronal gait speed factor
#define DEFAULT_ADAPTIVE_SPEED_FACTOR 1.1    // Default adaptive gait speed factor

// Velocity multipliers for different joints
#define FEMUR_VELOCITY_MULTIPLIER 1.1 // Femur velocity scaling multiplier
#define TIBIA_VELOCITY_MULTIPLIER 1.2 // Tibia velocity scaling multiplier

// ========================================================================
// ANGULAR POSITION CONSTANTS
// ========================================================================

// Joint angle limits (degrees)
#define TIBIA_ANGLE_MAX 45.0 // Maximum tibia angle (degrees)

// ========================================================================
// ROBOT PARAMETER DEFAULTS
// ========================================================================

// Default velocity limits
#define DEFAULT_MAX_LINEAR_VELOCITY 200.0 // Default maximum linear velocity (mm/s)
#define DEFAULT_MAX_ANGULAR_VELOCITY 90.0 // Default maximum angular velocity (degrees/s)

// ========================================================================
// GAIT CONFIGURATION CONSTANTS (OpenSHC equivalent)
// ========================================================================

// Step length factors as percentage of leg reach (OpenSHC equivalent)
#define GAIT_TRIPOD_LENGTH_FACTOR 0.35      // 35% for speed (OpenSHC: balanced speed/stability)
#define GAIT_WAVE_LENGTH_FACTOR 0.25        // 25% for stability (OpenSHC: most stable)
#define GAIT_RIPPLE_LENGTH_FACTOR 0.30      // 30% for balance (OpenSHC: faster movement)
#define GAIT_METACHRONAL_LENGTH_FACTOR 0.28 // 28% for smooth motion (HexaMotion specific)
#define GAIT_ADAPTIVE_LENGTH_FACTOR 0.30    // 30% base for adaptation (HexaMotion specific)

// Step height factors as percentage of robot height (OpenSHC equivalent)
#define GAIT_TRIPOD_HEIGHT_FACTOR 0.30      // 30% for obstacle clearance
#define GAIT_WAVE_HEIGHT_FACTOR 0.20        // 20% for stability
#define GAIT_RIPPLE_HEIGHT_FACTOR 0.25      // 25% for balance
#define GAIT_METACHRONAL_HEIGHT_FACTOR 0.25 // 25% for smooth motion
#define GAIT_ADAPTIVE_HEIGHT_FACTOR 0.25    // 25% base for adaptation

// ========================================================================
// BODY POSE CONFIGURATION CONSTANTS (OpenSHC equivalent)
// ========================================================================

// This is gait-independent, unlike gait-specific step heights
#define BODY_POSE_DEFAULT_SWING_HEIGHT_FACTOR 0.10 // 10% of standing height (OpenSHC: 0.020m for typical robot)

// Velocity scaling and coupling factors
#define DEFAULT_ANGULAR_SCALING 1.0        // Default angular velocity scaling
#define ANGULAR_ACCELERATION_FACTOR 2000.0 // Angular acceleration threshold (mm/s²)
#define WORKSPACE_SCALING_FACTOR 0.5       // Workspace scaling factor
#define MIN_SERVO_VELOCITY 0.1             // Minimum servo velocity (10% of max)
#define FULL_ROTATION_DEGREES 360.0        // Full rotation in degrees
#define HALF_ROTATION_DEGREES 180.0        // Half rotation in degrees

// Directional efficiency attenuation parameters (tunable)
// Exponent > 1 accentuates attenuation for off-axis bearings while preserving near-axis efficiency.
// Floor prevents pathological collapse of effective workspace when a subset of legs are misaligned.
#define DIRECTIONAL_EFFICIENCY_EXPONENT 1.4
#define DIRECTIONAL_EFFICIENCY_FLOOR 0.15

// ========================================================================
// INVERSE KINEMATICS CONSTANTS
// ========================================================================

// DLS (Damped Least Squares) IK parameters (OpenSHC-style)
#define IK_DLS_COEFFICIENT 0.02        // Damping factor for numerical stability in DLS method
#define IK_JOINT_LIMIT_COST_WEIGHT 0.1 // Gain used in determining cost weight for joints approaching limits (OpenSHC)
#define IK_TOLERANCE 1.0               // Position tolerance for IK convergence (1mm)
#define IK_DEFAULT_MAX_ITERATIONS 30   // Default maximum iterations for IK solver
#define IK_MAX_ANGLE_STEP 5.0          // Maximum angle change per IK iteration (degrees)

// Workspace analysis parameters (OpenSHC equivalent, converted to mm)
#define MAX_POSITION_DELTA 100.0   // Position delta increment for workspace generation (2mm, OpenSHC uses 0.002m)
#define MAX_WORKSPACE_RADIUS 359.0 // Maximum radius based on robot morphology: coxa(50) + femur(101) + tibia(208) = 359mm
#define BEARING_STEP 45            // Bearing step for workspace generation (45°, from OpenSHC)
#define WORKSPACE_LAYERS 5         // Number of height layers for workspace generation (from OpenSHC)
#define WORKSPACE_RESOLUTION 10    // Discretization resolution for workspace analysis

// ========================================================================
// TERRAIN ADAPTATION CONSTANTS
// ========================================================================

// Default FSR threshold values for terrain contact detection
// Normalized (0-1) default FSR contact thresholds (rolling average values)
#define DEFAULT_FSR_TOUCHDOWN_THRESHOLD 0.7 // Default normalized touchdown (enter contact)
#define DEFAULT_FSR_LIFTOFF_THRESHOLD 0.3   // Default normalized liftoff (exit contact)

// Terrain analysis parameters
#define STEP_DEPTH_DEFAULT 20.0   // Default step depth for terrain analysis (mm)
#define STEP_PLANE_CONFIDENCE 0.8 // Default confidence level for step plane validation

// ========================================================================
// TOLERANCE AND PRECISION CONSTANTS
// ========================================================================

#define POSITION_TOLERANCE 1.0 // Position tolerance (mm)
#define VELOCITY_THRESHOLD 1.0 // Minimum velocity to consider as moving (mm/s)

// Tip position tolerance for determining if a leg tip is at target
// HexaMotion uses millimeters
#define TIP_TOLERANCE 5.0 // mm

// Admittance safety limit (maximum absolute compliance delta per axis in mm)
#define ADMITTANCE_MAX_ABS_DELTA_MM 50.0

// ========================================================================
// SYSTEM STATE CONSTANTS
// ========================================================================

// System states (OpenSHC equivalent)
enum SystemState {
    SYSTEM_UNKNOWN = 0, // Unknown state
    SYSTEM_PACKED = 1,  // Robot is packed/disabled
    SYSTEM_READY = 2,   // Robot is ready but not walking
    SYSTEM_RUNNING = 3  // Robot is running/walking
};

// Progress constants
#define PROGRESS_COMPLETE 100 // Sequence completed

// ========================================================================
// STANCE CONFIGURATION CONSTANTS (OpenSHC equivalent)
// ========================================================================

// Per-leg base orientation offsets in radians - symmetric for opposite leg pairs
// Pairs (index tuples): (0,3)=(-30°,30°), (1,4)=(-90°,90°), (2,5)=(-150°,150°)
// Correct ordering restores phi_i + phi_j ≈ 0 in local frame for opposite legs
// and aligns with DH parameter orientation used elsewhere (see tests:
// coxa_tripod_symmetry_analytic_test expectations).
const double BASE_THETA_OFFSETS[NUM_LEGS] = {
    math_utils::degreesToRadians(-30.0),  // Leg 0 (AR)
    math_utils::degreesToRadians(-90.0),  // Leg 1 (BR)
    math_utils::degreesToRadians(-150.0), // Leg 2 (CR)
    math_utils::degreesToRadians(30.0),   // Leg 3 (CL)
    math_utils::degreesToRadians(90.0),   // Leg 4 (BL)
    math_utils::degreesToRadians(150.0)   // Leg 5 (AL)
};

#endif // HEXAMOTION_CONSTANTS_H
