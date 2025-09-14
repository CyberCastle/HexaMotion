// Dedicated header for gait type enumeration to avoid circular dependencies
#ifndef GAIT_TYPES_H
#define GAIT_TYPES_H

/**
 * @brief Enumeration of supported gait types.
 */
enum GaitType {
    NO_GAIT,
    TRIPOD_GAIT,
    WAVE_GAIT,
    RIPPLE_GAIT,
    METACHRONAL_GAIT,
    ADAPTIVE_GAIT
};

#endif // GAIT_TYPES_H
