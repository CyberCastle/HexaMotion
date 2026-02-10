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

/**
 * @brief Leg states for state machine control (OpenSHC equivalent)
 */
enum LegState {
    LEG_WALKING,                //< The leg is in a 'walking' state - participates in walking cycle
    LEG_MANUAL,                 //< The leg is in a 'manual' state - able to move via manual manipulation inputs
    LEG_STATE_COUNT,            //< Misc enum defining number of LegStates
    LEG_WALKING_TO_MANUAL = -1, //< The leg is in 'walking to manual' state
    LEG_MANUAL_TO_WALKING = -2, //< The leg is in 'manual to walking' state
};

/**
 * @brief Walk states for walk controller cycle (OpenSHC equivalent)
 */
enum WalkState {
    WALK_STARTING,    //< Transitioning from 'stopped' to 'moving'
    WALK_MOVING,      //< Primary walking state
    WALK_STOPPING,    //< Transitioning from 'moving' to 'stopped'
    WALK_STOPPED,     //< State whilst velocity input is zero
    WALK_STATE_COUNT, //< Misc enum defining number of Walk States
};

#endif // GAIT_TYPES_H
