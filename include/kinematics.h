#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>
#include <stddef.h>

// --- CORE CONSTANTS (Simple definition for use in C files) ---
// Define PI as a macro or const variable. We'll use const for type safety.
extern const float PI_CONST;

// --- STRUCTURE DEFINITION ---
// This struct holds all robot dimensions, calculated constants, and runtime
// state.
typedef struct
{
        // --- IMMUTABLE CONSTANTS (Calculated at Runtime, but fixed) ---
        // Physical Dimensions (Read-only once initialized)
        float length_a, length_b, length_c;
        float length_side, z_absolute;

        // Movement Parameters (Read-only once initialized)
        float z_default, z_up, z_boot;
        float x_default, x_offset;
        float y_start, y_step;

        // Derived Turn Constants (Calculated in init function)
        float temp_a, temp_b, temp_c;
        float temp_alpha;
        float turn_x0, turn_y0, turn_x1, turn_y1;

        // Speed Constants
        float speed_multiple;
        float spot_turn_speed;
        float leg_move_speed;

        float body_move_speed;
        float stand_seat_speed;

        // --- MUTABLE STATE (Volatile variables used by Control Thread) ---
        volatile float site_now[4][3];    // Real-time coordinates
        volatile float site_expect[4][3]; // Expected coordinates

        float temp_speed[4][3]; // Each axis' speed
        volatile int rest_counter;
        float move_speed;

        // Marker to ensure initialization has run
        bool initialized;

} robot_kinematics_t;

// --- GLOBAL INSTANCE DECLARATION ---
// This is the single global object that everything will access.
extern robot_kinematics_t g_kinematics;

// --- PUBLIC FUNCTIONS ---
// Function to perform all runtime calculations and state initialization
void kinematics_init(void);
void kinematics_print_debug(void);

#endif
