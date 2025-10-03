#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "zephyr/kernel.h"
#include <stdbool.h>
#include <stddef.h>

#define EPSILON 0.001

extern const float PI_CONST;
extern const float KEEP;
extern struct k_mutex g_state_mutex;
extern struct k_sem motion_finished;

/**
 * @typedef robot_state_t
 * @brief structure to hold global configs and leg positions
 *
 */
typedef struct robot_state_t
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

} robot_state_t;

// --- GLOBAL INSTANCE DECLARATION ---
extern robot_state_t g_state;

void init_robot_state(void);
void print_robot_state(void);
void set_site(int leg, float x, float y, float z);

#endif
