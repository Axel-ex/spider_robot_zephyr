#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "zephyr/kernel.h"
#include <stdbool.h>
#include <stddef.h>

#define EPSILON 0.001

extern const double PI_CONST;
extern const double KEEP;
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
        double length_a, length_b, length_c;
        double length_side, z_absolute;

        // Movement Parameters (Read-only once initialized)
        double z_default, z_up, z_boot;
        double x_default, x_offset;
        double y_start, y_step;

        // Derived Turn Constants (Calculated in init function)
        double temp_a, temp_b, temp_c;
        double temp_alpha;
        double turn_x0, turn_y0, turn_x1, turn_y1;

        // Speed Constants
        double speed_multiple;
        double spot_turn_speed;
        double leg_move_speed;

        double body_move_speed;
        double stand_seat_speed;

        // --- MUTABLE STATE (Volatile variables used by Control Thread) ---
        volatile double site_now[4][3];    // Real-time coordinates
        volatile double site_expect[4][3]; // Expected coordinates

        double temp_speed[4][3]; // Each axis' speed
        volatile int rest_counter;
        double move_speed;

        // Marker to ensure initialization has run
        bool initialized;

} robot_state_t;

// --- GLOBAL INSTANCE DECLARATION ---
extern robot_state_t g_state;

void init_robot_state(void);
void print_robot_state(void);

// Utils for movement calculations
void set_site(int leg, double x, double y, double z);
void cartesian_to_polar(volatile double* alpha, volatile double* beta,
                        volatile double* gamma, volatile double x,
                        volatile double y, volatile double z);
void polar_to_servo(int leg, double alpha, double beta, double gamma);

#endif
