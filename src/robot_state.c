#include "robot_state.h"
#include "zephyr/kernel.h"
#include <math.h>
#include <servos.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(robot_state, LOG_LEVEL_DBG);
const float PI_CONST = 3.1415926;
const float KEEP = 255.0;
K_MUTEX_DEFINE(g_state_mutex);

/**
 * @brief Global instance of the state
 */
robot_state_t g_state = {

    // Initialize simple constants directly where possible
    .length_a = 55.0,

    .length_b = 77.5,       .length_c = 27.5,        .length_side = 71.0,
    .z_absolute = -28.0,

    .z_default = -50.0,     .z_up = -30.0,           .x_default = 62.0,
    .y_step = 40.0,

    .speed_multiple = 1.0,  .spot_turn_speed = 4.0,  .leg_move_speed = 8.0,
    .body_move_speed = 3.0, .stand_seat_speed = 1.0,
};

/**
 * @brief inits the fields that need runtime initialisation
 */
void init_robot_state(void)
{
    if (g_state.initialized)
    {
        LOG_DBG("State already initialized.");

        return;
    }

    g_state.z_boot = g_state.z_absolute;

    // Runtime calculations
    float val_2x_l = (2.0 * g_state.x_default + g_state.length_side);

    g_state.temp_a = sqrt(pow(val_2x_l, 2.0) + pow(g_state.y_step, 2.0));

    g_state.temp_b =
        2.0 * (g_state.y_start + g_state.y_step) + g_state.length_side;

    g_state.temp_c = sqrt(
        pow(val_2x_l, 2.0) +
        pow(2.0 * g_state.y_start + g_state.y_step + g_state.length_side, 2.0));

    g_state.temp_alpha =
        acos((pow(g_state.temp_a, 2.0) + pow(g_state.temp_b, 2.0) -
              pow(g_state.temp_c, 2.0)) /
             (2.0 * g_state.temp_a * g_state.temp_b));

    // site for turn
    g_state.turn_x1 = (g_state.temp_a - g_state.length_side) / 2.0;
    g_state.turn_y1 = g_state.y_start + g_state.y_step / 2.0;

    g_state.turn_x0 =
        g_state.turn_x1 - g_state.temp_b * cos(g_state.temp_alpha);
    g_state.turn_y0 = g_state.temp_b * sin(g_state.temp_alpha) -
                      g_state.turn_y1 - g_state.length_side;

    g_state.initialized = true;
    LOG_INF("State initialized.");
}

/**
 * @brief Prints all fields of the global state structure for verification.
 * * Note: Uses LOG_INF level for visibility, ensuring the output is easy to
 * read.
 */
void print_robot_state(void)
{
    if (!g_state.initialized)
    {
        LOG_WRN("State not initialized. Skipping debug print.");
        return;
    }

    LOG_INF("--- Robot State Debug Dump ---");

    // 1. Core Dimensions
    LOG_INF("Core Dimensions:");
    LOG_INF("  Length A/B/C: %.2f / %.2f / %.2f", (double)g_state.length_a,
            (double)g_state.length_b, (double)g_state.length_c);
    LOG_INF("  Length Side: %.2f", (double)g_state.length_side);
    LOG_INF("  Z Absolute/Boot: %.2f / %.2f", (double)g_state.z_absolute,
            (double)g_state.z_boot);

    // 2. Movement Parameters
    LOG_INF("Movement Parameters:");
    LOG_INF("  X Default/Offset: %.2f / %.2f", (double)g_state.x_default,
            (double)g_state.x_offset);
    LOG_INF("  Y Start/Step: %.2f / %.2f", (double)g_state.y_start,

            (double)g_state.y_step);

    // 3. Calculated Constants (Crucial for verification)
    LOG_INF("Calculated Turn Constants:");
    LOG_INF("  Temp A/B/C: %.3f / %.3f / %.3f", (double)g_state.temp_a,
            (double)g_state.temp_b, (double)g_state.temp_c);
    LOG_INF("  Temp Alpha (rad): %.4f", (double)g_state.temp_alpha);
    LOG_INF("  Turn X0/Y0: %.2f / %.2f",

            (double)g_state.turn_x0, (double)g_state.turn_y0);
    LOG_INF("  Turn X1/Y1: %.2f / %.2f", (double)g_state.turn_x1,
            (double)g_state.turn_y1);

    // 4. Positions
    LOG_INF("Initial State (Site Expect/Now):");
    for (int i = 0; i < NB_LEGS; i++)
    {
        LOG_INF("  Leg %d: Expect: (%.1f, %.1f, %.1f) | Current: (%.1f, %.1f, "
                "%.1f)",
                i,

                (double)g_state.site_expect[i][0],
                (double)g_state.site_expect[i][1],
                (double)g_state.site_expect[i][2],

                (double)g_state.site_now[i][0],

                (double)g_state.site_now[i][1], (double)g_state.site_now[i][2]);
    }

    LOG_INF("-------------------------------------");
}
