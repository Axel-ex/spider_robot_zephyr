#include "kinematics.h"
#include <math.h>
#include <servos.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Kinematics, LOG_LEVEL_INF);
const double PI_CONST = 3.1415926;
const double KEEP = 255.0;

// --- GLOBAL INSTANCE DEFINITION ---
// 1. Define the single global instance (Initialized to zero/defaults)
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
 * @brief inits the fields that need runtime initialisation (using maths
 * functions)
 */
void kinematics_init(void)
{
    if (g_state.initialized)
    {
        LOG_DBG("Kinematics already initialized.");

        return;
    }

    g_state.z_boot = g_state.z_absolute;

    // Runtime calculations
    double val_2x_l = (2.0 * g_state.x_default + g_state.length_side);

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
    LOG_INF("Kinematics constants calculated and state initialized.");
}

/**
 * @brief Prints all fields of the global kinematics structure for verification.
 * * Note: Uses LOG_INF level for visibility, ensuring the output is easy to
 * read.
 */
void kinematics_print_debug(void)
{
    if (!g_state.initialized)
    {
        LOG_WRN("Kinematics not initialized. Skipping debug print.");
        return;
    }

    LOG_INF("--- Robot Kinematics Debug Dump ---");

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

void set_site(int leg, double x, double y, double z)
{
    double length_x = 0, length_y = 0, length_z = 0;

    if (x != KEEP)
        length_x = x - g_state.site_now[leg][0];
    if (y != KEEP)
        length_y = y - g_state.site_now[leg][1];
    if (z != KEEP)
        length_z = z - g_state.site_now[leg][2];

    double length =
        sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

    g_state.temp_speed[leg][0] =
        length_x / length * g_state.move_speed * g_state.speed_multiple;
    g_state.temp_speed[leg][1] =
        length_y / length * g_state.move_speed * g_state.speed_multiple;
    g_state.temp_speed[leg][2] =
        length_z / length * g_state.move_speed * g_state.speed_multiple;

    if (x != KEEP)
        g_state.site_expect[leg][0] = x;
    if (y != KEEP)
        g_state.site_expect[leg][1] = y;
    if (z != KEEP)
        g_state.site_expect[leg][2] = z;
}

void cartesian_to_polar(volatile double* alpha, volatile double* beta,
                        volatile double* gamma, volatile double x,
                        volatile double y, volatile double z)
{
    // calculate w-z degree
    double v, w;
    w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
    v = w - g_state.length_c;
    *alpha =
        atan2(z, v) + acos((pow(g_state.length_a, 2) -
                            pow(g_state.length_b, 2) + pow(v, 2) + pow(z, 2)) /
                           2 / g_state.length_a / sqrt(pow(v, 2) + pow(z, 2)));
    *beta = acos((pow(g_state.length_a, 2) + pow(g_state.length_b, 2) -
                  pow(v, 2) - pow(z, 2)) /
                 2 / g_state.length_a / g_state.length_b);
    // calculate x-y-z degree
    *gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);

    // trans degree pi->180
    *alpha = *alpha / PI_CONST * 180;
    *beta = *beta / PI_CONST * 180;
    *gamma = *gamma / PI_CONST * 180;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wself-assign"
/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, double alpha, double beta, double gamma)
{
    if (leg == 0)
    {
        alpha = 90 - alpha;
        beta = beta;
        gamma += 90;
    }
    else if (leg == 1)
    {
        alpha += 90;
        beta = 180 - beta;
        gamma = 90 - gamma;
    }
    else if (leg == 2)

    {
        alpha += 90;
        beta = 180 - beta;
        gamma = 90 - gamma;
    }
    else if (leg == 3)
    {
        alpha = 90 - alpha;
        beta = beta;
        gamma += 90;
    }

    set_angle(GET_SERVO_SPEC(leg, 0), alpha);
    set_angle(GET_SERVO_SPEC(leg, 1), beta);
    set_angle(GET_SERVO_SPEC(leg, 2), gamma);
}
#pragma clang diagnostic pop
