/*======================================================================
 * File:    motors_thread.c
 * Date:    2025-10-07
 * Purpose: Shares the state with the gait thread. wakes up every 20ms on high
 *priority and takes the appropriate steps based on the expected posisions of
 *the legs. Perform the inverse kinematic computation to convert the x,y,z
 *coordinates into angles.
 *====================================================================*/
#include "robot_state.h"
#include "servos.h"
#include "spider_robot.h"
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define MOTOR_THREAD_PRIORITY 1
#define MOTOR_THREAD_STACK_SIZE 1024
#define UPDATE_PERIOD 20

LOG_MODULE_REGISTER(motors_thread, LOG_LEVEL_DBG);
K_SEM_DEFINE(motion_finished, 0, 1);

/**
 * @brief update the legs positions every 20ms. When the positions of the legs
 * reach the expected,
 */
void motors_thread(void)
{
    static double alpha, beta, gamma;
    k_timeout_t period = K_MSEC(UPDATE_PERIOD);

    while (true)
    {
        if (k_mutex_lock(&g_state_mutex, K_MSEC(UPDATE_PERIOD / 2)) != 0)
        {
            LOG_ERR("Fail locking the mutex");
            continue;
        }

        for (int leg = 0; leg < NB_LEGS; leg++)
        {
            for (int joint = 0; joint < NB_JOINTS; joint++)
            {
                double current_pos = g_state.site_now[leg][joint];
                double target_pos = g_state.site_expect[leg][joint];
                double remaining_dist = target_pos - current_pos;

                double step_dist = g_state.temp_speed[leg][joint];

                // Prevent overshooting in the final step
                if (fabs(remaining_dist) < fabs(step_dist))
                    g_state.site_now[leg][joint] = target_pos;
                else
                    g_state.site_now[leg][joint] += step_dist;
            }
            cartesian_to_polar(&alpha, &beta, &gamma, g_state.site_now[leg][0],
                               g_state.site_now[leg][1],
                               g_state.site_now[leg][2]);
            polar_to_servo(leg, alpha, beta, gamma);
        }

        if (k_mutex_unlock(&g_state_mutex) != 0)
            LOG_ERR("Fail unlocking the mutex");

        k_sleep(period);
    }
}

K_THREAD_DEFINE(motor_thread_id, MOTOR_THREAD_STACK_SIZE, motors_thread, NULL,
                NULL, NULL, MOTOR_THREAD_PRIORITY, K_USER, 0);

void cartesian_to_polar(double* alpha, double* beta, double* gamma, double x,
                        double y, double z)
{
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

void polar_to_servo(int leg, double alpha, double beta, double gamma)
{
    if (leg == 0)
    {
        alpha = 90 - alpha;
        // beta = beta;
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
        // beta = beta;
        gamma += 90;
    }

    set_angle(leg, 0, alpha);
    set_angle(leg, 1, beta);
    set_angle(leg, 2, gamma);
}
