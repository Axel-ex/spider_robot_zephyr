#include "gait.h"
#include "robot_state.h"
#include "servos.h"
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define MOTOR_THREAD_PRIORITY 4
#define MOTOR_THREAD_STACK_SIZE 1024

LOG_MODULE_REGISTER(motors_thread, LOG_LEVEL_DBG);
K_SEM_DEFINE(motion_finished, 0, 1);

/**
 * @brief update the legs positions every 20ms. When the positions of the legs
 * reach the expected,
 */
void motors_thread(void)
{
    static double alpha, beta, gamma;
    k_timeout_t period = K_MSEC(20);

    while (true)
    {
        bool all_finished = true;

        if (k_mutex_lock(&g_state_mutex, K_MSEC(10)) != 0)
        {
            LOG_ERR("Fail locking the mutex");
            continue;
        }

        for (int leg = 0; leg < 4; leg++)
        {
            for (int joint = 0; joint < 3; joint++)
            {
                double current_pos = g_state.site_now[leg][joint];
                double target_pos = g_state.site_expect[leg][joint];
                double remaining_dist = target_pos - current_pos;

                if (fabs(remaining_dist) < EPSILON)
                    continue; // joint done
                all_finished = false;

                double step_dist = g_state.temp_speed[leg][joint];

                if (fabs(remaining_dist) < fabs(step_dist))
                {
                    // If the remaining distance is smaller than a full step,
                    // the step to take is just the remaining distance itself.
                    g_state.site_now[leg][joint] = target_pos;
                }
                else
                {
                    // Otherwise, we are far enough away to take a full step.
                    g_state.site_now[leg][joint] += step_dist;
                }
            }
            cartesian_to_polar(&alpha, &beta, &gamma, g_state.site_now[leg][0],
                               g_state.site_now[leg][1],
                               g_state.site_now[leg][2]);
            if (isnan(alpha) || isnan(beta) || isnan(gamma))
            {
                LOG_ERR("IK NaN for leg %d with inputs x:%.2f, y:%.2f, z:%.2f",
                        leg, g_state.site_now[leg][0], g_state.site_now[leg][1],
                        g_state.site_now[leg][2]);
                return;
            }
            polar_to_servo(leg, alpha, beta, gamma);
        }

        if (k_mutex_unlock(&g_state_mutex) != 0)
            LOG_ERR("Fail unlocking the mutex");

        // TODO: Make sure you give the sem only once (get count and print
        // debug)
        if (all_finished)
            k_sem_give(&motion_finished);

        k_sleep(period);
    }
}

K_THREAD_DEFINE(motor_thread_id, MOTOR_THREAD_STACK_SIZE, motors_thread, NULL,
                NULL, NULL, MOTOR_THREAD_PRIORITY, K_USER, 0);

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
