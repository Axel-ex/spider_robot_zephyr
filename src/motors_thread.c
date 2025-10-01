#include "robot_state.h"
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
void motor_thread(void)
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
                if (fabs(g_state.site_now[leg][joint] -
                         g_state.site_expect[leg][joint]) >=
                    fabs(g_state.temp_speed[leg][joint]))
                {
                    g_state.site_now[leg][joint] +=
                        g_state.temp_speed[leg][joint];
                    all_finished = false;
                }
                else
                {
                    g_state.site_now[leg][joint] =
                        g_state.site_expect[leg][joint];
                }
            }
            cartesian_to_polar(&alpha, &beta, &gamma, g_state.site_now[leg][0],
                               g_state.site_now[leg][1],
                               g_state.site_now[leg][2]);
            polar_to_servo(leg, alpha, beta, gamma);
        }

        // Give the semaphore when motion is done. The smaphore is taken by the
        // gait thread.
        if (all_finished)
            if (k_sem_count_get(&motion_finished) == 0)
                k_sem_give(&motion_finished);

        if (k_mutex_unlock(&g_state_mutex) != 0)
            LOG_ERR("Fail unlocking the mutex");

        k_sleep(period);
    }
}

K_THREAD_DEFINE(motor_thread_id, MOTOR_THREAD_STACK_SIZE, motor_thread, NULL,
                NULL, NULL, MOTOR_THREAD_PRIORITY, 0, K_USER);
