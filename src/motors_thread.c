#include "kinematics.h"
#include <math.h>
#include <zephyr/kernel.h>

#define MOTOR_THREAD_PRIORITY 4
#define MOTOR_THREAD_STACK_SIZE 1024

void motor_thread(void)
{
    static double alpha, beta, gamma;
    while (true)
    {
        // Deactivate interrupt?
        // Lock a mutex?
        for (int leg = 0; leg < 4; leg++)
        {
            for (int joint = 0; joint < 3; joint++)
            {
                if (fabs(g_state.site_now[leg][joint] -
                         g_state.site_expect[leg][joint]) >=
                    fabs(g_state.temp_speed[leg][joint]))
                    g_state.site_now[leg][joint] +=
                        g_state.temp_speed[leg][joint];
                else
                    g_state.site_now[leg][joint] =
                        g_state.site_expect[leg][joint];
                // raise the semaphore
            }
            cartesian_to_polar(&alpha, &beta, &gamma, g_state.site_now[leg][0],
                               g_state.site_now[leg][1],
                               g_state.site_now[leg][2]);
            polar_to_servo(leg, alpha, beta, gamma);
            k_msleep(20);
        }
    }
}

K_THREAD_DEFINE(motor_thread_id, MOTOR_THREAD_STACK_SIZE, motor_thread, NULL,
                NULL, NULL, MOTOR_THREAD_PRIORITY, 0, K_USER);
