#include "kinematics.h"
#include "servos.h"
#include "zephyr/logging/log_core.h"
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

extern const size_t NB_SERVOS;
extern const struct pwm_dt_spec servos[];
LOG_MODULE_REGISTER(ServoApp, LOG_LEVEL_DBG);

#define MOTOR_THREAD_PRIORITY 4
#define MOTOR_THREAD_STACK_SIZE 1024

void motor_thread(void)
{
    static float alpha, beta, gamma;
    while (true)
    {
        // Deactivate interrupt?
        // Lock a mutex?
        for (int leg = 0; leg < 4; leg++)
        {
            for (int joint = 0; joint < 3; joint++)
            {
                if (abs(g_kinematics.site_now[leg][joint] -
                        g_kinematics.site_expect[leg][joint]) >=
                    abs(g_kinematics.temp_speed[leg][joint]))
                    g_kinematics.site_now[leg][joint] +=
                        g_kinematics.temp_speed[leg][joint];
                else
                    g_kinematics.site_now[leg][joint] =
                        g_kinematics.site_expect[leg][joint];
            }
            cartesian_to_polar(
                &alpha, &beta, &gamma, g_kinematics.site_now[leg][0],
                g_kinematics.site_now[leg][1], g_kinematics.site_now[leg][2]);
            polar_to_servo(leg, alpha, beta, gamma);
            k_msleep(20);
        }
    }
}

K_THREAD_DEFINE(motor_thread_id, MOTOR_THREAD_STACK_SIZE, motor_thread, NULL,
                NULL, NULL, MOTOR_THREAD_PRIORITY, 0, K_USER);

void main(void)
{
    for (int i = 0; i < NB_SERVOS; i++)
    {
        if (!device_is_ready(servos[i].dev))
        {
            LOG_ERR("Fail initiating the servos");
            return;
        }
    }
    kinematics_init();

    // Initialize with default params
    set_site(0, g_kinematics.x_default - g_kinematics.x_offset,
             g_kinematics.y_start + g_kinematics.y_step, g_kinematics.z_boot);
    set_site(1, g_kinematics.x_default - g_kinematics.x_offset,
             g_kinematics.y_start + g_kinematics.y_step, g_kinematics.z_boot);
    set_site(2, g_kinematics.x_default + g_kinematics.x_offset,
             g_kinematics.y_start, g_kinematics.z_boot);
    set_site(3, g_kinematics.x_default + g_kinematics.x_offset,
             g_kinematics.y_start, g_kinematics.z_boot);

    for (int leg = 0; leg < 4; leg++)
        for (int joint = 0; joint < 3; joint++)
            g_kinematics.site_now[leg][joint] =
                g_kinematics.site_expect[leg][joint];
    kinematics_print_debug();

    LOG_DBG("All %zu servos are ready!", NB_SERVOS);
    k_sleep(K_SECONDS(1));
    center_all_servos(servos, NB_SERVOS);

    while (true)
        k_sleep(K_SECONDS(1));
}
