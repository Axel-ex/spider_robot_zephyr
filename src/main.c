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
    set_site(0, g_state.x_default - g_state.x_offset,
             g_state.y_start + g_state.y_step, g_state.z_boot);
    set_site(1, g_state.x_default - g_state.x_offset,
             g_state.y_start + g_state.y_step, g_state.z_boot);
    set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
             g_state.z_boot);
    set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
             g_state.z_boot);

    for (int leg = 0; leg < 4; leg++)
        for (int joint = 0; joint < 3; joint++)
            g_state.site_now[leg][joint] = g_state.site_expect[leg][joint];
    kinematics_print_debug();

    LOG_DBG("All %zu servos are ready!", NB_SERVOS);
    k_sleep(K_SECONDS(1));

    while (true)
        k_sleep(K_SECONDS(1));
}
