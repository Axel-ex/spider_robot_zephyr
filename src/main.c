#include "kinematics.h"
#include "zephyr/logging/log_core.h"
#include <servos.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ServoApp, LOG_LEVEL_DBG);
extern const size_t NB_SERVOS;

extern const struct pwm_dt_spec servos[];

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
    kinematics_print_debug();

    LOG_DBG("All %zu servos are ready!", NB_SERVOS);
    k_sleep(K_SECONDS(1));
    center_all_servos(servos, NB_SERVOS);

    while (true)
        k_sleep(K_SECONDS(1));
}
