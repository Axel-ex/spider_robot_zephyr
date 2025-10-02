#include "robot_state.h"
#include "servos.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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
    init_robot_state();

    while (true)
    {
        k_sleep(K_SECONDS(5));
    }
}
