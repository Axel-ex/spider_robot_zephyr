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
    init_robot_state();
    if (init_servos() < 0)
        return;

    while (true)
    {
        k_sleep(K_SECONDS(5));
    }
}
