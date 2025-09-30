#include "zephyr/drivers/pwm.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/util.h"
#include <servos.h>
#include <stddef.h>
#include <stdint.h>

LOG_MODULE_REGISTER(Servo, LOG_LEVEL_DBG);

/* Define the array of all servo specifications */
const struct pwm_dt_spec servos[] = {
    /* The macro iterates over all children of the 'pwmleds' node
     * (servo0, servo1, ..., servo11) and applies GET_PWM_SPEC to each.
     */
    DT_FOREACH_CHILD(DT_PATH(pwmleds), GET_PWM_SPEC)};

const size_t NB_SERVOS = ARRAY_SIZE(servos);

void set_angle(const struct pwm_dt_spec* servo, uint8_t angle)
{
    CLAMP(angle, 0, 180);

    // linear interpolation for the pulse
    uint32_t pulse = SERVO_PULSE_MIN_NS +
                     (angle * (SERVO_PULSE_MAX_NS - SERVO_PULSE_MIN_NS) / 180);

    int ret;
    ret = pwm_set_pulse_dt(servo, pulse);
    if (ret < 0)
        LOG_DBG("Error writing the angle");
}

void center_all_servos(const struct pwm_dt_spec* servos, size_t nb_servos)
{
    for (int i = 0; i < nb_servos; i++)
    {
        pwm_set_pulse_dt(&servos[i], SERVO_PULSE_CENTER_NS);
    }
}
