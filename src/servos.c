/*======================================================================
 * File:    servos.c
 * Date:    2025-10-07
 * Purpose: Contains the code that interact with the pwm driver. All writes of
 *pulse are done to a single interface (set_angle()).
 *====================================================================*/
#include "zephyr/device.h"
#include "zephyr/drivers/pwm.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/util.h"
#include <servos.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/errno.h>

LOG_MODULE_REGISTER(Servo, LOG_LEVEL_DBG);

const struct device* pwm = DEVICE_DT_GET(DT_NODELABEL(pca9685));

// servo_channel_map[leg_id][joint_id] = channel;
static const uint8_t servo_channel_map[NB_LEGS][NB_JOINTS] = {
    //{Coxa, Femur, Tibia}
    {0, 1, 2},    // Leg 0 (FRONT_LEFT)
    {3, 4, 5},    // Leg 1 (BOTTOM_LEFT)
    {6, 7, 8},    // Leg 2 (FRONT_RIGHT)
    {9, 10, 11}}; // Leg 3 (BOTTOM_RIGHT)

int init_servos(void)
{
    if (!device_is_ready(pwm))
    {
        LOG_ERR("PWM device %s is not ready", pwm->name);
        return -ENODEV;
    }

    int ret = pwm_set(pwm, 0, PERIOD_SERVO, 0, 0);
    if (ret < 0)
    {
        LOG_ERR("Failed tp set initial PWM period (%d)", ret);
        return ret;
    }
    return 0;
}

/**
 * @brief maps the angle to pulse and the servo ids to the right channel and
 * write it.
 *
 * @param leg_id
 * @param joint_id
 * @param angle
 */
void set_angle(uint8_t leg_id, uint8_t joint_id, uint8_t angle)
{
    CLAMP(angle, 0, 180);

    uint32_t pulse = SERVO_PULSE_MIN_NS +
                     (angle * (SERVO_PULSE_MAX_NS - SERVO_PULSE_MIN_NS) / 180);

    int ret = pwm_set(pwm, servo_channel_map[leg_id][joint_id], PERIOD_SERVO,
                      pulse, 0);
    if (ret < 0)
        LOG_ERR("Failed setting pwm for leg %d joint %d", leg_id, joint_id);
}

/**
 * @brief For calibration purpose, can be run when the servos are not locked
 * with the horn to set the robot initial positions (see:
 * https://www.instructables.com/DIY-Spider-RobotQuad-robot-Quadruped/)
 */
void center_all_servos(void)
{
    for (int leg = 0; leg < NB_LEGS; leg++)
        for (int joint = 0; joint < NB_JOINTS; joint++)
            set_angle(leg, joint, 90);
}
