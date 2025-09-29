#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Use the DT_ALIAS macro to safely reference the 'servo-0' alias defined in the
// overlay
static const struct pwm_dt_spec fading_led =
    PWM_DT_SPEC_GET(DT_NODELABEL(fading_led));
LOG_MODULE_REGISTER(ServoApp, LOG_LEVEL_INF);

void main(void)
{
    int ret;

    // 1. Always check if the device driver is initialized and ready
    if (!device_is_ready(fading_led.dev))
    {
        LOG_ERR("PWM device %s not ready", fading_led.dev->name);
        return;
    }

    LOG_INF("PWM Device Ready. Setting initial pulse...");

    // 2. Set the pulse using the highly portable 'pwm_set_pulse_dt'
    // 1,500,000 ns = 1.5 ms (Standard servo center pulse)
    ret = pwm_set_pulse_dt(&fading_led, 1500000);

    if (ret < 0)
    {
        LOG_ERR("Error setting pulse: %d", ret);
        return;
    }

    LOG_INF("Servo set to center position.");

    // Example: Sweep the servo back and forth
    while (true)
    {
        // Move to max-pulse (2.5 ms)
        // pwm_set_pulse_dt(&fading_led, fading_led.max_pulse);
        // k_msleep(1000);

        // Move to min-pulse (0.5 ms)
        // pwm_set_pulse_dt(&fading_led, fading_led.min_pulse);
        k_msleep(1000);
    }
}
