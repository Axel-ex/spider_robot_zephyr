#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Use the DT_ALIAS macro to safely reference the 'servo-0' alias defined in the
// overlay
static const struct pwm_dt_spec servo0 = PWM_DT_SPEC_GET(DT_NODELABEL(servo0));
LOG_MODULE_REGISTER(ServoApp, LOG_LEVEL_INF);

// Standard servo period (50 Hz = 20 ms = 20,000,000 ns)
// MIN: Use 10 ns *more* than 1 ms
#define SERVO_PULSE_MIN_NS 1000000UL
// MAX: Use 10 ns *less* than 2 ms
#define SERVO_PULSE_MAX_NS 2000000UL
// Center is still fine
#define SERVO_CENTER_PULSE 1500000UL

void main(void)
{
    int ret;
    if (!device_is_ready(servo0.dev))
        return;
    LOG_INF("Servo control initialized. Starting sweep...");

    ret = pwm_set_pulse_dt(&servo0, SERVO_CENTER_PULSE);
    if (ret < 0)
    {
        LOG_ERR("Error setting initial center pulse: %d",
                ret); // Check this log!
        return;
    }
    k_msleep(100);

    while (true)
    {

        // Use exact defined values to avoid rounding errors
        LOG_INF("Set angle: MAX");
        ret = pwm_set_pulse_dt(&servo0, SERVO_PULSE_MAX_NS);
        if (ret < 0)
        {
            LOG_ERR("Error setting MAX pulse: %d", ret); // CHECK THIS LOG!
            return;
        }
        k_msleep(5000);

        LOG_INF("Set angle: MIN");
        ret = pwm_set_pulse_dt(&servo0, SERVO_PULSE_MIN_NS);
        if (ret < 0)
        {
            LOG_ERR("Error setting MIN pulse: %d", ret); // CHECK THIS LOG!
            return;
        }
        k_msleep(5000);
    }
}
