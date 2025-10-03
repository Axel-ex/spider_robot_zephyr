#include <stddef.h>
#include <zephyr/drivers/pwm.h>

// Structural constants
#define NB_LEGS 4
#define JOINTS_PER_LEG 3

// Default values from Arduino Servo.cpp for calibration
#define SERVO_PULSE_MIN_NS 544000  // 544 microseconds
#define SERVO_PULSE_MAX_NS 2400000 // 2400 microseconds

// The center is the average of min and max
#define SERVO_PULSE_CENTER_NS                                                  \
    ((SERVO_PULSE_MIN_NS + SERVO_PULSE_MAX_NS) / 2) // 1472000 ns

/* Get the node ID for the parent 'pwmleds' node */
// Macro magic for it to work (coma at the end)
#define GET_PWM_SPEC(node_id) PWM_DT_SPEC_GET(node_id),

// Macro to retrive the pwm_dt_spec corresponding to a joint and a leg id
#define GET_SERVO_SPEC(leg_id, joint_id)                                       \
    (&servos[((leg_id) * JOINTS_PER_LEG) + (joint_id)])

extern const size_t NB_SERVOS;
extern const struct pwm_dt_spec servos[];

// helpers
void set_angle(const struct pwm_dt_spec* servo, uint8_t angle);
void center_all_servos(const struct pwm_dt_spec* servos, size_t nb_servos);
