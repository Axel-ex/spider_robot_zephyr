#include <zephyr/drivers/pwm.h>

// Structural constants
#define NB_LEGS 4
#define JOINTS_PER_LEG 3

// Standard servo period (50 Hz = 20 ms = 20,000,000 ns)
#define SERVO_PULSE_MIN_NS 1000000UL
#define SERVO_PULSE_MAX_NS 2000000UL
#define SERVO_PULSE_CENTER_NS 1500000UL

/* Get the node ID for the parent 'pwmleds' node */
// Macro magic for it to work (coma at the end)
#define GET_PWM_SPEC(node_id) PWM_DT_SPEC_GET(node_id),

// Macro to retrive the pwm_dt_spec corresponding to a joint and a leg id
#define GET_SERVO_SPEC(leg_id, joint_id)                                       \
    (&servos[((leg_id) * JOINTS_PER_LEG) + (joint_id)])

extern const struct pwm_dt_spec servos[];

// helpers
void set_angle(const struct pwm_dt_spec* servo, uint8_t angle);
void center_all_servos(const struct pwm_dt_spec* servos, size_t nb_servos);
