#include <stddef.h>
#include <zephyr/drivers/pwm.h>

// Structural constants
#define NB_LEGS 4
#define NB_JOINTS 3
#define NB_SERVOS 12
#define PERIOD_SERVO 20000000

// Default values from Arduino Servo.cpp for calibration
#define SERVO_PULSE_MIN_NS 544000  // 544 microseconds
#define SERVO_PULSE_MAX_NS 2400000 // 2400 microseconds

// The center is the average of min and max
#define SERVO_PULSE_CENTER_NS                                                  \
    ((SERVO_PULSE_MIN_NS + SERVO_PULSE_MAX_NS) / 2) // 1472000 ns

// /* Get the node ID for the parent 'pwmleds' node */
// // Macro magic for it to work (coma at the end)
// #define GET_PWM_SPEC(node_id) PWM_DT_SPEC_GET(node_id),
//
// // Macro to retrive the pwm_dt_spec corresponding to a joint and a leg id
// #define GET_SERVO_SPEC(leg_id, joint_id) \
//     (&servos[((leg_id) * JOINTS_PER_LEG) + (joint_id)])
//
// helpers
int init_servos(void);
void set_angle(uint8_t leg_id, uint8_t joint_id, uint8_t angle);
void center_all_servos(void);
