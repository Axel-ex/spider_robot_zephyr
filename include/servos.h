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

int init_servos(void);
void set_angle(uint8_t leg_id, uint8_t joint_id, uint8_t angle);
void center_all_servos(void);
