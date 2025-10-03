#ifndef GAIT
#define GAIT

void sit(void);
void stand(void);
void step_forward(unsigned int step);

// motor_thread.c
void cartesian_to_polar(volatile double* alpha, volatile double* beta,
                        volatile double* gamma, volatile double x,
                        volatile double y, volatile double z);
void polar_to_servo(int leg, double alpha, double beta, double gamma);

#endif // !GAIT
