#ifndef GAIT
#define GAIT

void sit(void);
void stand(void);
void step_forward(unsigned int step);

// motor_thread.c
void cartesian_to_polar(volatile float* alpha, volatile float* beta,
                        volatile float* gamma, volatile float x,
                        volatile float y, volatile float z);
void polar_to_servo(int leg, float alpha, float beta, float gamma);

#endif // !GAIT
