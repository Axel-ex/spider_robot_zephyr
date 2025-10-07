#ifndef GAIT
#define GAIT

void sit(unsigned int step);
void stand(unsigned int step);
void step_forward(unsigned int step);

// cmd table
struct cmd_entry
{
        const char* name;
        void (*fn)(unsigned int step);
};

static const struct cmd_entry cmd_table[] = {
    {"sit", sit}, {"stand", stand}, {"step_forward", step_forward}};

// motor_thread.c
void cartesian_to_polar(volatile double* alpha, volatile double* beta,
                        volatile double* gamma, volatile double x,
                        volatile double y, volatile double z);
void polar_to_servo(int leg, double alpha, double beta, double gamma);

#endif // !GAIT
