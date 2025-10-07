#ifndef GAIT
#define GAIT

void sit(unsigned int step);
void stand(unsigned int step);
void step_forward(unsigned int step);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_back(unsigned int step);
void hand_shake(unsigned int step);
void hand_wave(unsigned int step);

// cmd table
struct cmd_entry
{
        const char* name;
        void (*fn)(unsigned int step);
};

static const struct cmd_entry cmd_table[] = {
    {"sit", sit},          {"stand", stand},   {"step_forward", step_forward},
    {"sb", step_back},     {"tl", turn_left},  {"tr", turn_right},
    {"shake", hand_shake}, {"wave", hand_wave}};

// motor_thread.c
void cartesian_to_polar(volatile double* alpha, volatile double* beta,
                        volatile double* gamma, volatile double x,
                        volatile double y, volatile double z);
void polar_to_servo(int leg, double alpha, double beta, double gamma);

#endif // !GAIT
