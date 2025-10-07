#ifndef GAIT
#define GAIT

/*=====================================================================*
 *                       Gait moves & cmds
 *=====================================================================*/
void set_site(int leg, double x, double y, double z);
void wait_all_reach(void);

void sit(unsigned int step);
void stand(unsigned int step);
void step_forward(unsigned int step);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_back(unsigned int step);
void hand_shake(unsigned int step);
void hand_wave(unsigned int step);

struct cmd_entry
{
        const char* name;
        void (*fn)(unsigned int step);
};

static const struct cmd_entry cmd_table[] = {
    {"sit", sit},          {"stand", stand},   {"sf", step_forward},
    {"sb", step_back},     {"tl", turn_left},  {"tr", turn_right},
    {"shake", hand_shake}, {"wave", hand_wave}};

/*=====================================================================*
 *                          TCP command
 *=====================================================================*/
#define RX_BUF_SIZE 32

struct tcp_command
{
        char command[RX_BUF_SIZE];
        int times;
};
extern struct k_msgq tcp_command_q;

/*=====================================================================*
 *                           Kinematics
 *=====================================================================*/
void cartesian_to_polar(volatile double* alpha, volatile double* beta,
                        volatile double* gamma, volatile double x,
                        volatile double y, volatile double z);
void polar_to_servo(int leg, double alpha, double beta, double gamma);

#endif // !GAIT
