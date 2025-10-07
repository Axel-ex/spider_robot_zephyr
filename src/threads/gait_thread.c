#include "robot_state.h"
#include "servos.h"
#include "spider_robot.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/util.h"
#include <math.h>
#include <sys/_types.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gait_thread, LOG_LEVEL_DBG);
#define GAIT_STACK_SIZE 1024
#define GAIT_THREAD_PRIORITY 5

static const struct cmd_entry cmd_table[] = {
    {"sit", sit},          {"stand", stand},   {"sf", step_forward},
    {"sb", step_back},     {"tl", turn_left},  {"tr", turn_right},
    {"shake", hand_shake}, {"wave", hand_wave}};

void set_site(int leg, double x, double y, double z)
{
    double length_x = 0, length_y = 0, length_z = 0;

    if (x != KEEP)
        length_x = x - g_state.site_now[leg][0];
    if (y != KEEP)
        length_y = y - g_state.site_now[leg][1];
    if (z != KEEP)
        length_z = z - g_state.site_now[leg][2];

    double length =
        sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

    double speed_factor = g_state.move_speed * g_state.speed_multiple / length;
    g_state.temp_speed[leg][0] = length_x * speed_factor;
    g_state.temp_speed[leg][1] = length_y * speed_factor;
    g_state.temp_speed[leg][2] = length_z * speed_factor;

    if (x != KEEP)
        g_state.site_expect[leg][0] = x;
    if (y != KEEP)
        g_state.site_expect[leg][1] = y;
    if (z != KEEP)
        g_state.site_expect[leg][2] = z;
}

void process_tcp_command(const struct tcp_command* cmd)
{
    for (int i = 0; i < ARRAY_SIZE(cmd_table); i++)
    {
        if (strcmp(cmd->command, cmd_table[i].name) == 0)
        {
            cmd_table[i].fn(cmd->times);
            return;
        }
    }
    LOG_WRN("Unrecognised command %s", cmd->command);
}

void gait_thread(void)
{
    // Initialisation
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    set_site(0, g_state.x_default - g_state.x_offset,
             g_state.y_start + g_state.y_step, g_state.z_boot);
    set_site(1, g_state.x_default - g_state.x_offset,
             g_state.y_start + g_state.y_step, g_state.z_boot);
    set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
             g_state.z_boot);
    set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
             g_state.z_boot);

    for (int leg = 0; leg < NB_LEGS; leg++)
        for (int joint = 0; joint < NB_JOINTS; joint++)
            g_state.site_now[leg][joint] = g_state.site_expect[leg][joint];
    k_mutex_unlock(&g_state_mutex);

    k_msleep(1000);
    stand(0);

    while (true)
    {
        struct tcp_command cmd;

        k_msgq_get(&tcp_command_q, &cmd, K_FOREVER);
        LOG_DBG("Received: command: %s, times: %d", cmd.command, cmd.times);
        process_tcp_command(&cmd);
    }
}

K_THREAD_DEFINE(gait_thread_id, GAIT_STACK_SIZE, gait_thread, NULL, NULL, NULL,
                GAIT_THREAD_PRIORITY, K_USER, 2000);

void wait_all_reach(void)
{
    while (true)
    {
        bool motion_is_complete = true;

        if (k_mutex_lock(&g_state_mutex, K_MSEC(10)) != 0)
        {
            LOG_ERR("wait_all_reach: Failed to lock mutex");
            k_msleep(20);
            continue;
        }

        for (int leg = 0; leg < NB_LEGS; leg++)
        {
            for (int joint = 0; joint < NB_JOINTS; joint++)
            {
                if (g_state.site_now[leg][joint] !=
                    g_state.site_expect[leg][joint])
                {
                    motion_is_complete = false;
                    break;
                }
            }
            if (!motion_is_complete)
                break;
        }

        k_mutex_unlock(&g_state_mutex);

        if (motion_is_complete)
            break;

        k_msleep(2); // WARN: how frequent should this be checked
    }
    LOG_DBG("Motion done");
}
