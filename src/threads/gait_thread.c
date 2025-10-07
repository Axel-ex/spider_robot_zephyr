/*======================================================================
 * File:    gait_thread.c
 * Date:    2025-10-07
 * Purpose: Listens for command from the TCP server thread and execute a
 *precomputed sequence of legs moves. the execution of the gait is done by
 *locking and modifying a global state that is shared with the motor thread.
 *this thread then wait for the movement to be done before proceeding to the
 *next one.
 *====================================================================*/
#include "robot_state.h"
#include "servos.h"
#include "spider_robot.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/util.h"
#include <sys/_types.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gait_thread, LOG_LEVEL_DBG);
#define GAIT_STACK_SIZE 1024
#define GAIT_THREAD_PRIORITY 5

static const struct cmd_entry cmd_table[] = {
    {"sit", sit},          {"stand", stand},   {"sf", step_forward},
    {"sb", step_back},     {"tl", turn_left},  {"tr", turn_right},
    {"shake", hand_shake}, {"wave", hand_wave}};

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
