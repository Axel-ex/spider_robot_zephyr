#include "gait.h"
#include "robot_state.h"
#include "servos.h"
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gait_thread, LOG_LEVEL_DBG);
#define GAIT_STACK_SIZE 1024
#define GAIT_THREAD_PRIORITY 5

void gait_thread(void)
{

    // initialise the bot
    set_site(0, g_state.x_default - g_state.x_offset,
             g_state.y_start + g_state.y_step, g_state.z_boot);
    set_site(1, g_state.x_default - g_state.x_offset,
             g_state.y_start + g_state.y_step, g_state.z_boot);
    set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
             g_state.z_boot);
    set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
             g_state.z_boot);

    for (int leg = 0; leg < 4; leg++)
        for (int joint = 0; joint < 3; joint++)
            g_state.site_now[leg][joint] = g_state.site_expect[leg][joint];
    print_robot_state();

    LOG_DBG("All %zu servos are ready!", NB_SERVOS);
    k_msleep(1000);
    stand();
}

K_THREAD_DEFINE(gait_thread_id, GAIT_STACK_SIZE, gait_thread, NULL, NULL, NULL,
                GAIT_THREAD_PRIORITY, K_USER, 2000);

void wait_all_reach() { k_sem_take(&motion_finished, K_FOREVER); }

void sit(void)
{
    g_state.move_speed = g_state.stand_seat_speed;
    for (int leg = 0; leg < 4; leg++)
    {
        set_site(leg, KEEP, KEEP, g_state.z_boot);
    }
    wait_all_reach();
}

void stand(void)
{
    k_sem_reset(&motion_finished);
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    g_state.move_speed = g_state.stand_seat_speed;
    for (int leg = 0; leg < 4; leg++)
    {
        set_site(leg, KEEP, KEEP, g_state.z_default);
    }
    g_state.motion_state = MOTION_IN_PROGRESS;
    k_mutex_unlock(&g_state_mutex);

    wait_all_reach();
    LOG_DBG("Motion done");
}
