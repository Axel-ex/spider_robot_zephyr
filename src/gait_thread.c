#include "gait.h"
#include "robot_state.h"
#include "servos.h"
#include "zephyr/kernel.h"
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gait_thread, LOG_LEVEL_DBG);
#define GAIT_STACK_SIZE 1024
#define GAIT_THREAD_PRIORITY 5

void set_site(int leg, double x, double y, double z)
{
    LOG_DBG("set_site(leg=%d) called with target (x:%.2f, y:%.2f, z:%.2f). "
            "Current pos is (y:%.2f)",
            leg, x, y, z, g_state.site_now[leg][1]);
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

void gait_thread(void)
{

    // initialise the bot
    k_mutex_lock(&g_state_mutex, K_FOREVER);
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
    k_mutex_unlock(&g_state_mutex);

    print_robot_state();

    LOG_DBG("All %zu servos are ready!", NB_SERVOS);
    k_msleep(1000);
    stand();
    k_msleep(5000);
    step_forward(5);
}

K_THREAD_DEFINE(gait_thread_id, GAIT_STACK_SIZE, gait_thread, NULL, NULL, NULL,
                GAIT_THREAD_PRIORITY, K_USER, 2000);

void wait_all_reach()
{
    k_sem_reset(&motion_finished);
    k_sem_take(&motion_finished, K_FOREVER);
}

void sit(void)
{
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    g_state.move_speed = g_state.stand_seat_speed;
    for (int leg = 0; leg < 4; leg++)
    {
        set_site(leg, KEEP, KEEP, g_state.z_boot);
    }
    k_mutex_unlock(&g_state_mutex);
    wait_all_reach();
    LOG_DBG("Motion done");
}

void stand(void)
{

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    g_state.move_speed = g_state.stand_seat_speed;
    for (int leg = 0; leg < 4; leg++)
        set_site(leg, KEEP, KEEP, g_state.z_default);
    k_mutex_unlock(&g_state_mutex);

    LOG_DBG("Gait thread now waiting for motion to finish...");
    wait_all_reach();
    LOG_DBG("Motion done");
}

void step_forward(unsigned int step)
{
    double local_leg_move_speed, local_body_move_speed;

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    local_leg_move_speed = g_state.leg_move_speed;
    local_body_move_speed = g_state.body_move_speed;
    k_mutex_unlock(&g_state_mutex);

    while (step-- > 0)
    {
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        bool leg_2_is_home =
            (fabs(g_state.site_now[2][1] - g_state.y_start) < EPSILON);
        double current_leg2_y = g_state.site_now[2][1];
        k_mutex_unlock(&g_state_mutex);
        // Add this log right after the check
        //
        LOG_DBG("step_forward loop #%d: leg 2 y is %.2f, condition "
                "'leg_2_is_home' is %s",
                step, current_leg2_y, leg_2_is_home ? "true" : "false");

        if (leg_2_is_home)
        {
            // --- Gait Step 1: Leg 2 lifts up ---
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
            LOG_DBG("done");

            // --- Gait Step 2: Leg 2 moves forward ---
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
            LOG_DBG("done");

            // --- Gait Step 3: Leg 2 moves down to new position ---
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
            LOG_DBG("done");

            // --- Gait Step 4: Body moves to new center of gravity ---
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_body_move_speed;
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            set_site(1, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            set_site(2, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(3, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
        else
        {
            // --- Gait Step 5: Leg 0 lifts up ---
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // --- Gait Step 6: Leg 0 moves forward ---
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // --- Gait Step 7: Leg 0 moves down to new position ---
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // --- Gait Step 8: Body moves to new center of gravity ---
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_body_move_speed;
            set_site(0, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(1, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            set_site(3, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
    }
}
