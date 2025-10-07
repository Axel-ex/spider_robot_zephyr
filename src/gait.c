/*======================================================================
 * File:    gait.c
 * Date:    2025-10-07
 * Purpose: Implements gait and movement routines for the spider robot,
 *including walking, turning, and gesture behaviors. locks the g_state mutex and
 *sets the expected positions of the legs
 *====================================================================*/
#include "robot_state.h"
#include "servos.h"
#include "spider_robot.h"
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gait, LOG_LEVEL_DBG);

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

void sit(unsigned int step)
{
    (void)step;

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    g_state.move_speed = g_state.stand_seat_speed;
    for (int leg = 0; leg < NB_LEGS; leg++)
    {
        set_site(leg, KEEP, KEEP, g_state.z_boot);
    }
    k_mutex_unlock(&g_state_mutex);
    wait_all_reach();
}

void stand(unsigned int step)
{
    (void)step;

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    g_state.move_speed = g_state.stand_seat_speed;
    for (int leg = 0; leg < NB_LEGS; leg++)
        set_site(leg, KEEP, KEEP, g_state.z_default);
    k_mutex_unlock(&g_state_mutex);

    wait_all_reach();
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
        k_mutex_unlock(&g_state_mutex);

        if (leg_2_is_home)
        {
            /*********************************/
            /* Move Leg 2           */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            /*********************************/
            /* Shift Body           */
            /*********************************/
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

            /*********************************/
            /* Move Leg 1           */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(1, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
        else
        {
            /*********************************/
            /* Move Leg 0           */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);

            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            /*********************************/
            /* Shift Body           */
            /*********************************/
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

            /*********************************/
            /* Move Leg 3           */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(3, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
    }
}

void turn_left(unsigned int step)
{
    double local_spot_turn_speed;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    local_spot_turn_speed = g_state.spot_turn_speed;
    k_mutex_unlock(&g_state_mutex);

    while (step-- > 0)
    {
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        bool leg_3_is_home =

            (fabs(g_state.site_now[3][1] - g_state.y_start) < EPSILON);
        k_mutex_unlock(&g_state_mutex);

        if (leg_3_is_home)

        {
            /*********************************/

            /* Phase 1: Move Legs 3 & 1      */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_spot_turn_speed;
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x1 - g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);

            set_site(1, g_state.turn_x0 - g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            set_site(2, g_state.turn_x1 + g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(3, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x1 + g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(1, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            set_site(2, g_state.turn_x1 - g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(3, g_state.turn_x0 - g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            set_site(2, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(3, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
        else
        {
            /*********************************/
            /* Phase 2: Move Legs 0 & 2      */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_spot_turn_speed;
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            set_site(1, g_state.turn_x1 + g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(2, g_state.turn_x0 - g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            set_site(3, g_state.turn_x1 - g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x0 - g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            set_site(1, g_state.turn_x1 - g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(2, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            set_site(3, g_state.turn_x1 + g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(1, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
    }
}

void turn_right(unsigned int step)
{
    double local_spot_turn_speed;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    local_spot_turn_speed = g_state.spot_turn_speed;
    k_mutex_unlock(&g_state_mutex);

    while (step-- > 0)
    {

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        bool leg_2_is_home =
            (fabs(g_state.site_now[2][1] - g_state.y_start) < EPSILON);

        k_mutex_unlock(&g_state_mutex);

        if (leg_2_is_home)
        {
            /*********************************/
            /* Phase 1: Move Legs 2 & 0      */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_spot_turn_speed;
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);

            set_site(0, g_state.turn_x0 - g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            set_site(1, g_state.turn_x1 - g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(2, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            set_site(3, g_state.turn_x1 + g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            set_site(1, g_state.turn_x1 + g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(2, g_state.turn_x0 - g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);

            set_site(3, g_state.turn_x1 - g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            set_site(2, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(3, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
        else
        {
            /*********************************/
            /* Phase 2: Move Legs 1 & 3      */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_spot_turn_speed;
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x1 + g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(1, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            set_site(2, g_state.turn_x1 - g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(3, g_state.turn_x0 - g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x1 - g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(1, g_state.turn_x0 - g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            set_site(2, g_state.turn_x1 + g_state.x_offset, g_state.turn_y1,
                     g_state.z_default);
            set_site(3, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(1, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);

            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
    }
}

void step_back(unsigned int step)
{
    double local_leg_move_speed, local_body_move_speed;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    local_leg_move_speed = g_state.leg_move_speed;
    local_body_move_speed = g_state.body_move_speed;
    k_mutex_unlock(&g_state_mutex);

    while (step-- > 0)
    {
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        bool leg_3_is_home =
            (fabs(g_state.site_now[3][1] - g_state.y_start) < EPSILON);
        k_mutex_unlock(&g_state_mutex);

        if (leg_3_is_home)
        {
            /*********************************/
            /* Move Leg 3                    */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.x_default + g_state.x_offset,

                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            /*********************************/
            /* Shift Body Backward           */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_body_move_speed;
            set_site(0, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            set_site(2, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);

            set_site(3, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            /*********************************/
            /* Move Leg 0                    */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);

            g_state.move_speed = local_leg_move_speed;
            set_site(0, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);

            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }

        else
        {
            /*********************************/
            /* Move Leg 1                    */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            /*********************************/
            /* Shift Body Backward           */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_body_move_speed;
            set_site(0, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(1, g_state.x_default - g_state.x_offset,
                     g_state.y_start + g_state.y_step, g_state.z_default);
            set_site(2, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_default);
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            /*********************************/
            /* Move Leg 2                    */
            /*********************************/
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(2, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
    }
}

void body_left(unsigned int i)
{
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    set_site(0, g_state.site_now[0][0] + i, KEEP, KEEP);
    set_site(1, g_state.site_now[1][0] + i, KEEP, KEEP);
    set_site(2, g_state.site_now[2][0] - i, KEEP, KEEP);
    set_site(3, g_state.site_now[3][0] - i, KEEP, KEEP);
    k_mutex_unlock(&g_state_mutex);

    wait_all_reach();
}

void body_right(int i)
{
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    set_site(0, g_state.site_now[0][0] - i, KEEP, KEEP);
    set_site(1, g_state.site_now[1][0] - i, KEEP, KEEP);
    set_site(2, g_state.site_now[2][0] + i, KEEP, KEEP);
    set_site(3, g_state.site_now[3][0] + i, KEEP, KEEP);
    k_mutex_unlock(&g_state_mutex);

    wait_all_reach();
}

void hand_wave(unsigned int step)
{
    double x_tmp, y_tmp, z_tmp;
    double local_body_move_speed;

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    bool leg_3_is_home =
        (fabs(g_state.site_now[3][1] - g_state.y_start) < EPSILON);
    local_body_move_speed = g_state.body_move_speed;
    k_mutex_unlock(&g_state_mutex);

    if (leg_3_is_home)
    {
        /*********************************/
        /* Wave with Leg 2 (Front-Left)  */
        /*********************************/
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_right(15);

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        x_tmp = g_state.site_now[2][0];

        y_tmp = g_state.site_now[2][1];
        z_tmp = g_state.site_now[2][2];
        k_mutex_unlock(&g_state_mutex);

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = local_body_move_speed;
        k_mutex_unlock(&g_state_mutex);

        for (int j = 0; j < step; j++)
        {
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.turn_x1, g_state.turn_y1, 50.0);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.turn_x0, g_state.turn_y0, 50.0);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        set_site(2, x_tmp, y_tmp, z_tmp);
        k_mutex_unlock(&g_state_mutex);
        wait_all_reach();

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_left(15); // This function is already thread-safe
    }
    else
    {
        /*********************************/
        /* Wave with Leg 0 (Front-Right) */
        /*********************************/
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;

        k_mutex_unlock(&g_state_mutex);
        body_left(15);
        k_mutex_lock(&g_state_mutex, K_FOREVER);

        x_tmp = g_state.site_now[0][0];
        y_tmp = g_state.site_now[0][1];
        z_tmp = g_state.site_now[0][2];
        k_mutex_unlock(&g_state_mutex);

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = local_body_move_speed;

        k_mutex_unlock(&g_state_mutex);

        for (int j = 0; j < step; j++)
        {
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x1, g_state.turn_y1, 50.0);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x0, g_state.turn_y0, 50.0);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        set_site(0, x_tmp, y_tmp, z_tmp);
        k_mutex_unlock(&g_state_mutex);
        wait_all_reach();

        k_mutex_lock(&g_state_mutex, K_FOREVER);

        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);

        body_right(15);
    }
}

void hand_shake(unsigned int step)
{
    double x_tmp, y_tmp, z_tmp;
    double local_body_move_speed;

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    bool leg_3_is_home =
        (fabs(g_state.site_now[3][1] - g_state.y_start) < EPSILON);
    local_body_move_speed = g_state.body_move_speed;
    k_mutex_unlock(&g_state_mutex);

    if (leg_3_is_home)
    {
        /*************************************/
        /* Shake with Leg 2 (Front-Left)     */
        /*************************************/
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_right(15);
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        x_tmp = g_state.site_now[2][0];
        y_tmp = g_state.site_now[2][1];
        z_tmp = g_state.site_now[2][2];
        k_mutex_unlock(&g_state_mutex);

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = local_body_move_speed;

        k_mutex_unlock(&g_state_mutex);

        for (int j = 0; j < step; j++)
        {
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default - 30.0,
                     g_state.y_start + 2.0 * g_state.y_step, 55.0);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default - 30.0,
                     g_state.y_start + 2.0 * g_state.y_step, 10.0);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        set_site(2, x_tmp, y_tmp, z_tmp);
        k_mutex_unlock(&g_state_mutex);
        wait_all_reach();

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_left(15);
    }
    else
    {
        /*************************************/
        /* Shake with Leg 0 (Front-Right)    */
        /*************************************/
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_left(15);

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        x_tmp = g_state.site_now[0][0];
        y_tmp = g_state.site_now[0][1];
        z_tmp = g_state.site_now[0][2];
        k_mutex_unlock(&g_state_mutex);

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = local_body_move_speed;
        k_mutex_unlock(&g_state_mutex);

        for (int j = 0; j < step; j++)
        {
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default - 30.0,
                     g_state.y_start + 2.0 * g_state.y_step, 55.0);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default - 30.0,
                     g_state.y_start + 2.0 * g_state.y_step, 10.0);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        set_site(0, x_tmp, y_tmp, z_tmp);

        k_mutex_unlock(&g_state_mutex);
        wait_all_reach();

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_right(15);
    }
}
