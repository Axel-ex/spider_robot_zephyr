#include "gait.h"
#include "robot_state.h"
#include "servos.h"
#include "tcp_command.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/util.h"
#include <math.h>
#include <string.h>
#include <sys/_types.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gait_thread, LOG_LEVEL_DBG);
#define GAIT_STACK_SIZE 1024
#define GAIT_THREAD_PRIORITY 5

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

    for (int leg = 0; leg < 4; leg++)
        for (int joint = 0; joint < 3; joint++)
            g_state.site_now[leg][joint] = g_state.site_expect[leg][joint];
    k_mutex_unlock(&g_state_mutex);

    LOG_DBG("All %zu servos are ready!", NB_SERVOS);
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

void wait_all_reach()
{
    while (true)
    {
        bool motion_is_complete = true;

        if (k_mutex_lock(&g_state_mutex, K_MSEC(10)) != 0)
        {
            LOG_ERR("wait_all_reach: Failed to lock mutex");
            k_msleep(20); // Avoid spinning on lock failure
            continue;
        }

        for (int leg = 0; leg < 4; leg++)
        {
            for (int joint = 0; joint < 3; joint++)
            {
                // Check if any joint is still far from its target
                if (g_state.site_now[leg][joint] !=
                    g_state.site_expect[leg][joint])
                {
                    motion_is_complete = false;
                    break; // Exit inner loop
                }
            }
            if (!motion_is_complete)
                break; // Exit outer loop
        }

        k_mutex_unlock(&g_state_mutex);

        if (motion_is_complete)
        {
            // All legs have reached their destination, exit the wait loop
            return;
        }

        k_msleep(20);
    }
}

/*=====================================================================*
 *                          MOVES
 *=====================================================================*/
void sit(unsigned int step)
{
    (void)step;

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

void stand(unsigned int step)
{
    (void)step;

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    g_state.move_speed = g_state.stand_seat_speed;
    for (int leg = 0; leg < 4; leg++)
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
        // FIX #1: Correctly check if the leg is home using a tolerance
        bool leg_2_is_home =
            (fabs(g_state.site_now[2][1] - g_state.y_start) < EPSILON);
        k_mutex_unlock(&g_state_mutex);

        if (leg_2_is_home)
        {
            /*********************************/
            /* Move Leg 2           */
            /*********************************/
            // Lift leg 2
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Move leg 2 forward
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Place leg 2 down
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
            // Lift leg 1
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(1, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Move leg 1 back to home
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Place leg 1 down
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
            // Lift leg 0
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);

            wait_all_reach();

            // Move leg 0 forward
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Place leg 0 down
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
            // Lift leg 3
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(3, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Move leg 3 back to home
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Place leg 3 down
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
    // Copy speed to a local variable for consistency throughout the gait cycle

    double local_spot_turn_speed;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    // Assuming g_state has a spot_turn_speed field, similar to the original
    local_spot_turn_speed = g_state.spot_turn_speed;
    k_mutex_unlock(&g_state_mutex);

    while (step-- > 0)
    {
        // Check which phase of the gait we are in, using a tolerance
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        bool leg_3_is_home =

            (fabs(g_state.site_now[3][1] - g_state.y_start) < EPSILON);
        k_mutex_unlock(&g_state_mutex);

        if (leg_3_is_home)

        {
            /*********************************/

            /* Phase 1: Move Legs 3 & 1      */
            /*********************************/

            // Lift leg 3
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_spot_turn_speed;
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // First turn shift (while leg 3 is up)
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

            // Place leg 3 down
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Second turn shift (all legs on the ground)
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

            // Lift leg 1
            k_mutex_lock(&g_state_mutex, K_FOREVER);

            set_site(1, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Body shift to return leg 1 to home position
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

            // Place leg 1 down
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

            // Lift leg 0

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_spot_turn_speed;
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // First turn shift (while leg 0 is up)
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

            // Place leg 0 down
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Second turn shift (all legs on the ground)
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

            // Lift leg 2
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Body shift to return leg 2 to home position
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

            // Place leg 2 down
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();
        }
    }
}
