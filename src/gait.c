#include "robot_state.h"
#include "spider_robot.h"
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gait, LOG_LEVEL_DBG);

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

void turn_right(unsigned int step)
{
    // Copy speed to a local variable for consistency throughout the gait cycle
    double local_spot_turn_speed;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    local_spot_turn_speed = g_state.spot_turn_speed;
    k_mutex_unlock(&g_state_mutex);

    while (step-- > 0)
    {

        // Check which phase of the gait we are in, using a tolerance
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        bool leg_2_is_home =
            (fabs(g_state.site_now[2][1] - g_state.y_start) < EPSILON);

        k_mutex_unlock(&g_state_mutex);

        if (leg_2_is_home)
        {
            /*********************************/
            /* Phase 1: Move Legs 2 & 0      */
            /*********************************/

            // Lift leg 2
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_spot_turn_speed;
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // First turn shift (while leg 2 is up)
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

            // Place leg 2 down
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Second turn shift (all legs on the ground)
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

            // Lift leg 0
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Body shift to return leg 0 to home position
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

            // Place leg 0 down
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

            // Lift leg 1
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_spot_turn_speed;
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // First turn shift (while leg 1 is up)
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

            // Place leg 1 down
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_default);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Second turn shift (all legs on the ground)
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

            // Lift leg 3
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.turn_x0 + g_state.x_offset, g_state.turn_y0,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Body shift to return leg 3 to home position
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

            // Place leg 3 down
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
    // Copy speed values to local variables for a consistent gait cycle

    double local_leg_move_speed, local_body_move_speed;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    local_leg_move_speed = g_state.leg_move_speed;
    local_body_move_speed = g_state.body_move_speed;
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
            /* Move Leg 3                    */
            /*********************************/
            // Lift leg 3
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(3, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Move leg 3 to the forward position (in leg coordinates)
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(3, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Place leg 3 down
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
            // Lift leg 0
            k_mutex_lock(&g_state_mutex, K_FOREVER);

            g_state.move_speed = local_leg_move_speed;
            set_site(0, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Move leg 0 back to home

            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(0, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);

            wait_all_reach();

            // Place leg 0 down
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

            // Lift leg 1
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(1, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Move leg 1 to the forward position (in leg coordinates)
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(1, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);
            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Place leg 1 down

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
            // Lift leg 2
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            g_state.move_speed = local_leg_move_speed;
            set_site(2, g_state.x_default + g_state.x_offset,
                     g_state.y_start + 2 * g_state.y_step, g_state.z_up);

            k_mutex_unlock(&g_state_mutex);
            wait_all_reach();

            // Move leg 2 back to home
            k_mutex_lock(&g_state_mutex, K_FOREVER);
            set_site(2, g_state.x_default + g_state.x_offset, g_state.y_start,
                     g_state.z_up);
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

void body_left(unsigned int i)
{
    // Lock the mutex to ensure a consistent read of the current state
    // and to set the new target sites atomically.
    k_mutex_lock(&g_state_mutex, K_FOREVER);

    set_site(0, g_state.site_now[0][0] + i, KEEP, KEEP);
    set_site(1, g_state.site_now[1][0] + i, KEEP, KEEP);
    set_site(2, g_state.site_now[2][0] - i, KEEP, KEEP);
    set_site(3, g_state.site_now[3][0] - i, KEEP, KEEP);

    // Unlock the mutex after all commands have been issued
    k_mutex_unlock(&g_state_mutex);

    // Wait for all legs to reach their new target positions
    wait_all_reach();
}

void body_right(int i)
{
    // Lock the mutex to ensure a consistent read of the current state
    // and to set the new target sites atomically.
    k_mutex_lock(&g_state_mutex, K_FOREVER);

    set_site(0, g_state.site_now[0][0] - i, KEEP, KEEP);
    set_site(1, g_state.site_now[1][0] - i, KEEP, KEEP);
    set_site(2, g_state.site_now[2][0] + i, KEEP, KEEP);
    set_site(3, g_state.site_now[3][0] + i, KEEP, KEEP);

    // Unlock the mutex after all commands have been issued
    k_mutex_unlock(&g_state_mutex);

    // Wait for all legs to reach their new target positions
    wait_all_reach();
}

void hand_wave(unsigned int step)
{
    double x_tmp, y_tmp, z_tmp;
    double local_body_move_speed;

    // --- Start of Sequence ---

    // First, determine which leg to wave based on the robot's current stance
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    bool leg_3_is_home =
        (fabs(g_state.site_now[3][1] - g_state.y_start) < EPSILON);
    // Also grab the body_move_speed now for later use
    local_body_move_speed = g_state.body_move_speed;
    k_mutex_unlock(&g_state_mutex);

    if (leg_3_is_home)
    {
        /*********************************/
        /* Wave with Leg 2 (Front-Left)  */
        /*********************************/

        // 1. Set slow speed and shift body right for stability
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0; // Use a float for clarity
        k_mutex_unlock(&g_state_mutex);
        body_right(15); // This function is already thread-safe

        // 2. Store the current position of the leg we are about to lift

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        x_tmp = g_state.site_now[2][0];

        y_tmp = g_state.site_now[2][1];
        z_tmp = g_state.site_now[2][2];
        k_mutex_unlock(&g_state_mutex);

        // 3. Set waving speed and perform the wave
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

        // 4. Return the leg to its starting position
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        set_site(2, x_tmp, y_tmp, z_tmp);
        k_mutex_unlock(&g_state_mutex);
        wait_all_reach();

        // 5. Set slow speed and shift body back to center

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

        // 1. Set slow speed and shift body left for stability
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;

        k_mutex_unlock(&g_state_mutex);
        body_left(15); // This function is already thread-safe

        // 2. Store the current position of the leg we are about to lift
        k_mutex_lock(&g_state_mutex, K_FOREVER);

        x_tmp = g_state.site_now[0][0];
        y_tmp = g_state.site_now[0][1];
        z_tmp = g_state.site_now[0][2];
        k_mutex_unlock(&g_state_mutex);

        // 3. Set waving speed and perform the wave
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

        // 4. Return the leg to its starting position
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        set_site(0, x_tmp, y_tmp, z_tmp);
        k_mutex_unlock(&g_state_mutex);
        wait_all_reach();

        // 5. Set slow speed and shift body back to center
        k_mutex_lock(&g_state_mutex, K_FOREVER);

        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);

        body_right(15);
    }
}

void hand_shake(unsigned int step)
{
    // Using double for higher precision as requested
    double x_tmp, y_tmp, z_tmp;

    double local_body_move_speed;

    // --- Start of Sequence ---

    // First, determine which leg to use based on the robot's current stance
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    bool leg_3_is_home =
        (fabs(g_state.site_now[3][1] - g_state.y_start) < EPSILON);
    // Also grab the body_move_speed now for later use
    local_body_move_speed = g_state.body_move_speed;
    k_mutex_unlock(&g_state_mutex);

    if (leg_3_is_home)
    {
        /*************************************/
        /* Shake with Leg 2 (Front-Left)     */
        /*************************************/

        // 1. Set slow speed and shift body right for stability
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0; // Use a double literal
        k_mutex_unlock(&g_state_mutex);
        body_right(15); // This function is already thread-safe

        // 2. Store the current position of the leg we are about to lift
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        x_tmp = g_state.site_now[2][0];
        y_tmp = g_state.site_now[2][1];
        z_tmp = g_state.site_now[2][2];
        k_mutex_unlock(&g_state_mutex);

        // 3. Set shaking speed and perform the shake
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

        // 4. Return the leg to its starting position
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        set_site(2, x_tmp, y_tmp, z_tmp);
        k_mutex_unlock(&g_state_mutex);
        wait_all_reach();

        // 5. Set slow speed and shift body back to center
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_left(15); // This function is already thread-safe
    }
    else
    {
        /*************************************/
        /* Shake with Leg 0 (Front-Right)    */

        /*************************************/

        // 1. Set slow speed and shift body left for stability
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_left(15); // This function is already thread-safe

        // 2. Store the current position of the leg we are about to lift
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        x_tmp = g_state.site_now[0][0];
        y_tmp = g_state.site_now[0][1];
        z_tmp = g_state.site_now[0][2];
        k_mutex_unlock(&g_state_mutex);

        // 3. Set shaking speed and perform the shake
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

        // 4. Return the leg to its starting position
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        set_site(0, x_tmp, y_tmp, z_tmp);

        k_mutex_unlock(&g_state_mutex);
        wait_all_reach();

        // 5. Set slow speed and shift body back to center
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state.move_speed = 1.0;
        k_mutex_unlock(&g_state_mutex);
        body_right(15); // This function is already thread-safe
    }
}
