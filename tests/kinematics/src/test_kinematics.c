#include "gait.h"
#include "robot_state.h"
#include <zephyr/ztest.h>

// Define the tolerance for float comparisons
const float TEST_TOLERANCE = 0.001f;

// This is a "test fixture setup" function. It runs once before the tests in
// this suite. We use it to initialize the robot state, which cartesian_to_polar
// depends on.
static void kinematics_suite_setup(void) { init_robot_state(); }

/**
 * @brief Test case using the values from our log analysis.
 */
ZTEST(kinematics_suite, test_ik_calculation_from_log)
{
    volatile float alpha, beta, gamma;
    volatile float x = 62.0f;
    volatile float y = 50.0f;
    volatile float z = -50.0f;

    // Run the function we want to test
    cartesian_to_polar(&alpha, &beta, &gamma, x, y, z);

    // Print the values calculated by Zephyr/ESP32 for comparison
    printk("--- ZEPHYR CALCULATION ---\n");
    printk("Output: alpha=%.4f, beta=%.4f, gamma=%.4f\n", alpha, beta, gamma);

    printk("--------------------------\n");

    // --- ASSERTION ---
    // These are the "ground truth" values from the working Arduino.
    // I am using the ones you found with grep.
    float expected_alpha = 28.9082f;
    float expected_beta = 52.2906f;
    float expected_gamma = 13.6005f;

    // zassert_within(actual, expected, tolerance, message)
    // This will check if the Zephyr calculation matches the Arduino's result.
    zassert_within(alpha, expected_alpha, TEST_TOLERANCE, "Alpha mismatch!");
    zassert_within(beta, expected_beta, TEST_TOLERANCE, "Beta mismatch!");
    zassert_within(gamma, expected_gamma, TEST_TOLERANCE, "Gamma mismatch!");
}

// This defines and registers the test suite, and links our setup function.
ZTEST_SUITE(kinematics_suite, NULL, kinematics_suite_setup, NULL, NULL, NULL);
