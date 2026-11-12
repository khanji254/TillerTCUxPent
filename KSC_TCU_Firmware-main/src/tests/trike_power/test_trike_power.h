#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Runs an infinite loop toggling the BJT kill switch on IO21
 *        to verify hardware operation before integrating with the dashboard.
 */
void test_trike_power_run(void);

#ifdef __cplusplus
}
#endif