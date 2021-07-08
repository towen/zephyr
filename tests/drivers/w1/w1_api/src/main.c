/*
 * Copyright (c) 2022 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <ztest.h>

extern void test_w1_basic(void);
extern void test_w1_crc(void);
extern void test_w1_rom(void);
extern void test_w1_reset_empty(void);
extern void test_w1_search_empty(void);
extern void test_w1_fire_and_forget(void);
extern void test_w1_receive_nothing(void);
extern void test_w1_peripheral(void);

extern void test_w1_simple(void);
extern const struct device *get_w1_controller_dev(void);
extern const struct device *get_w1_dummy_device_1(void);
extern const struct device *get_w1_dummy_device_2(void);

void test_main(void)
{
	k_object_access_grant(get_w1_controller_dev(), k_current_get());

	ztest_test_suite(w1_api_test,
			 ztest_user_unit_test(test_w1_basic),
			 ztest_user_unit_test(test_w1_crc),
			 ztest_user_unit_test(test_w1_rom),
			 ztest_user_unit_test(test_w1_reset_empty),
			 ztest_user_unit_test(test_w1_search_empty),
			 ztest_user_unit_test(test_w1_fire_and_forget),
			 ztest_user_unit_test(test_w1_receive_nothing),
			 ztest_user_unit_test(test_w1_peripheral));
	ztest_run_test_suite(w1_api_test);
}
