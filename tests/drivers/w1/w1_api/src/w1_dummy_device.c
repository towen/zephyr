/*
 * Copyright (c) 2022 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT test_w1_dummy_device

#include <device.h>
#include <drivers/w1.h>

struct w1_dummy_api {
};

static const struct w1_dummy_api w1_dummy_api1 = {};

static int w1_dummy_init(const struct device *dev)
{
	return 0;
}

#define TEST_W1_DUMMY_DEVICE_DEFINE(inst)				\
	DEVICE_DT_INST_DEFINE(inst, w1_dummy_init, NULL, NULL, NULL,	\
			      POST_KERNEL, CONFIG_W1_INIT_PRIORITY,	\
			      &w1_dummy_api1);

DT_INST_FOREACH_STATUS_OKAY(TEST_W1_DUMMY_DEVICE_DEFINE)
