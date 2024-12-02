/*
 * Copyright (c) 2024 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include "zephyr/kernel.h"
#include "zephyr/devicetree.h"
#include "zephyr/sys/printk.h"
#include "zephyr/ztest_assert.h"
#include "zephyr/ztest_test.h"
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/counter.h>

struct counter_fixture {
	const struct device *counter;
	const struct device *counter2;
};

static void counter_callback_1(const struct device *dev, void *userdata)
{
	int *value = userdata;
	*value = 1;
}

static void counter_callback_2(const struct device *dev, void *userdata)
{
	int *value = userdata;
	*value += 1;
}

static void *counter_setup(void)
{
	static struct counter_fixture fixture = {
		.counter = DEVICE_DT_GET(DT_NODELABEL(counter1)),
		.counter2 = DEVICE_DT_GET(DT_NODELABEL(counter2)),
	};

	zassert_not_null(fixture.counter);
	zassert_not_null(fixture.counter2);
	return &fixture;
}

static void counter_before(void *f)
{
	struct counter_fixture *fixture = f;
	struct counter_top_cfg top_cfg = {.callback = NULL, .user_data = NULL, .flags = 0};
	top_cfg.ticks = UINT32_MAX;

	counter_set_top_value(fixture->counter, &top_cfg);
	counter_stop(fixture->counter);

	counter_set_top_value(fixture->counter2, &top_cfg);
	counter_stop(fixture->counter2);
}

ZTEST_SUITE(counter, NULL, counter_setup, counter_before, NULL, NULL);

ZTEST_F(counter, counter_starts)
{
	int ret = 1;
	ret = counter_start(fixture->counter);
	zassert_equal(0, ret, "Starting counter should return 0 but returned %d", ret);
}

ZTEST_F(counter, counter_stops)
{
	int ret = 1;
	ret = counter_start(fixture->counter);
	ret = counter_stop(fixture->counter);
	zassert_equal(0, ret, "Stopping counter should return 0 but returned %d", ret);
}

ZTEST_F(counter, counter_returns_frequency)
{
	uint32_t ret = 0;
	ret = counter_get_frequency(fixture->counter);
	zassert_equal(K_SECONDS(1).ticks, ret, "Frequency should be %d but is %u",
		      K_SECONDS(1).ticks, ret);
}

ZTEST_F(counter, counter_is_counting_up_flag)
{
	bool status = counter_is_counting_up(fixture->counter);
	zassert_true(status, "Counter 1 should be counting up");
}

ZTEST_F(counter, counter_is_counting_down_flag)
{
	bool status = counter_is_counting_up(fixture->counter2);
	zassert_false(status, "Counter 2 should be counting down");
}

ZTEST_F(counter, counter_is_counting_up)
{
	(void)counter_start(fixture->counter);
	(void)k_busy_wait(100000);
	uint32_t value = 0;
	(void)counter_get_value(fixture->counter, &value);
	uint32_t low = (uint32_t)K_MSEC(90).ticks;
	uint32_t high = (uint32_t)K_MSEC(110).ticks;
	zassert_true(low <= value && value <= high,
		     "Counter 1 value should be between %u and %u but is %u", low, high, value);
}

ZTEST_F(counter, counter_is_counting_down)
{
	(void)counter_start(fixture->counter2);
	(void)k_busy_wait(100000);
	uint32_t value = 0;
	(void)counter_get_value(fixture->counter2, &value);
	uint32_t low = UINT32_MAX - (uint32_t)K_MSEC(110).ticks;
	uint32_t high = UINT32_MAX - (uint32_t)K_MSEC(90).ticks;
	zassert_true(low <= value && value <= high,
		     "Counter 2 value should be between %u and %u but is %u", low, high, value);
}

ZTEST_F(counter, counter_get_top_value)
{
	uint32_t value = counter_get_top_value(fixture->counter);
	zassert_equal(UINT32_MAX, value, "Counter Top value should be %u but is %u", UINT32_MAX,
		      value);
}

ZTEST_F(counter, counter_set_top_value)
{
	uint32_t top_value = 200;
	struct counter_top_cfg top_cfg = {.callback = NULL, .user_data = NULL, .flags = 0};
	top_cfg.ticks = top_value;
	(void)counter_set_top_value(fixture->counter, &top_cfg);
	uint32_t value = counter_get_top_value(fixture->counter);
	zassert_equal(top_value, value, "Counter Top value should be %u but is %u", top_value, value);
}

ZTEST_F(counter, counter_top_callback_firing)
{
	uint32_t top_value = 10;
	int data = 0;
	struct counter_top_cfg top_cfg = {.callback = counter_callback_2, .user_data = &data, .flags = 0};
	top_cfg.ticks = top_value;
	(void)counter_set_top_value(fixture->counter, &top_cfg);
	k_msleep(150);
	counter_stop(fixture->counter);
	zassert_equal(1, data, "Top callback should have fired once but did fire %d times", data);
}
