/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "ipc.h"
#include "sensor.h"

#ifndef SENSOR_ATTR_INA3221_SELECTED_CHANNEL
#define SENSOR_ATTR_INA3221_SELECTED_CHANNEL	(SENSOR_ATTR_PRIV_START + 1)
#endif

#define SENSOR_THREAD_STACK_SIZE	2048
#define SENSOR_THREAD_PRIORITY		K_PRIO_PREEMPT(1)
#define SENSOR_FETCH_INTERVAL		K_MSEC(5000)

#define LM75_ENTRY(node_id)			\
	{					\
		.dev = DEVICE_DT_GET(node_id),	\
		.ret = -EAGAIN,			\
		.updated_at_ms = 0,		\
	},

#define INA3221_CHANNEL_ENTRY(node_id, ch)				\
	{								\
		.enabled = DT_PROP_BY_IDX(node_id, enable_channel, ch),	\
		.ret = -EAGAIN,						\
	}

#define INA3221_ENTRY(node_id)					\
	{							\
		.dev = DEVICE_DT_GET(node_id),			\
		.channel = {					\
			INA3221_CHANNEL_ENTRY(node_id, 0),	\
			INA3221_CHANNEL_ENTRY(node_id, 1),	\
			INA3221_CHANNEL_ENTRY(node_id, 2),	\
		},						\
		.ret = -EAGAIN,					\
		.updated_at_ms = 0,				\
	},

static struct sc_lm75_sensor_data lm75_sensors[] = {
	DT_FOREACH_STATUS_OKAY(lm75, LM75_ENTRY)
};

static struct sc_ina3221_sensor_data ina3221_sensors[] = {
	DT_FOREACH_STATUS_OKAY(ti_ina3221, INA3221_ENTRY)
};

static K_MUTEX_DEFINE(sensor_cache_lock);

static int ina3221_select_channel(const struct device *dev, int channel)
{
	struct sensor_value value = {
		.val1 = channel,
		.val2 = 0,
	};

	return sensor_attr_set(dev, SENSOR_CHAN_VOLTAGE,
			       SENSOR_ATTR_INA3221_SELECTED_CHANNEL, &value);
}

static void lm75_reset_result(struct sc_lm75_sensor_data *data)
{
	data->ret = -EAGAIN;
	data->updated_at_ms = 0;
}

static void ina3221_reset_result(struct sc_ina3221_sensor_data *data)
{
	size_t ch;

	data->ret = -EAGAIN;
	data->updated_at_ms = 0;

	for (ch = 0; ch < ARRAY_SIZE(data->channel); ch++) {
		data->channel[ch].ret = -EAGAIN;
	}
}

static int fetch_lm75_sensor(struct sc_lm75_sensor_data *data)
{
	const struct device *dev = data->dev;
	int ret;

	lm75_reset_result(data);

	if (!device_is_ready(dev)) {
		data->ret = -ENODEV;
		return data->ret;
	}

	ret = sensor_sample_fetch(dev);
	if (ret < 0) {
		data->ret = ret;
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &data->temp);
	if (ret < 0) {
		data->ret = ret;
		return ret;
	}

	data->ret = 0;
	data->updated_at_ms = k_uptime_get();

	return 0;
}

static int fetch_ina3221_channel(const struct device *dev, size_t ch,
					 struct sc_ina3221_channel_data *data)
{
	int ret;

	if (!data->enabled) {
		return 0;
	}

	ret = ina3221_select_channel(dev, (int)ch + 1);
	if (ret < 0) {
		data->ret = ret;
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &data->voltage);
	if (ret < 0) {
		data->ret = ret;
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &data->current);
	if (ret < 0) {
		data->ret = ret;
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_POWER, &data->power);
	if (ret < 0) {
		data->ret = ret;
		return ret;
	}

	data->ret = 0;

	return 0;
}

static int fetch_ina3221_sensor(struct sc_ina3221_sensor_data *data)
{
	const struct device *dev = data->dev;
	int first_error = 0;
	int ret;
	size_t ch;

	ina3221_reset_result(data);

	if (!device_is_ready(dev)) {
		data->ret = -ENODEV;
		return data->ret;
	}

	ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	if (ret < 0) {
		data->ret = ret;
		return ret;
	}

	for (ch = 0; ch < ARRAY_SIZE(data->channel); ch++) {
		ret = fetch_ina3221_channel(dev, ch, &data->channel[ch]);
		if (ret < 0 && first_error == 0) {
			first_error = ret;
		}
	}

	data->ret = first_error;
	data->updated_at_ms = k_uptime_get();

	return first_error;
}

static int fetch_lm75_sensors(void)
{
	struct sc_lm75_sensor_data data;
	int first_error = 0;
	int ret;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lm75_sensors); i++) {
		k_mutex_lock(&sensor_cache_lock, K_FOREVER);
		data = lm75_sensors[i];
		k_mutex_unlock(&sensor_cache_lock);

		ret = fetch_lm75_sensor(&data);

		k_mutex_lock(&sensor_cache_lock, K_FOREVER);
		lm75_sensors[i] = data;
		k_mutex_unlock(&sensor_cache_lock);

		if (ret < 0 && first_error == 0) {
			first_error = ret;
		}
	}

	return first_error;
}

static int fetch_ina3221_sensors(void)
{
	struct sc_ina3221_sensor_data data;
	int first_error = 0;
	int ret;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ina3221_sensors); i++) {
		k_mutex_lock(&sensor_cache_lock, K_FOREVER);
		data = ina3221_sensors[i];
		k_mutex_unlock(&sensor_cache_lock);

		ret = fetch_ina3221_sensor(&data);

		k_mutex_lock(&sensor_cache_lock, K_FOREVER);
		ina3221_sensors[i] = data;
		k_mutex_unlock(&sensor_cache_lock);

		if (ret < 0 && first_error == 0) {
			first_error = ret;
		}
	}

	return first_error;
}

static int fetch_all_sensors(void)
{
	int first_error = 0;
	int ret;

	ret = fetch_lm75_sensors();
	if (ret < 0 && first_error == 0) {
		first_error = ret;
	}

	ret = fetch_ina3221_sensors();
	if (ret < 0 && first_error == 0) {
		first_error = ret;
	}

	return first_error;
}

static void sensor_update_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		(void)fetch_all_sensors();
		sc_ipc_update_response_cache();
		k_sleep(SENSOR_FETCH_INTERVAL);
	}
}

K_THREAD_DEFINE(sensor_thread_id, SENSOR_THREAD_STACK_SIZE,
		sensor_update_thread, NULL, NULL, NULL,
		SENSOR_THREAD_PRIORITY, 0, 0);

size_t sc_lm75_get_sensor_count(void)
{
	return ARRAY_SIZE(lm75_sensors);
}

size_t sc_ina3221_get_sensor_count(void)
{
	return ARRAY_SIZE(ina3221_sensors);
}

int sc_lm75_get_sensor(size_t index, struct sc_lm75_sensor_data *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	if (index >= ARRAY_SIZE(lm75_sensors)) {
		return -ENOENT;
	}

	k_mutex_lock(&sensor_cache_lock, K_FOREVER);
	*data = lm75_sensors[index];
	k_mutex_unlock(&sensor_cache_lock);

	return 0;
}

int sc_ina3221_get_sensor(size_t index, struct sc_ina3221_sensor_data *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	if (index >= ARRAY_SIZE(ina3221_sensors)) {
		return -ENOENT;
	}

	k_mutex_lock(&sensor_cache_lock, K_FOREVER);
	*data = ina3221_sensors[index];
	k_mutex_unlock(&sensor_cache_lock);

	return 0;
}
