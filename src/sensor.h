/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SC_SENSOR_H_
#define SC_SENSOR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

struct sc_lm75_sensor_data {
	const struct device *dev;
	struct sensor_value temp;
	int ret;
	int64_t updated_at_ms;
};

struct sc_ina3221_channel_data {
	bool enabled;
	struct sensor_value voltage;
	struct sensor_value current;
	struct sensor_value power;
	int ret;
};

struct sc_ina3221_sensor_data {
	const struct device *dev;
	struct sc_ina3221_channel_data channel[3];
	int ret;
	int64_t updated_at_ms;
};

size_t sc_lm75_get_sensor_count(void);
size_t sc_ina3221_get_sensor_count(void);

int sc_lm75_get_sensor(size_t index, struct sc_lm75_sensor_data *data);
int sc_ina3221_get_sensor(size_t index, struct sc_ina3221_sensor_data *data);

#endif /* SC_SENSOR_H_ */
