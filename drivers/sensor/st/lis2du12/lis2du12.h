/* ST Microelectronics LIS2DU12 3-axis accelerometer sensor driver
 *
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lis2du12.pdf
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LIS2DU12_LIS2DU12_H_
#define ZEPHYR_DRIVERS_SENSOR_LIS2DU12_LIS2DU12_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <stmemsc.h>
#include "lis2du12_reg.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

#define LIS2DU12_EN_BIT  0x01
#define LIS2DU12_DIS_BIT 0x00

/* Accel sensor sensitivity grain is 61 ug/LSB */
#define GAIN_UNIT_XL	(61LL)

#define SENSOR_G_DOUBLE	(SENSOR_G / 1000000.0)

struct lis2du12_config {
	stmdev_ctx_t ctx;
	union {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
		const struct i2c_dt_spec i2c;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
		const struct spi_dt_spec spi;
#endif
	} stmemsc_cfg;
	uint8_t accel_pm;
	uint8_t accel_odr;
	uint8_t accel_range;
	uint8_t drdy_pulsed;
#ifdef CONFIG_LIS2DU12_TRIGGER
	const struct gpio_dt_spec int1_gpio;
	const struct gpio_dt_spec int2_gpio;
	uint8_t drdy_pin;
	bool trig_enabled;
// #ifdef CONFIG_LIS2DU12_TAP
	uint8_t tap_x_en;
	uint8_t tap_y_en;
	uint8_t tap_z_en;
	uint8_t tap_mode; //zero on single tap. 1 on double tap
	uint8_t tap_threshold[3];
	uint8_t tap_shock;
	uint8_t tap_latency;
	uint8_t tap_quiet;
	uint8_t tap_axis_priority;
#endif /* CONFIG_LIS2DU12_TRIGGER */
};

union samples {
	uint8_t raw[6];
	struct {
		int16_t axis[3];
	};
} __aligned(2);

struct lis2du12_data {
	const struct device *dev;
	int16_t acc[3];
	uint32_t acc_gain;
	uint16_t accel_freq;
	uint8_t accel_fs;

#ifdef CONFIG_LIS2DU12_TRIGGER
	struct gpio_dt_spec *drdy_gpio; //TODO: rename drdy_gpio as it's now used for tap detection as well.

	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t handler_drdy_acc;
	const struct sensor_trigger *trig_drdy_acc;

#if defined(CONFIG_LIS2DU12_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_LIS2DU12_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_LIS2DU12_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
//#ifdef CONFIG_LIS2DU12_TAP
	sensor_trigger_handler_t tap_handler;
	const struct sensor_trigger *tap_trig;
	sensor_trigger_handler_t double_tap_handler;
	const struct sensor_trigger *double_tap_trig;
#endif /* CONFIG_LIS2DU12_TRIGGER */
};

#ifdef CONFIG_LIS2DU12_TRIGGER
int lis2du12_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int lis2du12_init_interrupt(const struct device *dev);

int lis2du12_set_tap_x_threshold(const struct device *dev, const uint8_t threshold);
int lis2du12_set_tap_y_threshold(const struct device *dev, const uint8_t threshold);
int lis2du12_set_tap_z_threshold(const struct device *dev, const uint8_t threshold);
int lis2du12_set_tap_all_threshold(const struct device *dev, const uint8_t threshold);

#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_LIS2DU12_LIS2DU12_H_ */
