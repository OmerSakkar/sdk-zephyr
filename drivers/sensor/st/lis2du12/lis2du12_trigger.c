/* ST Microelectronics LIS2DU12 3-axis accelerometer sensor driver
 *
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lis2du12.pdf
 */

#define DT_DRV_COMPAT st_lis2du12

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "lis2du12.h"

LOG_MODULE_DECLARE(LIS2DU12, CONFIG_SENSOR_LOG_LEVEL);

/**sensor_trigger_set
 * lis2du12_enable_xl_int - XL enable selected int pin to generate interrupt
 */
static int lis2du12_enable_xl_int(const struct device *dev,
							enum sensor_trigger_type type, int enable)
{
	const struct lis2du12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2du12_pin_int_route_t val;
	int ret;


	switch (type) {
	case SENSOR_TRIG_DATA_READY:
		if (enable) {
			lis2du12_md_t md;
			lis2du12_data_t xl_data;

			/* dummy read: re-trigger interrupt */
			md.fs = cfg->accel_range;
			lis2du12_data_get(ctx, &md, &xl_data);
		}

		if (cfg->drdy_pin == 1) {

			ret = lis2du12_pin_int1_route_get(ctx, &val);
			if (ret < 0) {
				LOG_ERR("pin_int1_route_get error");
				return ret;
			}

			val.drdy_xl = enable;

			ret = lis2du12_pin_int1_route_set(ctx, &val);
		} else {

			ret = lis2du12_pin_int2_route_get(ctx, &val);
			if (ret < 0) {
				LOG_ERR("pin_int2_route_get error");
				return ret;
			}

			val.drdy_xl = enable;

			ret = lis2du12_pin_int2_route_set(ctx, &val);
		}
		break;
// #ifdef CONFIG_LIS2DU12_TAP
	case SENSOR_TRIG_TAP:

		/*set interrupt for pin INT1*/
		if (cfg->drdy_pin == 1) {
			ret = lis2du12_pin_int1_route_get(ctx, &val);
			if (ret < 0) {
				LOG_ERR("pin_int1_route_get error");
				return ret;
			}

			val.single_tap = enable;

			ret = lis2du12_pin_int1_route_set(ctx, &val);
		} else {
			ret = lis2du12_pin_int2_route_get(ctx, &val);
			if (ret < 0) {
				LOG_ERR("pin_int1_route_get error");
				return ret;
			}

			val.single_tap = enable;

			ret = lis2du12_pin_int2_route_set(ctx, &val);
		}
		break;
	case SENSOR_TRIG_DOUBLE_TAP:

		/*set interrupt for pin INT1*/
		if (cfg->drdy_pin == 1) {
			ret = lis2du12_pin_int1_route_get(ctx, &val);
			if (ret < 0) {
				LOG_ERR("pin_int1_route_get error");
				return ret;
			}

			val.double_tap = enable;

			ret = lis2du12_pin_int1_route_set(ctx, &val);
		} else {
			ret = lis2du12_pin_int2_route_get(ctx, &val);
			if (ret < 0) {
				LOG_ERR("pin_int1_route_get error");
				return ret;
			}

			val.double_tap = enable;

			ret = lis2du12_pin_int2_route_set(ctx, &val);
		}

		break;
	default:
		LOG_ERR("Unsupported trigger interrupt route %d", type);
		return -ENOTSUP;
	}
	return ret;
}

/**
 * lis2du12_trigger_set - link external trigger to event data ready
 */
int lis2du12_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	const struct lis2du12_config *cfg = dev->config;
	struct lis2du12_data *lis2du12 = dev->data;
	int state = (handler != NULL) ? LIS2DU12_EN_BIT : LIS2DU12_DIS_BIT;


	if (!cfg->trig_enabled) {
		LOG_ERR("trigger_set op not supported");
		return -ENOTSUP;
	}

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		switch (trig->chan) {
			case SENSOR_CHAN_ACCEL_XYZ:
				lis2du12->handler_drdy_acc = handler;
				lis2du12->trig_drdy_acc = trig;
				return lis2du12_enable_xl_int(dev, SENSOR_TRIG_DATA_READY, state);
			default:
				return -ENOTSUP;
		}
		break;
// #ifdef CONFIG_LIS2DU12_TAP
	case SENSOR_TRIG_TAP:
	case SENSOR_TRIG_DOUBLE_TAP:
		/*check if tap detection is enabled*/
		if ((cfg->tap_threshold[0] == 0) &&
			(cfg->tap_threshold[1] == 0) &&
			(cfg->tap_threshold[2] == 0)) {
			LOG_ERR("Unsupported sensor trigger");
			return -ENOTSUP;
		}

		/* set single tap trigger*/
		if (trig->type == SENSOR_TRIG_TAP) {
			lis2du12->tap_handler = handler;
			lis2du12->tap_trig = trig;
			return lis2du12_enable_xl_int(dev, SENSOR_TRIG_TAP, state);
		}

		/* set double tap trigger*/
		lis2du12->double_tap_handler = handler;
		lis2du12->double_tap_trig = trig;
		return lis2du12_enable_xl_int(dev, SENSOR_TRIG_DOUBLE_TAP, state);

	default:
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}

}

/**
 * lis2du12_handle_interrupt - handle the drdy event
 * read data and call handler if registered any
 */

/*
 * Note: This function assumes that interrupt will only fire on single tap event.
 * It should be changed asap.
*/
static void lis2du12_handle_interrupt(const struct device *dev)
{
	LOG_INF("handling interrrupt");
	struct lis2du12_data *lis2du12 = dev->data;
	const struct lis2du12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2du12_all_sources_t all_sources;

	if (lis2du12_all_sources_get(ctx, &all_sources) < 0) {
		LOG_ERR("lis2du12_all_sources_get error");
		return;
	}

	if ((all_sources.single_tap == 1) && (lis2du12->tap_handler != NULL)) {
		LOG_INF("Single tap event detected");
		lis2du12->tap_handler(dev, lis2du12->tap_trig); /* Call the handler defined by the use. */
	}
/*
	while (1) {
		if (lis2du12_status_get(ctx, &status) < 0) {
			LOG_ERR("failed reading status reg");
			return;
		}

		if (status.drdy_xl == 0) {
			break;
		}

		if ((status.drdy_xl) && (lis2du12->handler_drdy_acc != NULL)) {
			lis2du12->handler_drdy_acc(dev, lis2du12->trig_drdy_acc);
		}
	}
*/
	gpio_pin_interrupt_configure_dt(lis2du12->drdy_gpio,
					GPIO_INT_EDGE_TO_ACTIVE);
}

static void lis2du12_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	LOG_INF("lis2du12_gpio_callback is called");
	struct lis2du12_data *lis2du12 =
		CONTAINER_OF(cb, struct lis2du12_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(lis2du12->drdy_gpio, GPIO_INT_DISABLE);

#if defined(CONFIG_LIS2DU12_TRIGGER_OWN_THREAD)
	k_sem_give(&lis2du12->gpio_sem);
#elif defined(CONFIG_LIS2DU12_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&lis2du12->work);
#endif /* CONFIG_LIS2DU12_TRIGGER_OWN_THREAD */
}

#ifdef CONFIG_LIS2DU12_TRIGGER_OWN_THREAD
static void lis2du12_thread(struct lis2du12_data *lis2du12)
{
	while (1) {
		k_sem_take(&lis2du12->gpio_sem, K_FOREVER);
		LOG_INF("lis2du12_thread has taken sem");
		lis2du12_handle_interrupt(lis2du12->dev);
	}
}
#endif /* CONFIG_LIS2DU12_TRIGGER_OWN_THREAD */

#ifdef CONFIG_LIS2DU12_TRIGGER_GLOBAL_THREAD
static void lis2du12_work_cb(struct k_work *work)
{
	LOG_INF("lis2du12_work_cbd called");
	struct lis2du12_data *lis2du12 =
		CONTAINER_OF(work, struct lis2du12_data, work);

	lis2du12_handle_interrupt(lis2du12->dev);
}
#endif /* CONFIG_LIS2DU12_TRIGGER_GLOBAL_THREAD */


//#ifdef CONFIG_LIS2DU12_TAP
static int lis2du12_tap_init(const struct device *dev)
{
	const struct lis2du12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2du12_tap_md_t tap_md;
	int ret;
	
	tap_md.x_en = cfg->tap_x_en;
	tap_md.y_en = cfg->tap_y_en;
	tap_md.z_en = cfg->tap_z_en;
	tap_md.priority = cfg->tap_axis_priority;

	tap_md.tap_double.en = cfg->tap_mode;

    tap_md.threshold.x = cfg->tap_threshold[0];
    tap_md.threshold.y = cfg->tap_threshold[1];
    tap_md.threshold.z = cfg->tap_threshold[2];

    // Set.he tap timing parameters
    tap_md.shock = cfg->tap_shock;
    tap_md.quiet = cfg->tap_quiet;
    tap_md.tap_double.latency = cfg->tap_latency;

	ret = lis2du12_tap_mode_set(ctx, &tap_md);
	if (ret < 0) {
		LOG_ERR("Failed to set tap parameters");
		return ret;

	}

	return 0;
}



int lis2du12_init_interrupt(const struct device *dev)
{
	struct lis2du12_data *lis2du12 = dev->data;
	const struct lis2du12_config *cfg = dev->config;
	int ret;

	lis2du12->drdy_gpio = (cfg->drdy_pin == 1) ?
			(struct gpio_dt_spec *)&cfg->int1_gpio :
			(struct gpio_dt_spec *)&cfg->int2_gpio;

	/* setup data ready gpio interrupt (INT1 or INT2) */
	if (!gpio_is_ready_dt(lis2du12->drdy_gpio)) {
		LOG_ERR("Cannot get pointer to drdy_gpio device (%p)",
			lis2du12->drdy_gpio);
		return -EINVAL;
	}

#if defined(CONFIG_LIS2DU12_TRIGGER_OWN_THREAD)
	k_sem_init(&lis2du12->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&lis2du12->thread, lis2du12->thread_stack,
			CONFIG_LIS2DU12_THREAD_STACK_SIZE,
			(k_thread_entry_t)lis2du12_thread, lis2du12,
			NULL, NULL, K_PRIO_COOP(CONFIG_LIS2DU12_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&lis2du12->thread, dev->name);
#elif defined(CONFIG_LIS2DU12_TRIGGER_GLOBAL_THREAD)
	lis2du12->work.handler = lis2du12_work_cb;
#endif /* CONFIG_LIS2DU12_TRIGGER_OWN_THREAD */

	ret = gpio_pin_configure_dt(lis2du12->drdy_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure gpio: %d", ret);
		return ret;
	}

	gpio_init_callback(&lis2du12->gpio_cb,
			   lis2du12_gpio_callback,
			   BIT(lis2du12->drdy_gpio->pin));

	if (gpio_add_callback(lis2du12->drdy_gpio->port, &lis2du12->gpio_cb) < 0) {
		LOG_ERR("Could not set gpio callback");
		return -EIO;
	}

//#ifdef CONFIG_LIS2DU12_TAP
	lis2du12_int_mode_t int_mode;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	/* set tap detection parameters befor enabling it*/
	ret = lis2du12_tap_init(dev);
	if (ret < 0) {
		LOG_ERR("lis2du12_tap_init error");
		return ret;
	}

	ret = lis2du12_interrupt_mode_get(ctx, &int_mode);
	if (ret < 0) {
		LOG_ERR("lis2du12_interrupt_mode_get error");
		return ret;
	}

	/*set interrupt of base functions (FF, WU(W2S), 4/6D, Tap) to latched active high.
	 I am using these as defaults but they should be configured by the user
	 The user has to know the type of the intterrupt*/
	int_mode.enable = 1;
	int_mode.active_low = 0;
	int_mode.base_sig = LIS2DU12_INT_LATCHED;

	ret = lis2du12_interrupt_mode_set(ctx, &int_mode);
	if (ret < 0) {
		LOG_ERR("lis2du12_interrupt_mode_get errot");
		return ret;
	}


	return gpio_pin_interrupt_configure_dt(lis2du12->drdy_gpio,
					       GPIO_INT_EDGE_TO_ACTIVE);
}

int lis2du12_set_tap_x_threshold(const struct device *dev, const uint8_t threshold) {
	if(threshold > 32) {
		LOG_ERR("Threshold exceeds max possible value.");
		return -1;
	}

	const struct lis2du12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2du12_tap_ths_x_t tap_ths_x;
	int32_t ret;

	ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_X, (uint8_t *)&tap_ths_x, 1);

	tap_ths_x.tap_ths_x = threshold;
	ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_X, (uint8_t *)&tap_ths_x, 1);

	return ret;
}

int lis2du12_set_tap_y_threshold(const struct device *dev, const uint8_t threshold){
	if(threshold > 32) {
		LOG_ERR("Threshold exceeds max possible value.");
		return -1;
	}

	const struct lis2du12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2du12_tap_ths_y_t tap_ths_y;
	int32_t ret;

	ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_Y, (uint8_t *)&tap_ths_y, 1);

	tap_ths_y.tap_ths_y = threshold;
	ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_Y, (uint8_t *)&tap_ths_y, 1);

	return ret;
}

int lis2du12_set_tap_z_threshold(const struct device *dev, const uint8_t threshold){
	if(threshold > 32) {
		LOG_ERR("Threshold exceeds max possible value.");
		return -1;
	}

	const struct lis2du12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2du12_tap_ths_z_t tap_ths_z;
	int32_t ret;

	ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_Z, (uint8_t *)&tap_ths_z, 1);

	tap_ths_z.tap_ths_z = threshold;
	ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_Z, (uint8_t *)&tap_ths_z, 1);

	return ret;
}

int lis2du12_set_tap_all_threshold(const struct device *dev, const uint8_t threshold){
	if(threshold > 32) {
		LOG_ERR("Threshold exceeds max possible value.");
		return -1;
	}

	const struct lis2du12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2du12_tap_ths_x_t tap_ths_x;
	lis2du12_tap_ths_y_t tap_ths_y;
	lis2du12_tap_ths_z_t tap_ths_z;
	int32_t ret;

	ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_X, (uint8_t *)&tap_ths_x, 1);
	ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_Y, (uint8_t *)&tap_ths_y, 1);
	ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_Z, (uint8_t *)&tap_ths_z, 1);

	tap_ths_x.tap_ths_x = threshold;
	tap_ths_y.tap_ths_y = threshold;
	tap_ths_z.tap_ths_z = threshold;

	ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_X, (uint8_t *)&tap_ths_x, 1);
	ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_Y, (uint8_t *)&tap_ths_y, 1);
	ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_Z, (uint8_t *)&tap_ths_z, 1);


	return ret;
}