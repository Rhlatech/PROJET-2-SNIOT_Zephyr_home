/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include<stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "../inc/lcd_screen_i2c.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define LED_YELLOW_NODE DT_ALIAS(led_yellow)
#define DISPLAY_NODE DT_ALIAS(display)
#define CAPTOR_NODE DT_ALIAS(capteur)

const struct gpio_dt_spec led_yellow_gpio = GPIO_DT_SPEC_GET_OR(LED_YELLOW_NODE, gpios, {0});
const struct i2c_dt_spec display_i2c = I2C_DT_SPEC_GET(DISPLAY_NODE);
const struct device *capteur_T_H = DEVICE_DT_GET_ANY(aosong_dht);

int main(void)
{
	printf("COUCOU ! %s\n", CONFIG_BOARD);
	gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);
	init_lcd(&display_i2c);
	while (1) {
		struct sensor_value temp, humidity;

		sensor_sample_fetch(capteur_T_H);
		sensor_channel_get(capteur_T_H, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(capteur_T_H, SENSOR_CHAN_HUMIDITY, &humidity);

		printk("temp: %d.%06d; humidity: %d.%06d\n",
		      temp.val1, temp.val2, humidity.val1, humidity.val2);
		k_sleep(K_MSEC(10000));
	}
	printf("On est passé");
	return 0;
}
