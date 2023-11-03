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
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define LED_YELLOW_NODE DT_ALIAS(led_yellow)
#define DISPLAY_NODE DT_ALIAS(display)
#define CAPTOR_NODE DT_ALIAS(capteur)
#define SW0_NODE	DT_ALIAS(button1)
#define SW1_NODE	DT_ALIAS(button2)
#define BUZZER_NODE	DT_ALIAS(buzzer)
#define IR_SENSOR_NODE	DT_ALIAS(irsensor)

// La LED
const struct gpio_dt_spec led_yellow_gpio = GPIO_DT_SPEC_GET_OR(LED_YELLOW_NODE, gpios, {0});
// L'ecran
const struct i2c_dt_spec display_i2c = I2C_DT_SPEC_GET(DISPLAY_NODE);
//Le capteur temp / humi
const struct device *capteur_T_H = DEVICE_DT_GET_ANY(aosong_dht);
//Steam sensor
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
//static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user),0);
// Les bouttons
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button1_cb_data;
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static struct gpio_callback button2_cb_data;
// Buzzer
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET_OR(BUZZER_NODE, gpios, {0});
// SENSEUR IR
const struct gpio_dt_spec irsensor = GPIO_DT_SPEC_GET_OR(IR_SENSOR_NODE, gpios, {0});

int allumage = 0;
int disp_init = 0;
int alarme = 0;


void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button Allumage pressed \n");
	allumage = 1;

}

void button_pressed2(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button Extinction pressed \n");
	allumage = -1;
}


int main(void)
{
	int err;
	int32_t val_mv;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	printf("INITIALISATION sur ! %s\n", CONFIG_BOARD);
	// Allumage LED
	gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_LOW);
	//Init lcd
	init_lcd(&display_i2c);
	// init steam seansor
	(void)adc_sequence_init_dt(&adc_channel, &sequence);
	err = adc_channel_setup_dt(&adc_channel);
	//boutton 1 
	err = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	err = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button1_cb_data, button_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb_data);
	//boutton 2
	err = gpio_pin_configure_dt(&button2, GPIO_INPUT);
	err = gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button2_cb_data, button_pressed2, BIT(button2.pin));
	gpio_add_callback(button2.port, &button2_cb_data);
	while (1) {
		if(allumage == 1)
		{
			alarme = 0;
			//Clear command of the screen
			lcd_byte(&display_i2c, 0x01, LCD_CMD);
			// Allumage LED
			gpio_pin_set_dt(&led_yellow_gpio, 1);
			if(disp_init == 0)
			{
				printk(" Allumage ! \n");
				write_lcd(&display_i2c," Allumage ! ",LCD_LINE_1);
				disp_init = 1;
			}
			k_sleep(K_MSEC(1000));
			//Gestion du capteur humidity and temperature
			struct sensor_value temp, humidity;
			sensor_sample_fetch(capteur_T_H);
			sensor_channel_get(capteur_T_H, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(capteur_T_H, SENSOR_CHAN_HUMIDITY, &humidity);
			printk(" Temp: %d.%06d; Humidity: %d.%06d\n", temp.val1, temp.val2, humidity.val1, humidity.val2);
			//Gestion display LCD
			char caract[16];
			char caract2[16];
			sprintf(caract,"humidity: %d.%02d", humidity.val1, humidity.val2);
			sprintf(caract2,"temp: %d.%02d   ", temp.val1, temp.val2);
			write_lcd(&display_i2c,caract,LCD_LINE_1);
			write_lcd(&display_i2c,caract2,LCD_LINE_2);
			k_sleep(K_MSEC(1000));
			//Steam sensor definition and measure
			err = adc_read_dt(&adc_channel, &sequence);
			val_mv = (int32_t)buf;
			err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
			double volts = val_mv / 1000.0;
			char caract3[16];
			printf("Tension Steam sensor : %.3f\n", volts);
			sprintf(caract3," %f", volts);
			write_lcd(&display_i2c," Steam sensor :",LCD_LINE_1);
			write_lcd(&display_i2c,caract3,LCD_LINE_2);
			k_sleep(K_MSEC(1000));
			//Clear command of the screen
			lcd_byte(&display_i2c, 0x01, LCD_CMD);
		} else if(allumage == -1)
		{
			alarme = 0;
			//Clear command of the screen
			lcd_byte(&display_i2c, 0x01, LCD_CMD);
			// Allumage LED
			gpio_pin_set_dt(&led_yellow_gpio, 0);
			printk(" Extinction ! \n");
			write_lcd(&display_i2c," Extinction ! ",LCD_LINE_1);
			k_sleep(K_MSEC(1000));
			//Clear command of the screen
			lcd_byte(&display_i2c, 0x01, LCD_CMD);
			disp_init = 0;
			allumage = 0;
		} 
		if(alarme == 1)
		{
			//Clear command of the screen
			lcd_byte(&display_i2c, 0x01, LCD_CMD);
			//Affichage de detection
			write_lcd(&display_i2c," PRESENCE ",LCD_LINE_1);
			write_lcd(&display_i2c," DETECTE ",LCD_LINE_2);
			k_sleep(K_MSEC(1000));
		}
		k_sleep(K_MSEC(1000));
	}
	printf(" Sortie PRG \n");
	//Clear command of the screen
	lcd_byte(&display_i2c, 0x01, LCD_CMD);
	return 0;
}

void compute_thread(){
	int err;
	//IR sensor
	//err = gpio_pin_configure_dt(&irsensor, GPIO_INPUT);
	//printk("Value of error is %d\n", err);
	while(1)
	{
		alarme = gpio_pin_get_dt(&irsensor);
		if(alarme == 1)
		{
			//printk("Value of alarme is %d\n", alarme);
			//Lumière et buzzer
			gpio_pin_set_dt(&buzzer, 1);
			gpio_pin_set_dt(&led_yellow_gpio, 1);
			k_sleep(K_MSEC(100));
			gpio_pin_set_dt(&buzzer, 0);
			gpio_pin_set_dt(&led_yellow_gpio, 0);
			k_sleep(K_MSEC(100));
		}
	}
}

K_THREAD_DEFINE(compute_thread_id, 521, compute_thread, NULL, NULL, NULL, 9, 0, 0);