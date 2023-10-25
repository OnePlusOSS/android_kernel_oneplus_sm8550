/*
 * leds-aw210xx.c
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 *  Author: hushanping <hushanping@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <soc/oplus/system/oplus_project.h>
#include "leds_aw210xx.h"
#include "leds_aw210xx_reg.h"

struct aw210xx *aw210xx_glo;
static int max_led = 0;
static int dec_flag = 1;
static int ledbri[5] = {0};

static int aw210xx_hw_enable(struct aw210xx *aw210xx, bool flag);
static int aw210xx_led_init(struct aw210xx *aw210xx);
static int aw210xx_led_change_mode(struct aw210xx *led, enum AW2023_LED_MODE mode);
static int aw210xx_chipen_set(struct aw210xx *aw210xx, bool flag);


/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW210XX_DRIVER_VERSION "V0.4.0"
#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1
#define AW_START_TO_BREATH 200
#define LED_ESD_WORK_TIME					3

#define R_ISNK_ON_MASK					0x04
#define G_ISNK_ON_MASK					0x02
#define B_ISNK_ON_MASK					0x01

#define LED_SUPPORT_TYPE					"support"
// #define BLINK_USE_AW210XX
struct aw210xx *led_default;

/******************************************************
 *
 * aw210xx led parameter
 *
 ******************************************************/
aw210xx_cfg_t aw210xx_cfg_array[] = {
	{aw210xx_group_cfg_led_off, sizeof(aw210xx_group_cfg_led_off)},
	{aw21018_group_all_leds_on, sizeof(aw21018_group_all_leds_on)},
	{aw21018_group_red_leds_on, sizeof(aw21018_group_red_leds_on)},
	{aw21018_group_green_leds_on, sizeof(aw21018_group_green_leds_on)},
	{aw21018_group_blue_leds_on, sizeof(aw21018_group_blue_leds_on)},
	{aw21018_group_breath_leds_on, sizeof(aw21018_group_breath_leds_on)},
	{aw21012_group_all_leds_on, sizeof(aw21012_group_all_leds_on)},
	{aw21012_group_red_leds_on, sizeof(aw21012_group_red_leds_on)},
	{aw21012_group_green_leds_on, sizeof(aw21012_group_green_leds_on)},
	{aw21012_group_blue_leds_on, sizeof(aw21012_group_blue_leds_on)},
	{aw21012_group_breath_leds_on, sizeof(aw21012_group_breath_leds_on)},
	{aw21009_group_all_leds_on, sizeof(aw21009_group_all_leds_on)},
	{aw21009_group_red_leds_on, sizeof(aw21009_group_red_leds_on)},
	{aw21009_group_green_leds_on, sizeof(aw21009_group_green_leds_on)},
	{aw21009_group_blue_leds_on, sizeof(aw21009_group_blue_leds_on)},
	{aw21009_group_breath_leds_on, sizeof(aw21009_group_breath_leds_on)}
};

/******************************************************
 *
 * aw210xx i2c write/read
 *
 ******************************************************/
static int aw210xx_i2c_write(struct aw210xx *aw210xx,
		unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw210xx->i2c,
				reg_addr, reg_data);
		if (ret < 0)
			AW_ERR("id = %d i2c_write fail cnt=%d ret=%d addr=0x%x data=0x%x\n", aw210xx->id, cnt, ret, reg_addr, reg_data);
		else {
			AW_ERR("id = %d i2c_write suc cnt=%d ret=%d addr=0x%x data=0x%x\n", aw210xx->id, cnt, ret, reg_addr, reg_data);
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
				AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw210xx_i2c_read(struct aw210xx *aw210xx,
		unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw210xx->i2c, reg_addr);
		if (ret < 0) {
			AW_ERR("id = %d i2c_read fail cnt=%d ret=%d addr=0x%x\n", aw210xx->id, cnt, ret, reg_addr);
		} else {
			*reg_data = ret;
			AW_ERR("id = %d i2c_read suc cnt=%d ret=%d addr=0x%x\n", aw210xx->id, cnt, ret, reg_addr);
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
				AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw210xx_i2c_write_bits(struct aw210xx *aw210xx,
		unsigned char reg_addr, unsigned int mask,
		unsigned char reg_data)
{
	unsigned char reg_val;

	aw210xx_i2c_read(aw210xx, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	aw210xx_i2c_write(aw210xx, reg_addr, reg_val);

	return 0;
}


/*****************************************************
* led Interface: set effect
*****************************************************/
static void aw210xx_update_cfg_array(struct aw210xx *aw210xx,
		uint8_t *p_cfg_data, uint32_t cfg_size)
{
	unsigned int i = 0;

	for (i = 0; i < cfg_size; i += 2)
		aw210xx_i2c_write(aw210xx, p_cfg_data[i], p_cfg_data[i + 1]);
}

void aw210xx_cfg_update(struct aw210xx *aw210xx)
{
	AW_LOG("aw210xx->effect = %d", aw210xx->effect);

	aw210xx_update_cfg_array(aw210xx,
			aw210xx_cfg_array[aw210xx->effect].p,
			aw210xx_cfg_array[aw210xx->effect].count);
}

void aw210xx_uvlo_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_UVCR,
				AW210XX_BIT_UVPD_MASK,
				AW210XX_BIT_UVPD_DISENA);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_UVCR,
				AW210XX_BIT_UVDIS_MASK,
				AW210XX_BIT_UVDIS_DISENA);
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_UVCR,
				AW210XX_BIT_UVPD_MASK,
				AW210XX_BIT_UVPD_ENABLE);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_UVCR,
				AW210XX_BIT_UVDIS_MASK,
				AW210XX_BIT_UVDIS_ENABLE);
	}
}

void aw210xx_sbmd_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR2,
				AW210XX_BIT_SBMD_MASK,
				AW210XX_BIT_SBMD_ENABLE);
		aw210xx->sdmd_flag = 1;
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR2,
				AW210XX_BIT_SBMD_MASK,
				AW210XX_BIT_SBMD_DISENA);
		aw210xx->sdmd_flag = 0;
	}
}

void aw210xx_rgbmd_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR2,
				AW210XX_BIT_RGBMD_MASK,
				AW210XX_BIT_RGBMD_ENABLE);
		aw210xx->rgbmd_flag = 1;
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR2,
				AW210XX_BIT_RGBMD_MASK,
				AW210XX_BIT_RGBMD_DISENA);
		aw210xx->rgbmd_flag = 0;
	}
}

void aw210xx_apse_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_APSE_MASK,
				AW210XX_BIT_APSE_ENABLE);
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_APSE_MASK,
				AW210XX_BIT_APSE_DISENA);
	}
}

/*****************************************************
* aw210xx led function set
*****************************************************/
int32_t aw210xx_osc_pwm_set(struct aw210xx *aw210xx)
{
	switch (aw210xx->osc_clk) {
	case CLK_FRQ_16M:
		AW_LOG("id = %d osc is 16MHz!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_16MHz);
		break;
	case CLK_FRQ_8M:
		AW_LOG("id = %d osc is 8MHz!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_8MHz);
		break;
	case CLK_FRQ_1M:
		AW_LOG("id = %d osc is 1MHz!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_1MHz);
		break;
	case CLK_FRQ_512k:
		AW_LOG("id = %d osc is 512KHz!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_512kHz);
		break;
	case CLK_FRQ_256k:
		AW_LOG("id = %d osc is 256KHz!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_256kHz);
		break;
	case CLK_FRQ_125K:
		AW_LOG("id = %d osc is 125KHz!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_125kHz);
		break;
	case CLK_FRQ_62_5K:
		AW_LOG("id = %d osc is 62.5KHz!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_62_5kHz);
		break;
	case CLK_FRQ_31_25K:
		AW_LOG("id = %d osc is 31.25KHz!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_31_25kHz);
		break;
	default:
		AW_LOG("id = %d this clk_pwm is unsupported!\n", aw210xx->id);
		return -AW210XX_CLK_MODE_UNSUPPORT;
	}

	return 0;
}

int32_t aw210xx_br_res_set(struct aw210xx *aw210xx)
{
	switch (aw210xx->br_res) {
	case BR_RESOLUTION_8BIT:
		AW_LOG("id = %d br resolution select 8bit!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_PWMRES_MASK,
				AW210XX_BIT_PWMRES_8BIT);
		break;
	case BR_RESOLUTION_9BIT:
		AW_LOG("id = %d br resolution select 9bit!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_PWMRES_MASK,
				AW210XX_BIT_PWMRES_9BIT);
		break;
	case BR_RESOLUTION_12BIT:
		AW_LOG("id = %d br resolution select 12bit!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_PWMRES_MASK,
				AW210XX_BIT_PWMRES_12BIT);
		break;
	case BR_RESOLUTION_9_AND_3_BIT:
		AW_LOG("id = %d br resolution select 9+3bit!\n", aw210xx->id);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_PWMRES_MASK,
				AW210XX_BIT_PWMRES_9_AND_3_BIT);
		break;
	default:
		AW_LOG("id = %d this br_res is unsupported!\n", aw210xx->id);
		return -AW210XX_CLK_MODE_UNSUPPORT;
	}

	return 0;
}

/*****************************************************
* aw210xx debug interface set
*****************************************************/
static void aw210xx_update(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx, AW210XX_REG_UPDATE, AW210XX_UPDATE_BR_SL);
}

void aw210xx_global_set(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx,
			AW210XX_REG_GCCR, aw210xx->glo_current);
}
void aw210xx_current_set(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx, AW210XX_REG_GCCR, aw210xx->set_current);
}

static void aw210xx_brightness(struct aw210xx *led)
{
	int i = 0;
	int color[4] = {0};
	int temp_brightness[4] = {0};
	int led_brightness = 0;
	AW_LOG("id = %d brightness = %d\n", led->id, led->cdev.brightness);
	if (led->id > 5) {
		AW_LOG("id = %d ,return\n", led->id);
		return ;
	}
	switch(led->id) {
	case AW210xx_LED_RED:
		ledbri[0] = led->cdev.brightness;
		if (led->cdev.brightness > 0)
			led->pdata->led->rgb_isnk_on |= R_ISNK_ON_MASK;
		else
			led->pdata->led->rgb_isnk_on &= ~R_ISNK_ON_MASK;
		break;
	case AW210xx_LED_GREEN:
		ledbri[1] = led->cdev.brightness;
		if (led->cdev.brightness > 0)
			led->pdata->led->rgb_isnk_on |= G_ISNK_ON_MASK;
		else
			led->pdata->led->rgb_isnk_on &= ~G_ISNK_ON_MASK;
		break;
	case AW210xx_LED_BLUE:
		ledbri[2] = led->cdev.brightness;
		if (led->cdev.brightness > 0)
			led->pdata->led->rgb_isnk_on |= B_ISNK_ON_MASK;
		else
			led->pdata->led->rgb_isnk_on &= ~B_ISNK_ON_MASK;
		break;
	default:
		AW_LOG("no define id = %d brightness = %d\n", led->id, led->cdev.brightness);
		return;
	}
	AW_LOG("rgb_isnk_on = 0x%x,led_enable = 0x%x\n\n", led->pdata->led->rgb_isnk_on,led->pdata->led->led_enable);
	if (led->cdev.brightness > 0) {
		if (!led->pdata->led->led_enable && led->pdata->led->rgb_isnk_on) {
			aw210xx_i2c_write(led, AW210XX_REG_RESET, 0x00);
			usleep_range(2000, 2200);
			if (aw210xx_led_init(led->pdata->led)) {
				AW_LOG("id = %d aw210xx active failed", led->id);
			}
		}
	} else {
		if (led->pdata->led->led_enable && !(led->pdata->led->rgb_isnk_on)) {
			if (aw210xx_chipen_set(led->pdata->led, false)) {
				AW_LOG("id = %d aw210xx sleep failed", led->id);
			}
		}
	}

	/*aw210xx led music mode set up*/
	if (led->pdata->led_mode == AW210XX_LED_MUSICMODE || led->pdata->led_mode == AW210XX_LED_NEW_ALWAYSON || led->pdata->led_mode == AW210XX_LED_HORSE_RACE_LAMP) {
		led->br_res = BR_RESOLUTION_9_AND_3_BIT;
		aw210xx_br_res_set(led);

		if (led->cdev.brightness > 16) {
			led_brightness = (led->cdev.brightness * led->cdev.brightness) / 16;
		} else {
			led_brightness = led->cdev.brightness;
		}

		/* sbmd disable */
		aw210xx_sbmd_set(led, false);
		aw210xx_i2c_write(led, AW210XX_REG_BR00H + 2 * led->id, 0xff & (led_brightness >> 8));
		aw210xx_i2c_write(led, AW210XX_REG_BR00L + 2 * led->id, 0xff & led_brightness);
		aw210xx_i2c_write(led, AW210XX_REG_BR03H + 2 * led->id, 0xff & (led_brightness >> 8));
		aw210xx_i2c_write(led, AW210XX_REG_BR03L + 2 * led->id, 0xff & led_brightness);
		aw210xx_i2c_write(led, AW210XX_REG_BR06H + 2 * led->id, 0xff & (led_brightness >> 8));
		aw210xx_i2c_write(led, AW210XX_REG_BR06L + 2 * led->id, 0xff & led_brightness);
		aw210xx_i2c_write(led, AW210XX_REG_BR09H + 2 * led->id, 0xff & (led_brightness >> 8));
		aw210xx_i2c_write(led, AW210XX_REG_BR09L + 2 * led->id, 0xff & led_brightness);
		aw210xx_i2c_write(led, AW210XX_REG_ABMCFG, 0x01);

		/*red led, green led max current is 20ma, blue led max current is 10ma*/
		for (i = 0; i < led->pdata->led->led_groups_num; i++) {
			if (led->id == 0 && dec_flag) {
				color[i] = (led->pdata->color[i] * 2) / 3;
				//AW_LOG("id = %d set color[%d] = %d\n", led->id, i, color[i]);
			} else if (led->id == 1 && dec_flag) {
				color[i] = (led->pdata->color[i] * 2) / 3;
				//AW_LOG("id = %d set color[%d] = %d\n", led->id, i, color[i]);
			} else if (led->id == 2 && dec_flag) {
				color[i] = (led->pdata->color[i] * 2) / 3;
				//AW_LOG("id = %d set color[%d] = %d\n", led->id, i, color[i]);
			}
		}

		if (led->pdata->led->led_groups_num == 2) {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				color[i] = (color[i] * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id =%d color * (2/3) * %d/%d set reg color[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, color[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, color[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, color[1]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x40);
			AW_LOG("id = %d led_groups_num = %d \n", led->id, led->pdata->led->led_groups_num);
		} else if (led->pdata->led->led_groups_num == 4) {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				color[i] = (color[i] * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id = %d color * (2/3) * %d/%d set reg color[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, color[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, color[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, color[1]);
			aw210xx_i2c_write(led, AW210XX_REG_SL06 + led->id, color[2]);
			aw210xx_i2c_write(led, AW210XX_REG_SL09 + led->id, color[3]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x40);
			AW_LOG("id = %d led_groups_num = %d \n", led->id, led->pdata->led->led_groups_num);
		} else {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				color[i] = (color[i] * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id = %d color * (2/3) * %d/%d set reg color[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, color[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, color[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, color[1]);
			aw210xx_i2c_write(led, AW210XX_REG_SL06 + led->id, color[2]);
			aw210xx_i2c_write(led, AW210XX_REG_SL09 + led->id, color[3]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x40);
			AW_LOG("id = %d led_groups_num = %d \n", led->id, led->pdata->led->led_groups_num);
		}
	} else if (led->pdata->led_mode == AW210XX_LED_INDIVIDUAL_CTL_BREATH) {
		/*red led, green led max current is 20ma, blue led max current is 10ma*/
		for (i = 0; i < led->pdata->led->led_groups_num; i++) {
			if (led->id == 0 && dec_flag) {
				color[i] = (led->pdata->color[i] * 2) / 3;
				//AW_LOG("id = %d set color[%d] = %d\n", led->id, i, color[i]);
			} else if (led->id == 1 && dec_flag) {
				color[i] = (led->pdata->color[i] * 2) / 3;
				//AW_LOG("id = %d set color[%d] = %d\n", led->id, i, color[i]);
			} else if (led->id == 2 && dec_flag) {
				color[i] = (led->pdata->color[i] * 2) / 3;
				//AW_LOG("id = %d set color[%d] = %d\n", led->id, i, color[i]);
			}
		}

		led->br_res = BR_RESOLUTION_8BIT;
		aw210xx_br_res_set(led);
		/* sbmd enable */
		aw210xx_sbmd_set(led, true);

		if (led->pdata->led->led_groups_num == 2) {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				color[i] = (color[i] * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id = %d color * (2/3) * %d/%d set reg color[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, color[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, color[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, color[1]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x47);
			AW_LOG("id = %d led_groups_num = %d \n", led->id, led->pdata->led->led_groups_num);
		} else if (led->pdata->led->led_groups_num == 4) {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				color[i] = (color[i] * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id = %d color * (2/3) * %d/%d set reg color[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, color[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, color[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, color[1]);
			aw210xx_i2c_write(led, AW210XX_REG_SL06 + led->id, color[2]);
			aw210xx_i2c_write(led, AW210XX_REG_SL09 + led->id, color[3]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x4F);
			AW_LOG("id = %d led_groups_num = %d \n", led->id, led->pdata->led->led_groups_num);
		} else {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				color[i] = (color[i] * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id = %d color * (2/3) * %d/%d set reg color[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, color[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, color[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, color[1]);
			aw210xx_i2c_write(led, AW210XX_REG_SL06 + led->id, color[2]);
			aw210xx_i2c_write(led, AW210XX_REG_SL09 + led->id, color[3]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x4F);
			AW_LOG("id = %d led_groups_num = %d \n", led->id, led->pdata->led->led_groups_num);
		}
	} else {
		if (led->id == 0 && dec_flag) {
			led_brightness = (led->cdev.brightness * 2) / 3;
			//AW_LOG("id = %d set brightness = %d\n", led->id, led_brightness);
		} else if (led->id == 1 && dec_flag) {
			led_brightness = (led->cdev.brightness * 2) / 3;
			//AW_LOG("id = %d set brightness = %d\n", led->id, led_brightness);
		} else if (led->id == 2 && dec_flag) {
			led_brightness = (led->cdev.brightness * 2) / 3;
			//AW_LOG("id = %d set brightness = %d\n", led->id, led_brightness);
		}

		led->br_res = BR_RESOLUTION_8BIT;
		aw210xx_br_res_set(led);
		/* sbmd enable */
		aw210xx_sbmd_set(led, true);

		if (led->pdata->led->led_groups_num == 2) {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				temp_brightness[i] = (led_brightness * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id = %d led_brightness * (2/3) * %d/%d set reg led_brightness[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, temp_brightness[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, temp_brightness[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, temp_brightness[1]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x47);
			AW_LOG("id = %d led_groups_num = %d\n", led->id, led->pdata->led->led_groups_num);
		} else if (led->pdata->led->led_groups_num == 4) {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				temp_brightness[i] = (led_brightness * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id = %d led_brightness * (2/3) * %d/%d set reg led_brightness[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, temp_brightness[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, temp_brightness[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, temp_brightness[1]);
			aw210xx_i2c_write(led, AW210XX_REG_SL06 + led->id, temp_brightness[2]);
			aw210xx_i2c_write(led, AW210XX_REG_SL09 + led->id, temp_brightness[3]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x4F);
			AW_LOG("id = %d led_groups_num = %d\n", led->id, led->pdata->led->led_groups_num);
		} else {
			for (i = 0; i < led->pdata->led->led_groups_num; i++) {
				temp_brightness[i] = (led_brightness * led->pdata->led->led_current_numerator[i]) / led->pdata->led->led_current_denominator[i];
				AW_LOG("id = %d led_brightness * (2/3) * %d/%d set reg led_brightness[%d] = %d\n", led->id, led->pdata->led->led_current_numerator[i],
					led->pdata->led->led_current_denominator[i], i, temp_brightness[i]);
			}
			aw210xx_i2c_write(led, AW210XX_REG_SL00 + led->id, temp_brightness[0]);
			aw210xx_i2c_write(led, AW210XX_REG_SL03 + led->id, temp_brightness[1]);
			aw210xx_i2c_write(led, AW210XX_REG_SL06 + led->id, temp_brightness[2]);
			aw210xx_i2c_write(led, AW210XX_REG_SL09 + led->id, temp_brightness[3]);
			aw210xx_i2c_write(led, AW210XX_REG_GCFG, 0x4F);
			AW_LOG("id = %d led_groups_num = %d\n", led->id, led->pdata->led->led_groups_num);
		}
	}

	/*aw210xx led cc mode set up*/
	if (led->pdata->led_mode == AW210XX_LED_CCMODE) {
		aw210xx_i2c_write(led, AW210XX_REG_GBRH, 0x00);
		aw210xx_i2c_write(led, AW210XX_REG_GBRL, 0xff);
		aw210xx_i2c_write(led, AW210XX_REG_ABMCFG, 0x00);
	}

	/*aw210xx led breath set up*/
	if (led->pdata->led_mode == AW210XX_LED_BREATHMODE || led->pdata->led_mode == AW210XX_LED_INDIVIDUAL_CTL_BREATH) {
		aw210xx_i2c_write(led, AW210XX_REG_ABMT0 ,
				(led->pdata->rise_time_ms << 4 | led->pdata->hold_time_ms));
		aw210xx_i2c_write(led, AW210XX_REG_ABMT1 ,
				(led->pdata->fall_time_ms << 4 | led->pdata->off_time_ms));

		aw210xx_i2c_write(led, AW210XX_REG_GBRH, 0xff);
		aw210xx_i2c_write(led, AW210XX_REG_GBRL, 0x00);
		aw210xx_i2c_write(led, AW210XX_REG_ABMCFG, 0x03);

		for (i = 0; i < max_led; i++)
			cancel_delayed_work(&(&aw210xx_glo[i])->breath_work);

		schedule_delayed_work(&led->breath_work, msecs_to_jiffies(AW_START_TO_BREATH));
	}

	/*aw210xx led blink time*/
	if (led->pdata->led_mode == AW210XX_LED_BLINKMODE) {
		#ifdef BLINK_USE_AW210XX
			aw210xx_i2c_write(led, AW210XX_REG_ABMT0 , 0x04);
			aw210xx_i2c_write(led, AW210XX_REG_ABMT1 , 0x04);

			aw210xx_i2c_write(led, AW210XX_REG_GBRH, 0xff);
			aw210xx_i2c_write(led, AW210XX_REG_GBRL, 0x00);

			aw210xx_i2c_write(led, AW210XX_REG_ABMCFG, 0x03);

			for (i = 0; i < max_led; i++)
				cancel_delayed_work(&(&aw210xx_glo[i])->breath_work);

			schedule_delayed_work(&led->breath_work, msecs_to_jiffies(AW_START_TO_BREATH));
		#else
			aw210xx_i2c_write(led, AW210XX_REG_GBRH, 0x00);
			aw210xx_i2c_write(led, AW210XX_REG_GBRL, 0xff);
			aw210xx_i2c_write(led, AW210XX_REG_ABMCFG, 0x00);
		#endif
	}

	/* update */
	aw210xx_update(led);

	AW_LOG("%s:  brightness[%d]=%x led_mode[%d]=%x \n",__func__,led->id,led->cdev.brightness,led->id,led->pdata->led_mode);
}

static void aw210xx_breath_func(struct work_struct *work)
{
	struct aw210xx *led = container_of(work, struct aw210xx, breath_work.work);
	aw210xx_i2c_write(led, AW210XX_REG_ABMGO, 0x01);
}

static void aw210xx_brightness_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			brightness_work);

	AW_LOG("id = %d aw210xx_brightness_work enter\n", aw210xx->id);
	aw210xx_brightness(aw210xx);
}

static void aw210xx_set_brightness(struct led_classdev *cdev,
		enum led_brightness brightness)
{
	struct aw210xx *aw210xx = container_of(cdev, struct aw210xx, cdev);

	aw210xx->cdev.brightness = brightness;

	if(aw210xx->cdev.trigger != NULL)
	{
		if(strcmp(aw210xx->cdev.trigger->name,"timer") == 0)
		{
			aw210xx_led_change_mode(aw210xx, AW210XX_LED_BLINKMODE);
			AW_LOG("%s[%d]:  trigger = %s\n",__func__,aw210xx->id,aw210xx->cdev.trigger->name);
		}
	}

	schedule_work(&aw210xx->brightness_work);
}

/*****************************************************
* aw210xx basic function set
*****************************************************/
int aw210xx_chipen_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CHIPEN_MASK,
				AW210XX_BIT_CHIPEN_ENABLE);
		aw210xx->led_enable = true;
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CHIPEN_MASK,
				AW210XX_BIT_CHIPEN_DISENA);
		aw210xx->led_enable = false;
	}
	return 0;
}

static int aw210xx_hw_enable(struct aw210xx *aw210xx, bool flag)
{
	AW_LOG("enter\n");

	if (aw210xx && gpio_is_valid(aw210xx->enable_gpio) && gpio_is_valid(aw210xx->vbled_enable_gpio)) {
		if (flag) {
			gpio_set_value_cansleep(aw210xx->enable_gpio, 1);
			gpio_set_value_cansleep(aw210xx->vbled_enable_gpio, 1);
			aw210xx->led_enable = true;
			usleep_range(2000, 2500);
			aw210xx_led_init(aw210xx);
		} else {
			gpio_set_value_cansleep(aw210xx->enable_gpio, 0);
			gpio_set_value_cansleep(aw210xx->vbled_enable_gpio, 0);
			aw210xx->led_enable = false;
		}
	} else {
		AW_ERR("failed\n");
	}

	return 0;
}

/*****************************************************
 *
 * check chip id and version
 *
 *****************************************************/
static int aw210xx_read_chipid(struct aw210xx *aw210xx)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char chipid = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw210xx_i2c_read(aw210xx, AW210XX_REG_RESET, &chipid);
		if (ret < 0) {
			AW_ERR("failed to read chipid: %d\n", ret);
		} else {
			aw210xx->chipid = chipid;
			switch (aw210xx->chipid) {
			case AW21018_CHIPID:
				AW_LOG("AW21018, read chipid = 0x%02x!!\n",
						chipid);
				return 0;
			case AW21012_CHIPID:
				AW_LOG("AW21012, read chipid = 0x%02x!!\n",
						chipid);
				return 0;
			case AW21009_CHIPID:
				AW_LOG("AW21009, read chipid = 0x%02x!!\n",
						chipid);
				return 0;
			default:
				AW_LOG("chip is unsupported device id = %x\n",
						chipid);
				break;
			}
		}
		cnt++;
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
				AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}

	return -EINVAL;
}


static int aw210xx_led_init(struct aw210xx *aw210xx)
{
	AW_LOG("id = %d enter\n", aw210xx->id);

	aw210xx->sdmd_flag = 0;
	aw210xx->rgbmd_flag = 0;
	/* chip enable */
	if (aw210xx_chipen_set(aw210xx, true)) {
		AW_LOG("aw210xx sleep failed");
	}
	usleep_range(1000, 1500);
	/* sbmd enable */
	aw210xx_sbmd_set(aw210xx, true);
	/* rgbmd enable */
	aw210xx_rgbmd_set(aw210xx, false);
	/* clk_pwm selsect */
	aw210xx_osc_pwm_set(aw210xx);
	/* br_res select */
	aw210xx_br_res_set(aw210xx);
	/* global set */
	aw210xx_global_set(aw210xx);
	/* under voltage lock out */
	aw210xx_uvlo_set(aw210xx, false);
	/* apse enable */
	aw210xx_apse_set(aw210xx, false);

	return 0;
}

static int32_t aw210xx_group_gcfg_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		switch (aw210xx->chipid) {
		case AW21018_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21018_GROUP_ENABLE);
			return 0;
		case AW21012_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21012_GROUP_ENABLE);
			return 0;
		case AW21009_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21009_GROUP_ENABLE);
			return 0;
		default:
			AW_LOG("%s: chip is unsupported device!\n",
			       __func__);
			return -AW210XX_CHIPID_FAILD;
		}
	} else {
		switch (aw210xx->chipid) {
		case AW21018_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21018_GROUP_DISABLE);
			return 0;
		case AW21012_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21012_GROUP_DISABLE);
			return 0;
		case AW21009_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21009_GROUP_DISABLE);
			return 0;
		default:
			AW_LOG("%s: chip is unsupported device!\n",
			       __func__);
			return -AW210XX_CHIPID_FAILD;
		}
	}
}

void aw210xx_singleled_set(struct aw210xx *aw210xx,
		uint32_t rgb_reg,
		uint32_t rgb_sl,
		uint32_t rgb_br)
{
	/* chip enable */
	if (aw210xx_chipen_set(aw210xx, true)) {
		AW_LOG("aw210xx sleep failed");
	}
	/* global set */
	aw210xx->set_current = rgb_br;
	aw210xx_current_set(aw210xx);
	/* group set disable */
	aw210xx_group_gcfg_set(aw210xx, false);

	aw210xx_sbmd_set(aw210xx, true);
	aw210xx_rgbmd_set(aw210xx, false);
	aw210xx_uvlo_set(aw210xx, true);

	/* set sl */
	aw210xx->rgbcolor = rgb_sl & 0xff;
	aw210xx_i2c_write(aw210xx, AW210XX_REG_SL00 + rgb_reg,
			  aw210xx->rgbcolor);

	/* br set */
	if (aw210xx->sdmd_flag == 1) {
		if (aw210xx->rgbmd_flag == 1) {
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00L + rgb_reg,
					  rgb_br);
		} else {
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00L + rgb_reg,
					  rgb_br);
		}
	} else {
		if (aw210xx->rgbmd_flag == 1) {
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00L + rgb_reg,
					  rgb_br);
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00H + rgb_reg,
					  rgb_br);
		} else {
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00L + rgb_reg,
					  rgb_br);
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00H + rgb_reg,
					  rgb_br);
		}
	}
	/* update */
	aw210xx_update(aw210xx);
}

/*****************************************************
* open short detect
*****************************************************/
void aw210xx_open_detect_cfg(struct aw210xx *aw210xx)
{
	/*enable open detect*/
	aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_OPEN_DETECT_EN);
	/*set DCPWM = 1*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR,
							AW210XX_DCPWM_SET_MASK,
							AW210XX_DCPWM_SET);
	/*set Open threshold = 0.2v*/
	aw210xx_i2c_write_bits(aw210xx,
							AW210XX_REG_OSDCR,
							AW210XX_OPEN_THRESHOLD_SET_MASK,
							AW210XX_OPEN_THRESHOLD_SET);
}

void aw210xx_short_detect_cfg(struct aw210xx *aw210xx)
{

	/*enable short detect*/
	aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_SHORT_DETECT_EN);
	/*set DCPWM = 1*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR,
							AW210XX_DCPWM_SET_MASK,
							AW210XX_DCPWM_SET);
	/*set Short threshold = 1v*/
	aw210xx_i2c_write_bits(aw210xx,
							AW210XX_REG_OSDCR,
							AW210XX_SHORT_THRESHOLD_SET_MASK,
							AW210XX_SHORT_THRESHOLD_SET);
}

void aw210xx_open_short_dis(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_OPEN_SHORT_DIS);
	/*SET DCPWM = 0*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR,
							AW210XX_DCPWM_SET_MASK,
							AW210XX_DCPWM_CLEAN);
}
void aw210xx_open_short_detect(struct aw210xx *aw210xx,
										int32_t detect_flg, u8 *reg_val)
{
	/*config for open shor detect*/
	if (detect_flg == AW210XX_OPEN_DETECT)
		aw210xx_open_detect_cfg(aw210xx);
	else if (detect_flg == AW210XX_SHORT_DETECT)
		aw210xx_short_detect_cfg(aw210xx);
	/*read detect result*/
	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST0, &reg_val[0]);
	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST1, &reg_val[1]);
	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST2, &reg_val[2]);
	/*close for open short detect*/
	aw210xx_open_short_dis(aw210xx);
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw210xx_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		if (aw210xx_reg_access[(uint8_t)databuf[0]] & REG_WR_ACCESS)
			aw210xx_i2c_write(aw210xx, (uint8_t)databuf[0],
					(uint8_t)databuf[1]);
	}

	return len;
}

static ssize_t aw210xx_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	uint8_t br_max = 0;
	uint8_t sl_val = 0;

	aw210xx_read_chipid(aw210xx);
	aw210xx_i2c_read(aw210xx, AW210XX_REG_GCR, &reg_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"reg:0x%02x=0x%02x\n", AW210XX_REG_GCR, reg_val);
	switch (aw210xx->chipid) {
	case AW21018_CHIPID:
		br_max = AW210XX_REG_BR17H;
		sl_val = AW210XX_REG_SL17;
		break;
	case AW21012_CHIPID:
		br_max = AW210XX_REG_BR11H;
		sl_val = AW210XX_REG_SL11;
		break;
	case AW21009_CHIPID:
		br_max = AW210XX_REG_BR08H;
		sl_val = AW210XX_REG_SL08;
		break;
	default:
		AW_LOG("chip is unsupported device!\n");
		return len;
	}

	for (i = AW210XX_REG_BR00L; i <= br_max; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(aw210xx, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	for (i = AW210XX_REG_SL00; i <= sl_val; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(aw210xx, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	for (i = AW210XX_REG_GCCR; i <= AW210XX_REG_GCFG; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(aw210xx, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t aw210xx_hwen_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val > 0)
		aw210xx_hw_enable(aw210xx, true);
	else
		aw210xx_hw_enable(aw210xx, false);

	return len;
}

static ssize_t aw210xx_hwen_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "hwen=%d\n",
			gpio_get_value(aw210xx->enable_gpio));
	return len;
}

static ssize_t aw210xx_effect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;
	unsigned int i;

	for (i = 0; i < (sizeof(aw210xx_cfg_array) /
			sizeof(aw210xx_cfg_t)); i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "effect[%d]: %p\n",
				i, (unsigned char *)aw210xx_cfg_array[i].p);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "current effect[%d]: %p\n",
			aw210xx->effect, (unsigned char *)aw210xx_cfg_array[aw210xx->effect].p);
	return len;
}

static ssize_t aw210xx_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;
	if (val >= (sizeof(aw210xx_cfg_array) / sizeof(aw210xx_cfg_t))) {
		pr_err("%s, store effect num error.\n", __func__);
		return -EINVAL;
	}

	aw210xx->effect = val;
	pr_info("%s, line%d,val = %d\n", __func__, __LINE__, val);
	aw210xx_cfg_update(aw210xx);

	return len;
}

static ssize_t aw210xx_rgbcolor_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint32_t rgb_num = 0;
	uint32_t rgb_data = 0;

	if (sscanf(buf, "%x %x", &rgb_num, &rgb_data) == 2) {
		if (aw210xx_chipen_set(aw210xx, true)) {
			AW_LOG("aw210xx sleep failed");
		}
		aw210xx_sbmd_set(aw210xx, true);
		aw210xx_rgbmd_set(aw210xx, true);
		aw210xx_global_set(aw210xx);
		aw210xx_uvlo_set(aw210xx, true);

		/* set sl */
		aw210xx->rgbcolor = (rgb_data & 0xff0000) >> 16;
		aw210xx_i2c_write(aw210xx,
				AW210XX_REG_SL00 + (uint8_t)rgb_num * 3,
				aw210xx->rgbcolor);

		aw210xx->rgbcolor = (rgb_data & 0x00ff00) >> 8;
		aw210xx_i2c_write(aw210xx,
				AW210XX_REG_SL00 + (uint8_t)rgb_num * 3 + 1,
				aw210xx->rgbcolor);

		aw210xx->rgbcolor = (rgb_data & 0x0000ff);
		aw210xx_i2c_write(aw210xx,
				AW210XX_REG_SL00 + (uint8_t)rgb_num * 3 + 2,
				aw210xx->rgbcolor);

		/* br set */
		aw210xx_i2c_write(aw210xx,
				AW210XX_REG_BR00L + (uint8_t)rgb_num,
				AW210XX_GLOBAL_DEFAULT_SET);

		/* update */
		aw210xx_update(aw210xx);
	}

	return len;
}

static ssize_t aw210xx_singleled_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint32_t led_num = 0;
	uint32_t rgb_data = 0;
	uint32_t rgb_brightness = 0;

	if (sscanf(buf, "%x %x %x", &led_num, &rgb_data, &rgb_brightness) == 3) {
		if (aw210xx->chipid == AW21018_CHIPID) {
			if (led_num > AW21018_LED_NUM)
				led_num = AW21018_LED_NUM;
		} else if (aw210xx->chipid == AW21012_CHIPID) {
			if (led_num > AW21012_LED_NUM)
				led_num = AW21012_LED_NUM;
		} else {
			if (led_num > AW21009_LED_NUM)
				led_num = AW21009_LED_NUM;
		}
		aw210xx_singleled_set(aw210xx, led_num, rgb_data, rgb_brightness);
	}

	return len;
}

static ssize_t aw210xx_opdetect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;
	int i = 0;
	uint8_t reg_val[3] = {0};

	aw210xx_open_short_detect(aw210xx, AW210XX_OPEN_DETECT, reg_val);
	for (i = 0; i < sizeof(reg_val); i++)
		len += snprintf(buf + len, PAGE_SIZE - len,
					"OSST%d:%#x\n", i, reg_val[i]);

	return len;
}

static ssize_t aw210xx_stdetect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;
	int i = 0;
	uint8_t reg_val[3] = {0};

	aw210xx_open_short_detect(aw210xx, AW210XX_SHORT_DETECT, reg_val);
	for (i = 0; i < sizeof(reg_val); i++)
		len += snprintf(buf + len, PAGE_SIZE - len,
					"OSST%d:%#x\n", i, reg_val[i]);
	return len;
}

static ssize_t aw210xx_led_br_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d (max:4095)\n",led->pdata->br_brightness[0],led->pdata->br_brightness[1],
			led->pdata->br_brightness[2], led->pdata->br_brightness[3]);
}

static ssize_t aw210xx_led_br_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);
	char *delim = ",";
	char *str = NULL;
	char *p_buf, *tmp_buf;
	int i = 0, led_br[4] = {0}, led_order[4] = {0};
	ssize_t ret = -EINVAL;

	AW_LOG("br input is %s\n", buf);

	p_buf = kstrdup(buf, GFP_KERNEL);
	tmp_buf = p_buf;
	led_order[0] = led->pdata->led->led_allocation_order[0];
	led_order[1] = led->pdata->led->led_allocation_order[1];
	led_order[2] = led->pdata->led->led_allocation_order[2];
	led_order[3] = led->pdata->led->led_allocation_order[3];
	AW_LOG("[%d]: led_order=%d,%d,%d,%d.\n", led->id , led_order[0],
			led_order[1], led_order[2], led_order[3]);

	for (i = 0; i < 4; i++) {
		str = strsep(&tmp_buf, delim);
		if (str == NULL) {
			break;
		}
		ret = kstrtoul(str, 10, &data);
		if (ret) {
			kfree(p_buf);
			return ret;
		}
		AW_LOG("[%d]: led_color=%d, i=%d.\n",led->id ,data, i);
		led_br[i] = data;
	}

	if (i == 4) {
		AW_LOG("[%d]: led_color=%d,%d,%d,%d.\n",led->id ,data, led_br[0],
			led_br[1], led_br[2], led_br[3]);
		mutex_lock(&led->pdata->led->lock);
		led->pdata->br_brightness[0] = led_br[led_order[0]];
		led->pdata->br_brightness[1] = led_br[led_order[1]];
		led->pdata->br_brightness[2] = led_br[led_order[2]];
		led->pdata->br_brightness[3] = led_br[led_order[3]];
		mutex_unlock(&led->pdata->led->lock);
		AW_LOG("[%d]: led_br_brightness=%d,%d,%d,%d\n",led->id, led->pdata->br_brightness[0], led->pdata->br_brightness[1],
			led->pdata->br_brightness[2], led->pdata->br_brightness[3]);
	} else {
		AW_LOG("[%d]: input error.\n",led->id);
	}
	kfree(p_buf);
	return cnt;
}

static ssize_t aw210xx_led_color_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d (max:255)\n",led->pdata->color[0], led->pdata->color[1],
			led->pdata->color[2], led->pdata->color[3]);
}

static ssize_t aw210xx_led_color_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);
	char *delim = ",";
	char *str = NULL;
	char *p_buf, *tmp_buf;
	int i = 0, led_color[4] = {0}, led_order[4] = {0};
	ssize_t ret = -EINVAL;

	AW_LOG("color input is %s\n", buf);

	p_buf = kstrdup(buf, GFP_KERNEL);
	tmp_buf = p_buf;
	led_order[0] = led->pdata->led->led_allocation_order[0];
	led_order[1] = led->pdata->led->led_allocation_order[1];
	led_order[2] = led->pdata->led->led_allocation_order[2];
	led_order[3] = led->pdata->led->led_allocation_order[3];
	AW_LOG("[%d]: led_order=%d,%d,%d,%d.\n", led->id, led_order[0],
			led_order[1], led_order[2], led_order[3]);

	for (i = 0; i < 4; i++) {
		str = strsep(&tmp_buf, delim);
		if (str == NULL) {
			break;
		}
		ret = kstrtoul(str, 10, &data);
		if (ret) {
			kfree(p_buf);
			return ret;
		}
		AW_LOG("[%d]: led_color=%d, i=%d.\n",led->id ,data, i);
		led_color[i] = data;
	}

	if (i == 4) {
		AW_LOG("[%d]: led_color=%d,%d,%d,%d.\n",led->id ,data, led_color[0],
				led_color[1], led_color[2], led_color[3]);
		mutex_lock(&led->pdata->led->lock);
		led->pdata->color[0] = led_color[led_order[0]];
		led->pdata->color[1] = led_color[led_order[1]];
		led->pdata->color[2] = led_color[led_order[2]];
		led->pdata->color[3] = led_color[led_order[3]];
		mutex_unlock(&led->pdata->led->lock);
		AW_LOG("[%d]: led_color=%d,%d,%d,%d (max:255)\n", led->id, led->pdata->color[0], led->pdata->color[1],
				led->pdata->color[2], led->pdata->color[3]);
	} else {
		AW_LOG("[%d]: input error.\n",led->id);
	}
	kfree(p_buf);
	return cnt;
}

static ssize_t aw210xx_led_num_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n",led->pdata->led->led_groups_num);
}

static ssize_t aw210xx_led_color_ratio_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	return snprintf(buf, PAGE_SIZE, "%d,%d\n",led->color_ratio[0], led->color_ratio[1]);
}

static ssize_t aw210xx_led_ton_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->hold_time_ms);
}

static ssize_t aw210xx_led_ton_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->hold_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);
	AW_LOG("[%d]: hold_time_ms=%d (max:15)\n",led->id,led->pdata->hold_time_ms);

	return cnt;
}

static ssize_t aw210xx_led_tr1_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->rise_time_ms);
}

static ssize_t aw210xx_led_tr1_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->rise_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);

	AW_LOG("[%d]: rise_time_ms=%d (max:15)\n",led->id,led->pdata->rise_time_ms);

	return cnt;
}

static ssize_t aw210xx_led_tf1_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->fall_time_ms);

}

static ssize_t aw210xx_led_tf1_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->fall_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);

	AW_LOG("[%d]: fall_time_ms=%d (max:15)\n",led->id,led->pdata->fall_time_ms);

	return cnt;
}

static ssize_t aw210xx_led_toff_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->off_time_ms);
}

static ssize_t aw210xx_led_toff_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->off_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);
	AW_LOG("[%d]: off_time_ms=%d (max:15)\n",led->id,led->pdata->off_time_ms);

	return cnt;
}

static ssize_t aw210xx_led_support_attr_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int prj_id = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *led = container_of(led_cdev, struct aw210xx, cdev);

	prj_id = get_project();

	return snprintf(buf, PAGE_SIZE, "%s-%d\n",LED_SUPPORT_TYPE,led->cdev.max_brightness);
}

static DEVICE_ATTR(support, 0664, aw210xx_led_support_attr_show, NULL);
static DEVICE_ATTR(reg, 0664, aw210xx_reg_show, aw210xx_reg_store);
static DEVICE_ATTR(hwen, 0664, aw210xx_hwen_show, aw210xx_hwen_store);
static DEVICE_ATTR(effect, 0664, aw210xx_effect_show, aw210xx_effect_store);
static DEVICE_ATTR(rgbcolor, 0664, NULL, aw210xx_rgbcolor_store);
static DEVICE_ATTR(singleled, 0664, NULL, aw210xx_singleled_store);
static DEVICE_ATTR(opdetect, 0664, aw210xx_opdetect_show, NULL);
static DEVICE_ATTR(stdetect, 0664, aw210xx_stdetect_show, NULL);
static DEVICE_ATTR(br, 0664, aw210xx_led_br_attr_show, aw210xx_led_br_attr_store);
static DEVICE_ATTR(color, 0664, aw210xx_led_color_attr_show, aw210xx_led_color_attr_store);
static DEVICE_ATTR(led_num, 0664, aw210xx_led_num_attr_show, NULL);
static DEVICE_ATTR(color_ratio, 0664, aw210xx_led_color_ratio_attr_show, NULL);
static DEVICE_ATTR(ton, 0664, aw210xx_led_ton_attr_show, aw210xx_led_ton_attr_store);
static DEVICE_ATTR(toff, 0664, aw210xx_led_toff_attr_show, aw210xx_led_toff_attr_store);
static DEVICE_ATTR(tr1, 0664, aw210xx_led_tr1_attr_show, aw210xx_led_tr1_attr_store);
static DEVICE_ATTR(tf1, 0664, aw210xx_led_tf1_attr_show, aw210xx_led_tf1_attr_store);

static struct attribute *aw210xx_attributes[] = {
	&dev_attr_support.attr,
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_effect.attr,
	&dev_attr_rgbcolor.attr,
	&dev_attr_singleled.attr,
	&dev_attr_opdetect.attr,
	&dev_attr_stdetect.attr,
	&dev_attr_br.attr,
	&dev_attr_color.attr,
	&dev_attr_led_num.attr,
	&dev_attr_color_ratio.attr,
	&dev_attr_ton.attr,
	&dev_attr_toff.attr,
	&dev_attr_tr1.attr,
	&dev_attr_tf1.attr,
	NULL,
};

static struct attribute_group aw210xx_attribute_group = {
	.attrs = aw210xx_attributes
};

static int aw210xx_led_err_handle(struct aw210xx *led_array,
		int parsed_leds)
{
	int i=0;
	/*
	* If probe fails, cannot free resource of all LEDs, only free
	* resources of LEDs which have allocated these resource really.
	*/
	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw210xx_attribute_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&led_array->i2c->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	return i;
}

/******************************************************
 *
 * led class dev
 ******************************************************/
static int aw210xx_parse_led_cdev(struct aw210xx *aw210xx,
		struct device_node *node)
{
	struct aw210xx *led;
	struct aw210xx_platform_data *pdata;
	int rc = -1, parsed_leds = 0;
	int color_ratio[2] = {0};
	struct device_node *temp;

	AW_LOG("enter\n");
	aw210xx_glo = aw210xx;
	for_each_child_of_node(node, temp) {
		led = &aw210xx[parsed_leds];
		led->i2c = aw210xx->i2c;

		pdata = devm_kzalloc(&led->i2c->dev,
				sizeof(struct aw210xx_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&led->i2c->dev,
				"Failed to allocate memory\n");
			goto free_err;
		}
		pdata->led = aw210xx;
		led->pdata = pdata;

		rc = of_property_read_string(temp, "aw210xx,name",
			&led->cdev.name);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading led name, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw210xx,id",
			&led->id);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32_array(temp, "aw210xx,color_ratio", color_ratio, 2);
		if (rc) {
			dev_err(&led->i2c->dev,
				"Failure reading color_ratio, rc = %d\n", rc);
			if (led->id < 2) {
				led->color_ratio[0] = 2;
				led->color_ratio[1] = 3;
			} else {
				led->color_ratio[0] = 1;
				led->color_ratio[1] = 3;
			}
		} else {
			led->color_ratio[0] = color_ratio[0];
			led->color_ratio[1] = color_ratio[1];
		}

		rc = of_property_read_u32(temp, "aw210xx,imax",
			&led->pdata->imax);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading imax, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_string(temp, "aw210xx,led_default_trigger",
									&led->pdata->led_default_trigger);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure led_default_trigger, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw210xx,brightness",
			&led->cdev.brightness);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading brightness, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw210xx,max-brightness",
			&led->cdev.max_brightness);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading max-brightness, rc = %d\n",
				rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw210xx,rise-time-ms",
			&led->pdata->rise_time_ms);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading rise-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw210xx,hold-time-ms",
			&led->pdata->hold_time_ms);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading hold-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw210xx,fall-time-ms",
			&led->pdata->fall_time_ms);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading fall-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw210xx,off-time-ms",
			&led->pdata->off_time_ms);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading off-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		INIT_WORK(&led->brightness_work, aw210xx_brightness_work);
		INIT_DELAYED_WORK(&led->breath_work, aw210xx_breath_func);

		led->cdev.brightness_set = aw210xx_set_brightness;
//		led->cdev.default_trigger = pdata->led_default_trigger;
		rc = led_classdev_register(&led->i2c->dev, &led->cdev);
		if (rc) {
			dev_err(&led->i2c->dev,
				"unable to register led %d,rc=%d\n",
				led->id, rc);
			goto free_pdata;
		}

		rc = sysfs_create_group(&led->cdev.dev->kobj,
				&aw210xx_attribute_group);
		if (rc) {
			dev_err(&led->i2c->dev, "led sysfs rc: %d\n", rc);
			goto free_class;
		}
		parsed_leds++;
	}
	max_led = parsed_leds;
	return 0;

free_class:
	aw210xx_led_err_handle(aw210xx, parsed_leds);
	led_classdev_unregister(&aw210xx[parsed_leds].cdev);
	cancel_work_sync(&aw210xx[parsed_leds].brightness_work);
	devm_kfree(&led->i2c->dev, aw210xx[parsed_leds].pdata);
	aw210xx[parsed_leds].pdata = NULL;
	return rc;

free_pdata:
	aw210xx_led_err_handle(aw210xx, parsed_leds);
	devm_kfree(&led->i2c->dev, aw210xx[parsed_leds].pdata);
	return rc;

free_err:
	aw210xx_led_err_handle(aw210xx, parsed_leds);
	return rc;
}

static int aw210xx_led_change_mode(struct aw210xx *led,
		enum AW2023_LED_MODE mode)
{
	int ret=0;
	switch(mode) {
		case AW210XX_LED_CCMODE:
			led->pdata->led_mode = AW210XX_LED_CCMODE;
			break;
		case AW210XX_LED_NEW_ALWAYSON:
			led->pdata->led_mode = AW210XX_LED_NEW_ALWAYSON;
			break;
		case AW210XX_LED_BLINKMODE:
			led->pdata->hold_time_ms = 4;
			led->pdata->off_time_ms =  4;
			led->pdata->rise_time_ms = 0;
			led->pdata->fall_time_ms = 0;
			led->pdata->led_mode = AW210XX_LED_BLINKMODE;
			break;
		case AW210XX_LED_BREATHMODE:
			led->pdata->hold_time_ms = 0;
			led->pdata->off_time_ms =  0;
			led->pdata->rise_time_ms = 6;
			led->pdata->fall_time_ms = 6;
			led->pdata->led_mode = AW210XX_LED_BREATHMODE;
			break;
		case AW210XX_LED_INDIVIDUAL_CTL_BREATH:
			led->pdata->hold_time_ms = 0;
			led->pdata->off_time_ms =  0;
			led->pdata->rise_time_ms = 6;
			led->pdata->fall_time_ms = 6;
			led->pdata->led_mode = AW210XX_LED_INDIVIDUAL_CTL_BREATH;
			break;
		case AW210XX_LED_MUSICMODE:
			led->pdata->led_mode = AW210XX_LED_MUSICMODE;
			break;
		case AW210XX_LED_HORSE_RACE_LAMP:
			led->pdata->led_mode = AW210XX_LED_HORSE_RACE_LAMP;
			break;
		default:
			led->pdata->led_mode = AW210XX_LED_NONE;
			break;
	}
	return ret;
}
/*
static struct attribute *aw210xx_led_cc_mode_attrs[] = {
};

static struct attribute *aw210xx_led_blink_mode_attrs[] = {
};

static struct attribute *aw210xx_led_breath_mode_attrs[] = {
};

ATTRIBUTE_GROUPS(aw210xx_led_cc_mode);
ATTRIBUTE_GROUPS(aw210xx_led_blink_mode);
ATTRIBUTE_GROUPS(aw210xx_led_breath_mode);
*/
static int	aw210xx_led_cc_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw210xx_led_change_mode(led, AW210XX_LED_CCMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw210xx_led_change_mode fail\n", __func__);
		return ret;
	}

	return ret;
}

static void  aw210xx_led_cc_deactivate(struct led_classdev *cdev)
{
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: deactivate",led->id);
}

static int aw210xx_led_new_always_on_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: new_always_on activate",led->id);

	ret = aw210xx_led_change_mode(led, AW210XX_LED_NEW_ALWAYSON);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw210xx_led_change_mode fail\n", __func__);
		return ret;
	}

	return ret;
}

static void  aw210xx_led_new_always_on_deactivate(struct led_classdev *cdev)
{
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: new_always_on deactivate",led->id);
}

static int	aw210xx_led_blink_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("%s[%d]: activate",__func__,led->id);

	ret = aw210xx_led_change_mode(led, AW210XX_LED_BLINKMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw210xx_led_change_mode fail\n", __func__);
		return ret;
	}

	return ret;
}

static void aw210xx_led_blink_deactivate(struct led_classdev *cdev)
{
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: deactivate",led->id);
}

static int aw210xx_led_breath_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw210xx_led_change_mode(led, AW210XX_LED_BREATHMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw210xx_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw210xx_led_breath_deactivate(struct led_classdev *cdev)
{
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: deactivate",led->id);
}

static int aw210xx_led_individual_ctl_breath_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: individual_ctl_breath activate",led->id);

	ret = aw210xx_led_change_mode(led, AW210XX_LED_INDIVIDUAL_CTL_BREATH);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw210xx_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw210xx_led_individual_ctl_breath_deactivate(struct led_classdev *cdev)
{
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: individual_ctl_breath deactivate",led->id);
}

static int aw210xx_led_music_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: music activate",led->id);

	ret = aw210xx_led_change_mode(led, AW210XX_LED_MUSICMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw210xx_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static int aw210xx_led_horse_race_lamp_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: individual_ctl_breath activate",led->id);

	ret = aw210xx_led_change_mode(led, AW210XX_LED_HORSE_RACE_LAMP);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw210xx_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw210xx_led_horse_race_lamp_deactivate(struct led_classdev *cdev)
{
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: horse_race_lamp deactivate",led->id);
}


static void aw210xx_led_music_deactivate(struct led_classdev *cdev)
{
	struct aw210xx *led = container_of(cdev, struct aw210xx, cdev);

	AW_LOG("[%d]: music deactivate",led->id);
}

static struct led_trigger aw210xx_led_trigger[LED_MAX_NUM] = {
	{
		.name = "cc_mode",
		.activate = aw210xx_led_cc_activate,
		.deactivate = aw210xx_led_cc_deactivate,
	},
	{
		.name = "new_always_on_mode",
		.activate = aw210xx_led_new_always_on_activate,
		.deactivate = aw210xx_led_new_always_on_deactivate,
	},
	{
		.name = "blink_mode",
		.activate = aw210xx_led_blink_activate,
		.deactivate = aw210xx_led_blink_deactivate,
	},
	{
		.name = "breath_mode",
		.activate = aw210xx_led_breath_activate,
		.deactivate = aw210xx_led_breath_deactivate,
	},
	{
		.name = "individual_ctl_breath",
		.activate = aw210xx_led_individual_ctl_breath_activate,
		.deactivate = aw210xx_led_individual_ctl_breath_deactivate,
	},
	{
		.name = "music_mode",
		.activate = aw210xx_led_music_activate,
		.deactivate = aw210xx_led_music_deactivate,
	},
	{
		.name = "horse_race_lamp",
		.activate = aw210xx_led_horse_race_lamp_activate,
		.deactivate = aw210xx_led_horse_race_lamp_deactivate,
	},
};


/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw210xx_parse_dt(struct device *dev, struct aw210xx *aw210xx,
		struct device_node *np)
{
	int ret = -EINVAL;
	int i = 0;
	int led_order[4];
	int temp_denominator[4];
	int temp_numerator[4];

	aw210xx->enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (aw210xx->enable_gpio < 0) {
		aw210xx->enable_gpio = -1;
		AW_ERR("no enable gpio provided, HW enable unsupported\n");
		return ret;
	}

	aw210xx->vbled_enable_gpio = of_get_named_gpio(np, "vbled-enable-gpio", 0);
	if (aw210xx->vbled_enable_gpio < 0) {
		aw210xx->vbled_enable_gpio = -1;
		AW_ERR("no vbled enable gpio provided, HW enable unsupported\n");
	}

	ret = of_property_read_u32(np, "led_groups_num",
			&aw210xx->led_groups_num);
	if (ret < 0) {
		AW_ERR("led_groups_num is not set, set led_groups_num = 2\n");
		aw210xx->led_groups_num = 2;
	}

	ret = of_property_read_u32_array(np, "led_allocation_order", led_order, 4);

	if (ret) {
		AW_ERR("led_allocation_order is not set, set led_allocation_order = 1,2,3,4\n");
		aw210xx->led_allocation_order[0] = 0;
		aw210xx->led_allocation_order[1] = 1;
		aw210xx->led_allocation_order[2] = 2;
		aw210xx->led_allocation_order[3] = 3;
	} else {
		aw210xx->led_allocation_order[0] = led_order[0] - 1;
		aw210xx->led_allocation_order[1] = led_order[1] - 1;
		aw210xx->led_allocation_order[2] = led_order[2] - 1;
		aw210xx->led_allocation_order[3] = led_order[3] - 1;
		AW_ERR("led_allocation_order is %d,%d,%d,%d\n", aw210xx->led_allocation_order[0] + 1, aw210xx->led_allocation_order[1] + 1,
			aw210xx->led_allocation_order[2] + 1, aw210xx->led_allocation_order[3] + 1);
	}

	ret = of_property_read_u32(np, "vbled_volt",
			&aw210xx->vbled_volt);
	if (ret < 0) {
		AW_ERR("vbled_volt is not set\n");
		aw210xx->vbled_volt = 0;
	}

	ret = of_property_read_u32(np, "osc_clk",
			&aw210xx->osc_clk);
	if (ret < 0) {
		AW_ERR("no osc_clk provided, osc clk unsupported\n");
		return ret;
	}

	ret = of_property_read_u32(np, "br_res",
			&aw210xx->br_res);
	if (ret < 0) {
		AW_ERR("brightness resolution unsupported\n");
		return ret;
	}

	ret = of_property_read_u32(np, "global_current",
			&aw210xx->glo_current);
	if (ret < 0) {
		AW_ERR("global current resolution unsupported\n");
		return ret;
	}

	ret = of_property_read_u32_array(np, "led_current_denominator", temp_denominator, aw210xx->led_groups_num);
	if (ret) {
		for (i = 0; i < aw210xx->led_groups_num; i++) {
			aw210xx->led_current_denominator[i] = 1;
			AW_ERR("default set reg led_current_denominator = %d\n", aw210xx->led_current_denominator[i]);
		}
	} else {
		for (i = 0; i < aw210xx->led_groups_num; i++) {
			aw210xx->led_current_denominator[i] = temp_denominator[i];
			AW_ERR("set reg led_current_denominator = %d\n", aw210xx->led_current_denominator[i]);
		}
	}

	ret = of_property_read_u32_array(np, "led_current_numerator", temp_numerator, aw210xx->led_groups_num);
	if (ret) {
		for (i = 0; i < aw210xx->led_groups_num; i++) {
			aw210xx->led_current_numerator[i] = 1;
			AW_ERR("default set reg led_current_numerator = %d\n", aw210xx->led_current_numerator[i]);
		}
	} else {
		for (i = 0; i < aw210xx->led_groups_num; i++) {
			aw210xx->led_current_numerator[i] = temp_numerator[i];
			AW_ERR("set reg led_current_numerator = %d\n", aw210xx->led_current_numerator[i]);
		}
	}

	return 0;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw210xx_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct aw210xx *aw210xx;
	struct device_node *np = i2c->dev.of_node;
	int ret, num_leds = 0, i = 0;

	AW_LOG("enter\n");

	num_leds = of_get_child_count(np);

	if (!num_leds)
		return -EINVAL;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		AW_ERR("check_functionality failed\n");
		return -EIO;
	}

	aw210xx = devm_kzalloc(&i2c->dev,
			(sizeof(struct aw210xx) * num_leds), GFP_KERNEL);
	if (aw210xx == NULL)
		return -ENOMEM;

	aw210xx->dev = &i2c->dev;
	aw210xx->i2c = i2c;
	aw210xx->num_leds = num_leds;
	i2c_set_clientdata(i2c, aw210xx);
	aw210xx->rgb_isnk_on = 0;

	/* aw210xx parse device tree */
	if (np) {
		ret = aw210xx_parse_dt(&i2c->dev, aw210xx, np);
		if (ret) {
			AW_ERR("failed to parse device tree node\n");
			goto err_parse_dt;
		}
	}

	if (gpio_is_valid(aw210xx->enable_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw210xx->enable_gpio,
				GPIOF_OUT_INIT_LOW, "aw210xx_en");
		if (ret) {
			AW_ERR("enable gpio request failed\n");
			goto err_gpio_request;
		}
	}

	if (gpio_is_valid(aw210xx->vbled_enable_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw210xx->vbled_enable_gpio,
				GPIOF_OUT_INIT_LOW, "aw210xx_vbled_en");
		if (ret) {
			AW_ERR("vbled enable gpio request failed\n");
			goto err_gpio_request;
		}
	}

	/* hardware enable */
	aw210xx_hw_enable(aw210xx, true);

	/* aw210xx identify */
	ret = aw210xx_read_chipid(aw210xx);
	if (ret < 0) {
		AW_ERR("aw210xx_read_chipid failed ret=%d\n", ret);
		goto err_id;
	}

	dev_set_drvdata(&i2c->dev, aw210xx);
	aw210xx_parse_led_cdev(aw210xx, np);
	if (ret < 0) {
		AW_ERR("error creating led class dev\n");
		goto err_sysfs;
	}

	/* aw210xx led trigger register */
	for (i = 0; i < LED_MAX_NUM; i++) {
	    ret = led_trigger_register(&aw210xx_led_trigger[i]);
		if (ret < 0) {
			dev_err(&i2c->dev, "register %d trigger fail\n", i);
			goto fail_led_trigger;
		}
	}
	led_default = aw210xx;
	if (aw210xx_chipen_set(aw210xx, false)) {
		AW_LOG("aw210xx sleep failed");
	}

	//aw210xx_hw_enable(aw210xx, false);
	AW_LOG("probe completed!\n");


	led_default->aw210_led_wq = create_singlethread_workqueue("aw210_led_workqueue");
	if (!led_default->aw210_led_wq) {
		dev_err(&i2c->dev, "aw210_led_workqueue error\n");
		goto err_parse_dt;
	}
	return 0;

fail_led_trigger:
	while (--i >= 0)
		led_trigger_register(&aw210xx_led_trigger[i]);
	aw210xx_led_err_handle(aw210xx, num_leds);
err_sysfs:
err_id:
	devm_gpio_free(&i2c->dev, aw210xx->vbled_enable_gpio);
	devm_gpio_free(&i2c->dev, aw210xx->enable_gpio);
err_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, aw210xx);
	aw210xx = NULL;
	return ret;
}

static int aw210xx_i2c_remove(struct i2c_client *i2c)
{
	struct aw210xx *aw210xx = i2c_get_clientdata(i2c);

	AW_LOG("enter\n");
	sysfs_remove_group(&aw210xx->cdev.dev->kobj, &aw210xx_attribute_group);
	led_classdev_unregister(&aw210xx->cdev);
	if (gpio_is_valid(aw210xx->enable_gpio))
		devm_gpio_free(&i2c->dev, aw210xx->enable_gpio);
	if (gpio_is_valid(aw210xx->vbled_enable_gpio))
		devm_gpio_free(&i2c->dev, aw210xx->vbled_enable_gpio);
	devm_kfree(&i2c->dev, aw210xx);
	aw210xx = NULL;

	return 0;
}

static const struct i2c_device_id aw210xx_i2c_id[] = {
	{AW210XX_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw210xx_i2c_id);

static const struct of_device_id aw210xx_dt_match[] = {
	{.compatible = "awinic,aw210xx_led"},
	{}
};

static struct i2c_driver aw210xx_i2c_driver = {
	.driver = {
		.name = AW210XX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw210xx_dt_match),
		},
	.probe = aw210xx_i2c_probe,
	.remove = aw210xx_i2c_remove,
	.id_table = aw210xx_i2c_id,
};

static int __init aw210xx_i2c_init(void)
{
	int ret = 0;

	AW_LOG("enter, aw210xx driver version %s\n", AW210XX_DRIVER_VERSION);

	ret = i2c_add_driver(&aw210xx_i2c_driver);
	if (ret) {
		AW_ERR("failed to register aw210xx driver!\n");
		return ret;
	}

	return 0;
}
module_init(aw210xx_i2c_init);

static void __exit aw210xx_i2c_exit(void)
{
	i2c_del_driver(&aw210xx_i2c_driver);
}
module_exit(aw210xx_i2c_exit);

MODULE_DESCRIPTION("AW210XX LED Driver");
MODULE_LICENSE("GPL v2");
