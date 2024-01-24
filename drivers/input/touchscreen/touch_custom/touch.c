/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 . All rights reserved.
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/regulator/consumer.h>
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif
#include "touch.h"

#define MAX_CMDLINE_PARAM_LEN 512
char tp_dsi_display_primary[MAX_CMDLINE_PARAM_LEN];
char tp_dsi_display_secondary[MAX_CMDLINE_PARAM_LEN];

EXPORT_SYMBOL(tp_dsi_display_primary);
EXPORT_SYMBOL(tp_dsi_display_secondary);

#define MAX_LIMIT_DATA_LENGTH         100

struct touch_custom_data{
	int g_tp_prj_id;
	int g_tp_dev_vendor;
	int j;
	char *chip_name;
};

/*if can not compile success, please update vendor/oplus_touchsreen*/
struct tp_dev_name tp_dev_names[] = {
	{TP_OFILM, "OLIM"},
	{TP_BIEL, "BIEL"},
	{TP_TRULY, "TRULY"},
	{TP_BOE, "BOE"},
	{TP_G2Y, "G2Y"},
	{TP_TPK, "TPK"},
	{TP_JDI, "JDI"},
	{TP_TIANMA, "TIANMA"},
	{TP_SAMSUNG, "SAMSUNG"},
	{TP_DSJM, "DSJM"},
	{TP_BOE_B8, "BOEB8"},
	{TP_INNOLUX, "INNOLUX"},
	{TP_HIMAX_DPT, "DPT"},
	{TP_AUO, "AUO"},
	{TP_DEPUTE, "DEPUTE"},
	{TP_HUAXING, "HUAXING"},
	{TP_HLT, "HLT"},
	{TP_DJN, "DJN"},
	{TP_UNKNOWN, "UNKNOWN"},
};

#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

bool tp_judge_ic_match(char *tp_ic_name)
{
	pr_err("[TP] tp_ic_name = %s \n", tp_ic_name);
	pr_err("[TP] tp_dsi_display_primary = %s \n", tp_dsi_display_primary);
	pr_err("[TP] tp_dsi_display_secondary = %s \n", tp_dsi_display_secondary);

	if (strstr(tp_dsi_display_primary, tp_ic_name)) {
		pr_err("[TP] tp_judge_ic_match match ok\n");
		return true;
	}

	if (strstr(tp_dsi_display_secondary, tp_ic_name)) {
		pr_err("[TP] secondary disp match ok\n");
		return true;
	}

	pr_err("[TP] tp_judge_ic_match not match ok\n");
	return false;

}
EXPORT_SYMBOL(tp_judge_ic_match);

int tp_judge_ic_match_commandline(struct panel_info *panel_data)
{
	int prj_id = 0;
	int i = 0;
	struct touch_custom_data *p_touch_custom_data = NULL;

	prj_id = get_project();

	p_touch_custom_data = kzalloc(sizeof(struct touch_custom_data), GFP_KERNEL|GFP_DMA);
	if (p_touch_custom_data == NULL) {
		pr_err("[TP]panel_data.touch_custom_data kzalloc error\n");
		return -1;
	}
	panel_data->touch_custom_data = p_touch_custom_data;

	p_touch_custom_data->g_tp_dev_vendor = TP_UNKNOWN;
	p_touch_custom_data->chip_name = NULL;

	pr_err("[TP] prj_id = %d \n", prj_id);
	pr_err("[TP] tp_dsi_display_primary = %s \n", tp_dsi_display_primary);
	for(i = 0; i < panel_data->project_num; i++) {
		if(prj_id == panel_data->platform_support_project[i]) {
			p_touch_custom_data->g_tp_prj_id = panel_data->platform_support_project_dir[i];
			pr_err("[TP] Driver match support project [%d]\n", panel_data->platform_support_project[i]);

			for(p_touch_custom_data->j = 0; p_touch_custom_data->j < panel_data->panel_num; p_touch_custom_data->j++) {
				if(strstr(tp_dsi_display_primary, panel_data->platform_support_commandline[p_touch_custom_data->j]) \
				|| strstr(tp_dsi_display_secondary, panel_data->platform_support_commandline[p_touch_custom_data->j]) \
				|| strstr("default_commandline", panel_data->platform_support_commandline[p_touch_custom_data->j])) {
					panel_data->tp_type = panel_data->panel_type[p_touch_custom_data->j];
					if(panel_data->chip_num > 1) {
						p_touch_custom_data->chip_name = panel_data->chip_name[p_touch_custom_data->j];
						pr_err("[TP] WGL--1 chip_name = %s, panel_data->chip_name = %s",
						p_touch_custom_data->chip_name, panel_data->chip_name[p_touch_custom_data->j]);
					}
					pr_err("[TP] match panel type OK , panel type is [%d]\n", panel_data->tp_type);
					return p_touch_custom_data->j;
				}
				pr_err("[TP] Panel not found\n");
			}
		}
	}
	pr_err("[TP] Driver does not match the project\n");
	return -1;
}
EXPORT_SYMBOL(tp_judge_ic_match_commandline);

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data)
{
	char *vendor;
	struct touch_custom_data *p_touch_custom_data = NULL;

	panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL|GFP_DMA);
	if (panel_data->test_limit_name == NULL) {
		pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
	}

	if (panel_data->tp_type == TP_UNKNOWN) {
		pr_err("[TP]%s type is unknown\n", __func__);
		return 0;
	}

	if (IS_ERR_OR_NULL(panel_data->touch_custom_data)) {
		return 0;
	} else {
		p_touch_custom_data = (struct touch_custom_data *)panel_data->touch_custom_data;

		if (panel_data->firmware_name[p_touch_custom_data->j]) {
			memcpy(panel_data->manufacture_info.version, panel_data->firmware_name[p_touch_custom_data->j],
				strlen(panel_data->firmware_name[p_touch_custom_data->j]));
			panel_data->vid_len = strlen(panel_data->firmware_name[p_touch_custom_data->j]);
		}

		vendor = GET_TP_DEV_NAME(panel_data->tp_type);
		if(panel_data->chip_num == 1) {
			p_touch_custom_data->chip_name = panel_data->chip_name[0];
		}
		strcpy(panel_data->manufacture_info.manufacture, vendor);
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/%d/FW_%s_%s.img",
			p_touch_custom_data->g_tp_prj_id, p_touch_custom_data->chip_name, vendor);

		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/%d/LIMIT_%s_%s.img",
				p_touch_custom_data->g_tp_prj_id, p_touch_custom_data->chip_name, vendor);
		}

		panel_data->manufacture_info.fw_path = panel_data->fw_name;

		pr_info("[TP]vendor:%s fw:%s limit:%s\n",
			vendor,
			panel_data->fw_name,
			panel_data->test_limit_name==NULL?"NO Limit":panel_data->test_limit_name);

		kfree(panel_data->touch_custom_data);
		panel_data->touch_custom_data = NULL;

		return 0;
	}

}
EXPORT_SYMBOL(tp_util_get_vendor);
#endif
int preconfig_power_control(struct touchpanel_data *ts)
{
	return 0;
}
EXPORT_SYMBOL(preconfig_power_control);

int reconfig_power_control(struct touchpanel_data *ts)
{

	return 0;
}
EXPORT_SYMBOL(reconfig_power_control);

void display_esd_check_enable_bytouchpanel(bool enable)
{
	return;
}
EXPORT_SYMBOL(display_esd_check_enable_bytouchpanel);


module_param_string(dsi_display0, tp_dsi_display_primary, MAX_CMDLINE_PARAM_LEN,
                                        0600);
MODULE_PARM_DESC(dsi_display0,
	"oplus_bsp_tp_custom.dsi_display0=<display node> for primary dsi display node name");
module_param_string(dsi_display1, tp_dsi_display_secondary, MAX_CMDLINE_PARAM_LEN,
                                        0600);
MODULE_PARM_DESC(dsi_display1,
	"oplus_bsp_tp_custom.dsi_display1=<display node> for secondary dsi display node name");
MODULE_DESCRIPTION("Touchscreen Driver");
MODULE_LICENSE("GPL");
