// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/version.h>

#include "../oplus_touchscreen_v2/touchpanel_notify/touchpanel_event_notify.h"
#include "touchpanel_healthinfo/touchpanel_healthinfo.h"
#include "touchpanel_autotest/touchpanel_autotest.h"
#include "touch_comon_api/touch_comon_api.h"

/*irq_depth - For enable or disable irq
 * Output:
 * irq depth;
 * irq gpio state;
 */
static ssize_t proc_get_irq_depth_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGE_SIZE] = {0};
	struct syna_tcm *tcm = PDE_DATA(file_inode(file));
	struct irq_desc *desc = NULL;

	if (!tcm) {
		return 0;
	}

	desc = irq_to_desc(tcm->hw_if->bdata_attn.irq_id);

	if (!desc) {
		return 0;
	}

	snprintf(page, PAGE_SIZE - 1, "depth:%u, state:%d\n", desc->depth,
		 gpio_get_value(tcm->hw_if->bdata_attn.irq_gpio));
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	return ret;
}
/*irq_depth - For enable or disable irq
 * Input:
 * value:1, enable_irq;
 * value:other, disable_irq_nosync;
 */
static ssize_t proc_irq_status_write(struct file *file,
				     const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct syna_tcm *tcm = PDE_DATA(file_inode(file));

	if (!tcm) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(tcm->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	TP_INFO(tcm->tp_index, "%s %d, %s ts->irq=%d\n", __func__, value,
		value ? "enable" : "disable", tcm->hw_if->bdata_attn.irq_id);

	if (value == 1) {
		enable_irq(tcm->hw_if->bdata_attn.irq_id);
	} else {
		disable_irq_nosync(tcm->hw_if->bdata_attn.irq_id);
	}

	return count;
}

DECLARE_PROC_OPS(proc_get_irq_depth_fops, simple_open, proc_get_irq_depth_read, proc_irq_status_write, NULL);

/*proc/touchpanel/baseline_test*/
static int tp_auto_test_read_func(struct seq_file *s, void *v)
{
	int ret = 0;

	ret = tp_auto_test(s, v);
	return ret;
}

static int baseline_autotest_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_auto_test_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_auto_test_proc_fops, baseline_autotest_open, seq_read, NULL, single_release);



/*baseline_result - For GKI auto test result*/
static int tp_auto_test_result_read(struct seq_file *s, void *v)
{
	int ret = 0;

	ret = tp_auto_test_result(s, v);
	return ret;
}

static int tp_auto_test_result_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_auto_test_result_read, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_auto_test_result_fops, tp_auto_test_result_open, seq_read, NULL, single_release);


/*proc/touchpanel/framework_mode*/
static int tp_framework_mode_read_func(struct seq_file *s, void *v)
{
	seq_printf(s, "0\n");
	return 0;
}

static int tp_framework_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_framework_mode_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_framework_mode_proc_fops, tp_framework_mode_open, seq_read, NULL, single_release);

/*coordinate - For black screen gesture coordinate*/
static ssize_t proc_coordinate_read(struct file *file, char __user *buffer,
				    size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGE_SIZE] = {0};
	struct syna_tcm *tcm = PDE_DATA(file_inode(file));
	struct tcm_touch_data_blob *touch_data;

	uint32_t gesture_type = UNKOWN_GESTURE;
	uint32_t clockwise = 2;
	struct Coordinate Point_start;
	struct Coordinate Point_end;
	struct Coordinate Point_1st;
	struct Coordinate Point_2nd;
	struct Coordinate Point_3rd;
	struct Coordinate Point_4th;

	if (!tcm) {
		return 0;
	}

	mutex_lock(&tcm->mutex);

	touch_data = &tcm->tp_data;

	switch (touch_data->gesture_id) {
	case DTAP_DETECT:
		gesture_type = DOU_TAP;
		break;

	case CIRCLE_DETECT:
		gesture_type = CIRCLE_GESTURE;

		if (touch_data->extra_gesture_info[2] == 0x10) {
			clockwise = 1;

		} else if (touch_data->extra_gesture_info[2] == 0x20) {
			clockwise = 0;
		}

		break;

	case SWIPE_DETECT:
		if (touch_data->extra_gesture_info[4] == 0x41) { /*x+*/
			gesture_type = LEFT2RIGHT_SWIP;

		} else if (touch_data->extra_gesture_info[4] == 0x42) { /*x-*/
			gesture_type = RIGHT2LEFT_SWIP;

		} else if (touch_data->extra_gesture_info[4] == 0x44) { /*y+*/
			gesture_type = UP2DOWN_SWIP;

		} else if (touch_data->extra_gesture_info[4] == 0x48) { /*y-*/
			gesture_type = DOWN2UP_SWIP;

		} else if (touch_data->extra_gesture_info[4] == 0x81) { /*2x-*/
			gesture_type = DOU_SWIP;

		} else if (touch_data->extra_gesture_info[4] == 0x82) { /*2x+*/
			gesture_type = DOU_SWIP;

		} else if (touch_data->extra_gesture_info[4] == 0x84) { /*2y+*/
			gesture_type = DOU_SWIP;

		} else if (touch_data->extra_gesture_info[4] == 0x88) { /*2y-*/
			gesture_type = DOU_SWIP;
		}

		break;

	case M_UNICODE:
		gesture_type = M_GESTRUE;
		break;

	case W_UNICODE:
		gesture_type = W_GESTURE;
		break;

	case VEE_DETECT:
		if (touch_data->extra_gesture_info[2] == 0x02) { /*up*/
			gesture_type = UP_VEE;

		} else if (touch_data->extra_gesture_info[2] == 0x01) { /*down*/
			gesture_type = DOWN_VEE;

		} else if (touch_data->extra_gesture_info[2] == 0x08) { /*left*/
			gesture_type = LEFT_VEE;

		} else if (touch_data->extra_gesture_info[2] == 0x04) { /*right*/
			gesture_type = RIGHT_VEE;
		}

		break;

	case TOUCH_HOLD_DOWN:
		gesture_type = FINGER_PRINTDOWN;
		break;

	case TOUCH_HOLD_UP:
		gesture_type = FRINGER_PRINTUP;
		break;

	case HEART_DETECT:
		gesture_type = HEART;

		if (touch_data->extra_gesture_info[2] == 0x10) {
			clockwise = 1;

		} else if (touch_data->extra_gesture_info[2] == 0x20) {
			clockwise = 0;
		}
		break;

	case STAP_DETECT:
		gesture_type = SINGLE_TAP;
		break;

	case S_UNICODE:
		gesture_type = S_GESTURE;
		break;

	case TRIANGLE_DETECT:
	default:
		LOGW("not support\n");
		break;
	}

	if (gesture_type != UNKOWN_GESTURE) {
		Point_start.x = (touch_data->data_point[0] |
					   (touch_data->data_point[1] << 8)) / 10;
		Point_start.y = (touch_data->data_point[2] |
					   (touch_data->data_point[3] << 8)) / 10;
		Point_end.x    = (touch_data->data_point[4] |
					   (touch_data->data_point[5] << 8)) / 10;
		Point_end.y    = (touch_data->data_point[6] |
					   (touch_data->data_point[7] << 8)) / 10;
		Point_1st.x    = (touch_data->data_point[8] |
					   (touch_data->data_point[9] << 8)) / 10;
		Point_1st.y    = (touch_data->data_point[10] |
					   (touch_data->data_point[11] << 8)) / 10;
		Point_2nd.x    = (touch_data->data_point[12] |
					   (touch_data->data_point[13] << 8)) / 10;
		Point_2nd.y    = (touch_data->data_point[14] |
					   (touch_data->data_point[15] << 8)) / 10;
		Point_3rd.x    = (touch_data->data_point[16] |
					   (touch_data->data_point[17] << 8)) / 10;
		Point_3rd.y    = (touch_data->data_point[18] |
					   (touch_data->data_point[19] << 8)) / 10;
		Point_4th.x    = (touch_data->data_point[20] |
					   (touch_data->data_point[21] << 8)) / 10;
		Point_4th.y    = (touch_data->data_point[22] |
					   (touch_data->data_point[23] << 8)) / 10;
	}

	if (gesture_type == SINGLE_TAP || gesture_type == DOU_TAP) {
		Point_start.x = (touch_data->extra_gesture_info[0] |
					   (touch_data->extra_gesture_info[1] << 8)) / 10;
		Point_start.y = (touch_data->extra_gesture_info[2] |
					   (touch_data->extra_gesture_info[3] << 8)) / 10;
	}

	TPD_INFO("lpwg:0x%x, type:%d, clockwise: %d, points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n",
		 touch_data->gesture_id, gesture_type, clockwise, \
		 Point_start.x, Point_start.y, \
		 Point_end.x, Point_end.y, \
		 Point_1st.x, Point_1st.y, \
		 Point_2nd.x, Point_2nd.y, \
		 Point_3rd.x, Point_3rd.y, \
		 Point_4th.x, Point_4th.y);

	ret = snprintf(page, PAGE_SIZE - 1,
		       "%u,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%u\n", gesture_type,
		       Point_start.x, Point_start.y, Point_end.x, Point_end.y,
		       Point_1st.x,   Point_1st.y,   Point_2nd.x, Point_2nd.y,
		       Point_3rd.x,   Point_3rd.y,   Point_4th.x, Point_4th.y,
		       clockwise);

	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	mutex_unlock(&tcm->mutex);

	return ret;
}

DECLARE_PROC_OPS(proc_coordinate_fops, simple_open, proc_coordinate_read, NULL, NULL);

static void touch_call_notifier_fp(struct fp_underscreen_info *fp_info)
{
	struct touchpanel_event event_data;

	memset(&event_data, 0, sizeof(struct touchpanel_event));

	event_data.touch_state = fp_info->touch_state;
	event_data.area_rate = fp_info->area_rate;
	event_data.x = fp_info->x;
	event_data.y = fp_info->y;

	touchpanel_event_call_notifier(EVENT_ACTION_FOR_FINGPRINT,
		   (void *)&event_data);
}

static ssize_t proc_fingerprint_trigger_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *ppos)
{
	struct syna_tcm *tcm = PDE_DATA(file_inode(file));
	int is_down, x_pos, y_pos = 0;
	char buf[64] = {0};

	if (!tcm) {
		LOGE("ts not exist!\n");
		return count;
	}

	mutex_lock(&tcm->mutex);
	if (copy_from_user(buf, buffer, count)) {
		LOGE("%s: read proc input error.\n", __func__);
		goto EXIT;
	}

	if (sscanf(buf, "%d,%d,%d", &is_down, &x_pos, &y_pos)) {
		if(is_down) {
			tcm->fp_info.area_rate = 100;
			tcm->fp_info.x = x_pos;
			tcm->fp_info.y = y_pos;
			tcm->fp_info.touch_state = 1;
			tcm->is_fp_down = true;
			touch_call_notifier_fp(&tcm->fp_info);
			LOGI("screen on fingerprint down : (%d, %d)\n", tcm->fp_info.x, tcm->fp_info.y);
		} else {
			tcm->fp_info.touch_state = 0;
			tcm->is_fp_down = false;
			touch_call_notifier_fp(&tcm->fp_info);
			LOGI("screen on fingerprint up : (%d, %d)\n", tcm->fp_info.x, tcm->fp_info.y);
		}
	} else {
		LOGE("invalid content: '%s', length = %zd\n", buf, count);
	}

EXIT:
	mutex_unlock(&tcm->mutex);
	return count;
}

DECLARE_PROC_OPS(proc_fingerprint_trigger_fops, simple_open, NULL, proc_fingerprint_trigger_write, NULL);

/*proc/touchpanel/debug_info/health_monitor*/
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
static int tp_health_monitor_read_func(struct seq_file *s, void *v)
{
	struct syna_tcm *tcm = s->private;
	struct monitor_data *monitor_data = &tcm->monitor_data;

	mutex_lock(&tcm->mutex);

	if (monitor_data->fw_version) {
		memset(monitor_data->fw_version, 0, MAX_DEVICE_VERSION_LENGTH);
		strncpy(monitor_data->fw_version, tcm->panel_data.manufacture_info.version,
			strlen(tcm->panel_data.manufacture_info.version));
	}

	tp_healthinfo_read(s, monitor_data);

	mutex_unlock(&tcm->mutex);
	return 0;
}

static ssize_t health_monitor_control(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	struct syna_tcm *tcm = PDE_DATA(file_inode(file));
	struct monitor_data *monitor_data = &tcm->monitor_data;
	char buffer[4] = {0};
	int tmp = 0;

	mutex_lock(&tcm->mutex);
	if (count > 2) {
		goto EXIT;
	}
	if (copy_from_user(buffer, buf, count)) {
		TPD_INFO("%s: read proc input error.\n", __func__);
		goto EXIT;
	}

	if (1 == sscanf(buffer, "%d", &tmp) && tmp == 0) {
		tp_healthinfo_clear(monitor_data);
	} else {
		TPD_INFO("invalid operation\n");
	}

EXIT:
	mutex_unlock(&tcm->mutex);
	return count;
}

static int health_monitor_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_health_monitor_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_health_monitor_proc_fops, health_monitor_open, seq_read, health_monitor_control, single_release);
#endif

/*******Part5:Register node Function  Area********************/

typedef struct {
	char *name;
	umode_t mode;
	struct proc_dir_entry *node;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	const struct proc_ops *fops;
#else
	const struct file_operations *fops;
#endif
	void *data;
	bool is_created;/*proc node is creater or not*/
	bool is_support;/*feature is supported or not*/
} tp_proc_node;


/*proc/touchpanel/debug_info*/
static int init_debug_info_proc(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	struct proc_dir_entry *prEntry_debug_info = NULL;

	tp_proc_node proc_debug_node[] = {
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
		{
			"health_monitor", 0666, NULL, &tp_health_monitor_proc_fops, tcm, false,
			tcm->health_monitor_support
		},
#endif
	};

	TP_INFO(tcm->tp_index, "%s entry\n", __func__);

	/*proc/touchpanel/debug_info*/
	prEntry_debug_info = proc_mkdir("debug_info", tcm->prEntry_tp);

	if (prEntry_debug_info == NULL) {
		ret = -ENOMEM;
		TP_INFO(tcm->tp_index, "%s: Couldn't create debug_info proc entry\n", __func__);
	}

	tcm->prEntry_debug_tp = prEntry_debug_info;

	for (i = 0; i < ARRAY_SIZE(proc_debug_node); i++) {
		if (proc_debug_node[i].is_support) {
			proc_debug_node[i].node = proc_create_data(proc_debug_node[i].name,
						  proc_debug_node[i].mode,
						  prEntry_debug_info, proc_debug_node[i].fops, proc_debug_node[i].data);

			if (proc_debug_node[i].node == NULL) {
				proc_debug_node[i].is_created = false;
				TP_INFO(tcm->tp_index, "%s: Couldn't create proc/debug_info/%s\n", __func__,
					proc_debug_node[i].name);
				ret = -ENODEV;

			} else {
				proc_debug_node[i].is_created = true;
			}
		}
	}

	return ret;
}

/**
 * init_touchpanel_proc - Using for create proc interface
 * @tcm: syna_tcm struct using for common driver
 *
 * we need to set syna_tcm struct as private_data to those file_inode
 * Returning zero(success) or negative errno(failed)
 */
int init_touchpanel_proc(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	struct proc_dir_entry *prEntry_tp = NULL;
	char name[TP_NAME_SIZE_MAX];

	tp_proc_node tp_proc_node[] = {
		{
			"irq_depth", 0666, NULL, &proc_get_irq_depth_fops, tcm, false, true
		},
		{
			"baseline_test", 0666, NULL, &tp_auto_test_proc_fops, tcm, false, true
		},
		{
			"baseline_result", 0666, NULL, &tp_auto_test_result_fops, tcm, false, true
		},
		{
			"framework_mode", 0666, NULL, &tp_framework_mode_proc_fops, tcm, false, true
		},
		{
			"coordinate", 0666, NULL, &proc_coordinate_fops, tcm, false, true
		},
		{
			"fingerprint_trigger", 0666, NULL, &proc_fingerprint_trigger_fops, tcm, false, true
		},
	};

	LOGE("%s entry\n", __func__);

	/*proc files-step1:/proc/devinfo/tp  (touchpanel device info)*/
#ifndef REMOVE_OPLUS_FUNCTION

	/*if (tcm->tp_index == 0) {*/
	snprintf(name, TP_NAME_SIZE_MAX, "%s", "tp");
	/*} else {
		snprintf(name, TP_NAME_SIZE_MAX, "%s%d", "tp", tcm->tp_index);
	}*/

	register_devinfo(name, &tcm->panel_data.manufacture_info);

#endif

	/*proc files-step2:/proc/touchpanel*/
	/*if (tcm->tp_index == 0) {*/
	snprintf(name, TP_NAME_SIZE_MAX, "%s", "touchpanel");
	/*} else {
		snprintf(name, TP_NAME_SIZE_MAX, "%s%d", TPD_DEVICE, tcm->tp_index);
	}*/

	prEntry_tp = proc_mkdir(name, NULL);

	if (prEntry_tp == NULL) {
		ret = -ENOMEM;
		LOGE("%s: Couldn't create TP proc entry\n", __func__);
	}

	tcm->prEntry_tp = prEntry_tp;

	for (i = 0; i < ARRAY_SIZE(tp_proc_node); i++) {
		if (tp_proc_node[i].is_support) {
			tp_proc_node[i].node = proc_create_data(tp_proc_node[i].name,
								tp_proc_node[i].mode,
								prEntry_tp, tp_proc_node[i].fops, tp_proc_node[i].data);

			if (tp_proc_node[i].node == NULL) {
				tp_proc_node[i].is_created = false;
				LOGE("%s: Couldn't create proc/debug_info/%s\n", __func__,
					tp_proc_node[i].name);
				ret = -ENODEV;

			} else {
				tp_proc_node[i].is_created = true;
			}
		}
	}

	/*create debug_info node*/
	init_debug_info_proc(tcm, pdev);

	return ret;
}

void remove_touchpanel_proc(struct syna_tcm *tcm)
{
	char name[TP_NAME_SIZE_MAX];

	if (!tcm) {
		TPD_INFO("%s: tcm is null.\n", __func__);
		return;
	}
	/*if (tcm->tp_index == 0) {*/
		snprintf(name, TP_NAME_SIZE_MAX, "%s", "touchpanel");

	/*} else {
		snprintf(name, TP_NAME_SIZE_MAX, "%s%d", TPD_DEVICE, tcm->tp_index);
	}*/

	if (tcm->prEntry_tp) {
		remove_proc_subtree(name, NULL);
	}
}
