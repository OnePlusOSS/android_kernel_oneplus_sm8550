// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file syna_tcm2_sysfs.c
 *
 * This file implements cdev and ioctl interface in the reference driver.
 */

#include <linux/string.h>

#include "syna_tcm2.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_touch.h"
#ifdef HAS_TESTING_FEATURE
#include "syna_tcm2_testing.h"
#endif
#include "../oplus_touchscreen_v2/touchpanel_notify/touchpanel_event_notify.h"

#if (KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE) || \
	defined(HAVE_UNLOCKED_IOCTL)
#define USE_UNLOCKED_IOCTL
#endif

#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL)
#define USE_COMPAT_IOCTL
#endif

#if (KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE)
#define REPLACE_KTIME
#endif


#define ENABLE_PID_TASK

#define SIG_ATTN (46)

/* structure for IOCTLs
 */
struct syna_ioctl_data {
	unsigned int data_length;
	unsigned int buf_size;
	unsigned char __user *buf;
};

#ifdef USE_COMPAT_IOCTL
struct syna_tcm_ioctl_data_compat {
	unsigned int data_length;
	unsigned int buf_size;
	compat_uptr_t __user *buf;
};
#endif

/* defines the IOCTLs supported
 */
#define IOCTL_MAGIC 's'

/* Previous IOCTLs in early driver */
#define OLD_RESET_ID		(0x00)
#define OLD_SET_IRQ_MODE_ID	(0x01)
#define OLD_SET_RAW_MODE_ID	(0x02)
#define OLD_CONCURRENT_ID	(0x03)

#define IOCTL_OLD_RESET \
	_IO(IOCTL_MAGIC, OLD_RESET_ID)
#define IOCTL_OLD_SET_IRQ_MODE \
	_IOW(IOCTL_MAGIC, OLD_SET_IRQ_MODE_ID, int)
#define IOCTL_OLD_SET_RAW_MODE \
	_IOW(IOCTL_MAGIC, OLD_SET_RAW_MODE_ID, int)
#define IOCTL_OLD_CONCURRENT \
	_IOW(IOCTL_MAGIC, OLD_CONCURRENT_ID, int)

/* Standard IOCTLs in TCM2 driver */
#define STD_IOCTL_BEGIN		    (0x10)
#define STD_SET_PID_ID		    (0x11)
#define STD_ENABLE_IRQ_ID	    (0x12)
#define STD_RAW_READ_ID		    (0x13)
#define STD_RAW_WRITE_ID	    (0x14)
#define STD_GET_FRAME_ID	    (0x15)
#define STD_SEND_MESSAGE_ID     (0x16)
#define STD_SET_REPORTS_ID      (0x17)
#define STD_CHECK_FRAMES_ID     (0x18)
#define STD_CLEAN_OUT_FRAMES_ID (0x19)
#define STD_SET_APPLICATION_INFO_ID (0x1A)
#define STD_DO_HW_RESET_ID      (0x1B)

#define STD_DRIVER_CONFIG_ID	(0x21)
#define STD_DRIVER_GET_CONFIG_ID	(0x22)

#define CUS_INSERT_REQ_REPORT_DATA_ID (0xC0)
#define CUS_GET_POWER_STATUS_ID       (0xC1)  /* only used for TCM/TSM */
#define CUS_GET_DRIVER_STATUS_ID      (0xC2)  /* only used for tsDaemon */
#define CUS_SET_DRIVER_STATUS_ID      (0xC3)  /* only used for tsDaemon */

#define IOCTL_STD_IOCTL_BEGIN \
	_IOR(IOCTL_MAGIC, STD_IOCTL_BEGIN)
#define IOCTL_STD_SET_PID \
	_IOW(IOCTL_MAGIC, STD_SET_PID_ID, struct syna_ioctl_data *)
#define IOCTL_STD_ENABLE_IRQ \
	_IOW(IOCTL_MAGIC, STD_ENABLE_IRQ_ID, struct syna_ioctl_data *)
#define IOCTL_STD_RAW_READ \
	_IOR(IOCTL_MAGIC, STD_RAW_READ_ID, struct syna_ioctl_data *)
#define IOCTL_STD_RAW_WRITE \
	_IOW(IOCTL_MAGIC, STD_RAW_WRITE_ID, struct syna_ioctl_data *)
#define IOCTL_STD_GET_FRAME \
	_IOWR(IOCTL_MAGIC, STD_GET_FRAME_ID, struct syna_ioctl_data *)
#define IOCTL_STD_SEND_MESSAGE \
	_IOWR(IOCTL_MAGIC, STD_SEND_MESSAGE_ID, struct syna_ioctl_data *)
#define IOCTL_STD_SET_REPORT_TYPES \
	_IOW(IOCTL_MAGIC, STD_SET_REPORTS_ID, struct syna_ioctl_data *)
#define IOCTL_STD_CHECK_FRAMES \
	_IOWR(IOCTL_MAGIC, STD_CHECK_FRAMES_ID, struct syna_ioctl_data *)
#define IOCTL_STD_CLEAN_OUT_FRAMES \
	_IOWR(IOCTL_MAGIC, STD_CLEAN_OUT_FRAMES_ID, struct syna_ioctl_data *)
#define IOCTL_STD_SET_APPLICATION_INFO \
	_IOWR(IOCTL_MAGIC, STD_SET_APPLICATION_INFO_ID, struct syna_ioctl_data *)
#define IOCTL_STD_DO_HW_RESET \
	_IOWR(IOCTL_MAGIC, STD_DO_HW_RESET_ID, struct syna_ioctl_data *)

#define IOCTL_DRIVER_CONFIG \
	_IOW(IOCTL_MAGIC, STD_DRIVER_CONFIG_ID, struct syna_ioctl_data *)
#define IOCTL_DRIVER_GET_CONFIG \
	_IOR(IOCTL_MAGIC, STD_DRIVER_GET_CONFIG_ID, struct syna_ioctl_data *)

#define IOCTL_CUS_INSERT_REQ_REPORT_DATA \
	_IOWR(IOCTL_MAGIC, CUS_INSERT_REQ_REPORT_DATA_ID, struct syna_ioctl_data *)
#define IOCTL_CUS_GET_POWER_STATUS \
	_IOWR(IOCTL_MAGIC, CUS_GET_POWER_STATUS_ID, struct syna_ioctl_data *)
#define IOCTL_CUS_GET_DRIVER_STATUS \
	_IOWR(IOCTL_MAGIC, CUS_GET_DRIVER_STATUS_ID, struct syna_ioctl_data *)
#define IOCTL_CUS_SET_DRIVER_STATUS \
	_IOWR(IOCTL_MAGIC, CUS_SET_DRIVER_STATUS_ID, struct syna_ioctl_data *)

/* g_sysfs_dir represents the root directory of sysfs nodes being created
 */
static struct kobject *g_sysfs_dir;

/* g_extif_mutex is used to protect the access from the userspace application
 */
static syna_pal_mutex_t g_extif_mutex;

/* g_cdev_buf is a temporary buffer storing the data from userspace
 */
static struct tcm_buffer g_cdev_cbuf;

/* g_fifo_queue_mutex is used to protect the access from
 * the userspace application
 */
static syna_pal_mutex_t g_fifo_queue_mutex;

/* The g_sysfs_io_polling_interval is used to set the polling interval
 * for syna_tcm_send_command from syna_cdev_ioctl_send_message.
 * It will set to the mode SYSFS_FULL_INTERRUPT for using the full
 * interrupt mode. The way to update this variable is through the
 * syna_cdev_ioctl_enable_irq.
 */
unsigned int g_sysfs_io_polling_interval;

/* The g_sysfs_extra_bytes_read allows caller to ask extra bytes
 * to read. Thus, driver may need to append the requested bytes.
 */
static int g_sysfs_extra_bytes_read;

/* a buffer to record the streaming report
 * considering touch report and another reports may be co-enabled
 * at the same time, give a little buffer here (3 sec x 300 fps)
 */
#define FIFO_QUEUE_MAX_FRAMES		(1200)
#define SEND_MESSAGE_HEADER_LENGTH	(3)

/* Indicate the interrupt status especially for sysfs using */
#define SYSFS_DISABLED_INTERRUPT		(0)
#define SYSFS_ENABLED_INTERRUPT			(1)

/* Define a data structure that contains a list_head */
struct fifo_queue {
	struct list_head next;
	unsigned char *fifo_data;
	unsigned int data_length;
#ifdef REPLACE_KTIME
	struct timespec64 timestamp;
#else
	struct timeval timestamp;
#endif
};

/* Define a data structure for driver parameters configurations
 *
 *      [Integer]  [   Field   ]  [ Description         ]
 *      -------------------------------------------------
 *         1       Drv Connection  bit-0 ~ 23 reserved
 *                                 bit-24~ 31 current touchcomm version being connected
 *         2       reserved
 *         3       reserved
 *         4       Bus Data Chunk  bus config, max chunk size for data transfer
 *         5       reserved
 *         6       reserved
 *         7       reserved
 *         8       reserved
 *         9       Drv Features    bit-0: '1' to enable predict reading
 *                                 bit-1 ~ 7  reserved
 *                                 bit-8 ~15  to set up the number of bytes for extra reads
 *                                 bit-16~31  reserved
 */
struct drv_param {
	union {
		struct {
			/* reserve fields */
			unsigned char reserved_0[3];
			unsigned char connection_touchcomm_version;
			/* bus config */
			unsigned int reserved_2__3[2];
			unsigned int bus_chunk_size;
			/* reserve fields */
			unsigned int reserved_5__8[4];
			/* features */
			unsigned char feature_predict_reads:1;
			unsigned char feature_reserve_b1__7:7;
			unsigned char feature_extra_reads:8;
			unsigned char feature_reserve_b16__23:8;
			unsigned char feature_reserve_b24__31:8;
		} __packed;
		unsigned int parameters[9];
	};
};


/**
 * syna_sysfs_info_show()
 *
 * Attribute to show the device and driver information to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_sysfs_info_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	struct tcm_dev *tcm_dev;
	int i;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);
	tcm_dev = tcm->tcm_dev;

	syna_pal_mutex_lock(&g_extif_mutex);

	count = 0;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Driver version:     %d.%s\n",
			SYNAPTICS_TCM_DRIVER_VERSION,
			SYNAPTICS_TCM_DRIVER_SUBVER);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Core lib version:   %d.%02d\n\n",
			(unsigned char)(SYNA_TCM_CORE_LIB_VERSION >> 8),
			(unsigned char)SYNA_TCM_CORE_LIB_VERSION);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	if (!tcm->is_connected) {
		retval = snprintf(buf, PAGE_SIZE - count,
				"Device is NOT connected\n");
		count += retval;
		retval = count;
		goto exit;
	}


	retval = snprintf(buf, PAGE_SIZE - count,
			"TouchComm version:  %d\n", tcm_dev->id_info.version);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	switch (tcm_dev->id_info.mode) {
	case MODE_APPLICATION_FIRMWARE:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Application Firmware, 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	case MODE_BOOTLOADER:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Bootloader, 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	case MODE_ROMBOOTLOADER:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Rom Bootloader, 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	default:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Mode 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	}
	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Part number:        ");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = syna_pal_mem_cpy(buf,
			PAGE_SIZE - count,
			tcm_dev->id_info.part_number,
			sizeof(tcm_dev->id_info.part_number),
			sizeof(tcm_dev->id_info.part_number));
	if (retval < 0) {
		LOGE("Fail to copy part number string\n");
		goto exit;
	}
	buf += sizeof(tcm_dev->id_info.part_number);
	count += sizeof(tcm_dev->id_info.part_number);

	retval = snprintf(buf, PAGE_SIZE - count, "\n");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Packrat number:     %d\n\n", tcm_dev->packrat_number);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	if (tcm_dev->id_info.mode != MODE_APPLICATION_FIRMWARE) {
		retval = count;
		goto exit;
	}

	retval = snprintf(buf, PAGE_SIZE - count, "Config ID:          ");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	for (i = 0; i < MAX_SIZE_CONFIG_ID; i++) {
		retval = snprintf(buf, PAGE_SIZE - count,
			"0x%2x ", tcm_dev->config_id[i]);
		if (retval < 0)
			goto exit;
		buf += retval;
		count += retval;
	}

	retval = snprintf(buf, PAGE_SIZE - count, "\n");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"Max X & Y:          %d, %d\n", tcm_dev->max_x, tcm_dev->max_y);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"Num of objects:     %d\n", tcm_dev->max_objects);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"Num of cols & rows: %d, %d\n", tcm_dev->cols, tcm_dev->rows);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"report config:%*ph\n", tcm_dev->touch_config.data_length, tcm_dev->touch_config.buf);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = count;

exit:
	syna_pal_mutex_unlock(&g_extif_mutex);
	return retval;
}

static struct kobj_attribute kobj_attr_info =
	__ATTR(info, 0444, syna_sysfs_info_show, NULL);

/**
 * syna_sysfs_irq_en_store()
 *
 * Attribute to disable/enable the irq
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_irq_en_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (!tcm->hw_if->ops_enable_irq)
		return 0;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	syna_pal_mutex_lock(&g_extif_mutex);

	/* disable the interrupt line */
	if (input == 0) {
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
		if (retval < 0) {
			LOGE("Fail to disable interrupt\n");
			goto exit;
		}
	} else if (input == 1) {
	/* enable the interrupt line */
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		if (retval < 0) {
			LOGE("Fail to enable interrupt\n");
			goto exit;
		}
	} else {
		LOGW("Unknown option %d (0:disable / 1:enable)\n", input);
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	syna_pal_mutex_unlock(&g_extif_mutex);
	return retval;
}

static struct kobj_attribute kobj_attr_irq_en =
	__ATTR(irq_en, 0220, NULL, syna_sysfs_irq_en_store);

/**
 * syna_sysfs_reset_store()
 *
 * Attribute to issue a reset.
 * "1" for a sw reset; "2" for a hardware reset
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_reset_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	syna_pal_mutex_lock(&g_extif_mutex);

	if (input == 1) {
		retval = syna_tcm_reset(tcm->tcm_dev);
		if (retval < 0) {
			LOGE("Fail to do reset\n");
			goto exit;
		}
	} else if (input == 2) {
		if (!tcm->hw_if->ops_hw_reset) {
			LOGE("No hardware reset support\n");
			goto exit;
		}

		tcm->hw_if->ops_hw_reset(tcm->hw_if);

		/* enable the interrupt to process the identify report
		 * after the hardware reset.
		 */
		if (!tcm->hw_if->bdata_attn.irq_enabled) {
			tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
			/* disable it and back to original status */
			syna_pal_sleep_ms(100);
			tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
		}
	} else {
		LOGW("Unknown option %d (1:sw / 2:hw)\n", input);
		retval = -EINVAL;
		goto exit;
	}

	/* check the fw setup in case the settings is changed */
	if (IS_APP_FW_MODE(tcm->tcm_dev->dev_mode)) {
		retval = tcm->dev_set_up_app_fw(tcm);
		if (retval < 0) {
			LOGE("Fail to set up app fw\n");
			goto exit;
		}
	}

	retval = count;

exit:
	syna_pal_mutex_unlock(&g_extif_mutex);
	return retval;
}

static struct kobj_attribute kobj_attr_reset =
	__ATTR(reset, 0220, NULL, syna_sysfs_reset_store);


/**
 * syna_sysfs_pwr_store()
 *
 * Attribute to change the power state.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_pwr_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	syna_pal_mutex_lock(&g_extif_mutex);

	if (strncmp(buf, "resume", 6) == 0) {
		if (tcm->dev_resume)
			tcm->dev_resume(p_dev);
	} else if (strncmp(buf, "suspend", 7) == 0) {
		if (tcm->dev_suspend)
			tcm->dev_suspend(p_dev);
	} else {
		LOGW("Unknown option %s\n", buf);
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	syna_pal_mutex_unlock(&g_extif_mutex);
	return retval;
}

static struct kobj_attribute kobj_attr_pwr =
	__ATTR(power_state, 0220, NULL, syna_sysfs_pwr_store);


/**
 * syna_sysfs_gesture_coordinate_show()
 *
 */
static ssize_t syna_sysfs_gesture_coordinate_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	struct tcm_touch_data_blob *touch_data;
	uint32_t gesture_type = UNKOWN_GESTURE;
	uint32_t clockwise = 2;
	struct Coordinate Point_start;
	struct Coordinate Point_end;
	struct Coordinate Point_1st;
	struct Coordinate Point_2nd;
	struct Coordinate Point_3rd;
	struct Coordinate Point_4th;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);
	touch_data = &tcm->tp_data;

	syna_pal_mutex_lock(&g_extif_mutex);

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
		Point_start.x = (touch_data->data_point[0] | (touch_data->data_point[1]
					  << 8));
		Point_start.y = (touch_data->data_point[2] | (touch_data->data_point[3]
					  << 8));
		Point_end.x    = (touch_data->data_point[4] |
					   (touch_data->data_point[5] << 8));
		Point_end.y    = (touch_data->data_point[6] |
					   (touch_data->data_point[7] << 8));
		Point_1st.x    = (touch_data->data_point[8] |
					   (touch_data->data_point[9] << 8));
		Point_1st.y    = (touch_data->data_point[10] |
					   (touch_data->data_point[11] << 8));
		Point_2nd.x    = (touch_data->data_point[12] |
					   (touch_data->data_point[13] << 8));
		Point_2nd.y    = (touch_data->data_point[14] |
					   (touch_data->data_point[15] << 8));
		Point_3rd.x    = (touch_data->data_point[16] |
					   (touch_data->data_point[17] << 8));
		Point_3rd.y    = (touch_data->data_point[18] |
					   (touch_data->data_point[19] << 8));
		Point_4th.x    = (touch_data->data_point[20] |
					   (touch_data->data_point[21] << 8));
		Point_4th.y    = (touch_data->data_point[22] |
					   (touch_data->data_point[23] << 8));
	}

	if (gesture_type == SINGLE_TAP) {
		Point_start.x = (touch_data->extra_gesture_info[0] | (touch_data->extra_gesture_info[1] << 8));
		Point_start.y = (touch_data->extra_gesture_info[2] | (touch_data->extra_gesture_info[3] << 8));
	}

	LOGE("lpwg:0x%x, type:%d, clockwise: %d, points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n",
		 touch_data->gesture_id, gesture_type, clockwise, \
		 Point_start.x, Point_start.y, \
		 Point_end.x, Point_end.y, \
		 Point_1st.x, Point_1st.y, \
		 Point_2nd.x, Point_2nd.y, \
		 Point_3rd.x, Point_3rd.y, \
		 Point_4th.x, Point_4th.y);

	retval = snprintf(buf, PAGE_SIZE - 1,
		       "%u,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%u\n", gesture_type,
		       Point_start.x, Point_start.y, Point_end.x, Point_end.y,
		       Point_1st.x,   Point_1st.y,   Point_2nd.x, Point_2nd.y,
		       Point_3rd.x,   Point_3rd.y,   Point_4th.x, Point_4th.y,
		       clockwise);
	if (retval < 0)
		goto exit;

exit:
	syna_pal_mutex_unlock(&g_extif_mutex);
	return retval;
}

static struct kobj_attribute kobj_attr_gesture_coordinate =
	__ATTR(gesture_coordinate, 0444, syna_sysfs_gesture_coordinate_show, NULL);


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

/**
 * fingerprint_trigger()
 *
 * Attribute to disable/enable the irq
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_fingerprint_trigger_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int is_down, x_pos, y_pos = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	syna_pal_mutex_lock(&g_extif_mutex);

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

	syna_pal_mutex_unlock(&g_extif_mutex);
	return count;
}

static struct kobj_attribute kobj_attr_fingerprint_trigger =
	__ATTR(fingerprint_trigger, 0220, NULL, syna_sysfs_fingerprint_trigger_store);

/**
 * declaration of sysfs attributes
 */
static struct attribute *attrs[] = {
	&kobj_attr_info.attr,
	&kobj_attr_irq_en.attr,
	&kobj_attr_reset.attr,
	&kobj_attr_pwr.attr,
	&kobj_attr_gesture_coordinate.attr,
	&kobj_attr_fingerprint_trigger.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

/**
 * syna_sysfs_create_dir()
 *
 * Create a directory and register it with sysfs.
 * Then, create all defined sysfs files.
 *
 * @param
 *    [ in] tcm:  the driver handle
 *    [ in] pdev: an instance of platform device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_sysfs_create_dir(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int retval = 0;

	g_sysfs_dir = kobject_create_and_add("sysfs",
			&pdev->dev.kobj);
	if (!g_sysfs_dir) {
		LOGE("Fail to create sysfs directory\n");
		return -ENOTDIR;
	}

	tcm->sysfs_dir = g_sysfs_dir;

	retval = sysfs_create_group(g_sysfs_dir, &attr_group);
	if (retval < 0) {
		LOGE("Fail to create sysfs group\n");

		kobject_put(tcm->sysfs_dir);
		return retval;
	}

#ifdef HAS_TESTING_FEATURE
	retval = syna_testing_create_dir(tcm, g_sysfs_dir);
	if (retval < 0) {
		LOGE("Fail to create testing sysfs\n");

		sysfs_remove_group(tcm->sysfs_dir, &attr_group);
		kobject_put(tcm->sysfs_dir);
		return retval;
	}
#endif

	return 0;
}
/**
 * syna_sysfs_remove_dir()
 *
 * Remove the allocate sysfs directory
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
void syna_sysfs_remove_dir(struct syna_tcm *tcm)
{
	if (!tcm) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	if (tcm->sysfs_dir) {
#ifdef HAS_TESTING_FEATURE
		syna_testing_remove_dir();
#endif

		sysfs_remove_group(tcm->sysfs_dir, &attr_group);

		kobject_put(tcm->sysfs_dir);
	}

}
#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
/**
 * syna_cdev_insert_fifo()
 *
 * Insert/Push the data to the queue.
 *
 * This function is called by syna_cdev_update_report_queue(),
 * where the event data will be placed as the below format in byte
 * and use this function to store the data in queue.
 *     [0        ] : status / report code
 *     [1 :   2  ] : length of data frame
 *     [3 : N + 3] : N bytes of data payload
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] buf_ptr:  points to a data going to push
 *    [ in] length:   data length
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_insert_fifo(struct syna_tcm *tcm,
		unsigned char *buf_ptr, unsigned int length)
{
	int retval = 0;
	struct fifo_queue *pfifo_data;
	struct fifo_queue *pfifo_data_temp;
	static int pre_remaining_frames = -1;

	syna_pal_mutex_lock(&g_fifo_queue_mutex);

	/* check queue buffer limit */
	if (tcm->fifo_remaining_frame >= FIFO_QUEUE_MAX_FRAMES) {
		if (tcm->fifo_remaining_frame != pre_remaining_frames)
			LOGI("Reached %d and drop FIFO first frame\n",
				tcm->fifo_remaining_frame);

		pfifo_data_temp = list_first_entry(&tcm->frame_fifo_queue,
						struct fifo_queue, next);

		list_del(&pfifo_data_temp->next);
		kfree(pfifo_data_temp->fifo_data);
		kfree(pfifo_data_temp);
		pre_remaining_frames = tcm->fifo_remaining_frame;
		tcm->fifo_remaining_frame--;
	} else if (pre_remaining_frames >= FIFO_QUEUE_MAX_FRAMES) {
		LOGI("Reached limit, dropped oldest frame, remaining:%d\n",
			tcm->fifo_remaining_frame);
		pre_remaining_frames = tcm->fifo_remaining_frame;
	}

	pfifo_data = kmalloc(sizeof(*pfifo_data), GFP_KERNEL);
	if (!(pfifo_data)) {
		LOGE("Failed to allocate memory\n");
		LOGE("Allocation size = %zu\n", (sizeof(*pfifo_data)));
		retval = -ENOMEM;
		goto exit;
	}

	pfifo_data->fifo_data = kmalloc(length, GFP_KERNEL);
	if (!(pfifo_data->fifo_data)) {
		LOGE("Failed to allocate memory, size = %d\n", length);
		retval = -ENOMEM;
		goto exit;
	}

	pfifo_data->data_length = length;

	memcpy((void *)pfifo_data->fifo_data, (void *)buf_ptr, length);
#ifdef REPLACE_KTIME
	ktime_get_real_ts64(&(pfifo_data->timestamp));
#else
	do_gettimeofday(&(pfifo_data->timestamp));
#endif
	/* append the data to the tail for FIFO queueing */
	list_add_tail(&pfifo_data->next, &tcm->frame_fifo_queue);
	tcm->fifo_remaining_frame++;
	retval = 0;

exit:
	syna_pal_mutex_unlock(&g_fifo_queue_mutex);
	return retval;
}
#endif
/**
 * syna_cdev_ioctl_do_hw_reset()
 *
 * Perform the hardware reset with the selected reset method. The reset
 * option depends on the hardware design. The user has to add the
 * corresponding implementation in this function for the userspace
 * application.
 *
 * The reset options:
 *    [ Not support]: 0
 *    [   Reset Pin]: 1
 *    [ Power cycle]: 2
 *    [    ........]:
 *
 * @param
 *    [ in] tcm:           the driver handle
 *    [ in] ubuf_ptr:      points to a memory space from userspace
 *    [ in] buf_size:      size of given space
 *    [ in] data_size:     input data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_do_hw_reset(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned int param = 0;
	unsigned char data[4] = {0};

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return _EINVAL;
	}

	if (buf_size < sizeof(data) || data_size < sizeof(data)) {
		LOGE("Invalid sync data size, buf_size: %u\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

	retval = copy_from_user(data, ubuf_ptr, sizeof(data));
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	/* get the reset option param */
	param = syna_pal_le4_to_uint(&data[0]);
	LOGI("HW reset option: %u\n", param);

	if (param == 1) {
		if (!tcm->hw_if->ops_hw_reset) {
			LOGE("No hardware reset support\n");
			retval = -ENODEV;
			goto exit;
		}

		tcm->hw_if->ops_hw_reset(tcm->hw_if);

		/* to enable the interrupt for processing the identify report
		 * after the hardware reset.
		 */
		if (!tcm->hw_if->bdata_attn.irq_enabled) {
			tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
			/* disable it and back to original status */
			syna_pal_sleep_ms(100);
			//tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
			LOGW("HW reset: IRQ is forced to enable\n");
		} else {
			LOGI("HW reset: IRQ already enabled\n");
		}
	} else {
		LOGW("Unsupported HW reset option(%u) selected\n", param);
		retval = -EINVAL;
		goto exit;
	}

	/* check the fw setup in case the settings is changed */
	retval = tcm->dev_set_up_app_fw(tcm);
	if (retval < 0) {
		LOGE("HW reset: failed to set up the app fw\n");
		retval = -ENODATA;
		goto exit;
	}

exit:
	return retval;
}

/**
 * syna_cdev_ioctl_application_info()
 *
 * To keep the userspace application information, the user shall apply
 * the corresponding defined format on userspace. Otherwise, data will
 * be void type.
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_application_info(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	void *data = NULL;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if ((buf_size < 1) || (buf_size < data_size)) {
		LOGE("Invalid input buffer size, buf_size:%u, data_size:%u\n",
			buf_size, data_size);
		return -EINVAL;
	}

	/* free the allocated memory*/
	if (tcm->userspace_app_info != NULL)
		syna_pal_mem_free(tcm->userspace_app_info);

	tcm->userspace_app_info = syna_pal_mem_alloc(1, data_size);
	if (!(tcm->userspace_app_info)) {
		LOGE("Failed to allocate user app info memory, size = %u\n",
			data_size);
		retval = -ENOMEM;
		goto exit;
	}

	syna_pal_mem_set(tcm->userspace_app_info, 0, data_size);
	data = tcm->userspace_app_info;

	retval = copy_from_user(data, ubuf_ptr, data_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	/*
	 * The user shall cast the retrieved data to the format defined
	 * on userspace for the application.
	 */

exit:
	return retval;
}

/**
 * syna_cdev_ioctl_check_frame()
 *
 * Check the queuing status and wait for the data if it's empty.
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in/out] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: timeout value for queue waiting
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_check_frame(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	int result = 0;
	unsigned int timeout = 0;
	unsigned int frames = 0;
	unsigned char data[4] = {0};

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return _EINVAL;
	}

	if (buf_size < sizeof(data) || data_size < sizeof(data)) {
		LOGE("Invalid sync data size, buf_size: %u\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

	result = copy_from_user(data, ubuf_ptr,
			sizeof(data));
	if (result) {
		LOGE("Fail to copy data from user space\n");
		retval = -EBADE;
		goto exit;
	}

	/* Store the waiting duration length */
	timeout = syna_pal_le4_to_uint(&data[0]);
	LOGD("Time out: %d\n", timeout);

	if (list_empty(&tcm->frame_fifo_queue)) {
		LOGD("The queue is empty, wait for the frames\n");
		result = wait_event_interruptible_timeout(tcm->wait_frame,
				(tcm->fifo_remaining_frame > 0),
				msecs_to_jiffies(timeout));
		if (result == 0) {
			LOGD("Queue waiting timed out after %dms\n", timeout);
			retval = -ETIMEDOUT;
			goto exit;
		}
		LOGD("Data queued\n");
		retval = data_size;
	} else {
		LOGD("Queue is not empty\n");
		retval = data_size;
	}

exit:
	if (retval > 0) {
		frames = tcm->fifo_remaining_frame;
		data[0] = (unsigned char)(frames & 0xff);
		data[1] = (unsigned char)((frames >> 8) & 0xff);
		data[2] = (unsigned char)((frames >> 16) & 0xff);
		data[3] = (unsigned char)((frames >> 24) & 0xff);
		result = copy_to_user((void *)ubuf_ptr,
				data, sizeof(data));
		if (result) {
			LOGE("Fail to copy data to user space\n");
			retval = -EBADE;
		}
	}

	return retval;
}

/**
 * syna_cdev_clean_queue()
 *
 * Clean the data queue.
 * All data in the queue will be cleaned up in every time of device
 * open and close.
 *
 * @param
 *    [ in] tcm:       the driver handle
 *
 * @return
 *    void.
 */
static void syna_cdev_clean_queue(struct syna_tcm *tcm)
{
	struct fifo_queue *pfifo_data;

	syna_pal_mutex_lock(&g_fifo_queue_mutex);

	while (!list_empty(&tcm->frame_fifo_queue)) {
		pfifo_data = list_first_entry(&tcm->frame_fifo_queue,
				struct fifo_queue, next);
		list_del(&pfifo_data->next);
		kfree(pfifo_data->fifo_data);
		kfree(pfifo_data);
		if (tcm->fifo_remaining_frame != 0)
			tcm->fifo_remaining_frame--;
	}

	LOGD("Queue cleaned, frame: %d\n", tcm->fifo_remaining_frame);

	syna_pal_mutex_unlock(&g_fifo_queue_mutex);
}
/**
 * syna_cdev_ioctl_get_frame()
 *
 * Read the data from the queue and return to userspace if data is
 * copied or the specified timeout is expired.
 *
 * Please be noted that the retried data is formatted as follows.
 *     [0        ] : status / report code
 *     [1 :   2  ] : length of data frame
 *     [3 : N + 3] : N bytes of data payload
 *
 * @param
 *    [ in] tcm:           the driver handle
 *    [in/out] ubuf_ptr:   points to a memory space from userspace
 *    [ in] buf_size:      size of given space
 *    [out] frame_size:    frame size returned
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_get_frame(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *frame_size)
{
	int retval = 0;
	int timeout = 0;
	unsigned char timeout_data[4] = {0};
	struct fifo_queue *pfifo_data;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return _EINVAL;
	}

	if (buf_size < sizeof(timeout_data)) {
		LOGE("Invalid sync data size, buf_size:%d\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

#if !defined(ENABLE_EXTERNAL_FRAME_PROCESS)
	LOGE("ENABLE_EXTERNAL_FRAME_PROCESS is not enabled\n");
	return -EFAULT;
#endif

	retval = copy_from_user(timeout_data, ubuf_ptr, sizeof(timeout_data));
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	/* get the waiting duration */
	timeout = syna_pal_le4_to_uint(&timeout_data[0]);
	LOGD("Wait time: %dms\n", timeout);

	if (list_empty(&tcm->frame_fifo_queue)) {
		LOGD("The queue is empty, wait for the frame\n");
		retval = wait_event_interruptible_timeout(tcm->wait_frame,
				(tcm->fifo_remaining_frame > 0),
				msecs_to_jiffies(timeout));
		if (retval == 0) {
			LOGD("Queue waiting timed out after %dms\n", timeout);
			retval = -ETIMEDOUT;
			*frame_size = 0;
			goto exit;
		}
		LOGD("Data queued\n");
	}

    /* confirm the queue status */
	if (list_empty(&tcm->frame_fifo_queue)) {
		LOGD("Is queue empty? The remaining frame = %d\n",
			tcm->fifo_remaining_frame);
		retval = -ENODATA;
		goto exit;
	}

	syna_pal_mutex_lock(&g_fifo_queue_mutex);

	pfifo_data = list_first_entry(&tcm->frame_fifo_queue,
			struct fifo_queue, next);

	LOGD("Pop data from the queue, data length = %d\n",
		pfifo_data->data_length);

	if (buf_size >= pfifo_data->data_length) {
		retval = copy_to_user((void *)ubuf_ptr,
				pfifo_data->fifo_data,
				pfifo_data->data_length);
		if (retval) {
			LOGE("Fail to copy data to user space, size:%d\n",
				retval);
			retval = -EBADE;
		}

		*frame_size = pfifo_data->data_length;

	} else {
		LOGE("No enough space for data copy, buf_size:%d data:%d\n",
			buf_size, pfifo_data->data_length);

		retval = -EOVERFLOW;
		goto exit;
	}

	LOGD("Pop data: 0x%02x, 0x%02x, 0x%02x ...\n",
		pfifo_data->fifo_data[0], pfifo_data->fifo_data[1],
		pfifo_data->fifo_data[2]);

	list_del(&pfifo_data->next);

	if (retval >= 0)
		retval = pfifo_data->data_length;

	kfree(pfifo_data->fifo_data);
	kfree(pfifo_data);
	if (tcm->fifo_remaining_frame != 0)
		tcm->fifo_remaining_frame--;

	syna_pal_mutex_unlock(&g_fifo_queue_mutex);

exit:
	return retval;
}

/**
 * syna_cdev_ioctl_set_reports()
 *
 * Assign the report types for queuing. The enabled reports will be queued
 * into the FIFO queue.
 *
 * @param
 *    [ in] tcm:            the driver handle
 *    [ in] ubuf_ptr:       points to a memory space from userspace
 *    [ in] buf_size:       size of given space
 *    [ in] report_size:    report types data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_set_reports(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int report_size)
{
	int retval = 0;
	unsigned char data[REPORT_TYPES] = {0};
	unsigned int reports = 0;
	unsigned int report_set = 0;

	if (buf_size < sizeof(data)) {
		LOGE("Invalid sync data size, buf_size:%d, expected:%d\n",
			buf_size, (unsigned int)sizeof(data));
		return -EINVAL;
	}

#if !defined(ENABLE_EXTERNAL_FRAME_PROCESS)
	LOGE("ENABLE_EXTERNAL_FRAME_PROCESS is not enabled\n");
	return -EINVAL;
#endif

	if (report_size == 0) {
		LOGE("Invalid written size\n");
		return -EINVAL;
	}

	retval = copy_from_user(data, ubuf_ptr, report_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	retval = syna_pal_mem_cpy(tcm->report_to_queue, REPORT_TYPES,
			data, sizeof(data), REPORT_TYPES);
	for (reports = 0 ; reports < REPORT_TYPES ; reports++) {
		if (tcm->report_to_queue[reports] == EFP_ENABLE) {
			report_set++;
			LOGD("Set report 0x%02x for queue\n", reports);
		}
	}

	LOGI("Forward %d types of reports to the Queue.\n", report_set);

	retval = report_set;

	tcm->hbp_enabled = (report_set) ? true : false;

exit:
	return retval;
}

/**
 * syna_sysfs_set_fingerprint_prepare()
 *
 * prepare to set touch and hold
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_sysfs_set_fingerprint_prepare(struct syna_tcm *tcm)
{
	int retval = 0;
	struct syna_hw_interface *hw_if = tcm->hw_if;
	int retryCnt = 0;
	#define SYNA_RETRY_CNT 60

	/* update tcm->lpwg_enabled */
	syna_dev_update_lpwg_status(tcm);

	if((tcm->sub_pwr_state == SUB_PWR_RESUME_DONE) && (tcm->pwr_state == PWR_ON)) {
		//screen on
		goto exit;
	}else if (tcm->sub_pwr_state >= SUB_PWR_EARLY_SUSPENDING) {
		//screen off
		if(tcm->sub_pwr_state < SUB_PWR_SUSPEND_DONE) {
			/* wait the early suspend and suspend */
			retryCnt = SYNA_RETRY_CNT;
retry:
			syna_pal_sleep_ms(5);
			retryCnt--;
			if ((tcm->sub_pwr_state < SUB_PWR_SUSPEND_DONE) && (retryCnt > 0))
				goto retry;

			if(retryCnt <= 0) {
				LOGE("retryCnt is too small, Please Incress the retryCnt\n");
				goto exit;
			}
		}

		if (!tcm->tcm_dev->is_sleep) {
			/* not in sleep */
			goto exit;
		} else {
			// deep sleep, need to exit deepsleep firstly
			/* set irq back to active mode if not enabled yet */
			/* enable irq */
			if (hw_if->ops_enable_irq)
				hw_if->ops_enable_irq(hw_if, true);

			/* bring out of sleep mode. */
			retval = syna_tcm_sleep(tcm->tcm_dev, false);
			if (retval < 0) {
				LOGE("Fail to exit deep sleep\n");
				goto exit;
			}

			/* enable LPWG */
			retval = syna_dev_enable_lowpwr_gesture(tcm, true);
			if (retval < 0) {
				LOGE("Fail to enable low power gesture mode\n");
				goto exit;
			}

			goto exit;
		}
	} else {
		/* should not come here */
		LOGE("Fail to set fingerPrint prepare(%d %d)\n", tcm->sub_pwr_state, tcm->pwr_state);
		goto exit;
	}

exit:
	if(tcm->sub_pwr_state == SUB_PWR_SUSPEND_DONE) {
		/* enable the report to queue */
		syna_cdev_clean_queue(tcm);
		syna_pal_mem_set(tcm->report_to_queue, EFP_ENABLE, (STATUS_ERROR + 1));
		LOGI("enable response to report_to_queue for touch_and_hold\n");
		tcm->hbp_enabled = true;
	}
	return retval;
}

/**
 * syna_sysfs_set_fingerprint_post()
 *
 * post to set touch and hold
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_sysfs_set_fingerprint_post(struct syna_tcm *tcm)
{
	int retval = 0;
	struct syna_hw_interface *hw_if = tcm->hw_if;

	/* update tcm->lpwg_enabled */
	syna_dev_update_lpwg_status(tcm);

	if((tcm->sub_pwr_state == SUB_PWR_RESUME_DONE) && (tcm->pwr_state == PWR_ON)) {
		//screen on, nothing to do
		goto exit;
	} else if (tcm->sub_pwr_state == SUB_PWR_SUSPEND_DONE){
		/* do not fill any report/response to queue */
		LOGI("Disable all Report and response to report_to_queue\n");
		syna_pal_mem_set(tcm->report_to_queue, EFP_DISABLE, REPORT_TYPES);
		tcm->hbp_enabled = false;

		//screen off
		if(tcm->touch_and_hold)
			goto exit;

		// if disable touch_and_hold, need to recovery to sleep if there is no gesture
		if (tcm->lpwg_enabled) {
			/* LPWG, nothing to do */
			goto exit;
		} else {
			/* enter sleep mode. */
			retval = syna_tcm_sleep(tcm->tcm_dev, true);
			if (retval < 0) {
				LOGE("Fail to enter deep sleep\n");
				goto exit;
			}

			/* once lpwg is enabled, irq should be alive.
			* otherwise, disable irq in suspend.
			*/
			/* disable irq */
			if ((hw_if->ops_enable_irq))
				hw_if->ops_enable_irq(hw_if, false);

			goto exit;
		}
	} else {
		/* should not come here */
		LOGE("Fail to set fingerPrint post(%d %d)\n", tcm->sub_pwr_state, tcm->pwr_state);
		goto exit;
	}

exit:
	return retval;
}

/**
 * syna_cdev_ioctl_send_message()
 *
 * Send the command/message from userspace.
 *
 * For updating the g_sysfs_io_polling_interval, it need to be configured
 * by syna_cdev_ioctl_enable_irq from userspace.
 *
 * @param
 *    [ in] tcm:           the driver handle
 *    [ in/out] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:      size of given space
 *    [ in/out] msg_size:  size of message
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_send_message(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *msg_size)
{
	int retval = 0;
	unsigned char *data = NULL;
	unsigned char resp_code = 0;
	unsigned int payload_length = 0;
	unsigned int delay_ms_resp = RESP_IN_POLLING;
	struct tcm_buffer resp_data_buf;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if (buf_size < SEND_MESSAGE_HEADER_LENGTH) {
		LOGE("Invalid sync data size, buf_size:%d\n", buf_size);
		return -EINVAL;
	}

	if (*msg_size == 0) {
		LOGE("Invalid message length, msg size: 0\n");
		return -EINVAL;
	}

	mutex_lock(&tcm->mutex);
	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, buf_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for g_cdev_cbuf, size: %d\n",
			buf_size);
		goto exit;
	}

	data = g_cdev_cbuf.buf;

	retval = copy_from_user(data, ubuf_ptr, *msg_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", *msg_size);
		retval = -EBADE;
		goto exit;
	}

	payload_length = syna_pal_le2_to_uint(&data[1]);
	LOGD("Command = 0x%02x, payload length = %d\n",
		data[0], payload_length);

	/* init a buffer for the response data */
	syna_tcm_buf_init(&resp_data_buf);

	if (g_sysfs_io_polling_interval == RESP_IN_ATTN)
		delay_ms_resp = RESP_IN_ATTN;
	else
		delay_ms_resp = g_sysfs_io_polling_interval;

	if ((data[0] == CMD_SET_DYNAMIC_CONFIG) && (payload_length == 3)) {
		if (data[3] == DC_GESTURE_TYPE_ENABLE) {
			tcm->gesture_type = (unsigned short)syna_pal_le2_to_uint(&data[4]);
			syna_dev_update_lpwg_status(tcm);
			syna_sysfs_set_fingerprint_prepare(tcm);
			LOGI("HBP set gesture_type(0x%04x)\n", tcm->gesture_type);
		} else if (data[3] == DC_TOUCH_AND_HOLD) {
			tcm->touch_and_hold = (unsigned short)syna_pal_le2_to_uint(&data[4]);
			syna_dev_update_lpwg_status(tcm);
			syna_sysfs_set_fingerprint_prepare(tcm);
			LOGI("HBP set touch_and_hold(0x%04x)\n", tcm->touch_and_hold);
		}
	}

	retval = syna_tcm_send_command(tcm->tcm_dev,
			data[0],
			&data[3],
			payload_length,
			&resp_code,
			&resp_data_buf,
			delay_ms_resp);
	if (retval < 0) {
		LOGE("Fail to run command 0x%02x with payload len %d\n",
			data[0], payload_length);
		/* even if resp_code returned is not success
		 * this ioctl shall return the packet to caller
		 */
	}

	if ((data[0] == CMD_SET_DYNAMIC_CONFIG) && (payload_length == 3)) {
		if ((data[3] == DC_GESTURE_TYPE_ENABLE) || (data[3] == DC_TOUCH_AND_HOLD)) {
			syna_sysfs_set_fingerprint_post(tcm);
		}
	}

	syna_pal_mem_set(data, 0, buf_size);
	/* status code */
	data[0] = resp_code;
	/* the length for response data */
	data[1] = (unsigned char)(resp_data_buf.data_length & 0xff);
	data[2] = (unsigned char)((resp_data_buf.data_length >> 8) & 0xff);

	LOGD("resp data: 0x%02x 0x%02x 0x%02x\n",
		data[0], data[1], data[2]);

	/* response data */
	if (resp_data_buf.data_length > 0) {
		retval = syna_pal_mem_cpy(&g_cdev_cbuf.buf[3],
			(g_cdev_cbuf.buf_size - SEND_MESSAGE_HEADER_LENGTH),
			resp_data_buf.buf,
			resp_data_buf.buf_size,
			resp_data_buf.data_length);
		if (retval < 0) {
			LOGE("Fail to copy resp data\n");
			goto exit;
		}
	}

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
    /* It's for queuing the data when user is polling the command
     * response for the selected responses. The response will not be
     * queued if the user doesn't set the report/response types through
     * syna_cdev_ioctl_set_reports.
     */
	if (delay_ms_resp != RESP_IN_ATTN) {
		if (tcm->report_to_queue[resp_code] == EFP_ENABLE) {
			syna_cdev_update_report_queue(tcm, resp_code,
				&resp_data_buf);
		}
	}
#endif

	if (buf_size < resp_data_buf.data_length) {
		LOGE("No enough space for data copy, buf_size:%d data:%d\n",
			buf_size, resp_data_buf.data_length);
		retval = -EOVERFLOW;
		goto exit;
	}

	retval = copy_to_user((void *)ubuf_ptr,
			data, resp_data_buf.data_length + 3);
	if (retval) {
		LOGE("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	*msg_size = resp_data_buf.data_length + 3;
	retval = *msg_size;

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);
	mutex_unlock(&tcm->mutex);

	syna_tcm_buf_release(&resp_data_buf);

	return retval;
}

/**
 * syna_cdev_ioctl_enable_irq()
 *
 * Enable or disable the irq via IOCTL.
 *
 * Expect to get 4 bytes unsigned int parameter from userspace:
 *    0:         disable the irq.
 *    1:         enable the irq and set g_sysfs_io_polling_interval
 *               to RESP_IN_ATTN
 *    otherwise: enable the irq and also assign the polling interval
 *               to a specific time, which will be used when calling
 *               syna_cdev_ioctl_send_message.
 *               the min. polling time is RESP_IN_POLLING
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_enable_irq(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned int data;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if ((buf_size < sizeof(data)) || (data_size < sizeof(data))) {
		LOGE("Invalid sync data size, buf_size:%d, data_size:%d\n",
		    buf_size, data_size);
		return -EINVAL;
	}

	if (!tcm->hw_if->ops_enable_irq) {
		LOGW("Not support irq control\n");
		return -EFAULT;
	}

	retval = copy_from_user(&data, ubuf_ptr, buf_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		return -EBADE;
	}

	switch (data) {
	case SYSFS_DISABLED_INTERRUPT:
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
		if (retval < 0) {
			LOGE("Fail to disable interrupt\n");
			return retval;
		}

		g_sysfs_io_polling_interval =
			tcm->tcm_dev->msg_data.default_resp_reading;

		LOGI("IRQ is disabled by userspace application\n");

		break;
	case SYSFS_ENABLED_INTERRUPT:
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		if (retval < 0) {
			LOGE("Fail to enable interrupt\n");
			return retval;
		}

		g_sysfs_io_polling_interval = RESP_IN_ATTN;

		LOGI("IRQ is enabled by userspace application\n");

		break;
	default:
		/* recover the interrupt and also assign the polling interval */
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		if (retval < 0) {
			LOGE("Fail to enable interrupt\n");
			return retval;
		}

		g_sysfs_io_polling_interval = data;
		if (g_sysfs_io_polling_interval < RESP_IN_POLLING)
			g_sysfs_io_polling_interval = RESP_IN_POLLING;

		LOGI("IRQ is enabled by userspace application\n");
		LOGI("Polling interval is set to %d ms\n",
			g_sysfs_io_polling_interval);

		break;
	}

	return 0;
}
/**
 * syna_cdev_ioctl_store_pid()
 *
 * Save PID through IOCTL interface
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_store_pid(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned char *data = NULL;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if (buf_size < 4) {
		LOGE("Invalid sync data size, buf_size:%d\n", buf_size);
		return -EINVAL;
	}

	if (data_size < 4) {
		LOGE("Invalid data_size\n");
		return -EINVAL;
	}

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, buf_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for g_cdev_buf, size: %d\n",
			buf_size);
		goto exit;
	}

	data = g_cdev_cbuf.buf;

	retval = copy_from_user(data, ubuf_ptr, data_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	tcm->proc_pid = syna_pal_le4_to_uint(&data[0]);

	LOGD("PID: %d\n", (unsigned int)tcm->proc_pid);
#ifdef ENABLE_PID_TASK
	if (tcm->proc_pid) {
		tcm->proc_task = pid_task(
				find_vpid(tcm->proc_pid),
				PIDTYPE_PID);
		if (!tcm->proc_task) {
			LOGE("Fail to locate task, pid: %d\n",
				(unsigned int)tcm->proc_pid);
			retval = -ESRCH;
			goto exit;
		}
	}
#endif
exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	return retval;
}
/**
 * syna_cdev_ioctl_raw_read()
 *
 * Read the data from device directly without routing to command wrapper
 * interface.
 *
 * @param
 *    [ in] tcm:         the driver handle
 *    [in/out] ubuf_ptr: ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size:    size of given space
 *    [ in] rd_size:     reading size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_raw_read(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int rd_size)
{
	int retval = 0;
	unsigned char *data = NULL;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if ((buf_size < 0) || (rd_size > buf_size)) {
		LOGE("Invalid sync data size, buf_size:%d, rd_size:%d\n",
			buf_size, rd_size);
		return -EINVAL;
	}

	if (rd_size == 0) {
		LOGE("The read length is 0\n");
		return 0;
	}

	syna_pal_mutex_lock(&tcm->tcm_dev->msg_data.rw_mutex);

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, rd_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for g_cdev_cbuf, size: %d\n",
			rd_size);
		goto exit;
	}

	data = g_cdev_cbuf.buf;

	retval = syna_tcm_read(tcm->tcm_dev,
			data,
			rd_size);
	if (retval < 0) {
		LOGE("Fail to read raw data, size: %d\n", rd_size);
		goto exit;
	}

	if (copy_to_user((void *)ubuf_ptr, data, rd_size)) {
		LOGE("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	retval = rd_size;

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	syna_pal_mutex_unlock(&tcm->tcm_dev->msg_data.rw_mutex);

	return retval;
}
/**
 * syna_cdev_ioctl_raw_write()
 *
 * Write the given data to device directly without routing to command wrapper
 * interface.
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] wr_size:  size to write
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_raw_write(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int wr_size)
{
	int retval = 0;
	unsigned char *data = NULL;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if ((buf_size < 0) || (wr_size > buf_size)) {
		LOGE("Invalid sync data size, buf_size:%d, wr_size:%d\n",
			buf_size, wr_size);
		return -EINVAL;
	}

	if (wr_size == 0) {
		LOGE("Invalid written size\n");
		return -EINVAL;
	}

	syna_pal_mutex_lock(&tcm->tcm_dev->msg_data.rw_mutex);

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, wr_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for g_cdev_cbuf, size: %d\n",
			wr_size);
		goto exit;
	}

	data = g_cdev_cbuf.buf;

	retval = copy_from_user(data, ubuf_ptr, wr_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	LOGD("Write command: 0x%02x, length: 0x%02x, 0x%02x (size:%u)\n",
		data[0], data[1], data[2], wr_size);

	retval = syna_tcm_write(tcm->tcm_dev,
			data,
			wr_size);
	if (retval < 0) {
		LOGE("Fail to write raw data, size: %u\n", wr_size);
		goto exit;
	}

	retval = wr_size;

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	syna_pal_mutex_unlock(&tcm->tcm_dev->msg_data.rw_mutex);

	return retval;
}
/**
 * syna_cdev_ioctl_get_config_params()
 *
 * Return current configuration settings to user-space
 * The returned buffer array should be same as struct drv_param
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] size:     size of array
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_get_config_params(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int size)
{
	int retval = 0;
	struct syna_hw_bus_data *bdata_io = &tcm->hw_if->bdata_io;
	struct drv_param *param;

	if (buf_size < 0) {
		LOGE("Invalid sync data size, out of range\n");
		return -EINVAL;
	}

	if (size < sizeof(struct drv_param)) {
		LOGE("Invalid data input, size: %d (expected: %d)\n",
			size, sizeof(struct drv_param));
		return -EINVAL;
	}

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, sizeof(struct drv_param));
	if (retval < 0) {
		LOGE("Fail to allocate memory for g_cdev_cbuf, size: %d\n",
			sizeof(struct drv_param));
		goto exit;
	}

	syna_pal_mem_set(&g_cdev_cbuf.buf[0], 0x00, sizeof(struct drv_param));

	param = (struct drv_param *)&g_cdev_cbuf.buf[0];

	param->parameters[0] |= (tcm->tcm_dev->id_info.version) << 24;

	param->parameters[3] = MIN(bdata_io->rd_chunk_size, bdata_io->wr_chunk_size);

	param->parameters[8] = (unsigned int)((tcm->tcm_dev->msg_data.predict_reads & 0x01) |
						(g_sysfs_extra_bytes_read & 0xff) << 8);

	/* copy the info to user-space */
	retval = copy_to_user((void *)ubuf_ptr,
		(unsigned char *)param,
		sizeof(struct drv_param));
	if (retval) {
		LOGE("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	retval = sizeof(struct drv_param);

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	return retval;
}
/**
 * syna_cdev_ioctl_config()
 *
 * Set up and connect to touch controller.
 * The given buffer array should be same as struct drv_param
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] in_size:  input data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_config(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int in_size)
{
	int retval = 0;
	struct drv_param *param;
	bool enable = false;
	int extra_bytes = 0;
	struct syna_hw_bus_data *ptr = &tcm->hw_if->bdata_io;

	if (buf_size < 0) {
		LOGE("Invalid sync data size, out of range\n");
		return -EINVAL;
	}

	if (in_size < sizeof(struct drv_param)) {
		LOGE("Invalid data input, size: %d (expected: %d)\n",
			in_size, sizeof(struct drv_param));
		return -EINVAL;
	}

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, sizeof(struct drv_param));
	if (retval < 0) {
		LOGE("Fail to allocate memory for g_cdev_cbuf, size: %d\n",
			sizeof(struct drv_param));
		goto exit;
	}

	retval = copy_from_user(&g_cdev_cbuf.buf[0], ubuf_ptr, sizeof(struct drv_param));
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	param = (struct drv_param *)&g_cdev_cbuf.buf[0];

	/* update the config based on given data */
	if ((tcm->tcm_dev) && (tcm->is_connected)) {
		/* config the read/write chunk, if user provided */
		if (param->bus_chunk_size > 0) {
			if (tcm->tcm_dev->max_rd_size != param->bus_chunk_size)
				tcm->tcm_dev->max_rd_size = param->bus_chunk_size;
			if (tcm->tcm_dev->max_wr_size != param->bus_chunk_size)
				tcm->tcm_dev->max_wr_size = param->bus_chunk_size;
		}
		/* config the feature of predict reading */
		enable = (param->feature_predict_reads == 1);
		if (tcm->tcm_dev->msg_data.predict_reads != enable) {
			LOGI("request to %s predict reading\n", (enable) ? "enable":"disable");
			syna_tcm_enable_predict_reading(tcm->tcm_dev, enable);
		}
		/* config the feature of extra bytes reading */
		extra_bytes = param->feature_extra_reads;
		if (g_sysfs_extra_bytes_read != extra_bytes) {
			g_sysfs_extra_bytes_read = extra_bytes;
			LOGI("request to read in %d extra bytes\n", extra_bytes);
		}
	} else {
		/* restore the features */
		if (tcm->tcm_dev->max_wr_size != ptr->wr_chunk_size)
			tcm->tcm_dev->max_wr_size = ptr->wr_chunk_size;
		if (tcm->tcm_dev->max_rd_size != ptr->rd_chunk_size)
			tcm->tcm_dev->max_rd_size = ptr->rd_chunk_size;
		g_sysfs_extra_bytes_read = 0;
		tcm->tcm_dev->msg_data.predict_reads = false;
	}

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	return retval;
}

/**
 * syna_cdev_ioctl_get_power_status()
 *
 * get driver power status (only used for TCM/TSM)
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in/out] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ out] data_size: power status size returned
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_get_power_status(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *data_size)
{
	int retval = 0;
	struct tcm_power_state_info info;
	unsigned int data_length = sizeof(info);

	if (buf_size < data_length) {
		LOGE("Invalid size, buf_size: %u\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

	info.power_state     = tcm->pwr_state;
	info.driver_req_mode = ((tcm->pwr_state == LOW_PWR) || (tcm->pwr_state == PWR_OFF)) ?
							DRIVER_REQ_MODE_SUSPEND : DRIVER_REQ_MODE_NONE;

	info.lpwg_enabled    = tcm->lpwg_enabled;
	if((tcm->tcm_dev != NULL) && (tcm->tcm_dev->hw_if != NULL)) {
		info.irq_enabled = tcm->tcm_dev->hw_if->bdata_attn.irq_enabled;
	} else {
		info.irq_enabled = 0;
	}

	retval = copy_to_user((void *)ubuf_ptr, (void *)&info, data_length);
	if (retval) {
		LOGE("Fail to copy data to user space, size:%d\n",
			retval);
		retval = -EBADE;
	}

	*data_size = data_length;

exit:
	return retval;
}

/**
 * syna_cdev_ioctl_get_driver_status()
 *
 * get driver status (only used for tsDaemon)
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in/out] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ out] data_size: power status size returned
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_get_driver_status(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *data_size)
{
	int retval = 0;
	int level = 0;
	struct syna_hw_rst_data *rst = &tcm->tcm_dev->hw_if->bdata_rst;
	struct tcm_driver_state_info info;
	unsigned int data_length = sizeof(info);

	if (buf_size < data_length) {
		LOGE("Invalid size, buf_size: %u\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

	info.power_state     = tcm->pwr_state;
	level = gpio_get_value(rst->reset_gpio);
	info.hw_resetting    = (rst->reset_on_state == level) ? 1 : 0;

	info.lpwg_enabled    = tcm->lpwg_enabled;
	if((tcm->tcm_dev != NULL) && (tcm->tcm_dev->hw_if != NULL)) {
		info.irq_enabled = tcm->tcm_dev->hw_if->bdata_attn.irq_enabled;
	} else {
		info.irq_enabled = 0;
	}

	retval = copy_to_user((void *)ubuf_ptr, (void *)&info, data_length);
	if (retval) {
		LOGE("Fail to copy data to user space, size:%d\n",
			retval);
		retval = -EBADE;
	}

	*data_size = data_length;

exit:
	return retval;
}

/**
 * syna_cdev_ioctl_set_driver_status()
 *
 * Transmit the tsDaemon info to drvier
 *
 * @param
 *    [ in] tcm:            the driver handle
 *    [ in] ubuf_ptr:       points to a memory space from userspace
 *    [ in] buf_size:       size of given space
 *    [ in] data_size:      input data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_set_driver_status(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned int param = 0;
	unsigned char data[4] = {0};

	if (buf_size < sizeof(data) || data_size < sizeof(data)) {
		LOGE("Invalid sync data size, buf_size:%d, expected:%d; data_size:%d, expected:%d\n",
			buf_size, (unsigned int)sizeof(data), data_size, (unsigned int)sizeof(data));
		return -EINVAL;
	}

	retval = copy_from_user(data, ubuf_ptr, sizeof(data));
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	// convert to int
	param = syna_pal_le4_to_uint(&data[0]);
	LOGI("Daemon Set: %u\n", param);

exit:
	return retval;
}

/**
 * syna_cdev_ioctl_dispatch()
 *
 * Dispatch the IOCTLs operation based on the given code
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] code:      code for the target operation
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] ubuf_size: size of given space
 *    [ in] wr_size:   written data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_dispatch(struct syna_tcm *tcm,
		unsigned int code, const unsigned char *ubuf_ptr,
		unsigned int ubuf_size, unsigned int *data_size)
{
	int retval = 0;

	switch (code) {
	case STD_SET_PID_ID:
		retval = syna_cdev_ioctl_store_pid(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_ENABLE_IRQ_ID:
		retval = syna_cdev_ioctl_enable_irq(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_RAW_WRITE_ID:
		retval = syna_cdev_ioctl_raw_write(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_RAW_READ_ID:
		retval = syna_cdev_ioctl_raw_read(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_GET_FRAME_ID:
		retval = syna_cdev_ioctl_get_frame(tcm,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case STD_SEND_MESSAGE_ID:
		retval = syna_cdev_ioctl_send_message(tcm,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case STD_SET_REPORTS_ID:
		retval = syna_cdev_ioctl_set_reports(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_CHECK_FRAMES_ID:
		retval = syna_cdev_ioctl_check_frame(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_CLEAN_OUT_FRAMES_ID:
		LOGD("STD_CLEAN_OUT_FRAMES_ID called\n");
		syna_cdev_clean_queue(tcm);
		retval = 0;
		break;
	case STD_SET_APPLICATION_INFO_ID:
		retval = syna_cdev_ioctl_application_info(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_DO_HW_RESET_ID:
		LOGD("STD_DO_HW_RESET_ID called\n");
		retval = syna_cdev_ioctl_do_hw_reset(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_DRIVER_CONFIG_ID:
		retval = syna_cdev_ioctl_config(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_DRIVER_GET_CONFIG_ID:
		retval = syna_cdev_ioctl_get_config_params(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case CUS_INSERT_REQ_REPORT_DATA_ID:
		LOGE("Not support this ioctl code: 0x%x\n", code);
		break;
	case CUS_GET_POWER_STATUS_ID:
		retval = syna_cdev_ioctl_get_power_status(tcm,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case CUS_GET_DRIVER_STATUS_ID:
		retval = syna_cdev_ioctl_get_driver_status(tcm,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case CUS_SET_DRIVER_STATUS_ID:
		retval = syna_cdev_ioctl_set_driver_status(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	default:
		LOGE("Unknown ioctl code: 0x%x\n", code);
		return -EINVAL;
	}

	return retval;
}
/**
 * syna_cdev_ioctl_old_dispatch()
 *
 * Dispatch the old IOCTLs operation based on the given code
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] code:     code for the target operation
 *    [ in] arg:      argument passed from user-space
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_old_dispatch(struct syna_tcm *tcm,
		unsigned int code, unsigned long arg)
{
	int retval = 0;

	switch (code) {
	case OLD_RESET_ID:
		retval = syna_tcm_reset(tcm->tcm_dev);
		if (retval < 0) {
			LOGE("Fail to do reset\n");
			break;
		}

		retval = tcm->dev_set_up_app_fw(tcm);
		if (retval < 0) {
			LOGE("Fail to set up app fw\n");
			break;
		}

		break;
	case OLD_SET_IRQ_MODE_ID:
		if (!tcm->hw_if->ops_enable_irq) {
			retval = -EFAULT;
			break;
		}

		if (arg == 0)
			retval = tcm->hw_if->ops_enable_irq(tcm->hw_if,
					false);
		else if (arg == 1)
			retval = tcm->hw_if->ops_enable_irq(tcm->hw_if,
					true);
		break;
	case OLD_SET_RAW_MODE_ID:
		if (arg == 0)
			tcm->is_attn_redirecting = false;
		else if (arg == 1)
			tcm->is_attn_redirecting = true;

		break;
	case OLD_CONCURRENT_ID:
		retval = 0;
		break;

	default:
		LOGE("Unknown ioctl code: 0x%x\n", code);
		retval = -EINVAL;
		break;
	}

	return retval;
}

/**
 * syna_cdev_ioctls()
 *
 * Used to implements the IOCTL operations
 *
 * @param
 *    [ in] filp: represents the file descriptor
 *    [ in] cmd:  command code sent from userspace
 *    [ in] arg:  arguments sent from userspace
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
#ifdef USE_UNLOCKED_IOCTL
static long syna_cdev_ioctls(struct file *filp, unsigned int cmd,
		unsigned long arg)
#else
static int syna_cdev_ioctls(struct inode *inp, struct file *filp,
		unsigned int cmd, unsigned long arg)
#endif
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	struct syna_ioctl_data ioc_data;
	unsigned char *ptr = NULL;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	syna_pal_mutex_lock(&g_extif_mutex);

	retval = 0;

	LOGD("IOC_ID:0x%02X received\n", (unsigned int)_IOC_NR(cmd));

	/* handle the old IOCTLs */
	if ((_IOC_NR(cmd)) < STD_IOCTL_BEGIN) {
		retval = syna_cdev_ioctl_old_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd), arg);

		goto exit;
	} else if ((_IOC_NR(cmd)) == STD_IOCTL_BEGIN) {
		retval = 1;
		goto exit;
	}

	retval = copy_from_user(&ioc_data,
			(void __user *) arg,
			sizeof(struct syna_ioctl_data));
	if (retval) {
		LOGE("Fail to copy ioctl_data from user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

	ptr = ioc_data.buf;

	retval = syna_cdev_ioctl_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd),
			(const unsigned char *)ptr,
			ioc_data.buf_size,
			&ioc_data.data_length);
	if (retval < 0)
		goto exit;

	retval = copy_to_user((void __user *) arg,
			&ioc_data,
			sizeof(struct syna_ioctl_data));
	if (retval) {
		LOGE("Fail to update ioctl_data to user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

exit:
	syna_pal_mutex_unlock(&g_extif_mutex);

	return retval;
}

#ifdef USE_COMPAT_IOCTL
/**
 * syna_cdev_compat_ioctls()
 *
 * Used to implements the IOCTL compatible operations
 *
 * @param
 *    [ in] filp: represents the file descriptor
 *    [ in] cmd: command code sent from userspace
 *    [ in] arg: arguments sent from userspace
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static long syna_cdev_compat_ioctls(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	struct syna_tcm_ioctl_data_compat ioc_data;
	unsigned char *ptr = NULL;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	syna_pal_mutex_lock(&g_extif_mutex);

	retval = 0;

	/* handle the old IOCTLs */
	if ((_IOC_NR(cmd)) < STD_IOCTL_BEGIN) {
		retval = syna_cdev_ioctl_old_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd), arg);

		goto exit;
	} else if ((_IOC_NR(cmd)) == STD_IOCTL_BEGIN) {
		retval = 1;
		goto exit;
	}

	retval = copy_from_user(&ioc_data,
		(struct syna_tcm_ioctl_data_compat __user *) compat_ptr(arg),
		sizeof(struct syna_tcm_ioctl_data_compat));
	if (retval) {
		LOGE("Fail to copy ioctl_data from user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

	ptr = compat_ptr((unsigned long)ioc_data.buf);

	retval = syna_cdev_ioctl_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd),
			(const unsigned char *)ptr,
			ioc_data.buf_size,
			&ioc_data.data_length);
	if (retval < 0)
		goto exit;

	retval = copy_to_user(compat_ptr(arg),
			&ioc_data,
			sizeof(struct syna_tcm_ioctl_data_compat));
	if (retval) {
		LOGE("Fail to update ioctl_data to user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

exit:
	syna_pal_mutex_unlock(&g_extif_mutex);

	return retval;
}
#endif

/**
 * syna_cdev_llseek()
 *
 * Used to change the current position in a file.
 *
 * @param
 *    [ in] filp:   represents the file descriptor
 *    [ in] off:    the file position
 *    [ in] whence: flag for seeking
 *
 * @return
 *    not support
 */
static loff_t syna_cdev_llseek(struct file *filp,
		loff_t off, int whence)
{
	return -EFAULT;
}
/**
 * syna_cdev_read()
 *
 * Used to read data through the device file.
 * Function will use raw write approach.
 *
 * @param
 *    [ in] filp:  represents the file descriptor
 *    [out] buf:   given buffer from userspace
 *    [ in] count: size of buffer
 *    [ in] f_pos: the file position
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static ssize_t syna_cdev_read(struct file *filp,
		char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (count == 0)
		return 0;

	syna_pal_mutex_lock(&g_extif_mutex);

	retval = syna_cdev_ioctl_raw_read(tcm,
			(const unsigned char *)buf, count, count);
	if (retval != count) {
		LOGE("Invalid read operation, request:%d, return:%d\n",
			(unsigned int)count, retval);
	}

	syna_pal_mutex_unlock(&g_extif_mutex);

	return retval;
}
/**
 * syna_cdev_write()
 *
 * Used to send data to device through the device file.
 * Function will use raw write approach.
 *
 * @param
 *    [ in] filp:  represents the file descriptor
 *    [ in] buf:   given buffer from userspace
 *    [ in] count: size of buffer
 *    [ in] f_pos: the file position
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static ssize_t syna_cdev_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (count == 0)
		return 0;

	syna_pal_mutex_lock(&g_extif_mutex);

	retval = syna_cdev_ioctl_raw_write(tcm,
			(const unsigned char *)buf, count, count);
	if (retval != count) {
		LOGE("Invalid write operation, request:%d, return:%d\n",
			(unsigned int)count, retval);
	}

	syna_pal_mutex_unlock(&g_extif_mutex);

	return retval;
}
/**
 * syna_cdev_open()
 *
 * Invoked when the device file is being open, which should be
 * always the first operation performed on the device file
 *
 * @param
 *    [ in] inp:  represents a file in rootfs
 *    [ in] filp: represents the file descriptor
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_open(struct inode *inp, struct file *filp)
{
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	syna_pal_mutex_lock(&g_extif_mutex);

	if (tcm->char_dev_ref_count != 0) {
		LOGN("cdev already open, %d\n",
			tcm->char_dev_ref_count);
		syna_pal_mutex_unlock(&g_extif_mutex);
		return -EBUSY;
	}

	tcm->char_dev_ref_count++;

	g_sysfs_io_polling_interval = 0;

	g_sysfs_extra_bytes_read = 0;

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	syna_cdev_clean_queue(tcm);
#endif
	syna_pal_mutex_unlock(&g_extif_mutex);

	LOGI("cdev open\n");

	return 0;
}
/**
 * syna_cdev_release()
 *
 * Invoked when the device file is being released
 *
 * @param
 *    [ in] inp:  represents a file in rootfs
 *    [ in] filp: represents the file descriptor
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_release(struct inode *inp, struct file *filp)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	syna_pal_mutex_lock(&g_extif_mutex);

	if (tcm->char_dev_ref_count <= 0) {
		LOGN("cdev already closed, %d\n",
			tcm->char_dev_ref_count);
		syna_pal_mutex_unlock(&g_extif_mutex);
		return 0;
	}

	tcm->char_dev_ref_count--;

	tcm->is_attn_redirecting = false;
	syna_pal_mem_set(tcm->report_to_queue, 0, REPORT_TYPES);
#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	if(tcm->hbp_enabled) {
		retval = syna_dev_disable_hbp_mode(tcm);
		if (retval < 0) {
			LOGE("Fail to disable hbp mode\n");
		}
	}
	syna_cdev_clean_queue(tcm);
#endif
	syna_pal_mutex_unlock(&g_extif_mutex);

	g_sysfs_io_polling_interval = 0;

	g_sysfs_extra_bytes_read = 0;

	LOGI("cdev close\n");

	return 0;
}

/**
 * Declare the operations of TouchCom device file
 */
static const struct file_operations device_fops = {
	.owner = THIS_MODULE,
#ifdef USE_UNLOCKED_IOCTL
	.unlocked_ioctl = syna_cdev_ioctls,
#ifdef USE_COMPAT_IOCTL
	.compat_ioctl = syna_cdev_compat_ioctls,
#endif
#else
	.ioctl = syna_cdev_ioctls,
#endif
	.llseek = syna_cdev_llseek,
	.read = syna_cdev_read,
	.write = syna_cdev_write,
	.open = syna_cdev_open,
	.release = syna_cdev_release,
};
/**
 * syna_cdev_redirect_attn()
 *
 * Expose the status of ATTN signal to userspace
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    none.
 */
void syna_cdev_redirect_attn(struct syna_tcm *tcm)
{
	if (tcm->proc_pid)
		return;
}
#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
/**
 * syna_cdev_update_report_queue()
 *
 * Push the selected data to the queue.
 * The followings are the format of reported data
 *
 *      [Bytes]     [ Description         ]
 *      -------------------------------------------
 *      [   0   ]  status of report code
 *      [ 1 - 2 ]  length of payload data
 *      [ 3 -N+3]  N bytes of payload data
 *
 * If the extra bytes are requested, the format will become
 *      [N+3]       the original packet with N bytes of payload
 *           [0-1]  crc bytes
 *           [ 2 ]  extra rc byte
 *
 * @param
 *    [ in] tcm:         the driver handle
 *    [ in] code:        report type
 *    [ in] pevent_data: report payload
 *
 * @return
 *    none.
 */
void syna_cdev_update_report_queue(struct syna_tcm *tcm,
		unsigned char code, struct tcm_buffer *pevent_data)
{
	int retval;
	unsigned char *frame_buffer = NULL;
	unsigned int size = 0;
	unsigned short val;
	unsigned char *extrabytes = NULL;
	unsigned char *extraptr = NULL;
	const int header_size = 3;

	if (pevent_data == NULL) {
		LOGE("Returned, invalid event data pointer\n");
		return;
	}

	size = pevent_data->data_length + header_size;
	if (g_sysfs_extra_bytes_read > 0)
		size += g_sysfs_extra_bytes_read;

	LOGD("Length of queuing data = %d\n", pevent_data->data_length);
	LOGD("Total size = %d\n", size);

	frame_buffer = (unsigned char *)syna_pal_mem_alloc(size,
					sizeof(unsigned char));
	if (!frame_buffer) {
		LOGE("Fail to allocate buffer, size: %d, data_length: %d\n",
			size, pevent_data->data_length);
		return;
	}

	if (g_sysfs_extra_bytes_read > 0) {
		extrabytes = (unsigned char *)syna_pal_mem_alloc(
					g_sysfs_extra_bytes_read,
					sizeof(unsigned char));
		if (!extrabytes) {
			syna_pal_mem_free((void *)frame_buffer);

			LOGE("Fail to allocate extra buffer, size: %d\n",
				g_sysfs_extra_bytes_read);
			return;
		}
	}

	frame_buffer[0] = code;
	frame_buffer[1] = (unsigned char)pevent_data->data_length;
	frame_buffer[2] = (unsigned char)(pevent_data->data_length >> 8);

	if (pevent_data->data_length > 0) {
		retval = syna_pal_mem_cpy(&frame_buffer[header_size],
				(size - header_size),
				pevent_data->buf,
				pevent_data->data_length,
				pevent_data->data_length);
		if (retval < 0) {
			LOGE("Fail to copy data to buffer, size: %d\n",
				pevent_data->data_length);
			goto exit;
		}
	}

	if (g_sysfs_extra_bytes_read >= TCM_MSG_CRC_LENGTH) {
		val = syna_tcm_get_message_crc(tcm->tcm_dev);
		extrabytes[0] = (unsigned char)val;
		extrabytes[1] = (unsigned char)(val >> 8);

		val = g_sysfs_extra_bytes_read - TCM_MSG_CRC_LENGTH;
		if (val >= TCM_EXTRA_RC_LENGTH)
			extrabytes[2] = syna_tcm_get_extra_rc_byte(tcm->tcm_dev);

		extraptr = &frame_buffer[pevent_data->data_length + header_size];
		retval = syna_pal_mem_cpy(extraptr,
				(size - header_size - pevent_data->data_length),
				extrabytes,
				g_sysfs_extra_bytes_read,
				g_sysfs_extra_bytes_read);
		if (retval < 0) {
			LOGE("Fail to copy extra bytes to buffer\n");
			goto exit;
		}
	}

	retval = syna_cdev_insert_fifo(tcm, frame_buffer, size);
	if (retval < 0) {
		LOGE("Fail to push data to fifo\n");
		goto exit;
	}

	wake_up_interruptible(&(tcm->wait_frame));

exit:
	syna_pal_mem_free((void *)extrabytes);

	syna_pal_mem_free((void *)frame_buffer);
}

/**
 * syna_cdev_update_power_state_report_queue()
 *
 * Push the driver state info to the queue.
 *
 * @param
 *    [ in] tcm:         the driver handle
 *    [ in] wakeup:      wake up wait_frame or not
 *
 * @return
 *    none.
 */
void syna_cdev_update_power_state_report_queue(struct syna_tcm *tcm, bool wakeup)
{
	int retval;
	unsigned char *frame_buffer = NULL;
	unsigned int frame_length = 0;
	struct tcm_power_state_info info;
	unsigned int data_length = sizeof(info);

	info.power_state     = tcm->pwr_state;
	info.driver_req_mode = ((tcm->pwr_state == LOW_PWR) || (tcm->pwr_state == PWR_OFF)) ?
			DRIVER_REQ_MODE_SUSPEND : DRIVER_REQ_MODE_NONE;

	info.lpwg_enabled    = tcm->lpwg_enabled;
	if((tcm->tcm_dev != NULL) && (tcm->tcm_dev->hw_if != NULL)) {
		info.irq_enabled = tcm->tcm_dev->hw_if->bdata_attn.irq_enabled;
	} else {
		info.irq_enabled = 0;
	}

	frame_length = data_length + 3;
	LOGD("The overall queuing data length = %d\n", frame_length);
	frame_buffer = (unsigned char *)syna_pal_mem_alloc(frame_length,
					sizeof(unsigned char));
	if (!frame_buffer) {
		LOGE("Fail to allocate buffer, size: %d, data_length: %d\n",
			data_length + 3, data_length);
		return;
	}

	frame_buffer[0] = REPORT_POWER_STATE_INFO;
	frame_buffer[1] = (unsigned char)data_length;
	frame_buffer[2] = (unsigned char)(data_length >> 8);

	if (data_length > 0) {
		retval = syna_pal_mem_cpy(&frame_buffer[3],
				(frame_length - 3),
				&info,
				data_length,
				data_length);
		if (retval < 0) {
			LOGE("Fail to copy data to buffer, size: %d\n", data_length);
			goto exit;
		}
	}
	retval = syna_cdev_insert_fifo(tcm, frame_buffer, frame_length);
	if (retval < 0) {
		LOGE("Fail to insert data to fifo\n");
		goto exit;
	}

	if(wakeup)
	{
		wake_up_interruptible(&(tcm->wait_frame));
	}

exit:
	syna_pal_mem_free((void *)frame_buffer);
}
#endif
/**
 * syna_cdev_devnode()
 *
 * Provide the declaration of devtmpfs
 *
 * @param
 *    [ in] dev:  an instance of device
 *    [ in] mode: mode of created node
 *
 * @return
 *    the string of devtmpfs
 */
static char *syna_cdev_devnode(struct device *dev, umode_t *mode)
{
	if (!mode)
		return NULL;

	/* S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH */
	*mode = CHAR_DEVICE_MODE;

	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}
/**
 * syna_cdev_create_sysfs()
 *
 * Create a device node and register it with sysfs.
 *
 * @param
 *    [ in] tcm: the driver handle
 *    [ in] pdev: an instance of platform device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_cdev_create_sysfs(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int retval = 0;
	struct class *device_class = NULL;
	struct device *device = NULL;
	static int cdev_major_num;

	tcm->device_class = NULL;
	tcm->device = NULL;

	tcm->is_attn_redirecting = false;

	syna_pal_mutex_alloc(&g_extif_mutex);
#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	syna_pal_mutex_alloc(&g_fifo_queue_mutex);
#endif
	syna_tcm_buf_init(&g_cdev_cbuf);

	if (cdev_major_num) {
		tcm->char_dev_num = MKDEV(cdev_major_num, 0);
		retval = register_chrdev_region(tcm->char_dev_num, 1,
				PLATFORM_DRIVER_NAME);
		if (retval < 0) {
			LOGE("Fail to register char device\n");
			goto err_register_chrdev_region;
		}
	} else {
		retval = alloc_chrdev_region(&tcm->char_dev_num, 0, 1,
				PLATFORM_DRIVER_NAME);
		if (retval < 0) {
			LOGE("Fail to allocate char device\n");
			goto err_alloc_chrdev_region;
		}

		cdev_major_num = MAJOR(tcm->char_dev_num);
	}

	cdev_init(&tcm->char_dev, &device_fops);
	tcm->char_dev.owner = THIS_MODULE;

	retval = cdev_add(&tcm->char_dev, tcm->char_dev_num, 1);
	if (retval < 0) {
		LOGE("Fail to add cdev_add\n");
		goto err_add_chardev;
	}

	device_class = class_create(THIS_MODULE, PLATFORM_DRIVER_NAME);
	if (IS_ERR(device_class)) {
		LOGE("Fail to create device class\n");
		retval = PTR_ERR(device_class);
		goto err_create_class;
	}

	device_class->devnode = syna_cdev_devnode;

	device = device_create(device_class, NULL,
			tcm->char_dev_num, NULL,
			CHAR_DEVICE_NAME"%d", MINOR(tcm->char_dev_num));
	if (IS_ERR(tcm->device)) {
		LOGE("Fail to create character device\n");
		retval = -ENOENT;
		goto err_create_device;
	}

	tcm->device_class = device_class;

	tcm->device = device;

	tcm->char_dev_ref_count = 0;
	tcm->proc_pid = 0;

	g_sysfs_extra_bytes_read = 0;

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	INIT_LIST_HEAD(&tcm->frame_fifo_queue);
	init_waitqueue_head(&tcm->wait_frame);
#endif
	syna_pal_mem_set(tcm->report_to_queue, 0, REPORT_TYPES);

	retval = syna_sysfs_create_dir(tcm, pdev);
	if (retval < 0) {
		LOGE("Fail to create sysfs dir\n");
		retval = -ENOTDIR;
		goto err_create_dir;
	}

	return 0;

err_create_dir:
	device_destroy(device_class, tcm->char_dev_num);
err_create_device:
	class_destroy(device_class);
err_create_class:
	cdev_del(&tcm->char_dev);
err_add_chardev:
	unregister_chrdev_region(tcm->char_dev_num, 1);
err_alloc_chrdev_region:
err_register_chrdev_region:
	return retval;
}
/**
 * syna_cdev_remove_sysfs()
 *
 * Remove the allocate cdev device node and release the resource
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    none.
 */
void syna_cdev_remove_sysfs(struct syna_tcm *tcm)
{
	if (!tcm) {
		LOGE("Invalid tcm driver handle\n");
		return;
	}

	syna_sysfs_remove_dir(tcm);

	syna_pal_mem_set(tcm->report_to_queue, 0, REPORT_TYPES);
	syna_cdev_clean_queue(tcm);
	syna_pal_mutex_free(&g_fifo_queue_mutex);

	tcm->char_dev_ref_count = 0;
	tcm->proc_pid = 0;

	if (tcm->device) {
		device_destroy(tcm->device_class, tcm->char_dev_num);
		class_destroy(tcm->device_class);
		cdev_del(&tcm->char_dev);
		unregister_chrdev_region(tcm->char_dev_num, 1);
	}

	syna_tcm_buf_release(&g_cdev_cbuf);

	syna_pal_mutex_free(&g_extif_mutex);

	tcm->device_class = NULL;

	tcm->device = NULL;
}


