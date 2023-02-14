/* SPDX-License-Identifier: GPL-2.0
 *
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
 * @file syna_tcm2.h
 *
 * The header file is used for the Synaptics TouchComm reference driver.
 * Platform-specific functions and included headers are implemented in
 * syna_touchcom_platform.h and syna_touchcom_runtime.h.
 */

#ifndef _SYNAPTICS_TCM2_DRIVER_H_
#define _SYNAPTICS_TCM2_DRIVER_H_

#include <soc/oplus/device_info.h>
#include "syna_tcm2_platform.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_touch.h"
#include "../oplus_touchscreen_v2/tp_devices.h"
#include "../oplus_touchscreen_v2/touchpanel_common.h"

#define PLATFORM_DRIVER_NAME "synaptics_tcm_hbp"

#define TOUCH_INPUT_NAME "synaptics_tcm_touch"
#define TOUCH_INPUT_PHYS_PATH "synaptics_tcm/touch_input"

#define CHAR_DEVICE_NAME "tcm_hbp"
#define CHAR_DEVICE_MODE (0x0600)

#define SIG_DISPLAY_ON  44
#define SIG_DISPLAY_OFF 45

/*#define TP_NAME_SIZE_MAX 25*/
/*
#define MAX_FW_NAME_LENGTH        60
#define MAX_DEVICE_VERSION_LENGTH 16
#define MAX_DEVICE_MANU_LENGTH    16
*/
#define SYNAPTICS_TCM_DRIVER_ID (1 << 0)
#define SYNAPTICS_TCM_DRIVER_VERSION 1
#define SYNAPTICS_TCM_DRIVER_SUBVER "4.0"


#define FW_IMAGE_NAME "tp/21131/FW_S3910_SAMSUNG_HBP.img"
#define LIMIT_IMAGE_NAME "tp/21131/LIMIT_S3910_SAMSUNG_HBP.img"
#define TX_NUM 17
#define RX_NUM 38
/**
 * @section: Driver Configurations
 *
 * The macros in the driver files below are used for doing compile time
 * configuration of the driver.
 */

/**
 * @brief: HAS_SYSFS_INTERFACE
 *         Open to enable the sysfs interface
 *
 * @brief: HAS_REFLASH_FEATURE
 *         Open to enable firmware reflash features
 *
 * @brief: HAS_ROMBOOT_REFLASH_FEATURE
 *         Open to enable ROMBOOT reflash features
 *
 * @brief: HAS_TESTING_FEATURE
 *         Open to enable testing features
 */
#if 1//mod by zhangle
#define HAS_SYSFS_INTERFACE
#define HAS_REFLASH_FEATURE
#define HAS_ROMBOOT_REFLASH_FEATURE
#define HAS_TESTING_FEATURE
#else
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_SYSFS)
#define HAS_SYSFS_INTERFACE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_REFLASH)
#define HAS_REFLASH_FEATURE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_ROMBOOT)
#define HAS_ROMBOOT_REFLASH_FEATURE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_TESTING)
#define HAS_TESTING_FEATURE
#endif
#endif

/**
 * @brief: TYPE_B_PROTOCOL
 *         Open to enable the multi-touch (MT) protocol
 */
#define TYPE_B_PROTOCOL

/**
 * @brief: RESET_ON_RESUME
 *         Open if willing to issue a reset to the touch controller
 *         from suspend.
 *         Set "disable" in default.
 */
#define RESET_ON_RESUME

/**
 * @brief ENABLE_WAKEUP_GESTURE
 *        Open if having wake-up gesture support.
 */
#define ENABLE_WAKEUP_GESTURE

/**
 * @brief REPORT_SWAP_XY
 *        Open if trying to swap x and y position coordinate reported.
 * @brief REPORT_FLIP_X
 *        Open if trying to flip x position coordinate reported.
 * @brief REPORT_FLIP_Y
 *        Open if trying to flip x position coordinate reported.
 */
/* #define REPORT_SWAP_XY */
/* #define REPORT_FLIP_X */
/* #define REPORT_FLIP_Y */

/**
 * @brief REPORT_TOUCH_WIDTH
 *        Open if willing to add the width data to the input event.
 */
#define REPORT_TOUCH_WIDTH

/**
 * @brief USE_CUSTOM_TOUCH_REPORT_CONFIG
 *        Open if willing to set up the format of touch report.
 *        The custom_touch_format[] array in syna_tcm2.c can be used
 *        to describe the customized report format.
 */
/* #define USE_CUSTOM_TOUCH_REPORT_CONFIG */

/**
 * @brief STARTUP_REFLASH
 *        Open if willing to do fw checking and update at startup.
 *        The firmware image will be obtained by request_firmware() API,
 *        so please ensure the image is built-in or included properly.
 *
 *        This property is available only when SYNA_TCM2_REFLASH
 *        feature is enabled.
 */
#if defined(HAS_REFLASH_FEATURE) || defined(HAS_ROMBOOT_REFLASH_FEATURE)
#define STARTUP_REFLASH
#endif
/**
 * @brief  MULTICHIP_DUT_REFLASH
 *         Open if willing to do fw update and the DUT belongs to multi-chip
 *         product. This property dependent on STARTUP_REFLASH property.
 *
 *         Set "disable" in default.
 */
#if defined(HAS_ROMBOOT_REFLASH_FEATURE) && defined(STARTUP_REFLASH)
/* #define MULTICHIP_DUT_REFLASH */
#endif

/**
 * @brief  ENABLE_DISP_NOTIFIER
 *         Open if having display notification event and willing to listen
 *         the event from display driver.
 *
 *         Set "disable" in default due to no generic notifier for DRM
 */

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER) \
|| IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY) || IS_ENABLED(CONFIG_DRM_MSM) \
|| IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || IS_ENABLED(CONFIG_FB)
#define ENABLE_DISP_NOTIFIER
#endif
/**
 * @brief RESUME_EARLY_UNBLANK
 *        Open if willing to resume in early un-blanking state.
 *
 *        This property is available only when ENABLE_DISP_NOTIFIER
 *        feature is enabled.
 */
#ifdef ENABLE_DISP_NOTIFIER
/* #define RESUME_EARLY_UNBLANK */
#endif
/**
 * @brief  USE_DRM_PANEL_NOTIFIER
 *         Open if willing to listen the notification event from
 *         DRM_PANEL. Please be noted that 'struct drm_panel_notifier'
 *         must be implemented in the target BSP.
 *
 *        This property is available only when ENABLE_DISP_NOTIFIER
 *        feature is enabled.
 *
 *         Set "disable" in default due to no generic notifier for DRM
 */
#if defined(ENABLE_DISP_NOTIFIER) && (IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY) \
|| IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER))
#define USE_DRM_PANEL_NOTIFIER
#endif

/**
 * @brief ENABLE_EXTERNAL_FRAME_PROCESS
 *        Open if having external frame process to the userspace application.
 *
 *        Set "enable" in default
 *
 * @brief REPORT_TYPES
 *        Total types of report being used for external frame process.
 *
 * @brief EFP_ENABLE / EFP_DISABLE
 *        Specific value to label whether the report is required to be
 *        process or not.
 *
 * @brief REPORT_CONCURRENTLY
 *        Open if willing to concurrently handle reports for both kernel
 *        and userspace application.
 *
 *        Set "disable" in default
 */
#define ENABLE_EXTERNAL_FRAME_PROCESS
#define REPORT_TYPES (256)
#define EFP_ENABLE	(1)
#define EFP_DISABLE (0)
/* #define REPORT_CONCURRENTLY */

/**
 * @brief TCM_CONNECT_IN_PROBE
 *        Open if willing to detect and connect to TouchComm device at
 *        probe function; otherwise, please invoke connect() manually.
 *
 *        Set "enable" in default
 */
#define TCM_CONNECT_IN_PROBE

/**
 * @brief FORCE_CONNECTION
 *        Open if willing to connect to TouchComm device w/o error outs.
 *
 *        Set "disable" in default
 */
/* #define FORCE_CONNECTION */

/**
 * @brief ENABLE_CUSTOM_TOUCH_ENTITY
 *        Open if having custom requirements to parse the custom code
 *        entity in the touch report.
 *
 *        Set "disable" in default
 */
/* #define ENABLE_CUSTOM_TOUCH_ENTITY */

/**
 * @brief ENABLE_HELPER
 *        Open if willing to do additional handling upon helper workqueue
 *
 *        Set "disable" in default
 */
/* #define ENABLE_HELPER */

#define TOUCH_BIT_CHECK           0x3FF  /*max support 10 point report.using for detect non-valid points*/
/*
typedef enum debug_level {
	LEVEL_BASIC,    /printk basic tp debug info/
	LEVEL_DETAIL,   /printk tp detail log for stress test/
	LEVEL_DEBUG,    /printk all tp debug info/
} tp_debug_level;
*/
/**
 * @brief: Power States
 *
 * Enumerate the power states of device
 */
enum power_state {
	PWR_OFF = 0,
	PWR_ON,
	LOW_PWR,
};

/**
 * @brief: Sub Power States
 *
 * Enumerate the power states of device
 */
enum sub_power_state {
	SUB_NONE = 0,
	SUB_PWR_RESUMING,
	SUB_PWR_RESUME_DONE,
	SUB_PWR_EARLY_SUSPENDING,
	SUB_PWR_SUSPENDING,
	SUB_PWR_SUSPEND_DONE,
};

/**
 * @brief: Driver Request Mode to host
 *
 * Enumerate the driver request mode of device
 */
enum driver_req_mode {
	DRIVER_REQ_MODE_NONE    = 0,
	DRIVER_REQ_MODE_SUSPEND = 1,
};

#if defined(ENABLE_HELPER)
/**
 * @brief: Tasks for helper
 *
 * Tasks being supported in the helper thread and the structure
 */
enum helper_task {
	HELP_NONE = 0,
	HELP_RESET_DETECTED,
};

struct syna_tcm_helper {
	syna_pal_atomic_t task;
	struct work_struct work;
	struct workqueue_struct *workqueue;
};
#endif
/*
struct fp_underscreen_info {
	uint8_t touch_state;
	uint8_t area_rate;
	uint16_t x;
	uint16_t y;
};

struct com_api_data {
	spinlock_t tp_irq_lock;
	int tp_irq_disable;
};

struct com_test_data {
	const struct firmware *limit_fw;      /test limit fw/
	const struct firmware *black_test_fw;      /test limit fw/
	void *chip_test_ops;
	/save auto test result data/
	void *result_data;
	size_t result_max_len;
	size_t result_flag;
	size_t result_cur_len;
	/save black screen test result data/
	void *bs_result_data;
	size_t bs_result_max_len;
	size_t bs_result_flag;
	size_t bs_result_cur_len;
};
*/
struct tcm_engineer_test_operations {
	/*int (*black_screen_test)(struct black_gesture_test *p,
				 struct syna_tcm *tcm);                 //message of black gesture test*/
	int (*auto_test)(struct seq_file *s,  struct device *dev);         /*message of auto test*/
};
/*
typedef enum {
	TP_RATE_START,
	TP_RATE_CALC,
	TP_RATE_CLEAR,
} tp_rate;

typedef enum {
	TYPE_RECORD_INT = 0,
	TYPE_RECORD_STR,
} value_record_type;

struct health_value_count {
	struct list_head head;
	value_record_type value_type;
	void *value;
	int count;
};

#define RECORD_POINTS_COUNT 5
struct points_record {
	int count;
	struct Coordinate points[RECORD_POINTS_COUNT];
};

struct swipes_record {
	int count;
	struct Coordinate start_points[RECORD_POINTS_COUNT];
	struct Coordinate end_points[RECORD_POINTS_COUNT];
};

struct monitor_data {
	struct debug_info_proc_operations  *debug_info_ops; /debug info data

	u64 boot_time;
	u64 stat_time;
	u64 probe_time;
	u64 max_resume_time;
	u64 max_suspend_time;
	u64 max_fw_update_time;
	char *fw_version;
	const char *tp_ic;
	char *vendor;

	bool health_monitor_support;
	int max_finger_support;
	int tx_num;
	int rx_num;
	uint32_t max_x;
	uint32_t max_y;
	uint32_t long_swipe_judge_distance;
	uint32_t swipe_broken_judge_distance;
	uint32_t jumping_point_judge_distance;

	int report_rate;
	int report_rate_in_game;
	bool in_game_mode;

	int touch_num;
	int direction;

	struct point_state_monitor *points_state;

	int32_t *jumping_points_count_array;
	int32_t *stuck_points_count_array;
	int32_t *lanscape_stuck_points_count_array;
	int32_t *broken_swipes_count_array;
	struct swipes_record    long_swipes;

	int32_t *jumping_point_delta_data;
	int32_t *stuck_point_delta_data;

	int max_jumping_times;
	int max_touch_num;
	int max_touch_num_in_game;
	int current_touch_num;

	int click_count;
	int swipe_count;
	/int32_t *click_count_array;/

	u64 touch_timer;
	u64 holding_touch_time;
	u64 total_touch_time;
	u64 *total_touch_time_in_game;
	u64 max_holding_touch_time;

	u64 screenon_timer;
	u64 total_screenon_time;

	int auto_test_total_times;
	int auto_test_failed_times;

	int blackscreen_test_total_times;
	int blackscreen_test_failed_times;

	int gesture_waiting;
	bool is_gesture_waiting_read;
	u64 gesture_received_time;

	struct list_head        gesture_values_list;
	struct list_head        invalid_gesture_values_list;
	struct list_head        fp_area_rate_list;
	struct list_head        fd_values_list;
	struct list_head        health_report_list;
	struct list_head        bus_errs_list;
	struct list_head        bus_errs_buff_list;
	struct list_head        alloc_err_funcs_list;
	struct list_head        fw_update_result_list;

	unsigned char *bus_buf;
	uint16_t bus_len;

	long alloced_size;
	long max_alloc_err_size;
	long min_alloc_err_size;

	u32 smooth_level_chosen;
	u32 sensitive_level_chosen;
	int RATE_MIN;
	int below_rate_counts;
	tp_rate tp_rate_type;
	int work_freq;

	int avdd;
	int vddi;
};

#define MAX_BUS_ERROR_COUNT 30
struct exception_data {
	void  *chip_data; /debug info data/
	bool exception_upload_support;
	u32 exception_upload_count;
	u32 bus_error_count;
	u32 bus_error_upload_count;
};
*/
/**
 * @brief: context of the synaptics linux-based driver
 *
 * The structure defines the kernel specific data in linux-based driver
 */
struct syna_tcm {

	int tp_index;

	/* TouchComm device core context */
	struct tcm_dev *tcm_dev;

	/* PLatform device driver */
	struct platform_device *pdev;

	/* Generic touched data generated by tcm core lib */
	struct tcm_touch_data_blob tp_data;

	syna_pal_mutex_t tp_event_mutex;

	struct mutex		mutex;

	unsigned char prev_obj_status[MAX_NUM_OBJECTS];

	/* Buffer stored the irq event data */
	struct tcm_buffer event_data;

	/* Hardware interface layer */
	struct syna_hw_interface *hw_if;
	struct hw_resource hw_res;      /*hw resourc information*/

	/* ISR-related variables */
	pid_t isr_pid;
	bool irq_wake;

	/* cdev and sysfs nodes creation */
	struct cdev char_dev;
	dev_t char_dev_num;
	int char_dev_ref_count;

	struct class *device_class;
	struct device *device;

	struct kobject *sysfs_dir;

	struct proc_dir_entry *prEntry_tp;
	/*struct proc_dir_entry of "/proc/touchpanel/debug_info"*/
	struct proc_dir_entry *prEntry_debug_tp;

	/* Input device registration */
	struct input_dev *input_dev;
	struct input_params {
		unsigned int max_x;
		unsigned int max_y;
		unsigned int max_objects;
	} input_dev_params;

	int tx_num;
	int rx_num;
	struct panel_info panel_data;	/*GPIO control(id && pinctrl && tp_type)*/
	char *fw_name_fae;                 /*fw name fae*/

	/* Workqueue used for fw update */
	struct delayed_work reflash_work;
	struct workqueue_struct *reflash_workqueue;

	/* IOCTL-related variables */
	pid_t proc_pid;
	struct task_struct *proc_task;

	/* flags */
	int pwr_state;
	int sub_pwr_state;
	bool slept_in_early_suspend;
	bool lpwg_enabled;
	bool is_attn_redirecting;
	unsigned char fb_ready;
	bool is_connected;
	bool has_custom_tp_config;
	bool helper_enabled;
	bool startup_reflash_enabled;
	bool rst_on_resume_enabled;
	bool hbp_enabled; /* report data to report_to_queue[] */

	unsigned short gesture_type;
	unsigned short touch_and_hold;
	bool is_fp_down;
	struct fp_underscreen_info fp_info;	/*tp info used for underscreen fingerprint*/

	/* framebuffer callbacks notifier */
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
	struct drm_panel *active_panel;
	struct notifier_block fb_notif; /*register to control suspend/resume*/
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	struct drm_panel *active_panel;
	void *notifier_cookie;
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	struct notifier_block disp_notifier;
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) \
|| IS_ENABLED(CONFIG_FB)
	struct notifier_block fb_notif;	/*register to control suspend/resume*/
#endif

	struct com_api_data com_api_data;
	struct com_test_data com_test_data;	/*test comon data*/
	struct tcm_engineer_test_operations   *engineer_ops;     /*call_back function*/
	bool in_test_process;

	bool health_monitor_support;                        /*health_monitor is used*/
	struct monitor_data    monitor_data;                /*health monitor data*/

	/* fifo to pass the data to userspace */
	unsigned int fifo_remaining_frame;
	struct list_head frame_fifo_queue;
	wait_queue_head_t wait_frame;
	unsigned char report_to_queue[REPORT_TYPES];

#if defined(ENABLE_HELPER)
	/* helper workqueue */
	struct syna_tcm_helper helper;
#endif

	struct work_struct     speed_up_work;               /*using for speedup resume*/
	/*using for touchpanel speedup resume wq*/
	struct workqueue_struct *speedup_resume_wq;

	/* the pointer of userspace application info data */
	void *userspace_app_info;

	/* Specific function pointer to do device connection.
	 *
	 * This function will power on and identify the connected device.
	 * At the end of function, the ISR will be registered as well.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_connect)(struct syna_tcm *tcm);

	/* Specific function pointer to disconnect the device
	 *
	 * This function will power off the connected device.
	 * Then, all the allocated resource will be released.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_disconnect)(struct syna_tcm *tcm);

	/* Specific function pointer to set up app fw firmware
	 *
	 * This function should be called whenever the device initially
	 * powers up, resets, or firmware update.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_set_up_app_fw)(struct syna_tcm *tcm);

	/* Specific function pointer to resume the device from suspend state.
	 *
	 * @param
	 *    [ in] dev: an instance of device
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_resume)(struct device *dev);

	/* Specific function pointer to put device into suspend state.
	 *
	 * @param
	 *    [ in] dev: an instance of device
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_suspend)(struct device *dev);
};

/**
 * syna_dev_disable_hbp_mode()
 *
 * Enable or disable the hbp mode.
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_dev_disable_hbp_mode(struct syna_tcm *tcm);

/**
 * syna_dev_update_lpwg_status()
 *
 * update tcm->lpwg_enabled.
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *
 * @return
 *    none
 */
void syna_dev_update_lpwg_status(struct syna_tcm *tcm);

void syna_send_signal(struct syna_tcm *tcm, int signal_num);

/**
 * syna_dev_enable_lowpwr_gesture()
 *
 * Enable or disable the low power gesture mode.
 * Furthermore, set up the wake-up irq.
 *
 * @param
 *    [ in] tcm: tcm driver handle
 *    [ in] en:  '1' to enable low power gesture mode; '0' to disable
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_dev_enable_lowpwr_gesture(struct syna_tcm *tcm, bool en);

/**
 * @brief: Helpers for chardev nodes and sysfs nodes creation
 *
 * These functions are implemented in syna_touchcom_sysfs.c
 * and available only when HAS_SYSFS_INTERFACE is enabled.
 */
#ifdef HAS_SYSFS_INTERFACE

int syna_cdev_create_sysfs(struct syna_tcm *ptcm,
		struct platform_device *pdev);

void syna_cdev_remove_sysfs(struct syna_tcm *ptcm);

void syna_cdev_redirect_attn(struct syna_tcm *ptcm);

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
void syna_cdev_update_report_queue(struct syna_tcm *tcm,
		unsigned char code, struct tcm_buffer *pevent_data);
void syna_cdev_update_power_state_report_queue(struct syna_tcm *tcm, bool wakeup);
#endif

#endif

#endif /* end of _SYNAPTICS_TCM2_DRIVER_H_ */

