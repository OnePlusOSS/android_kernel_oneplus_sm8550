// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/err.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/stacktrace.h>
#include <asm/stack_pointer.h>
#include <asm/stacktrace.h>
#include <asm/current.h>
#include <linux/version.h>

#include "../syna_tcm2.h"
#include "../touch_comon_api/touch_comon_api.h"
#include "touchpanel_healthinfo.h"

#ifdef CONFIG_OPLUS_KEVENT_UPLOAD_DELETE
#include <linux/oplus_kevent.h>

static DEFINE_MUTEX(touchpanel_kevent_lock);

int upload_touchpanel_kevent_data(unsigned char *payload)
{
	struct kernel_packet_info *user_msg_info;
	char log_tag[] = KEVENT_LOG_TAG;
	char event_id_touchpanel[] = KEVENT_EVENT_ID;
	void *buffer = NULL;
	int len, size;

	mutex_lock(&touchpanel_kevent_lock);

	len = strlen(payload);

	size = sizeof(struct kernel_packet_info) + len + 1;

	buffer = kmalloc(size, GFP_KERNEL);

	if (!buffer) {
		TPD_INFO("%s: Allocation failed\n", __func__);
		mutex_unlock(&touchpanel_kevent_lock);
		return -ENOMEM;
	}

	memset(buffer, 0, size);
	user_msg_info = (struct kernel_packet_info *)buffer;
	user_msg_info->type = 1;

	memcpy(user_msg_info->log_tag, log_tag, strlen(log_tag) + 1);
	memcpy(user_msg_info->event_id, event_id_touchpanel,
	       strlen(event_id_touchpanel) + 1);

	user_msg_info->payload_length = len + 1;
	memcpy(user_msg_info->payload, payload, user_msg_info->payload_length);

	kevent_send_to_user(user_msg_info);
	msleep(20);
	kfree(buffer);

	mutex_unlock(&touchpanel_kevent_lock);

	return 0;
}
/*EXPORT_SYMBOL(upload_touchpanel_kevent_data);*/
#else
int upload_touchpanel_kevent_data(unsigned char *payload)
{
	return 0;
}
#endif /* CONFIG_OPLUS_KEVENT_UPLOAD_DELETE */

void reset_healthinfo_time_counter(u64 *time_counter)
{
	*time_counter = ktime_to_ms(ktime_get());
}

u64 check_healthinfo_time_counter_timeout(u64 time_counter, int ms)
{
	u64 curr = ktime_to_ms(ktime_get());

	if (curr < time_counter + ms) {
		return 0;

	} else {
		return (curr - time_counter);
	}
}

int print_swipe_from_record(struct seq_file *s,
			    struct swipes_record *swipes_record, char *prefix)
{
	int i = 0;
	int length = swipes_record->count < RECORD_POINTS_COUNT ? swipes_record->count :
		     RECORD_POINTS_COUNT;

	if (!swipes_record->count) {
		return 0;
	}

	if (s) {
		seq_printf(s, "%sswipes count:%d\n", prefix ? prefix : "",
			   swipes_record->count);
	}

	TPD_DETAIL("%sswipes count:%d\n", prefix ? prefix : "", swipes_record->count);

	for (i = 0; i < length; i++) {
		if (s) {
			seq_printf(s, "%sswipe[%d]:[%d %d] to [%d %d]\n", prefix ? prefix : "", i,
				   swipes_record->start_points[i].x, swipes_record->start_points[i].y,
				   swipes_record->end_points[i].x, swipes_record->end_points[i].y);
		}

		TPD_DETAIL("%sswipe[%d]:[%d %d] to [%d %d]\n", prefix ? prefix : "", i,
			   swipes_record->start_points[i].x, swipes_record->start_points[i].y,
			   swipes_record->end_points[i].x, swipes_record->end_points[i].y);
	}

	return 0;
}

int update_value_count_list(struct list_head *list, void *value,
			    value_record_type value_type)
{
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;
	char *value_str = (char *)value;
	int *value_int = (int *)value;
	int *vc_value = NULL;

	list_for_each(pos, list) {
		vc = (struct health_value_count *)pos;

		switch (value_type) {
		case TYPE_RECORD_INT:
			vc_value = vc->value;

			if (vc->value_type == value_type && *vc_value == *value_int) {
				vc->count++;
				TPD_DETAIL("%s int=%d, count=%d\n", __func__, *vc_value, vc->count);
				return vc->count;
			}

			break;

		case TYPE_RECORD_STR:
			if (vc->value_type == value_type && !strcmp((char *)vc->value, value_str)) {
				vc->count++;
				TPD_DETAIL("%s str=%s, count=%d\n", __func__, (char *)vc->value, vc->count);
				return vc->count;
			}

			break;

		default:
			break;
		}
	}

	vc = kzalloc(sizeof(struct health_value_count), GFP_KERNEL);

	if (vc) {
		switch (value_type) {
		case TYPE_RECORD_INT:
			vc->value = kzalloc(sizeof(int), GFP_KERNEL);

			if (vc->value) {
				vc_value = vc->value;
				*vc_value = *value_int;
				vc->value_type = value_type;
				vc->count = 1;
				list_add_tail(&vc->head, list);
				TPD_DETAIL("%s int=%d, count=%d\n", __func__, *value_int, vc->count);
				return vc->count;

			} else {
				TPD_INFO("vc->value kzalloc failed.\n");
				kfree(vc);
				return -1;
			}

		case TYPE_RECORD_STR:
			vc->value = kzalloc(sizeof(char) * (strlen(value_str) + 1), GFP_KERNEL);

			if (vc->value) {
				strncpy((char *)vc->value, value_str, strlen(value_str) + 1);
				vc->value_type = value_type;
				vc->count = 1;
				list_add_tail(&vc->head, list);
				TPD_DETAIL("%s str=%s, count=%d\n", __func__, (char *)vc->value, vc->count);
				return vc->count;

			} else {
				TPD_INFO("vc->value kzalloc failed.\n");
				kfree(vc);
				return -1;
			}

		default:
			break;
		}

	} else {
		TPD_INFO("kzalloc failed.\n");
		return -1;
	}

	return 0;
}

int clear_value_count_list(struct list_head *list)
{
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;

	while (!list_empty(list)) {
		pos = list->next;
		list_del(pos);
		vc = list_entry(pos, struct health_value_count, head);
		if (vc->value_type == TYPE_RECORD_STR) {
			kfree(vc->value);
		} else {
			kfree(vc->value);
		}
		kfree(vc);
	}
	if (list_empty(list)) {
		TPD_INFO("list is cleared success.\n");
	} else {
		TPD_INFO("list is cleared fail.\n");
	}

	return 0;
}

int print_value_count_list(struct seq_file *s, struct list_head *list,
			   value_record_type value_type, char *prefix)
{
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;
	int *vc_value = NULL;

	list_for_each(pos, list) {
		vc = (struct health_value_count *)pos;

		if (value_type == vc->value_type) {
			switch (value_type) {
			case TYPE_RECORD_INT:
				vc_value = vc->value;

				if (s) {
					seq_printf(s, "%s%d:%d\n", prefix ? prefix : "", *vc_value, vc->count);
				}

				TPD_DETAIL("%s%d:%d\n", prefix ? prefix : "", *vc_value, vc->count);
				break;

			case TYPE_RECORD_STR:
				if (s) {
					seq_printf(s, "%s%s:%d\n", prefix ? prefix : "", (char *)vc->value, vc->count);
				}

				TPD_DETAIL("%s%s:%d\n", prefix ? prefix : "", (char *)vc->value, vc->count);
				break;

			default:
				break;
			}
		}
	}

	return 0;
}

char *print_as_matrix(struct seq_file *s, void *value, int len, int linebreak,
		      bool need_feedback)
{
	uint8_t *tmp_uint8 = NULL;
	char *tmp_str = NULL;
	char *tmp_str_child = NULL;
	char *tmp_str_feedback = NULL;
	int i = 0;
	int retval;

	if (!linebreak) {
		linebreak = DEFAULT_BUF_MATRIX_LINEBREAK;
	}

	tmp_uint8 = tp_kzalloc(len, GFP_KERNEL);

	if (!tmp_uint8) {
		TPD_INFO("tmp_uint8 tp_kzalloc failed.\n");
		goto out;
	}

	tmp_str = tp_kzalloc(linebreak * DEFAULT_CHILD_STR_LEN, GFP_KERNEL);

	if (!tmp_str) {
		TPD_INFO("tmp_str tp_kzalloc failed.\n");
		goto out;
	}

	tmp_str_child = tp_kzalloc(DEFAULT_CHILD_STR_LEN, GFP_KERNEL);

	if (!tmp_str_child) {
		TPD_INFO("tmp_str_child tp_kzalloc failed.\n");
		goto out;
	}

	if (need_feedback) {
		tmp_str_feedback = tp_kzalloc(linebreak * DEFAULT_CHILD_STR_LEN, GFP_KERNEL);
	}

	retval = tp_memcpy(tmp_uint8, len, value, len, len);

	if (retval < 0) {
		TPD_INFO("tmp_str_child tp_kzalloc failed.\n");
		goto out;
	}

	memset(tmp_str, 0, linebreak * DEFAULT_CHILD_STR_LEN);

	for (i = 0; i < len; i++) {
		memset(tmp_str_child, 0, DEFAULT_CHILD_STR_LEN);
		snprintf(tmp_str_child, DEFAULT_CHILD_STR_LEN, "0x%02x, ", tmp_uint8[i]);
		strcat(tmp_str, tmp_str_child);

		if (i % linebreak == linebreak - 1) {
			if (s) {
				seq_printf(s, "%s\n", tmp_str);
			}

			TPD_DETAIL("%s\n", tmp_str);

			if (tmp_str_feedback && (i / linebreak == 0)) {
				strncpy(tmp_str_feedback, tmp_str, strlen(tmp_str));
			}

			memset(tmp_str, 0, linebreak * DEFAULT_CHILD_STR_LEN);
		}
	}

	if (i % linebreak) {
		if (s) {
			seq_printf(s, "%s\n", tmp_str);
		}

		TPD_DETAIL("%s\n", tmp_str);

		if (tmp_str_feedback && (i / linebreak == 0)) {
			strncpy(tmp_str_feedback, tmp_str, strlen(tmp_str));
		}
	}

out:
	tp_kfree((void **)&tmp_uint8);
	tp_kfree((void **)&tmp_str);
	tp_kfree((void **)&tmp_str_child);

	return tmp_str_feedback;
}

char *record_buffer_data(struct list_head *list, char *record,
			 uint16_t recordlen, int max_num)
{
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;
	void *pre_str = NULL;
	void *next_str = NULL;
	char *feedback = NULL;
	int pre_count = 0;
	int next_count = 0;
	int list_num = 0;
	int retval;

	list_for_each(pos, list) {
		vc = (struct health_value_count *)pos;

		if (!list_num) {
			next_str = vc->value;
			vc->value = tp_kzalloc(recordlen, GFP_KERNEL);

			if (vc->value) {
				retval = tp_memcpy(vc->value, recordlen, record, recordlen, recordlen);
				if (retval < 0) {
					TPD_INFO("tp_memcpy failed.\n");
					vc->value = next_str;
					return feedback;
				} else {
					next_count = vc->count;
					vc->count = recordlen;
					TPD_DETAIL("len = %d\n", vc->count);
					feedback = print_as_matrix(NULL, vc->value, vc->count,
								   DEFAULT_BUF_MATRIX_LINEBREAK, true);
				}
			} else {
				TPD_INFO("vc->value tp_kzalloc failed.\n");
				vc->value = next_str;
				return feedback;
			}

		} else {
			pre_str = next_str;
			next_str = vc->value;
			vc->value = pre_str;

			pre_count = next_count;
			next_count = vc->count;
			vc->count = pre_count;
			TPD_DETAIL("len = %d\n", vc->count);
			print_as_matrix(NULL, vc->value, vc->count, DEFAULT_BUF_MATRIX_LINEBREAK,
					false);
		}

		list_num++;
	}

	if (list_num < max_num) {
		vc = tp_kzalloc(sizeof(struct health_value_count), GFP_KERNEL);

		if (vc) {
			if (!list_num) {
				vc->value = tp_kzalloc(recordlen, GFP_KERNEL);

				if (vc->value) {
					retval = tp_memcpy(vc->value, recordlen, record, recordlen, recordlen);
					if (retval < 0) {
						TPD_INFO("tp_memcpy failed.\n");
						tp_kfree((void **)&(vc->value));
						tp_kfree((void **)&vc);
						return feedback;
					} else {
						vc->count = recordlen;
						TPD_DETAIL("len = %d\n", vc->count);
						feedback = print_as_matrix(NULL, vc->value, vc->count,
									   DEFAULT_BUF_MATRIX_LINEBREAK, true);
					}
				} else {
					TPD_INFO("vc->value tp_kzalloc failed.\n");
					tp_kfree((void **)&vc);
					return feedback;
				}

			} else {
				vc->value = next_str;
				vc->count = next_count;
				TPD_DETAIL("len = %d\n", vc->count);
				print_as_matrix(NULL, vc->value, vc->count, DEFAULT_BUF_MATRIX_LINEBREAK, false);
			}

			vc->value_type = TYPE_RECORD_STR;
			list_add_tail(&vc->head, list);
			list_num++;

		} else {
			TPD_INFO("tp_kzalloc failed.\n");
			return feedback;
		}

	} else {
		tp_kfree((void **)&next_str);
	}

	return feedback;
}

void print_buffer_list(struct seq_file *s, struct list_head *list, char *prefix)
{
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;

	list_for_each(pos, list) {
		vc = (struct health_value_count *)pos;

		if (s) {
			seq_printf(s, "%slen=%d\n", prefix ? prefix : "", vc->count);
		}

		print_as_matrix(s, vc->value, vc->count, DEFAULT_BUF_MATRIX_LINEBREAK, false);
	}
}

void print_delta_data(struct seq_file *s, int32_t *delta_data, int tx_num,
		      int rx_num)
{
	char *tmp_str = NULL;
	char *tmp_str_child = NULL;
	int i = 0, j = 0;

	if (!delta_data) {
		TPD_INFO("delta data hasn't alloc space.\n");
		return;
	}

	tmp_str = tp_kzalloc(DEFAULT_CHILD_STR_LEN * (rx_num + 1), GFP_KERNEL);

	if (!tmp_str) {
		TPD_INFO("tmp_str kzaloc failed\n");
		return;
	}

	tmp_str_child = tp_kzalloc(DEFAULT_CHILD_STR_LEN, GFP_KERNEL);

	if (!tmp_str_child) {
		TPD_INFO("tmp_str_child kzaloc failed\n");
		tp_kfree((void **)&tmp_str);
		return;
	}

	for (i = 0; i < tx_num; i++) {
		memset(tmp_str, 0, DEFAULT_CHILD_STR_LEN * (rx_num + 1));
		memset(tmp_str_child, 0, DEFAULT_CHILD_STR_LEN);
		snprintf(tmp_str_child, DEFAULT_CHILD_STR_LEN, "[%2d]", i);
		strcat(tmp_str, tmp_str_child);

		for (j = 0; j < rx_num; j++) {
			memset(tmp_str_child, 0, DEFAULT_CHILD_STR_LEN);
			snprintf(tmp_str_child, DEFAULT_CHILD_STR_LEN, "%4d, ",
				 delta_data[rx_num * i + j]);
			strcat(tmp_str, tmp_str_child);
		}

		if (s) {
			seq_printf(s, "%s", tmp_str);
		}

		TPD_DETAIL("%s\n", tmp_str);
	}
	if (s) {
		seq_printf(s, "\n");
	}

	tp_kfree((void **)&tmp_str);
	tp_kfree((void **)&tmp_str_child);
}

void clear_delta_data(int32_t *delta_data, int tx_num, int rx_num)
{
	int i = 0, j = 0;

	for (i = 0; i < tx_num; i++) {
		for (j = 0; j < rx_num; j++) {
			delta_data[rx_num * i + j] = 0;
		}
	}
}

bool is_delta_data_allzero(int32_t *delta_data, int tx_num, int rx_num)
{
	int i = 0;

	for (i = 0; i < tx_num * rx_num; i++) {
		if (delta_data[i]) {
			return false;
		}
	}

	return true;
}

int tp_gesture_healthinfo_handle(struct monitor_data *monitor_data,
				 int gesture_type)
{
	if (!monitor_data) {
		return 0;
	}

	if (monitor_data->is_gesture_waiting_read) {
		update_value_count_list(&monitor_data->invalid_gesture_values_list,
					&monitor_data->gesture_waiting, TYPE_RECORD_INT);
	}

	monitor_data->is_gesture_waiting_read = true;
	reset_healthinfo_time_counter(&monitor_data->gesture_received_time);
	monitor_data->gesture_waiting = gesture_type;

	return 0;
}

int tp_gesture_read_healthinfo_handle(struct monitor_data *monitor_data,
				 int gesture_type)
{
	if (!monitor_data) {
		return 0;
	}

	if (monitor_data->is_gesture_waiting_read) {
		if (check_healthinfo_time_counter_timeout(monitor_data->gesture_received_time,
				GESTURE_RESPONSE_TIME) || gesture_type != monitor_data->gesture_waiting) {
			update_value_count_list(&monitor_data->invalid_gesture_values_list,
						&monitor_data->gesture_waiting, TYPE_RECORD_INT);

		} else {
			update_value_count_list(&monitor_data->gesture_values_list,
						&monitor_data->gesture_waiting, TYPE_RECORD_INT);
		}

		monitor_data->is_gesture_waiting_read = false;
	}

	return 0;
}

int tp_fingerprint_healthinfo_handle(struct monitor_data *monitor_data,
				     uint8_t area_rate)
{
	int rate_rate = 0;

	if (!monitor_data || !area_rate) {
		return 0;
	}

	rate_rate =  area_rate / 10;

	return update_value_count_list(&monitor_data->fp_area_rate_list, &rate_rate,
				       TYPE_RECORD_INT);
}

int tp_face_detect_healthinfo_handle(struct monitor_data *monitor_data,
				     int ps_state)
{
	if (!ps_state) {
		return 0;
	}

	return update_value_count_list(&monitor_data->fd_values_list, &ps_state,
				       TYPE_RECORD_INT);
}

int tp_report_healthinfo_handle(struct monitor_data *monitor_data, char *report)
{
	int ret = 0;

	if (!monitor_data) {
		return 0;
	}

	ret = update_value_count_list(&monitor_data->health_report_list, report,
				      TYPE_RECORD_STR);

	return ret;
}

int tp_irq_interval_handle(struct monitor_data *monitor_data, int count)
{
	if (!monitor_data) {
		return 0;
	}
	if (count > 0) {
		monitor_data->below_rate_counts++;
	}
	return 0;
}

int tp_probe_healthinfo_handle(struct monitor_data *monitor_data,
			       u64 start_time)
{
	if (!monitor_data) {
		return 0;
	}

	monitor_data->boot_time = start_time;
	monitor_data->stat_time = start_time;
	monitor_data->probe_time = check_healthinfo_time_counter_timeout(start_time, 0);

	reset_healthinfo_time_counter(&monitor_data->screenon_timer);

	return 0;
}

int update_max_time(struct monitor_data *monitor_data, u64 *max_time,
		    u64 start_time)
{
	u64 time_cost = 0;

	if (!monitor_data) {
		return 0;
	}

	time_cost = check_healthinfo_time_counter_timeout(start_time, 0);

	if (time_cost > *max_time) {
		*max_time = time_cost;
	}

	return 0;
}

int tp_resume_healthinfo_handle(struct monitor_data *monitor_data,
				u64 start_time)
{
	if (!monitor_data) {
		return 0;
	}

	if (monitor_data->is_gesture_waiting_read) {
		update_value_count_list(&monitor_data->invalid_gesture_values_list,
					&monitor_data->gesture_waiting, TYPE_RECORD_INT);

		monitor_data->is_gesture_waiting_read = false;
	}

	update_max_time(monitor_data, &monitor_data->max_resume_time, start_time);

	reset_healthinfo_time_counter(&monitor_data->screenon_timer);

	return 0;
}

int tp_suspend_healthinfo_handle(struct monitor_data *monitor_data,
				 u64 start_time)
{
	u64 screenon_time = 0;

	if (!monitor_data) {
		return 0;
	}

	update_max_time(monitor_data, &monitor_data->max_suspend_time, start_time);

	screenon_time = monitor_data->screenon_timer ?
			check_healthinfo_time_counter_timeout(monitor_data->screenon_timer, 0) : 0;

	monitor_data->total_screenon_time += screenon_time;
	monitor_data->screenon_timer = 0;

	return 0;
}

int tp_test_healthinfo_handle(struct monitor_data *monitor_data,
			      healthinfo_type test_type, int error_count)
{
	int ret = 0;

	if (!monitor_data) {
		return 0;
	}

	switch (test_type) {
	case HEALTH_TEST_AUTO:
		monitor_data->auto_test_total_times++;

		if (error_count) {
			monitor_data->auto_test_failed_times++;
		}

		ret = monitor_data->auto_test_failed_times;

		TPD_INFO("TEST FAILED RATE:%d/%d.\n", monitor_data->auto_test_failed_times,
			 monitor_data->auto_test_total_times);
		break;

	case HEALTH_TEST_BLACKSCREEN:
		monitor_data->blackscreen_test_total_times++;

		if (error_count) {
			monitor_data->blackscreen_test_failed_times++;
		}

		ret = monitor_data->blackscreen_test_failed_times;

		TPD_INFO("TEST FAILED RATE:%d/%d.\n",
			 monitor_data->blackscreen_test_failed_times,
			 monitor_data->blackscreen_test_total_times);
		break;

	default:
		break;
	}

	return ret;
}

int tp_bus_err_healthinfo_handle(struct monitor_data *monitor_data, int errno,
				 unsigned char *writebuf, uint16_t writelen)
{
	char *report = NULL;
	char *buff_part = NULL;

	if (!monitor_data || errno >= 0 || !writebuf) {
		return 0;
	}

	buff_part = record_buffer_data(&monitor_data->bus_errs_buff_list, writebuf,
				       writelen, MAX_BUS_ERROR_BUFF_RECORD);

	if (buff_part) {
		report = tp_kzalloc(DEFAULT_REPORT_STR_LEN + strlen(buff_part) + 1, GFP_KERNEL);

		if (report) {
			snprintf(report, DEFAULT_REPORT_STR_LEN, "BusErr$$Errno@@%d$$Buff@@%s", errno,
				 buff_part);
			upload_touchpanel_kevent_data(report);
			tp_kfree((void **)&report);
		} else {
			TPD_INFO("alloc report kzalloc failed.\n");
		}

		tp_kfree((void **)&buff_part);
	}

	return update_value_count_list(&monitor_data->bus_errs_list, &errno,
				       TYPE_RECORD_INT);
}

int tp_alloc_healthinfo_handle(struct monitor_data *monitor_data,
			       long alloc_size, bool alloc_success)
{
	int ret = 0;
	int deep = 2;
#if IS_BUILTIN(CONFIG_TOUCHPANEL_OPLUS)
	char *report = NULL;
	char *func = NULL;
#endif /*CONFIG_TOUCHPANEL_OPLUS*/
	struct stackframe frame;

	if (!monitor_data) {
		return 0;
	}

	if (alloc_success) {
		monitor_data->alloced_size = monitor_data->alloced_size + alloc_size;
		TPD_DETAIL("%ld(%ld) bytes has been alloced.\n", monitor_data->alloced_size,
			   alloc_size);

	} else {
		if (!monitor_data->max_alloc_err_size && !monitor_data->min_alloc_err_size) {
			monitor_data->max_alloc_err_size = alloc_size;
			monitor_data->min_alloc_err_size = alloc_size;

		} else if (alloc_size > monitor_data->max_alloc_err_size) {
			monitor_data->max_alloc_err_size = alloc_size;

		} else if (alloc_size < monitor_data->min_alloc_err_size) {
			monitor_data->min_alloc_err_size = alloc_size;
		}

		TPD_INFO("alloc_err_size [%ld - %ld].\n", monitor_data->min_alloc_err_size,
			 monitor_data->max_alloc_err_size);

		frame.fp = (unsigned long)__builtin_frame_address(0);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 9, 0)
		frame.sp = current_stack_pointer;
#endif
		frame.pc = (unsigned long)tp_alloc_healthinfo_handle;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
                start_backtrace(&frame,
                                (unsigned long)__builtin_frame_address(0),
                                (unsigned long)tp_alloc_healthinfo_handle);
#endif
		while (deep--) {
#if IS_BUILTIN(CONFIG_TOUCHPANEL_OPLUS)
			ret = unwind_frame(current, &frame);
#else
			ret = -1;
#endif
			if (ret < 0) {
				return ret;
			}
		}
#if IS_BUILTIN(CONFIG_TOUCHPANEL_OPLUS)
		report = tp_kzalloc(DEFAULT_REPORT_STR_LEN, GFP_KERNEL);

		if (report) {
			TPD_INFO("%pS alloc %ld failed\n", (void *)frame.pc, alloc_size);
			snprintf(report, DEFAULT_REPORT_STR_LEN, "AllocErr$$Func@@%pS$$Size@@%ld",
				 (void *)frame.pc, alloc_size);
			upload_touchpanel_kevent_data(report);

			func = tp_kzalloc(DEFAULT_REPORT_STR_LEN, GFP_KERNEL);

			if (func) {
				memset(report, 0, DEFAULT_REPORT_STR_LEN);
				snprintf(report, DEFAULT_REPORT_STR_LEN, "%pS", (void *)frame.pc);
				strncpy(func, report, strstr(report, "+") - report);
				update_value_count_list(&monitor_data->alloc_err_funcs_list, func,
							TYPE_RECORD_STR);
			}

			tp_kfree((void **)&report);
			tp_kfree((void **)&func);

		} else {
			TPD_INFO("alloc report kzalloc failed.\n");
		}
#endif /*IS_BUILTIN(CONFIG_TOUCHPANEL_OPLUS)*/
	}

	return ret;
}

int tp_fw_update_healthinfo_handle(struct monitor_data *monitor_data,
				   char *result)
{
	int ret = 0;
	char *report = NULL;

	if (!monitor_data) {
		return 0;
	}

	ret = update_value_count_list(&monitor_data->fw_update_result_list, result,
				      TYPE_RECORD_STR);

	if (!strstr(result, FIRMWARE_UPDATE_SUCCESS_KEYWORD)) {
		report = tp_kzalloc(DEFAULT_REPORT_STR_LEN, GFP_KERNEL);

		if (report) {
			snprintf(report, DEFAULT_REPORT_STR_LEN, "FwUpdateErr$$ErrMsg@@%s", result);
			upload_touchpanel_kevent_data(report);
			tp_kfree((void **)&report);
		}
	}

	return ret;
}


int tp_voltage_healthinfo_handle(struct monitor_data *monitor_data,
			      healthinfo_type test_type, int volt)
{
	int ret = 0;

	if (!monitor_data) {
		return 0;
	}

	switch (test_type) {
	case HEALTH_AVDD:
		monitor_data->avdd = volt;
		TPD_INFO("AVDD voltage error:%d\n", volt);
		break;

	case HEALTH_VDDI:
		monitor_data->vddi = volt;
		TPD_INFO("VDDI voltage error:%d\n", volt);
		break;

	default:
		break;
	}

	return ret;
}

int tp_healthinfo_report(void *tp_monitor_data, healthinfo_type type,
			 void *value)
{
	int ret = 0;
	uint8_t *value_uint8 = (uint8_t *)value;
	char *value_str = (char *)value;
	int *value_int = (int *)value;
	long *value_long = (long *)value;
	u64 *value_u64 = (u64 *)value;
	struct monitor_data *monitor_data = (struct monitor_data *)tp_monitor_data;

	if (!monitor_data || !monitor_data->health_monitor_support) {
		return 0;
	}

	switch (type) {
	case HEALTH_TOUCH:
		break;

	case HEALTH_GESTURE:
		ret = tp_gesture_healthinfo_handle(monitor_data, *value_int);
		break;

	case HEALTH_GESTURE_READ:
		ret = tp_gesture_read_healthinfo_handle(monitor_data, *value_int);
		break;

	case HEALTH_FINGERPRINT:
		ret = tp_fingerprint_healthinfo_handle(monitor_data, *value_uint8);
		break;

	case HEALTH_FACE_DETECT:
		ret = tp_face_detect_healthinfo_handle(monitor_data, *value_int);
		break;

	case HEALTH_REPORT:
		ret = tp_report_healthinfo_handle(monitor_data, value_str);
		break;

	case HEALTH_PROBE:
		ret = tp_probe_healthinfo_handle(monitor_data, *value_u64);
		break;

	case HEALTH_RESUME:
		ret = tp_resume_healthinfo_handle(monitor_data, *value_u64);
		break;

	case HEALTH_SUSPEND:
		ret = tp_suspend_healthinfo_handle(monitor_data, *value_u64);
		break;

	case HEALTH_TEST_AUTO:
	case HEALTH_TEST_BLACKSCREEN:
		ret = tp_test_healthinfo_handle(monitor_data, type, *value_int);
		break;

	case HEALTH_BUS:
		if (monitor_data->bus_buf) {
			ret = tp_bus_err_healthinfo_handle(monitor_data, *value_int,
							   monitor_data->bus_buf, monitor_data->bus_len);
			monitor_data->bus_buf = NULL;
		}

		break;

	case HEALTH_ALLOC_SUCCESS:
		ret = tp_alloc_healthinfo_handle(monitor_data, *value_long, true);
		break;

	case HEALTH_ALLOC_FAILED:
		ret = tp_alloc_healthinfo_handle(monitor_data, *value_long, false);
		break;

	case HEALTH_FW_UPDATE:
		ret = tp_fw_update_healthinfo_handle(monitor_data, value_str);
		break;

	case HEALTH_FW_UPDATE_COST:
		ret = update_max_time(monitor_data, &monitor_data->max_fw_update_time,
				      *value_u64);
		break;

	case HEALTH_BELOW_RATE:
		ret = tp_irq_interval_handle(monitor_data, *value_int);
		break;

	case HEALTH_AVDD:
	case HEALTH_VDDI:
		ret = tp_voltage_healthinfo_handle(monitor_data, type, *value_int);
		break;

	default:
		break;
	}

	return ret;
}

int tp_healthinfo_read(struct seq_file *s, void *tp_monitor_data)
{
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;
	struct monitor_data *monitor_data = (struct monitor_data *)tp_monitor_data;
	int *vc_value = NULL;
	u64 screenon_time = 0;

	if (!monitor_data->health_monitor_support) {
		seq_printf(s, "health monitor not supported\n");
		return 0;
	}

	if (monitor_data->tp_ic) {
		if (monitor_data->vendor) {
			seq_printf(s, "tp_ic:%s+%s\n", monitor_data->tp_ic, monitor_data->vendor);
		} else {
			seq_printf(s, "tp_ic:%s\n", monitor_data->tp_ic);
		}
	} else {
		seq_printf(s, "tp_ic:%s\n", TPD_DEVICE);
	}
	seq_printf(s, "boot_time:%llds\n",
		   check_healthinfo_time_counter_timeout(monitor_data->boot_time, 0)
				   / MS_PER_SECOND);
	seq_printf(s, "stat_time:%llds\n",
		   check_healthinfo_time_counter_timeout(monitor_data->stat_time, 0)
				   / MS_PER_SECOND);
	seq_printf(s, "probe_time:%lldms\n", monitor_data->probe_time);
	seq_printf(s, "max_resume_time:%lldms\n", monitor_data->max_resume_time);
	seq_printf(s, "max_suspend_time:%lldms\n", monitor_data->max_suspend_time);
	/*seq_printf(s, "RATE_MIN:%d\n", monitor_data->RATE_MIN);
	seq_printf(s, "below_rate_counts:%d\n", monitor_data->below_rate_counts);*/

	/*touch time rate*/
	screenon_time = monitor_data->screenon_timer ?
			check_healthinfo_time_counter_timeout(monitor_data->screenon_timer, 0) : 0;
	seq_printf(s, "total_screen_on_time:%llds\n",
		   (monitor_data->total_screenon_time + screenon_time) / MS_PER_SECOND);
	/*seq_printf(s, "total_touch_time:%llds\n",
		   monitor_data->total_touch_time / MS_PER_SECOND);

	if (monitor_data->max_touch_num_in_game) {
		seq_printf(s,
			   "total_touch_time(in_game):%lld[%lld %lld %lld %lld %lld %lld %lld %lld %lld %lld]s\n",
			   monitor_data->total_touch_time_in_game[0] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[1] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[2] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[3] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[4] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[5] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[6] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[7] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[8] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[9] / MS_PER_SECOND,
			   monitor_data->total_touch_time_in_game[10] / MS_PER_SECOND);
	}

	seq_printf(s, "max_holding_touch_time:%llds\n",
		   monitor_data->max_holding_touch_time / MS_PER_SECOND);
	seq_printf(s, "max_touch_number:%d\n", monitor_data->max_touch_num);

	if (monitor_data->max_touch_num_in_game) {
		seq_printf(s, "max_touch_number(in_game):%d\n",
			   monitor_data->max_touch_num_in_game);
	}*/
	seq_printf(s, "panel_coords:%d*%d\n", monitor_data->max_x, monitor_data->max_y);
	seq_printf(s, "tx_rx_num:%d*%d\n", monitor_data->tx_num, monitor_data->rx_num);
	/*seq_printf(s, "click_count:%d\n", monitor_data->click_count);
	print_delta_data(s, monitor_data->click_count_array, CLICK_COUNT_ARRAY_HEIGHT,
			 CLICK_COUNT_ARRAY_WIDTH);
	seq_printf(s, "swipe_count:%d\n", monitor_data->swipe_count);*/

	/*firmware update*/
	if (monitor_data->fw_version) {
		seq_printf(s, "fw_version:%s\n" "max_fw_update_time:%lldms\n",
			   monitor_data->fw_version, monitor_data->max_fw_update_time);
	}

	print_value_count_list(s, &monitor_data->fw_update_result_list, TYPE_RECORD_STR,
			       PREFIX_FW_UPDATE_RESULT);

	/*bus transfer*/
	print_value_count_list(s, &monitor_data->bus_errs_list, TYPE_RECORD_INT,
			       PREFIX_BUS_TRANS_ERRNO);
	print_buffer_list(s, &monitor_data->bus_errs_buff_list,
			  PREFIX_BUS_TRANS_ERRBUF);

	/*alloc*/
	/*seq_printf(s, "alloced_size:%ld\n", monitor_data->alloced_size);

	if (monitor_data->min_alloc_err_size) {
		seq_printf(s, "alloc_error_size:%ld~%ld\n", monitor_data->min_alloc_err_size,
			   monitor_data->max_alloc_err_size);
		print_value_count_list(s, &monitor_data->alloc_err_funcs_list, TYPE_RECORD_STR,
				       PREFIX_ALLOC_ERR_FUNC);
	}*/

	/*debug info*/
	print_value_count_list(s, &monitor_data->health_report_list, TYPE_RECORD_STR,
			       PREFIX_HEALTH_REPORT);

	/*abnormal touch and swipe*/
	if (monitor_data->max_jumping_times > JUMPING_POINT_TIMES) {
		seq_printf(s, "max_point-jumping_times:%d\n", monitor_data->max_jumping_times);
		/*print_point_from_record(s, &monitor_data->jumping_points, PREFIX_POINT_JUMPING);*/
		if (!is_delta_data_allzero(monitor_data->jumping_points_count_array,
					monitor_data->rx_num / 2, monitor_data->tx_num / 2)) {
			seq_printf(s, "%spoint:", PREFIX_POINT_JUMPING);
			print_delta_data(s, monitor_data->jumping_points_count_array,
						monitor_data->rx_num / 2, monitor_data->tx_num / 2);
		}
		if (!is_delta_data_allzero(monitor_data->jumping_point_delta_data,
					monitor_data->tx_num, monitor_data->rx_num)) {
			print_delta_data(s, monitor_data->jumping_point_delta_data,
						monitor_data->tx_num, monitor_data->rx_num);
		}
	}

	/*print_point_from_record(s, &monitor_data->stuck_points, PREFIX_POINT_STUCK);
	print_point_from_record(s, &monitor_data->lanscape_stuck_points,
				PREFIX_POINT_LANSCAPE_STUCK);*/
	if (!is_delta_data_allzero(monitor_data->stuck_points_count_array,
				monitor_data->rx_num / 2, monitor_data->tx_num / 2)) {
		seq_printf(s, "%spoint:", PREFIX_POINT_STUCK);
		print_delta_data(s, monitor_data->stuck_points_count_array, monitor_data->rx_num / 2,
					monitor_data->tx_num / 2);
	}
	if (!is_delta_data_allzero(monitor_data->lanscape_stuck_points_count_array,
				monitor_data->rx_num / 2, monitor_data->tx_num / 2)) {
		seq_printf(s, "%spoint:", PREFIX_POINT_LANSCAPE_STUCK);
		print_delta_data(s, monitor_data->lanscape_stuck_points_count_array,
					monitor_data->rx_num / 2, monitor_data->tx_num / 2);
	}
	if (!is_delta_data_allzero(monitor_data->stuck_point_delta_data,
				monitor_data->tx_num, monitor_data->rx_num)) {
		seq_printf(s, "stuck-point:");
		print_delta_data(s, monitor_data->stuck_point_delta_data, monitor_data->tx_num,
					monitor_data->rx_num);
	}

	if (!is_delta_data_allzero(monitor_data->broken_swipes_count_array,
				monitor_data->rx_num / 2, monitor_data->tx_num / 2)) {
		seq_printf(s, "%sswipe:", PREFIX_SWIPE_BROKER);
		print_delta_data(s, monitor_data->broken_swipes_count_array,
					monitor_data->rx_num / 2, monitor_data->tx_num / 2);
	}
	/*print_swipe_from_record(s, &monitor_data->broken_swipes, PREFIX_SWIPE_BROKER);*/
	print_swipe_from_record(s, &monitor_data->long_swipes, PREFIX_SWIPE_SUDDNT_LONG);

	if (monitor_data->smooth_level_chosen) {
		seq_printf(s, "smooth_lv:%d\n", monitor_data->smooth_level_chosen);
	}
	if (monitor_data->sensitive_level_chosen) {
		seq_printf(s, "sensitive_lv:%d\n", monitor_data->sensitive_level_chosen);
	}

		/*black gesture*/
	list_for_each(pos, &monitor_data->gesture_values_list) {
		vc = (struct health_value_count *)pos;
        vc_value = vc->value;
		seq_printf(s, "%s%s:%d\n", PREFIX_GESTURE, *vc_value == DOU_TAP ? "double_tap" :
			   *vc_value == UP_VEE ? "up_vee" :
			   *vc_value == DOWN_VEE ? "down_vee" :
			   *vc_value == LEFT_VEE ? "(>)" :
			   *vc_value == RIGHT_VEE ? "(<)" :
			   *vc_value == CIRCLE_GESTURE ? "circle" :
			   *vc_value == DOU_SWIP ? "(||)" :
			   *vc_value == LEFT2RIGHT_SWIP ? "(-->)" :
			   *vc_value == RIGHT2LEFT_SWIP ? "(<--)" :
			   *vc_value == UP2DOWN_SWIP ? "up_to_down_|" :
			   *vc_value == DOWN2UP_SWIP ? "down_to_up_|" :
			   *vc_value == M_GESTRUE ? "(M)" :
			   *vc_value == W_GESTURE ? "(W)" :
			   *vc_value == FINGER_PRINTDOWN ? "(fingerprintdown)" :
			   *vc_value == FRINGER_PRINTUP ? "(fingerprintup)" :
			   *vc_value == SINGLE_TAP ? "single_tap" :
			   *vc_value == HEART ? "heart" : "unknown", vc->count);
	}

	list_for_each(pos, &monitor_data->invalid_gesture_values_list) {
		vc = (struct health_value_count *)pos;
        vc_value = vc->value;
		seq_printf(s, "%s%s:%d\n", PREFIX_GESTURE_INVLID,
			   *vc_value == DOU_TAP ? "double_tap" :
			   *vc_value == UP_VEE ? "up_vee" :
			   *vc_value == DOWN_VEE ? "down_vee" :
			   *vc_value == LEFT_VEE ? "(>)" :
			   *vc_value == RIGHT_VEE ? "(<)" :
			   *vc_value == CIRCLE_GESTURE ? "circle" :
			   *vc_value == DOU_SWIP ? "(||)" :
			   *vc_value == LEFT2RIGHT_SWIP ? "(-->)" :
			   *vc_value == RIGHT2LEFT_SWIP ? "(<--)" :
			   *vc_value == UP2DOWN_SWIP ? "up_to_down_|" :
			   *vc_value == DOWN2UP_SWIP ? "down_to_up_|" :
			   *vc_value == M_GESTRUE ? "(M)" :
			   *vc_value == W_GESTURE ? "(W)" :
			   *vc_value == FINGER_PRINTDOWN ? "(fingerprintdown)" :
			   *vc_value == FRINGER_PRINTUP ? "(fingerprintup)" :
			   *vc_value == SINGLE_TAP ? "single_tap" :
			   *vc_value == HEART ? "heart" : "unknown", vc->count);
	}

	/*fingerprint area rate*/
	print_value_count_list(s, &monitor_data->fp_area_rate_list, TYPE_RECORD_INT,
			       PREFIX_FP_AREA_RATE);

	/*face detect*/
	print_value_count_list(s, &monitor_data->fd_values_list, TYPE_RECORD_INT,
			       PREFIX_FD);

	/*auto test*/
	if (monitor_data->auto_test_total_times) {
		seq_printf(s, "auto_test_failed_rate:%d/%d\n",
			   monitor_data->auto_test_failed_times, monitor_data->auto_test_total_times);
	}

	if (monitor_data->blackscreen_test_total_times) {
		seq_printf(s, "blackscreen_test_failed_rate:%d/%d\n",
			   monitor_data->blackscreen_test_failed_times,
			   monitor_data->blackscreen_test_total_times);
	}

	if (monitor_data->avdd != VOLTAGE_STATE_DEFAULT) {
		seq_printf(s, "avdd:%d\n", monitor_data->avdd);
	}

	if (monitor_data->vddi != VOLTAGE_STATE_DEFAULT) {
		seq_printf(s, "vddi:%d\n", monitor_data->vddi);
	}

	seq_printf(s, "--kernel_end--\n");

	return 0;
}

int tp_healthinfo_clear(void *tp_monitor_data)
{
	struct monitor_data *monitor_data = (struct monitor_data *)tp_monitor_data;
	int i = 0;

	if (!monitor_data->health_monitor_support) {
		return 0;
	}

	TPD_INFO("Clear health info Now!\n");

	reset_healthinfo_time_counter(&monitor_data->stat_time);
	monitor_data->max_resume_time = 0;
	monitor_data->max_suspend_time = 0;

	/*touch time rate*/
	reset_healthinfo_time_counter(&monitor_data->screenon_timer);
	monitor_data->total_screenon_time = 0;
	monitor_data->total_touch_time = 0;
	for (i = 0; i <= 10; i++) {
		monitor_data->total_touch_time_in_game[i] = 0;
	}
	monitor_data->max_holding_touch_time = 0;
	monitor_data->max_touch_num = 0;
	monitor_data->max_touch_num_in_game = 0;
	monitor_data->click_count = 0;
	monitor_data->swipe_count = 0;

	/*clear_delta_data(monitor_data->click_count_array, CLICK_COUNT_ARRAY_HEIGHT, CLICK_COUNT_ARRAY_WIDTH);*/

	monitor_data->max_fw_update_time = 0;

	clear_value_count_list(&monitor_data->fw_update_result_list);

	/*bus transfer*/
	clear_value_count_list(&monitor_data->bus_errs_list);
	clear_value_count_list(&monitor_data->bus_errs_buff_list);

	monitor_data->min_alloc_err_size = 0;
	monitor_data->max_alloc_err_size = 0;
	clear_value_count_list(&monitor_data->alloc_err_funcs_list);

	/*debug info*/
	clear_value_count_list(&monitor_data->health_report_list);

	/*abnormal touch and swipe*/

	monitor_data->max_jumping_times = 0;

	clear_delta_data(monitor_data->jumping_points_count_array, monitor_data->rx_num / 2, monitor_data->tx_num / 2);
	clear_delta_data(monitor_data->jumping_point_delta_data, monitor_data->tx_num, monitor_data->rx_num);
	clear_delta_data(monitor_data->stuck_points_count_array, monitor_data->rx_num / 2, monitor_data->tx_num / 2);
	clear_delta_data(monitor_data->lanscape_stuck_points_count_array, monitor_data->rx_num / 2, monitor_data->tx_num / 2);
	clear_delta_data(monitor_data->stuck_point_delta_data, monitor_data->tx_num, monitor_data->rx_num);
	clear_delta_data(monitor_data->broken_swipes_count_array, monitor_data->rx_num / 2, monitor_data->tx_num / 2);

	monitor_data->long_swipes.count = 0;

	monitor_data->smooth_level_chosen = 0;
	monitor_data->sensitive_level_chosen = 0;

	clear_value_count_list(&monitor_data->gesture_values_list);
	clear_value_count_list(&monitor_data->invalid_gesture_values_list);

	/*fingerprint area rate*/
	clear_value_count_list(&monitor_data->fp_area_rate_list);

	/*face detect*/
	clear_value_count_list(&monitor_data->fd_values_list);

	/*auto test*/
	monitor_data->auto_test_failed_times = 0;
	monitor_data->auto_test_total_times = 0;
	monitor_data->blackscreen_test_failed_times = 0;
	monitor_data->blackscreen_test_total_times = 0;

	monitor_data->avdd = VOLTAGE_STATE_DEFAULT;
	monitor_data->vddi = VOLTAGE_STATE_DEFAULT;

	TPD_INFO("Clear health info Finish!\n");

	return 0;
}

int tp_healthinfo_init(struct device *dev, void *tp_monitor_data)
{
	int ret = 0;
	int temp_array[2] = {0};
	struct monitor_data *monitor_data = (struct monitor_data *)tp_monitor_data;

	if (!monitor_data) {
		TPD_INFO("monitor_data is NULL.\n");
		return -1;
	}

	ret = of_property_read_u32(dev->of_node, "touchpanel,max-num-support",
				   &monitor_data->max_finger_support);

	if (ret) {
		TPD_INFO("monitor_data->max_finger_support not specified\n");
		monitor_data->max_finger_support = 10;
	}

	ret = of_property_read_u32_array(dev->of_node, "touchpanel,tx-rx-num",
					 temp_array, 2);

	if (ret) {
		TPD_INFO("tx-rx-num not set\n");
		monitor_data->tx_num = TX_NUM;
		monitor_data->rx_num = RX_NUM;

	} else {
		monitor_data->tx_num =  temp_array[0];
		monitor_data->rx_num =  temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, "touchpanel,panel-coords",
					 temp_array, 2);

	if (ret) {
		monitor_data->max_x = 1080;
		monitor_data->max_y = 2340;
		TPD_INFO("panel coords using default.\n");

	} else {
		monitor_data->max_x = temp_array[0];
		monitor_data->max_y = temp_array[1];
	}
	monitor_data->long_swipe_judge_distance = int_sqrt(monitor_data->max_x *
				monitor_data->max_x + monitor_data->max_y * monitor_data->max_y) /
				LONG_SWIPE_JUDGE_RATIO;
	monitor_data->swipe_broken_judge_distance = int_sqrt(monitor_data->max_x *
				monitor_data->max_x + monitor_data->max_y * monitor_data->max_y) /
				SWIPE_BROKEN_JUDGE_RATIO;
	monitor_data->jumping_point_judge_distance = int_sqrt(monitor_data->max_x *
				monitor_data->max_x + monitor_data->max_y * monitor_data->max_y) /
				JUMPING_POINT_JUDGE_RATIO;
	TPD_INFO("long_swipe_judge_distance=%d swipe_broken_judge_distance=%d "
				"jumping_point_judge_distance=%d\n",
				monitor_data->long_swipe_judge_distance,
				monitor_data->swipe_broken_judge_distance,
				monitor_data->jumping_point_judge_distance);

	ret = of_property_read_u32_array(dev->of_node, "touchpanel,report-rate",
					 temp_array, 2);

	if (ret) {
		monitor_data->report_rate = 120;
		monitor_data->report_rate_in_game = 120;
		TPD_INFO("report rate using default.\n");

	} else {
		monitor_data->report_rate = temp_array[0];
		monitor_data->report_rate_in_game = temp_array[1];
		TPD_INFO("report rate %d-%d.\n", monitor_data->report_rate,
					monitor_data->report_rate_in_game);
	}

	ret = of_property_read_string(dev->of_node, "chip-name", &monitor_data->tp_ic);
	if (ret) {
		TPD_INFO("failed to get tp ic\n");
	}

	monitor_data->fw_version = tp_kzalloc(MAX_DEVICE_VERSION_LENGTH, GFP_KERNEL);

	if (!monitor_data->fw_version) {
		TPD_INFO("tp_kzalloc fw_version failed.\n");
		ret = -1;
		goto err;
	}

	monitor_data->jumping_point_delta_data = tp_kzalloc(sizeof(
				int32_t) * monitor_data->tx_num * monitor_data->rx_num, GFP_KERNEL);

	if (!monitor_data->jumping_point_delta_data) {
		TPD_INFO("tp_kzalloc jumping_point_delta_data failed.\n");
		ret = -1;
		goto err;
	}

	monitor_data->stuck_point_delta_data = tp_kzalloc(sizeof(
			int32_t) * monitor_data->tx_num * monitor_data->rx_num, GFP_KERNEL);

	if (!monitor_data->stuck_point_delta_data) {
		TPD_INFO("tp_kzalloc stuck_point_delta_data failed.\n");
		ret = -1;
		goto err;
	}

/*
	monitor_data->click_count_array = tp_kzalloc(sizeof(int32_t) *
					  CLICK_COUNT_ARRAY_HEIGHT * CLICK_COUNT_ARRAY_WIDTH, GFP_KERNEL);

	if (!monitor_data->click_count_array) {
		TPD_INFO("tp_kzalloc click_count_array failed.\n");
		ret = -1;
		goto err;
	}
*/
	monitor_data->jumping_points_count_array = tp_kzalloc(sizeof(int32_t) *
				(monitor_data->tx_num / 2) * (monitor_data->rx_num / 2), GFP_KERNEL);
	if (!monitor_data->jumping_points_count_array) {
		TPD_INFO("tp_kzalloc jumping_points_count_array failed.\n");
		ret = -1;
		goto err;
	}

	monitor_data->stuck_points_count_array = tp_kzalloc(sizeof(int32_t) *
				(monitor_data->tx_num / 2) * (monitor_data->rx_num / 2), GFP_KERNEL);
	if (!monitor_data->stuck_points_count_array) {
		TPD_INFO("tp_kzalloc stuck_points_count_array failed.\n");
		ret = -1;
		goto err;
	}

	monitor_data->lanscape_stuck_points_count_array = tp_kzalloc(sizeof(int32_t) *
				(monitor_data->tx_num / 2) * (monitor_data->rx_num / 2), GFP_KERNEL);
	if (!monitor_data->lanscape_stuck_points_count_array) {
		TPD_INFO("tp_kzalloc lanscape_stuck_points_count_array failed.\n");
		ret = -1;
		goto err;
	}

	monitor_data->broken_swipes_count_array = tp_kzalloc(sizeof(int32_t) *
				(monitor_data->tx_num / 2) * (monitor_data->rx_num / 2), GFP_KERNEL);
	if (!monitor_data->broken_swipes_count_array) {
		TPD_INFO("tp_kzalloc broken_swipes_count_array failed.\n");
		ret = -1;
		goto err;
	}

	monitor_data->total_touch_time_in_game = tp_kzalloc(sizeof(u64) *
			(monitor_data->max_finger_support + 1), GFP_KERNEL);

	if (!monitor_data->total_touch_time_in_game) {
		TPD_INFO("tp_kzalloc total_touch_time_in_game failed.\n");
		ret = -1;
		goto err;
	}

	/*values list init*/
	INIT_LIST_HEAD(&monitor_data->gesture_values_list);
	INIT_LIST_HEAD(&monitor_data->invalid_gesture_values_list);
	INIT_LIST_HEAD(&monitor_data->fp_area_rate_list);
	INIT_LIST_HEAD(&monitor_data->fd_values_list);
	INIT_LIST_HEAD(&monitor_data->health_report_list);
	INIT_LIST_HEAD(&monitor_data->bus_errs_list);
	INIT_LIST_HEAD(&monitor_data->bus_errs_buff_list);
	INIT_LIST_HEAD(&monitor_data->alloc_err_funcs_list);
	INIT_LIST_HEAD(&monitor_data->fw_update_result_list);

	monitor_data->avdd = VOLTAGE_STATE_DEFAULT;
	monitor_data->vddi = VOLTAGE_STATE_DEFAULT;

	return 0;
err:
	tp_kfree((void **)&monitor_data->fw_version);
	tp_kfree((void **)&monitor_data->jumping_point_delta_data);
	tp_kfree((void **)&monitor_data->stuck_point_delta_data);
	/*tp_kfree((void **)&monitor_data->click_count_array);*/
	tp_kfree((void **)&monitor_data->jumping_points_count_array);
	tp_kfree((void **)&monitor_data->stuck_points_count_array);
	tp_kfree((void **)&monitor_data->lanscape_stuck_points_count_array);
	tp_kfree((void **)&monitor_data->broken_swipes_count_array);
	tp_kfree((void **)&monitor_data->total_touch_time_in_game);

	return ret;
}
