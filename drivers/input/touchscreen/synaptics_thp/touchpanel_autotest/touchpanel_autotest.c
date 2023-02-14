// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include "../syna_tcm2.h"
#include "touchpanel_autotest.h"
#include "../touch_comon_api/touch_comon_api.h"
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/proc_fs.h>

/*******Start of LOG TAG Declear**********************************/
#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "touchpanel_autotest"
#else
#define TPD_DEVICE "touchpanel_autotest"
#endif
/*******End of LOG TAG Declear***********************************/
/**
 * tp_test_write - instead of vfs_write,save test result to memory
 * @data_start: pointer to memory buffer
 * @max_count: max length for memory buffer
 * @buf: new buffer
 * @count: count of new buffer
 * @pos: pos of current length for memory buffer
 * we can using this function to get item offset form index item
 * Returning parameter number(success) or negative errno(failed)
 */
ssize_t tp_test_write(void *data_start, size_t max_count,
		      const char *buf, size_t count, ssize_t *pos)
{
	ssize_t ret = 0;
	char *p = NULL;

	if (!data_start) {
		return -1;
	}

	if (!buf) {
		return -1;
	}

	if (*pos >= max_count) {
		TPD_INFO("%s: pos:%ld is out of memory\n", *pos, __func__);
		return -1;
	}

	p = (char *)data_start + *pos;

	memcpy(p, buf, count);
	*pos += count;

	return ret;
}

/*EXPORT_SYMBOL(tp_test_write);*/

/**
 * search_for_item_offset - get each item offset form test limit fw
 * @fw: pointer to fw
 * @item_cnt: max item number
 * @item_index: item index
 * we can using this function to get item offset form index item
 * Returning parameter number(success) or negative errno(failed)
 */
uint32_t search_for_item_offset(const struct firmware *fw, int item_cnt,
				uint8_t item_index)
{
	int i = 0;
	uint32_t item_offset = 0;
	struct auto_test_item_header *item_header = NULL;
	uint32_t *p_item_offset = (uint32_t *)(fw->data + 16);

	/*check the matched item offset*/
	for (i = 0; i < item_cnt; i++) {
		item_header = (struct auto_test_item_header *)(fw->data + p_item_offset[i]);

		if (item_header->item_bit == item_index) {
			item_offset = p_item_offset[i];
		}
	}

	return item_offset;
}

/**
 * get_info_for_item - get all infomation from item
 * @fw: pointer to fw
 * @item_index: item index
 * we can using this function to get infomation form index item
 * Returning pointer to test_item_info buffer
 */
struct test_item_info *get_test_item_info(const struct firmware *fw,
		uint8_t item_index)
{
	uint32_t item_offset = 0;
	int i = 0;
	uint32_t item_cnt = 0;
	struct auto_test_item_header *item_header = NULL;
	struct auto_test_header *test_header = NULL;
	int32_t *p_buffer = NULL;

	/*result: test_item_info */
	struct test_item_info *p = NULL;

	p = tp_kzalloc(sizeof(*p), GFP_KERNEL);

	if (!p) {
		return NULL;
	}

	/*step1: check item index is support or not*/
	test_header = (struct auto_test_header *)fw->data;

	if (!(test_header->test_item & (1 << item_index))) {
		TPD_INFO("item_index:%d is not support\n", item_index);
		goto ERROR;
	}

	/*step2: get max item*/
	for (i = 0; i < 8 * sizeof(test_header->test_item); i++) {
		if ((test_header->test_item >> i) & 0x01) {
			item_cnt++;
		}
	}

	/*step3: find item_index offset from the limit img*/
	item_offset = search_for_item_offset(fw, item_cnt, item_index);

	if (item_offset == 0) {
		TPD_INFO("search for item limit offset failed\n");
		goto ERROR;
	}

	/*get item_offset*/
	p->item_offset = item_offset;

	/*step4: check the item magic is support or not*/
	item_header = (struct auto_test_item_header *)(fw->data + item_offset);

	if (item_header->item_magic != Limit_ItemMagic && item_header->item_magic != Limit_ItemMagic_V2) {
		TPD_INFO("test item: %d magic number(%4x) is wrong\n", item_index,
			 item_header->item_magic);
		goto ERROR;
	}

	/*get item_header*/
	p->item_magic = item_header->item_magic;
	p->item_size = item_header->item_size;
	p->item_bit = item_header->item_bit;
	p->item_limit_type = item_header->item_limit_type;
	p->top_limit_offset = item_header->top_limit_offset;
	p->floor_limit_offset = item_header->floor_limit_offset;

	/*step5: get the parameter from the limit img*/
	if (item_header->para_num == 0) {
		TPD_INFO("item: %d has %d no parameter\n", item_index, item_header->para_num);
		goto ERROR;

	} else {
		p_buffer = (int32_t *)(fw->data + item_offset + sizeof(struct
				       auto_test_item_header));

		for (i = 0; i < item_header->para_num; i++) {
			TPD_INFO("item: %d has parameter:%d\n", item_index, p_buffer[i]);
		}
	}

	/*get item para number and para buffer*/
	p->para_num = item_header->para_num;
	p->p_buffer = p_buffer;

	return p;

ERROR:
	tp_kfree((void **)&p);
	return NULL;
}

/*
static int tp_test_limit_switch(struct syna_tcm *tcm)
{
	char *p_node = NULL;
	char *postfix = "_AGING";
	uint8_t copy_len = 0;

	if (!tcm) {
		return -1;
	}

	tcm->panel_data.aging_test_limit_name = tp_devm_kzalloc(tcm->device, MAX_FW_NAME_LENGTH, GFP_KERNEL);
	if (tcm->panel_data.aging_test_limit_name == NULL) {
		TP_INFO(tcm->tp_index, "[TP]panel_data.test_limit_name kzalloc error\n");
		return -1;
	}

	**change **.img to **_AGING.img*
	p_node	= strstr(LIMIT_IMAGE_NAME, ".");
	if (p_node == NULL) {
		TP_INFO(tcm->tp_index, "p_node strstr error!\n");
		goto EXIT;
	}

	copy_len = p_node - LIMIT_IMAGE_NAME;
	memcpy(tcm->panel_data.aging_test_limit_name, LIMIT_IMAGE_NAME, copy_len);
	strlcat(tcm->panel_data.aging_test_limit_name, postfix, MAX_LIMIT_DATA_LENGTH_COM);
	strlcat(tcm->panel_data.aging_test_limit_name, p_node, MAX_LIMIT_DATA_LENGTH_COM);
	TP_INFO(tcm->tp_index, "aging_test_limit_name is %s\n", tcm->panel_data.aging_test_limit_name);
	return 0;

EXIT:
	tp_devm_kfree(tcm->device, (void **)&tcm->panel_data.aging_test_limit_name, MAX_FW_NAME_LENGTH);

	return -1;
}
*/
static int request_test_limit(const struct firmware **fw, char *test_limit_name, struct device *device)
{
	int ret = 0;
	int retry = 5;

	do {
		ret = request_firmware(fw, test_limit_name, device);

		if (!ret) {
			break;
		}
	} while ((ret < 0) && (--retry > 0));

	TPD_INFO("retry times %d\n", 5 - retry);

	if (fw) {
		TPD_INFO("%s:fw is not null\n", __func__);
	} else {
		TPD_INFO("%s:fw is NULL!!\n", __func__);
	}

	return ret;
}

static int request_real_test_limit(struct syna_tcm *tcm,
	const struct firmware **fw, char *test_limit_name, struct device *device)
{
	int ret = 0;

	/*if (AGING_TEST_MODE == tcm->aging_mode) {
		ret = tp_test_limit_switch(tcm);
		if (ret < 0) {
			return ret;
		}
		ret = request_test_limit(fw, tcm->panel_data.aging_test_limit_name, device);
		if (ret < 0) {
			ret = request_test_limit(fw, LIMIT_IMAGE_NAME, device);
		}
		tp_devm_kfree(tcm->device, (void **)&tcm->panel_data.aging_test_limit_name, MAX_FW_NAME_LENGTH);
	} else {*/
	ret = request_test_limit(fw, test_limit_name, device);
	/*}*/

	return ret;
}
/*
void tp_limit_read(struct seq_file *s, struct syna_tcm *tcm)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	struct auto_test_header *ph = NULL;
	int m = 0, i = 0, j = 0, item_cnt = 0;
	struct auto_test_item_header *item_head = NULL;
	uint32_t *p_item_offset = NULL;
	int32_t *p_data32 = NULL;

	ret =  request_real_test_limit(tcm, &fw, LIMIT_IMAGE_NAME, tcm->device);

	if (ret < 0) {
		TPD_INFO("Request firmware failed - %s (%d)\n", LIMIT_IMAGE_NAME,
			 ret);
		seq_printf(s, "Request failed, Check the path %s\n",
			   LIMIT_IMAGE_NAME);
		return;
	}

	ph = (struct auto_test_header *)(fw->data);
	p_item_offset = (uint32_t *)(fw->data + 16);

	if ((ph->magic1 != Limit_MagicNum1) || (ph->magic2 != Limit_MagicNum2)) {
		TPD_INFO("limit image is not generated by oplus\n");
		seq_printf(s, "limit image is not generated by oplus\n");
		return;
	}

	TPD_INFO("magic1:%x,magic2:%x,test_item:%llu\n", ph->magic1, ph->magic2,
		 ph->test_item);

	for (i = 0; i < 8 * sizeof(ph->test_item); i++) {
		if ((ph->test_item >> i) & 0x01) {
			item_cnt++;
		}
	}

	TPD_INFO("item_cnt :%d\n", item_cnt);

	if (!item_cnt) {
		TPD_INFO("limit image has no test item\n");
		seq_printf(s, "limit image has no test item\n");
	}

	for (m = 0; m < item_cnt; m++) {
		item_head = (struct auto_test_item_header *)(fw->data + p_item_offset[m]);
		TPD_INFO("item %d[size %d, limit type %d,top limit %d, floor limit %d para num %d] :\n",
			 item_head->item_bit, item_head->item_size, item_head->item_limit_type,
			 item_head->top_limit_offset, item_head->floor_limit_offset,
			 item_head->para_num);

		if (item_head->item_magic != Limit_ItemMagic && item_head->item_magic != Limit_ItemMagic_V2) {
			seq_printf(s, "item: %d limit data has some problem\n", item_head->item_bit);
			continue;
		}

		seq_printf(s, "item[%d]:", m);

		if (item_head->item_limit_type == LIMIT_TYPE_NO_DATA) {
			seq_printf(s, "no limit data\n");

		} else if (item_head->item_limit_type == LIMIT_TYPE_CERTAIN_DATA) {
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);
			seq_printf(s, "top limit data: %d\n", *p_data32);
			p_data32 = (int32_t *)(fw->data + item_head->floor_limit_offset);
			seq_printf(s, "floor limit data: %d\n", *p_data32);

		} else if (item_head->item_limit_type == LIMIT_TYPE_EACH_NODE_DATA) {
			seq_printf(s, "raw top data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);

			for (i = 0; i < (TX_NUM * RX_NUM); i++) {
				if (i % RX_NUM == 0) {
					seq_printf(s, "\n[%2d] ", (i / RX_NUM));
				}

				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

			seq_printf(s, "\n\ngap raw top data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset + 4 *
					       TX_NUM * RX_NUM);

			for (i = 0; i < (TX_NUM * RX_NUM); i++) {
				if (i % RX_NUM == 0) {
					seq_printf(s, "\n[%2d] ", (i / RX_NUM));
				}

				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

			seq_printf(s, "\n\nraw floor data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->floor_limit_offset);

			for (i = 0; i < (TX_NUM * RX_NUM); i++) {
				if (i % RX_NUM == 0) {
					seq_printf(s, "\n[%2d] ", (i / RX_NUM));
				}

				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

			seq_printf(s, "\n\ngap raw floor data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->floor_limit_offset + 4 *
					       TX_NUM * RX_NUM);

			for (i = 0; i < (TX_NUM * RX_NUM); i++) {
				if (i % RX_NUM == 0) {
					seq_printf(s, "\n[%2d] ", (i / RX_NUM));
				}

				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

		} else if (item_head->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
			seq_printf(s, "raw top data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);

			for (i = 0; i < (TX_NUM * RX_NUM); i++) {
				if (i % RX_NUM == 0) {
					seq_printf(s, "\n[%2d] ", (i / RX_NUM));
				}

				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

			seq_printf(s, "\n\nraw floor data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->floor_limit_offset);

			for (i = 0; i < (TX_NUM * RX_NUM); i++) {
				if (i % RX_NUM == 0) {
					seq_printf(s, "\n[%2d] ", (i / RX_NUM));
				}

				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

		} else if (item_head->item_limit_type == LIMIT_TYPE_SLEF_TX_RX_DATA) {
			seq_printf(s, "tx + rx data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);

			for (i = 0; i < (TX_NUM + RX_NUM); i++) {
				if (i % RX_NUM == 0) {
					seq_printf(s, "\n[%2d] ", (i / RX_NUM));
				}

				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

		} else if (item_head->item_limit_type == LIMIT_TYPE_SLEF_TX_RX_DATA_DOUBLE) {
			seq_printf(s, "tx floor data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset);
			seq_printf(s, "\n[ 0] ");

			for (i = 0; i < TX_NUM; i++) {
				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

			seq_printf(s, "\n\ntx top data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset + 4 *
					       TX_NUM);
			seq_printf(s, "\n[ 1] ");

			for (i = 0; i < TX_NUM; i++) {
				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

			seq_printf(s, "\n\nrx floor data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset + 2 * 4 *
					       TX_NUM);
			seq_printf(s, "\n[ 2] ");

			for (i = 0; i < RX_NUM; i++) {
				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}

			seq_printf(s, "\n\nrx top data: \n");
			p_data32 = (int32_t *)(fw->data + item_head->top_limit_offset + 2 * 4 *
					       TX_NUM + 4 * RX_NUM);
			seq_printf(s, "\n[ 3] ");

			for (i = 0; i <  RX_NUM; i++) {
				seq_printf(s, "%4d, ", p_data32[i]);
				TPD_DEBUG("%d, ", p_data32[i]);
			}
		}

		p_data32 = (int32_t *)(fw->data + p_item_offset[m] + sizeof(
					       struct auto_test_item_header));

		if (item_head->para_num) {
			seq_printf(s, "\n\nparameter:");

			for (j = 0; j < item_head->para_num; j++) {
				seq_printf(s, "%d, ", p_data32[j]);
			}

			seq_printf(s, "\n");
		}

		seq_printf(s, "\n");
	}

	release_firmware(fw);
}
EXPORT_SYMBOL(tp_limit_read);
*/
int tp_auto_test(struct seq_file *s, void *v)
{
	struct syna_tcm *tcm = s->private;
	int ret = 0;
	int error_count = 0;

	if (!tcm) {
		return 0;
	}

	if (!tcm->engineer_ops) {
		seq_printf(s, "Not support auto-test proc node\n");
		return 0;
	}

	if (!tcm->engineer_ops->auto_test) {
		seq_printf(s, "Not support auto-test proc node\n");
		return 0;
	}

	/*step1:disable_irq && get mutex locked*/
	/*if (tcm->int_mode == BANNABLE) {
		disable_irq_nosync(tcm->irq);
	}*/

	mutex_lock(&tcm->mutex);

	/*step2:malloc space to store test data*/
	/*set buffer pos to first position every time*/
	tcm->com_test_data.result_cur_len = 0;

	if (!tcm->com_test_data.result_flag) {
		tcm->com_test_data.result_max_len = PAGE_SIZE * 15;/*60k*/
		tcm->com_test_data.result_data = kvzalloc(tcm->com_test_data.result_max_len,
						GFP_KERNEL);

		if (!tcm->com_test_data.result_data) {
			TP_INFO(tcm->tp_index, "%s kvzalloc failed\n", __func__);
			tcm->com_test_data.result_flag = 0;

		} else {
			tcm->com_test_data.result_flag = 1;
			TP_INFO(tcm->tp_index, "%s kvzalloc data ok\n", __func__);
		}
	}

	/*step3:request test limit data from userspace*/
	ret =  request_real_test_limit(tcm, &tcm->com_test_data.limit_fw,
		LIMIT_IMAGE_NAME, tcm->device);

	if (ret < 0) {
		TP_INFO(tcm->tp_index, "Request firmware failed - %s (%d)\n",
			LIMIT_IMAGE_NAME, ret);
		seq_printf(s, "No limit IMG\n");
		mutex_unlock(&tcm->mutex);

		/*if (tcm->int_mode == BANNABLE) {
			tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		}*/

		return 0;
	}

	if (tcm->com_test_data.limit_fw) {
		TPD_INFO("%s:fw is not null\n", __func__);
	} else {
		TPD_INFO("%s:fw is NULL!!\n", __func__);
	}

	tcm->in_test_process = true;

	error_count = tcm->engineer_ops->auto_test(s, tcm->device);

	/*step5: release test limit firmware*/

	release_firmware(tcm->com_test_data.limit_fw);

	/*step6: return to normal mode*/
	/*operate_mode_switch(syna_tcm);*/

	/*step7: unlock the mutex && enable irq trigger*/
	mutex_unlock(&tcm->mutex);

	/*if (tcm->int_mode == BANNABLE) {
		tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
	}*/

	tcm->in_test_process = false;
	return 0;
}

int tp_auto_test_result(struct seq_file *s, void *v)
{
	struct syna_tcm *tcm = s->private;

	TP_INFO(tcm->tp_index, "%s:s->size:%d,s->count:%d\n", __func__, s->size,
		s->count);
	mutex_lock(&tcm->mutex);

	/*the result data is big than one page, so do twice.*/
	if (s->size <= (tcm->com_test_data.result_cur_len)) {
		s->count = s->size;
		mutex_unlock(&tcm->mutex);
		return 0;
	}

	if (tcm->com_test_data.result_flag) {
		if (tcm->com_test_data.result_data) {
			if (tcm->com_test_data.result_cur_len) {
				seq_write(s, tcm->com_test_data.result_data, tcm->com_test_data.result_cur_len);
			}

			kvfree(tcm->com_test_data.result_data);
			TP_INFO(tcm->tp_index, "%s:free data ok\n", __func__);
			tcm->com_test_data.result_flag = 0;
		}

	} else {
		TP_INFO(tcm->tp_index, "%s:it must do auto test frist\n", __func__);
		seq_printf(s, "it must do auto test frist\n");
	}

	mutex_unlock(&tcm->mutex);

	return 0;
}
