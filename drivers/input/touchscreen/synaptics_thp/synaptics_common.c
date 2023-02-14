// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include "synaptics_common.h"
#include "syna_tcm2.h"
#include "tcm/synaptics_touchcom_func_base.h"
#include <linux/crc32.h>
#include <linux/module.h>
#include <linux/version.h>
/*******Part0:LOG TAG Declear********************/
#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "synaptics_common"
#else
#define TPD_DEVICE "synaptics_common"
#endif
/*******Part1:Call Back Function implement*******/

unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
	       (unsigned int)ptr[1] * 0x100 +
	       (unsigned int)ptr[2] * 0x10000 +
	       (unsigned int)ptr[3] * 0x1000000;
}

static void store_to_file(void *fp, size_t max_count,
			  size_t *pos, char *format, ...)
{
	va_list args;
	char buf[64] = {0};

	va_start(args, format);
	vsnprintf(buf, 64, format, args);
	va_end(args);

	if (!IS_ERR_OR_NULL(fp)) {
		tp_test_write(fp, max_count, buf, strlen(buf), pos);
	}
}
/*************************************auto test Funtion**************************************/
static int syna_auto_test_irq(struct syna_tcm *tcm,
			      struct syna_auto_test_operations *syna_test_ops,
			      struct auto_testdata *syna_testdata, bool false)
{
	int ret = 0;
	int eint_status, eint_count = 0, read_gpio_num = 0;

	if (syna_test_ops->syna_auto_test_enable_irq) {
		ret = syna_test_ops->syna_auto_test_enable_irq(tcm, false);

		if (ret) {
			TPD_INFO("%s: syna_auto_test_enable_irq failed !\n", __func__);
		}
	}

	eint_count = 0;
	read_gpio_num = 10;

	while (read_gpio_num--) {
		msleep(5);
		eint_status = gpio_get_value(syna_testdata->irq_gpio);

		if (eint_status == 1) {
			eint_count--;

		} else {
			eint_count++;
		}

		TPD_INFO("%s: eint_count = %d  eint_status = %d\n", __func__, eint_count,
			 eint_status);
	}

	return eint_count;
}

int syna_trx_short_test(struct seq_file *s, void *chip_data,
			       struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	uint8_t u_data8 = 0;
	int i = 0, j = 0, ret = 0;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	unsigned int checked_bits = 0, total_bits = 0;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	total_bits = syna_testdata->tx_num + syna_testdata->rx_num;

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_TRX_SHORT);*/
	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_TRX_SHORT,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run trx short test failed.\n");

		if (!error_count) {
			seq_printf(s, "run trx short test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "trx_short:\n");

	for (i = 0; i < test_data.data_length;) {
		u_data8 = buf[i];
		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "0x%02x, ", u_data8);

		for (j = 0; j < 8; j++) {
			if (1 == (u_data8 & (1 << j))) {
				TPD_INFO("trx short test failed at %d bits.\n", checked_bits + 1);

				if (!error_count) {
					seq_printf(s, "trx short test failed at %d bits.\n", checked_bits + 1);
				}

				error_count++;
			}

			checked_bits++;

			if (checked_bits >= total_bits) {
				goto full_out;
			}
		}

		i += 1;
	}

full_out:
	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_trx_open_test(struct seq_file *s, void *chip_data,
			      struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	uint8_t u_data8 = 0;
	int i = 0, j = 0, ret = 0;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	unsigned int checked_bits = 0, total_bits = 0;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	total_bits = syna_testdata->tx_num + syna_testdata->rx_num;

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_TRX_OPEN);*/
	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_TRX_OPEN,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run trx open test failed.\n");

		if (!error_count) {
			seq_printf(s, "run trx open test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "tx_tx_open:\n");

	for (i = 0; i < test_data.data_length;) {
		u_data8 = buf[i];
		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "0x%02x, ", u_data8);

		for (j = 0; j < 8; j++) {
			if (0 == (u_data8 & (1 << j))) {
				TPD_INFO("trx open test failed at %d bits.\n", checked_bits + 1);

				if (!error_count) {
					seq_printf(s, "trx open test failed at %d bits.\n", checked_bits + 1);
				}

				error_count++;
			}

			checked_bits++;

			if (checked_bits >= total_bits) {
				goto full_out;
			}
		}

		i += 1;
	}

full_out:
	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_trx_gndshort_test(struct seq_file *s, void *chip_data,
				  struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	uint8_t u_data8 = 0;
	int i = 0, j = 0, ret = 0;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	unsigned int checked_bits = 0, total_bits = 0;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	total_bits = syna_testdata->tx_num + syna_testdata->rx_num;

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_TRXGND_SHORT);*/
	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_TRXGND_SHORT,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run trx gndshort test failed.\n");

		if (!error_count) {
			seq_printf(s, "run trx gndshort test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "tx_tx_gndshort:\n");

	for (i = 0; i < test_data.data_length;) {
		u_data8 = buf[i];
		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "0x%02x, ", u_data8);

		for (j = 0; j < 8; j++) {
			if (0 == (u_data8 & (1 << j))) {
				TPD_INFO("trx gndshort test failed at %d bits.\n", checked_bits + 1);

				if (!error_count) {
					seq_printf(s, "trx gndshort test failed at %d bits.\n", checked_bits + 1);
				}

				error_count++;
			}

			checked_bits++;

			if (checked_bits >= total_bits) {
				goto full_out;
			}
		}

		i += 1;
	}

full_out:
	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp,  syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_full_rawcap_test(struct seq_file *s, void *chip_data,
				 struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	uint16_t u_data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	} else {
		TPD_INFO("full rawcap test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "full rawcap test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_FULLRAW_CAP);*/
	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_FULLRAW_CAP,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run full rawcap test failed.\n");

		if (!error_count) {
			seq_printf(s, "run full rawcap test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp,  syna_testdata->length,
		      syna_testdata->pos, "full_rawcap:");

	for (i = 0; i < test_data.data_length;) {
		index = i / byte_cnt;
		u_data16 = (buf[i] | (buf[i + 1] << 8));

		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				      syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", u_data16);

		if ((u_data16 < p_mutual_n[index]) || (u_data16 > p_mutual_p[index])) {
			TPD_INFO("full rawcap test failed at node[%d]=%d [%d %d].\n", index, u_data16,
				 p_mutual_n[index], p_mutual_p[index]);

			if (!error_count) {
				seq_printf(s, "full rawcap test failed at node[%d]=%d [%d %d].\n", index,
					   u_data16, p_mutual_n[index], p_mutual_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_delta_noise_test(struct seq_file *s, void *chip_data,
				 struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data +
			p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	} else {
		TPD_INFO("delta noise test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "delta noise test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	TPD_INFO("%s start.\n", __func__);
	ret = syna_tcm_run_production_test(tcm->tcm_dev,
            TYPE_DELTA_NOISE,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run delta noise rawcap test failed.\n");

		if (!error_count) {
			seq_printf(s, "run delta noise test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "delta_noise:");

	for (i = 0; i < test_data.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));

		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				      syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			TPD_INFO("delta noise test failed at node[%d]=%d [%d %d].\n", index, data16,
				 p_mutual_n[index], p_mutual_p[index]);

			if (!error_count) {
				seq_printf(s, "delta noise test failed at node[%d]=%d [%d %d].\n", index,
					   data16, p_mutual_n[index], p_mutual_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_hybrid_rawcap_test(struct seq_file *s, void *chip_data,
				   struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int32_t data32 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 4;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_hybridcap_p = NULL, *p_hybridcap_n = NULL;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_SLEF_TX_RX_DATA) {
		p_hybridcap_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_hybridcap_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);
	} else {
		TPD_INFO("hybrid_rawcap test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "hybrid_rawcap test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_HYBRIDRAW_CAP);*/
	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_HYBRIDRAW_CAP,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run hybrid rawcap test failed.\n");

		if (!error_count) {
			seq_printf(s, "run hybrid rawcap test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "hybrid_rawcap:\n");

	for (i = 0; i < test_data.data_length;) {
		index = i / byte_cnt;
		data32 = (buf[i] | (buf[i + 1] << 8) | (buf[i + 2] << 16) | (buf[i + 3] << 24));
		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%08d, ", data32);

		if ((data32 < p_hybridcap_n[index]) || (data32 > p_hybridcap_p[index])) {
			TPD_INFO("hybrid rawcap test failed at node[%d]=%d [%d %d].\n", index, data32,
				 p_hybridcap_n[index], p_hybridcap_p[index]);

			if (!error_count) {
				seq_printf(s, "hybrid rawcap test failed at node[%d]=%d [%d %d].\n", index,
					   data32, p_hybridcap_n[index], p_hybridcap_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_rawcap_test(struct seq_file *s, void *chip_data,
			    struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	} else {
		TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "raw cap test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_RAW_CAP);*/
	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_RAW_CAP,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run raw cap test failed.\n");

		if (!error_count) {
			seq_printf(s, "run raw cap test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "raw_cap:");

	for (i = 0; i < test_data.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));

		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				      syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
				 p_mutual_n[index], p_mutual_p[index]);

			if (!error_count) {
				seq_printf(s, "rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
					   p_mutual_n[index], p_mutual_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_trex_shortcustom_test(struct seq_file *s, void *chip_data,
				      struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	uint16_t u_data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_tx_p = NULL, *p_tx_n = NULL;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_SLEF_TX_RX_DATA) {
		p_tx_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_tx_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	} else {
		TPD_INFO("trex short custom test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "trex short custom test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_TREXSHORT_CUSTOM);*/
	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_TREXSHORT_CUSTOM,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run trex short custom test failed.\n");

		if (!error_count) {
			seq_printf(s, "run trex short custom test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "trex_shorcustom:\n");

	for (i = 0; i < test_data.data_length;) {
		index = i / byte_cnt;
		u_data16 = (buf[i] | (buf[i + 1] << 8));
		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", u_data16);

		if ((u_data16 < p_tx_n[index]) || (u_data16 > p_tx_p[index])) {
			TPD_INFO("trex_shorcustom test failed at node[%d]=%d [%d %d].\n", index,
				 u_data16, p_tx_n[index], p_tx_p[index]);

			if (!error_count) {
				seq_printf(s, "trex_shorcustom test failed at node[%d]=%d [%d %d].\n", index,
					   u_data16, p_tx_n[index], p_tx_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_hybrid_diffcbc_test(struct seq_file *s, void *chip_data,
				    struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	uint16_t u_data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_selfdata_p = NULL, *p_selfdata_n = NULL;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data +
			p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_SLEF_TX_RX_DATA) {
		p_selfdata_p = (int32_t *)(syna_testdata->fw->data +
					   item_header->top_limit_offset);
		p_selfdata_n = (int32_t *)(syna_testdata->fw->data +
					   item_header->floor_limit_offset);

	} else {
		TPD_INFO("hybrid diffcbc test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "hybrid diffcbc test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_HYBRIDABS_DIFF_CBC);*/

	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_HYBRIDABS_DIFF_CBC,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run hybrid diffcbc test failed.\n");

		if (!error_count) {
			seq_printf(s, "run hybrid diffcbc test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "hybrid_diffwithcbc:\n");

	for (i = 0; i < test_data.data_length;) {
		index = i / byte_cnt;
		u_data16 = (buf[i] | (buf[i + 1] << 8));
		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", u_data16);

		if ((u_data16 < p_selfdata_n[index]) || (u_data16 > p_selfdata_p[index])) {
			TPD_INFO("hybrid diffcbc test failed at node[%d]=%d [%d %d].\n", index,
				 u_data16, p_selfdata_n[index], p_selfdata_p[index]);

			if (!error_count) {
				seq_printf(s, "hybrid diffcbc test failed at node[%d]=%d [%d %d].\n", index,
					   u_data16, p_selfdata_n[index], p_selfdata_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_hybrid_absnoise_test(struct seq_file *s, void *chip_data,
				     struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_selfdata_p = NULL, *p_selfdata_n = NULL;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data +
			p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_SLEF_TX_RX_DATA) {
		p_selfdata_p = (int32_t *)(syna_testdata->fw->data +
					   item_header->top_limit_offset);
		p_selfdata_n = (int32_t *)(syna_testdata->fw->data +
					   item_header->floor_limit_offset);

	} else {
		TPD_INFO("hybrid abs noise test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "hybrid abs noise test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	TPD_INFO("%s start.\n", __func__);

	syna_tcm_reset(tcm->tcm_dev); /* before the test, need to reset the IC*/

	/*ret = testing_run_prod_test_item(tcm, TYPE_HYBRIDABS_NOSIE);*/

	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_HYBRIDABS_NOSIE,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run hybrid abs noise test failed.\n");

		if (!error_count) {
			seq_printf(s, "run hybrid abs noise test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "hybrid_absnoise:\n");

	for (i = 0; i < test_data.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));
		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_selfdata_n[index]) || (data16 > p_selfdata_p[index])) {
			TPD_INFO("hybrid abs noise test failed at node[%d]=%d [%d %d].\n", index,
				 data16, p_selfdata_n[index], p_selfdata_p[index]);

			if (!error_count) {
				seq_printf(s, "hybrid abs noise test failed at node[%d]=%d [%d %d].\n", index,
					   data16, p_selfdata_n[index], p_selfdata_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int syna_hybrid_rawcap_test_ad(struct seq_file *s, void *chip_data,
				   struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int32_t data32 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm *tcm = (struct syna_tcm *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_hybridcap_p = NULL, *p_hybridcap_n = NULL;
	unsigned char *buf = NULL;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_SLEF_TX_RX_DATA) {
		p_hybridcap_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_hybridcap_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);
	} else {
		TPD_INFO("hybrid_rawcap test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "hybrid_rawcap test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	TPD_INFO("%s start.\n", __func__);
	/*ret = testing_run_prod_test_item(tcm, TYPE_HYBRIDRAW_CAP_WITH_AD + 24);*/

	ret = syna_tcm_run_production_test(tcm->tcm_dev,
			TYPE_HYBRIDRAW_CAP_WITH_AD + 24,
			&test_data);

	if (ret < 0) {
		TPD_INFO("run hybrid rawcap test failed.\n");

		if (!error_count) {
			seq_printf(s, "run hybrid rawcap test failed.\n");
		}

		error_count++;
		return error_count;
	}

	syna_tcm_buf_lock(&test_data);
	buf = test_data.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_data.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "hybrid_rawcap:\n");

	for (i = 0; i < test_data.data_length;) {
		index = i / byte_cnt;
		data32 = (buf[i] | (buf[i + 1] << 8));
		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%08d, ", data32);

		if ((data32 < p_hybridcap_n[index]) || (data32 > p_hybridcap_p[index])) {
			TPD_INFO("hybrid rawcap test failed at node[%d]=%d [%d %d].\n", index, data32,
				 p_hybridcap_n[index], p_hybridcap_p[index]);

			if (!error_count) {
				seq_printf(s, "hybrid rawcap test failed at node[%d]=%d [%d %d].\n", index,
					   data32, p_hybridcap_n[index], p_hybridcap_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	syna_tcm_buf_unlock(&test_data);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

int synaptics_auto_test(struct seq_file *s,  struct device *dev)
{
	int ret = 0;
	int error_count = 0;
	int curr_pwr_state = 0;
	/*struct syna_tcm *tcm = dev_get_drvdata(dev);*/
	struct syna_tcm *tcm = s->private;

	/*for save result file buffer*/
	uint8_t  data_buf[64];

	/*for limit fw*/
	struct auto_test_header *test_head = NULL;
	/*for item limit data*/
	uint32_t *p_data32 = NULL;
	uint32_t item_cnt = 0;
	uint32_t i = 0;

	struct test_item_info *p_test_item_info = NULL;
	struct syna_auto_test_operations *syna_test_ops = NULL;
	struct com_test_data *com_test_data_p = NULL;

	struct auto_testdata syna_testdata = {
		.tx_num = 0,
		.rx_num = 0,
		.fp = NULL,
		.pos = NULL,
		.irq_gpio = -1,
		.key_tx = 0,
		.key_rx = 0,
		.tp_fw = 0,
		.fw = NULL,
		.test_item = 0,
	};
	TPD_INFO("%s  is called\n", __func__);

	com_test_data_p = &tcm->com_test_data;

	if (!tcm) {
		TPD_INFO("%s: tcm is null\n", __func__);
		return -1;
	}

	if (!com_test_data_p->limit_fw) {
		TPD_INFO("%s: limit_fw is null\n", __func__);
		return -1;
	}

	if (!com_test_data_p->chip_test_ops) {
		TPD_INFO("%s: chip_test_ops is null\n", __func__);
		return -1;
	}

	if (!com_test_data_p->limit_fw || !tcm || !com_test_data_p->chip_test_ops) {
		TPD_INFO("%s: data is null\n", __func__);
		return -1;
	}

	syna_test_ops = (struct syna_auto_test_operations *)
			com_test_data_p->chip_test_ops;

	/*decode the limit image*/
	test_head = (struct auto_test_header *)com_test_data_p->limit_fw->data;
	p_data32 = (uint32_t *)(com_test_data_p->limit_fw->data + 16);

	if ((test_head->magic1 != Limit_MagicNum1)
			|| (test_head->magic2 != Limit_MagicNum2)) {
		TPD_INFO("limit image is not generated by oplus\n");
		seq_printf(s, "limit image is not generated by oplus\n");
		return -1;
	}

	TPD_INFO("current test item: %llx\n", test_head->test_item);

	for (i = 0; i < 8 * sizeof(test_head->test_item); i++) {
		if ((test_head->test_item >> i) & 0x01) {
			item_cnt++;
		}
	}

	/*check limit support any item or not*/
	if (!item_cnt) {
		TPD_INFO("no any test item\n");
		error_count++;
		seq_printf(s, "no any test item\n");
		return -1;
	}

	/*init syna_testdata*/
	syna_testdata.fp        = tcm->com_test_data.result_data;
	syna_testdata.length    = tcm->com_test_data.result_max_len;
	syna_testdata.pos       = &tcm->com_test_data.result_cur_len;
	syna_testdata.tx_num    = TX_NUM;
	syna_testdata.rx_num    = RX_NUM;
	syna_testdata.irq_gpio  = tcm->hw_if->bdata_attn.irq_gpio;
	syna_testdata.key_tx    = TX_NUM;
	syna_testdata.key_rx    = RX_NUM;
	/*syna_testdata.tp_fw     = FW_IMAGE_NAME;*/
	syna_testdata.fw        =  tcm->com_test_data.limit_fw;
	syna_testdata.test_item = test_head->test_item;

	TPD_INFO("%s, : send display off signal\n", __func__);
	syna_send_signal(tcm, SIG_DISPLAY_OFF);

	ret = syna_dev_disable_hbp_mode(tcm);
	if (ret < 0) {
		LOGE("error :  Fail to disable hbp mode\n");
		error_count++;
		seq_printf(s, "fail to disable hbp mode\n");
		sprintf(data_buf, "fail to disable hbp mode\n");
		tp_test_write(syna_testdata.fp, syna_testdata.length, data_buf,
			      strlen(data_buf), syna_testdata.pos);
		ret = 0;
		goto END;
	}
	curr_pwr_state = tcm->pwr_state;
	tcm->pwr_state = LOW_PWR;
	syna_cdev_update_power_state_report_queue(tcm, true);

	TPD_INFO("%s, step 0: begin to check INT-GND short item\n", __func__);
	ret = syna_auto_test_irq(tcm, syna_test_ops, &syna_testdata, false);

	TPD_INFO("TP EINT PIN direct short! eint_count ret = %d\n", ret);

	if (ret == 10) {
		TPD_INFO("error :  TP EINT PIN direct short!\n");
		error_count++;
		seq_printf(s, "eint_status is low, TP EINT direct stort\n");
		sprintf(data_buf, "eint_status is low, TP EINT direct stort, \n");
		tp_test_write(syna_testdata.fp, syna_testdata.length, data_buf,
			      strlen(data_buf), syna_testdata.pos);
		ret = 0;
		goto END;
	}

	if (!syna_test_ops->syna_auto_test_preoperation) {
		TPD_INFO("not support syna_test_ops->syna_auto_test_preoperation callback\n");

	} else {
		syna_test_ops->syna_auto_test_preoperation(s, tcm, &syna_testdata,
				p_test_item_info);
	}

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST1);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST1);

	} else {
		ret = syna_test_ops->test1(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST2);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST2);

	} else {
		ret = syna_test_ops->test2(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST3);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST3);

	} else {
		ret = syna_test_ops->test3(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST4);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST4);

	} else {
		ret = syna_test_ops->test4(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST5);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST5);

	} else {
		ret = syna_test_ops->test5(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST6);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST6);

	} else {
		ret = syna_test_ops->test6(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST7);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST7);

	} else {
		ret = syna_test_ops->test7(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST8);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST8);

	} else {
		ret = syna_test_ops->test8(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST9);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST9);

	} else {
		ret = syna_test_ops->test9(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST10);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST10);

	} else {
		ret = syna_test_ops->test10(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}

	tp_kfree((void **)&p_test_item_info);

	p_test_item_info = get_test_item_info(syna_testdata.fw, TYPE_TEST11);

	if (!p_test_item_info) {
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_TEST11);

	} else {
		ret = syna_test_ops->test11(s, tcm, &syna_testdata, p_test_item_info);

		if (ret > 0) {
			TPD_INFO("synaptics_capacity_test failed! ret is %d\n", ret);
			error_count++;
			goto END_TEST;
		}
	}


END_TEST:

	if (!syna_test_ops->syna_auto_test_endoperation) {
		TPD_INFO("not support syna_test_ops->syna_auto_test_endoperation callback\n");

	} else {
		syna_test_ops->syna_auto_test_endoperation(s, tcm, &syna_testdata,
				p_test_item_info);
	}

	tcm->hw_if->ops_hw_reset(tcm->hw_if);

	msleep(100);

	TPD_INFO("%s, : send display on signal\n", __func__);

	tcm->pwr_state = curr_pwr_state;
	syna_send_signal(tcm, SIG_DISPLAY_ON);

END:

	seq_printf(s, "imageid = 0x%llx, deviceid = 0x%llx\n", syna_testdata.tp_fw,
		   syna_testdata.tp_fw);
	seq_printf(s, "%d error(s). %s\n", error_count,
		   error_count ? "" : "All test passed.");
	TPD_INFO(" TP auto test %d error(s). %s\n", error_count,
		 error_count ? "" : "All test passed.");
	TPD_INFO("\n\nstep5 reset and open irq complete\n");

	tp_kfree((void **)&p_test_item_info);
	return error_count;
}
/*EXPORT_SYMBOL(synaptics_auto_test);*/
/*************************************auto test Funtion**************************************/

MODULE_DESCRIPTION("Touchscreen Synaptics Common Interface");
MODULE_LICENSE("GPL");
