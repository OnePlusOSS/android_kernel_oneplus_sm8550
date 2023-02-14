/*******************************************************************************
 * 器件模拟与仿真, 配置驱动的功能开启
 *
 * 文件名称: pseudo_sensor.c
 * 文件标识:
 * 内容摘要: 器件模拟与仿真, 实现与底层SP处理器驱动进行数据通讯的功能
 * 其它说明:
 * 当前版本: 1.1
 * 作    者: 80353364
 * 完成日期: 2021-12-01
 *
 * 修订记录
 * 修改日期    版本  修改人      修改内容
 * 2021-12-01   1.0  80353364    功能实现
 * 2021-01-15   1.1  80353364    方便共享内存的环回BUF处理, 将相关地址计数器改为32bit
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/io.h>

// 高通子系统的共享内存分配
#if defined(CONFIG_ARCH_QCOM)

#include <linux/soc/qcom/smem.h>
#elif defined(CONFIG_MTK_PLATFORM)
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include "../../misc/mediatek/scp/mt6885/scp_helper.h"
#else
#error Macro not defined, check definition in kernel of the SOC manufacturer and revise the code.
#error 定义的宏没有出现，请查看是否SOC厂商内核定义有变化，请修订代码或宏定义.
#endif

#define DRV_TAG                             "[pseudo]"
#define DBG(fmt, ...)                       pr_debug(DRV_TAG fmt, ##__VA_ARGS__);
#define INF(fmt, ...)                       pr_info(DRV_TAG fmt, ##__VA_ARGS__);
#define ERR(fmt, ...)                       pr_err(DRV_TAG fmt, ##__VA_ARGS__);

#define CFG_BUFSIZE                         512
// 支持 16 个SENSOR(数据通道), 定义需是 8 的倍数
#define CFG_MAX_SENSORS                     16
// 默认内存分配为 64KB, 注意高通由SLPI分配, 本定义不产生影响
#define CFG_SMEM_SZ                         SZ_64K
#define SZ_6K                               0x1800
#define MIN(a, b)                           ((a) <= (b) ? (a) : (b))

#define SMEM_PSEUDO_SENSOR_MEM_TYPE     490
#define SMEM_PSEUDO_SENSOR_SMEM_SIZE    0x10000

// 设置默认的数据大小, 以 32KB 为计量, 注意需要减去 smem_head 所占的空间
// 所有分配的空间大小需以 64Byte 对齐，因为数据序列发生器以 0x40 对齐
// 最终默认内存分配为 64KB

static uint16_t g_def_sz[CFG_MAX_SENSORS] = {
	SZ_6K,  SZ_6K,  SZ_6K,  SZ_6K,
	SZ_1K,  SZ_1K,  SZ_1K,  SZ_1K,
	SZ_512, SZ_512, SZ_512, SZ_512,
	SZ_512, SZ_512, SZ_512, SZ_256
};

static char *g_chn_type[CFG_MAX_SENSORS] = {
	"acc1",    "gyro1",    "mag",      "temp",
	"press",   "als1",     "proxi",    "halls",
	"acc2",    "gyro2",    "als2",     "als3",
	"n/a",     "n/a",      "n/a",      "halla"
};

#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
typedef struct _sensor_debug_kit_data_t{
	int32_t fault_code;
	int32_t frequency;
	int32_t repeat;
	int32_t test_type;
}sensor_debug_kit_data_t;
#endif

struct psensor_data {
	uint64_t phy_address;
	uint64_t vir_address;
	uint32_t size;
	int16_t enable;                             // 功能是否启用
	int16_t cur_chn;                            // 当前使用的通道[0,15]
#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
	sensor_debug_kit_data_t debug_kit_data;
#endif
	int32_t time_left[CFG_MAX_SENSORS];         // timeleft
	struct proc_dir_entry   *proc;
};

typedef struct smem_data_seq
{
	float begin;                                // 初始值
	float end;                                  // 结束值
	float delta;                                // 增量： 0 <0 >0
	float rand;                                 // 随机修订值:    rand * [-0.01 - 0.99]
}   smem_data_seq;


// 数据发生器: 最多支持 3个数据
typedef struct smem_data_type1 {
	uint32_t magic;                              // 魔数
	uint32_t times;                              // 数据执行次数
	uint32_t times_run;                          // 数据剩余执行次数
	uint32_t type:16;                            // 数据类型，固定为 1
	uint32_t axis:16;                            // 单次上报数据个数
	smem_data_seq seq[3];
}   smem_data_type1;

// 支持 CFG_MAX_SENSORS 个 SENSOR 同时工作,
// 为什么不用宏定义 16 ? 因为使用宏时可能存在一些问题如对齐、Cache命中等
// 注意定义的数据对齐 每个SENSOR 为 16Byte
typedef struct smem_head {
	uint32_t bp[CFG_MAX_SENSORS];               // 当前通道内存的起始偏移
	uint32_t size[CFG_MAX_SENSORS];             // 分配给指定 SENSOR 的内存大小,
												// 使用的环回缓冲区不能超时该地址
												// 单位为 float 字数
	uint32_t wp[CFG_MAX_SENSORS];               // 当前写入数据的地址偏址, 单位为 float 字数
	uint32_t rp[CFG_MAX_SENSORS];               // 当前读取数据的地址偏址, 单位为 float 字数
	uint8_t type[CFG_MAX_SENSORS];              // 数据类型
												// 0: 数据序列
												// 1: 数据序列发生器产生的数据
	uint8_t start[CFG_MAX_SENSORS];             // 数据启动标记
												// 0: 数据未准备好
												// 1: 数据已准备好
												// 2: 数据正在读取中
												// 3: 数据已取完
	uint16_t sst[CFG_MAX_SENSORS];              // 传感器工作状态:
												// 0: 初始态
												// 1 正常启用
												// 2: 手动关闭
												// 3: 启用错误测试功能
	uint32_t ecode[CFG_MAX_SENSORS];            // 错误代码反馈
}   smem_head;


// 分配的全局数据
static struct psensor_data *g_data = NULL;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
static void psensor_timer_out(unsigned long data);
static DEFINE_TIMER(g_timer, psensor_timer_out, 0, 0);
#else
static void psensor_timer_out(struct timer_list *data);
static DEFINE_TIMER(g_timer, psensor_timer_out);
#endif

// 定时器函数: 每秒钟执行一次
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
static void psensor_timer_out(unsigned long data)
#else
static void psensor_timer_out(struct timer_list *data)
#endif
{
	int i;
	int cnt = 0;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (0 == g_data->vir_address) {
		mod_timer(&g_timer, jiffies + msecs_to_jiffies(1000));
		return;
	}

	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		if (g_data->time_left[i] <= 0)  {
			continue;
		} else {
			g_data->time_left[i]--;
			cnt++;
		}

		if (g_data->time_left[i] <= 0)  {
			smem->start[i] = 1;
		}
	}

	if (cnt > 0) {
		mod_timer(&g_timer, jiffies + msecs_to_jiffies(1000));
	}
}


#if defined(CONFIG_ARCH_QCOM)
static int psensor_smem_get(unsigned host, unsigned id, uint64_t *vaddr, size_t *size)
{
	void *vmem = qcom_smem_get(host, id, size);
	if (IS_ERR(vmem)) {
		ERR("qcom_smem_get failed\n");
		*vaddr = 0;
		*size = 0;
		return -ENOMEM;
	} else {
		INF("qcom_smem_get true size: %lld Bytes\n", *size);
	}

	*vaddr = (uint64_t)vmem;
	return 0;
}
#endif


// 初始化共享内存空间
// 根据总 size 大小以及 sz 数组的长度为 smem 进行空间划分
// 注意 size 为 byte 计量, sz 中也为 byte 计量, 但 smem 中为 float 计量
static int psensor_smem_init(smem_head *smem, uint32_t size, uint16_t *sz)
{
	int i;
	uint32_t offset = sizeof(smem_head);
	uint32_t total_sz = 0;

	// 校验数据空间分配
	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		total_sz += sz[i];
	}

	if (0 == total_sz) {
		ERR("allocate size total is 0, failed\n");
		return -EPERM;
	}

	if (total_sz > size - offset) {
		ERR("allocate size(%X) more than free size %X\n", total_sz, size - offset);
		return -ENOMEM;
	}

	// 为什么要除以 4? 是因为转换为 float 个数
	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		smem->size[i] = sz[i] >> 2;
		smem->bp[i] = offset >> 2;
		smem->wp[i] = 0;
		smem->rp[i] = 0;
		offset = offset + sz[i];
		smem->type[i] = 0;
		smem->start[i] = 0;
	}

	return 0;
}

// 将用户态的内存复制到共享内存通路, 复制时不允许 wp == rp
// smem:    共享内存的起始地址
// chn:     共享内存通道号
// buf:     用户空间buf
// count:   用户空间传输的数据bytes, 注意该数据为 4Byte对齐(float32)
static int psensor_smem_put(smem_head *smem, int chn, const char __user *buf, size_t count)
{
	char *p;
	uint32_t len = 0;
	uint32_t size = count >> 2;           // 数据为  4Byte 的整数倍
	size = MIN(size, smem->size[chn] - smem->wp[chn] + smem->rp[chn]);
	len = MIN(size, smem->size[chn] - (smem->wp[chn] % smem->size[chn]));

	p = (char*)g_data->vir_address;
	p += smem->bp[chn] << 2;
	p += (smem->wp[chn] % smem->size[chn]) << 2;

	if (copy_from_user(p, buf, len << 2)) {
		ERR("ecode val copy from user error\n");
		return -EIO;
	}


	if (size - len > 0) {
		p = (char*)g_data->vir_address;
		p += smem->bp[chn] << 2;
		if (copy_from_user(p, buf + len, (size - len) << 2)) {
			ERR("ecode val copy from user error\n");
			return -EIO;
		}
	}

	smem->wp[chn] += size;
	return size << 2;
}


static ssize_t proc_data_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[2048];
	int i, len, total = 0;
	char *d = (char *)&data[0];
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (*ppos > 0 || count < 2048) {
		return 0;
	}

	if (0 == g_data->vir_address) {
		total = snprintf(d, 2048, "memory invalid, plese enable first\n");
		ERR("memory invalid, plese enable first\n");
		if (copy_to_user(buf, data, total)) {
			return -EFAULT;
		}
		*ppos = total;
		return total;
	}

	len = snprintf(d, 2048-total, "addr\t\t: %016lX\n", g_data->vir_address);
	d += len;
	total += len;
	len = snprintf(d, 2048-total, "size\t\t: %X\n", g_data->size);
	d += len;
	total += len;
	len = snprintf(d, 2048-total, "current chn\t: %d\n", g_data->cur_chn);
	d += len;
	total += len;
	len = snprintf(d, 2048-total,
				   "chn sst name\toff.f\tsize.f\tdtype\tstart\twp.f\trp.f\tecode\ttimeleft\n");
	d += len;
	total += len;

	for(i = 0; i < CFG_MAX_SENSORS; i++) {
		if (i == g_data->cur_chn) {
			len = snprintf(d, 2048-total,
						   "%02d< %-4X%s\t%X\t%X\t%X\t%X\t%X\t%X\t%X\t%d\n",
							i, smem->sst[i], g_chn_type[i], smem->bp[i],
							smem->size[i], smem->type[i], smem->start[i],
							smem->wp[i],
							smem->rp[i], smem->ecode[i], g_data->time_left[i]);
		} else {
			len = snprintf(d, 2048-total,
						   "%02d  %-4X%s\t%X\t%X\t%X\t%X\t%X\t%X\t%X\t%d\n",
						   i, smem->sst[i], g_chn_type[i], smem->bp[i],
						   smem->size[i], smem->type[i], smem->start[i],
						   smem->wp[i], smem->rp[i], smem->ecode[i],
						   g_data->time_left[i]);

		}
		d += len;
		total += len;
	}

	if (copy_to_user(buf, data, total)) {
		return -EFAULT;
	}

	*ppos = total;
	return total;
}


static uint32_t IEEE754_INT_TO_FLOAT(int v)
{
	uint32_t data = 0;
	uint32_t e = 23;

	// 特殊值处理
	if (v == 0) {
		return 0x0;
	}
	if (v == -1) {
		return 0xBF800000;
	}
	if (v == 1) {
		return 0x3F800000;
	}

	if (v < 0) {
		data = 1 << 31;
	}
	v = v < 0 ? -v: v;

	// 00..22 为 有效数字，不处理 >= 1<<23 的数据
	if (v >= (1<<23)) {
		v = (1 << 23);
	}

	// 将 v 中的 1 移到 第 23 位
	while ((v & (1 << 23)) == 0x0) {
		v = v << 1;
		e--;
	}
	v = v & 0x7FFFFF;
	e = e + 127;
	data = data | e << 23 | v;

	return data;
}


// 注意：真实数据序列允许绕回
// 数据发生器不支持绕回
static ssize_t proc_data_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char data[128];
	int v[8];
	uint32_t *p;                 // 其实是 float
	int i;
	int ret = 0;
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;
	int len = MIN(count, 128);

	if (0 == g_data->vir_address) {
		ERR("no share memory, plese bind share memory first\n");
		return -ENOMEM;
	}

	if (2 == smem->type[chn]) {
		if (smem->size[chn] <= 16) {
			return -ENOMEM;
		}

		memset(data, 0, sizeof(data));
		memset(v, 0, sizeof(v));

		if (copy_from_user(data, buf, len)) {
			ERR("sizes val copy from user error\n");
			return -EIO;
		}
		// 内核代码中的 sscanf 不支持 float 读取，故只能使用整型数据
		ret = sscanf(data,
					 "%d,%d,%d,%d,%d,%d,%d,%d",
					 &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]);
		INF("get %d values\n", ret);

		// 数据重新写入到头部
		smem->rp[chn] = 0;
		smem->wp[chn] = ret;
		p = (uint32_t*)g_data->vir_address;       // 其实是 float
		p += smem->bp[chn];
		for (i = 0; i < ret; i++) {
			*p++ = IEEE754_INT_TO_FLOAT(v[i]);
		}

		ret = count;
	} else {
		ret = psensor_smem_put(smem, chn, buf, count);
	}

	if (ret < 0) {
		ERR("data copy failed, ret = %d\n", ret);
	}

	return ret;
}


static ssize_t proc_sizes_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[512];
	int chn, i, len = 0;
	char *d = (char *)&data[0];

	if (*ppos > 0 || count < sizeof(data)) {
		return 0;
	}

	for(chn = 0; chn < CFG_MAX_SENSORS; chn++) {
		i = snprintf(d, 512-len, "chn%2d :\t%04X\n", chn, g_def_sz[chn]);
		d += i;
		len += i;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_sizes_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int ret;
	int i;
	char data[128];
	uint16_t s[CFG_MAX_SENSORS];
	int len = MIN(count, 128);

	if (0 == g_data->vir_address) {
		ERR("no share memory, plese bind share memory first\n");
		return -ENOMEM;
	}

	memset(data, 0, sizeof(data));
	if (copy_from_user(data, buf, len)) {
		ERR("sizes val copy from user error\n");
		return -EIO;
	}

	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		s[i] = 0;
	}

	// 读取数据, 注意数据单位为 KB
	ret = sscanf(data,
				 "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
				 &s[0], &s[1], &s[2], &s[3], &s[4], &s[5], &s[6], &s[7],
				 &s[8], &s[9], &s[10],&s[11],&s[12],&s[13],&s[14],&s[15]);

	INF("get %d valid data\n", ret);
	// 转换数据 KB 为 B
	for (i = 0; i < ret; i++) {
		s[i] <<= 10;
	}

	// 尝试分配各通道空间并返回分配结果
	ret = psensor_smem_init((smem_head*)g_data->vir_address, g_data->size, s);

	// 如果分配成功，将数据写入 g_def_sz, 其实不写入也没有问题, 只是为了后续可能的操作
	if (ret >= 0) {
		for (i = 0; i < CFG_MAX_SENSORS; i++) {
			g_def_sz[i] = s[i];
		}
	} else {
		return ret;
	}

	return count;
}


static ssize_t proc_datareset_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char *p;
	char data[16];
	int len, reset = 0;
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (0 == g_data->vir_address) {
		ERR("invalid memory address\n");
		return -EIO;
	}

	memset(data, 0, sizeof(data));

	len = MIN(count, 16);

	if (copy_from_user(data, buf, len)) {
		ERR("datareset val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &reset)) {
		INF("datareset val input: %d\n", reset);
	} else {
		ERR("datareset val input invalid\n");
		return -EINVAL;
	}

	if (1 != reset) {
		return count;
	}

	p = (char*)g_data->vir_address;
	p += smem->bp[chn] << 2;
	memset(p, 0, smem->size[chn] << 2);
	smem->wp[chn] = 0;
	smem->rp[chn] = 0;
	smem->type[chn] = 0;
	smem->start[chn] = 0;

	return count;
}


static ssize_t proc_timeleft_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[512];
	int chn, i, len = 0;
	char *d = (char *)&data[0];

	if (*ppos > 0 || count < 512) {
		return 0;
	}

	for(chn = 0; chn < CFG_MAX_SENSORS; chn++) {
		i = snprintf(d, 512-len, "chn%2d timeleft\t: %d\n", chn, g_data->time_left[chn]);
		d += i;
		len += i;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


// 写入格式1: 直接写入数据表示使用当前通道
// 写入格式2: "0xFF,4" > timeleft  前一个 指定的通道编号，后一个是多少秒后启用
static ssize_t proc_timeleft_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int i;
	char data[32];
	int timeleft = 0;
	u32 chns = 0;
	int len = MIN(count, sizeof(data));

	memset(data, 0, sizeof(data));

	if (copy_from_user(data, buf, len)) {
		ERR("timeleft val copy from user error\n");
		return -EIO;
	}

	if (2 == sscanf(data, "0x%X,%d", &chns, &timeleft)) {
		INF("timeleft val input: chn: 0x%X, %d\n", chns, timeleft);
	} else if (2 == sscanf(data, "0x%X %d", &chns, &timeleft)) {
		INF("timeleft val input: chn: 0x%X, %d\n", chns, timeleft);
	} else if (2 == sscanf(data, "0x%X, %d", &chns, &timeleft)) {
		INF("timeleft val input: chn: 0x%X, %d\n", chns, timeleft);
	} else if (1 == sscanf(data, "%d", &timeleft)) {
		INF("timeleft val input: %d\n", timeleft);
	} else {
		ERR("input invalid, it like: echo \"0xFF 10\" > timeleft\n");
		return -EINVAL;
	}

	if (0 == chns) {
		g_data->time_left[g_data->cur_chn] = timeleft;
	} else {
		for (i = 0; i < CFG_MAX_SENSORS; i++) {
			if (chns & (1 << i)) {
				g_data->time_left[i] = timeleft;
			}
		}
	}

	// 启动数据处理
	mod_timer(&g_timer, jiffies + msecs_to_jiffies(1000));

	return count;
}


static ssize_t proc_sst_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	static char *mod_txt[4] = {"init", "valid", "invalid", "error"};

	char data[512];
	int i, len = 0;
	char *d = (char *)&data[0];
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (*ppos > 0 || count < 512) {
		return 0;
	}

	i = snprintf(d, 512-len, "chn\tstatus\tmode\n");
	d += i;
	len += i;

	for(chn = 0; chn < CFG_MAX_SENSORS; chn++) {
		i = snprintf(d, 512-len, "%d\t%u\t%s\n", chn, smem->sst[chn], mod_txt[smem->sst[chn]]);
		d += i;
		len += i;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


// 写入格式: 直接写入数据表示使用当前通道
static ssize_t proc_sst_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char data[32];
	int st = 0;
	smem_head *smem = (smem_head*)g_data->vir_address;
	int len = MIN(count, sizeof(data));

	memset(data, 0, sizeof(data));

	if (copy_from_user(data, buf, len)) {
		ERR("timeleft val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &st)) {
		INF("sensor status val input: %d\n", st);
	} else {
		ERR("input invalid, it like: echo 1 > sst\n");
		return -EINVAL;
	}

	// 注意, sst 的值设置范围为：
	// 0: SENSOR 没有工作(初始态)
	// 1: SENSOR 工作在正常模式
	// 2: SENSOR 被手动关闭
	// 3: SENSOR 工作在异常模拟模式，此时 ecode 值才起作用。
	if (0 <= st && st < 4) {
		smem->sst[g_data->cur_chn] = st;
	} else {
		ERR("input value verify invalid\n");
		return -EINVAL;
	}

	return count;
}


static ssize_t proc_ecode_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[512];
	int i, len = 0;
	char *d = (char *)&data[0];
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (*ppos > 0 || count < 512) {
		return 0;
	}

	for(chn = 0; chn < CFG_MAX_SENSORS; chn++) {
		i = snprintf(d, 512-len,
					"chn%2d ecode\t: %08X(%u)\n",
					chn, smem->ecode[chn], smem->ecode[chn]);
		d += i;
		len += i;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


// 写入格式: 直接写入数据表示使用当前通道
static ssize_t proc_ecode_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char data[32];
	int ecode = 0;
	smem_head *smem = (smem_head*)g_data->vir_address;
	int len = MIN(count, sizeof(data));

	memset(data, 0, sizeof(data));

	if (copy_from_user(data, buf, len)) {
		ERR("ecode val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &ecode)) {
		INF("ecode val input: %d\n", ecode);
	} else {
		ERR("input invalid, it like: echo 1 > ecode\n");
		return -EINVAL;
	}

	if (ecode >= 0) {
		smem->ecode[g_data->cur_chn] = ecode;
	} else {
		ERR("input value verify invalid\n");
		return -EINVAL;
	}

	return count;
}


static ssize_t proc_datatype_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len;
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE,"%u\n", smem->type[chn]);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_datatype_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char data[16];
	int len;
	int type = 0;
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	memset(data, 0, sizeof(data));

	len = MIN(count, 16);

	if (copy_from_user(data, buf, len)) {
		ERR("datatype val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &type)) {
		INF("datatype val input: %d\n", type);
	} else {
		ERR("datatype val input invalid\n");
		return -EINVAL;
	}

	if (type < 0 || type > 2) {
		ERR("datatype val out range\n");
		return -EINVAL;
	}

	smem->type[chn] = type;
	return count;
}


static ssize_t proc_phy_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "0x%016lX\n", g_data->phy_address);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;

}


static ssize_t proc_vir_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "0x%016lX\n", g_data->vir_address);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


#if defined(CONFIG_ARCH_QCOM)
/******************************************************************************
 * 以下是高通对各子系统的定义
enum subsystem_pid {
	PID_APSS = 0,
	PID_MPSS = 1,
	PID_ADSP = 2,
	PID_SLPI = 3,
	PID_CDSP = 5,
	PID_WPSS = 13,
	PID_GPU = PID_APSS,
	PID_DISPLAY = PID_APSS,
	PID_OTHERS = -2,
};
******************************************************************************/
static ssize_t proc_freemem_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	uint32_t bytes;
	int len = 0;
	char *d = &data[0];

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	bytes = (uint32_t)qcom_smem_get_free_space(-1);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone comm]\t: 0x%08X\n", bytes);
	d = &data[len];

	bytes = (uint32_t)qcom_smem_get_free_space(0);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone apss]\t: 0x%08X\n", bytes);
	d = &data[len];

	bytes = (uint32_t)qcom_smem_get_free_space(1);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone mpss]\t: 0x%08X\n", bytes);
	d = &data[len];

	bytes = (uint32_t)qcom_smem_get_free_space(2);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone adsp]\t: 0x%08X\n", bytes);
	d = &data[len];

	bytes = (uint32_t)qcom_smem_get_free_space(3);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone slpi]\t: 0x%08X\n", bytes);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

#elif defined(CONFIG_MTK_PLATFORM)

static ssize_t proc_freemem_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int i;
	int len = 0;
	char *d = &data[0];

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(d, CFG_BUFSIZE - len, "id\tphy\t \t \tsize\n");

	for (i = 0; i < 16; i++) {
		d = &data[len];
		phys_addr_t phy = scp_get_reserve_mem_phys(i);
		phys_addr_t sz = scp_get_reserve_mem_size(i);
		len += snprintf(d, CFG_BUFSIZE - len, "%d\t%016lX\t%X\n", i, phy, sz);
	}

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

#else

#error Macro not defined, check definition in kernel of the SOC manufacturer and revise the code.
#error 定义的宏没有出现，请查看是否SOC厂商内核定义有变化，请修订代码或宏定义.

#endif


// 显示当前正在处理的通道 [0,15]
static ssize_t proc_current_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "%d\n", g_data->cur_chn);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_current_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len, cur_chn;
	char data[16];

	memset(data, 0, sizeof(data));

	len = MIN(count, 16);

	if (copy_from_user(data, buf, len)) {
		ERR("current val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_chn)) {
		INF("current val input: %d\n", cur_chn);
	} else {
		ERR("current val input invalid\n");
		return -EINVAL;
	}

	if (cur_chn < 0 || cur_chn > (CFG_MAX_SENSORS - 1)) {
		ERR("current val out range\n");
		return -EINVAL;
	}

	g_data->cur_chn = cur_chn;
	return count;
}


static ssize_t proc_bind_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "%d\n", g_data->enable);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_bind_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	void *vmem;
	int len;
	int ret;
	uint64_t vaddr;
	size_t size = 0;
	int enable = 0;
	char data[16];

	memset(data, 0, sizeof(data));

	// 不允许重新分配内存
	if (0 != g_data->vir_address) {
		ERR("alread bind\n");
		return -EBUSY;
	}

	len = MIN(count, 16);

	if (copy_from_user(data, buf, len)) {
		ERR("read bind val input error\n");
		return -EIO;
	}

	if (1 != sscanf(data, "%d", &enable)) {
		ERR("invalid input\n");
		return -EINVAL;
	}

#if defined(CONFIG_ARCH_QCOM)

	// 通过高通 qcom_smem_alloc 方式匹配已分配的内存
	// 参数 3, 490 为已约定值  PID_APSS = 0,   PID_MPSS = 1,   PID_ADSP = 2,   PID_SLPI = 3,
	ret = psensor_smem_get(3, 490, &vaddr, &size);

	if (ret < 0) {
		ret = psensor_smem_get(2, 490, &vaddr, &size);
	}

	g_data->vir_address = (uint64_t)vaddr;

	if (0 != vaddr) {
		g_data->phy_address = (uint64_t)qcom_smem_virt_to_phys((void*)vaddr);
	}
	vmem = (void*)vaddr;

#elif defined(CONFIG_MTK_PLATFORM)

	// 申请内存并将其物理、虚拟地址填入 phy_address vir_address
	// MTK 内存分配，注意是全部的内存, 所用到只占最后 64KB
	// vaddr 使用是为了避免 -Wunused-variable
	uint64_t smem_base_phys;
	uint64_t smem_size;
	struct device_node *nd;

	// 测试代码
	nd = of_find_compatible_node(NULL, NULL, "mediatek,reserve-memory-scp_share");

	if (nd != NULL) {
		of_property_read_u64_index(nd, "size", 0, &smem_size);
		INF("get size: %X\n", smem_size);
	}

	smem_base_phys = (uint64_t)scp_get_reserve_mem_phys(0);
	vaddr = (uint64_t)(size_t)ioremap_wc(smem_base_phys, smem_size);;
	g_data->vir_address = vaddr;
	g_data->phy_address = smem_base_phys;
	size = smem_size;
	vmem = (void*)g_data->vir_address;

	INF("mtk share memory info: phy: %016lX, vir: %016lX, size: %08lX\n",
		smem_base_phys, g_data->vir_address, size);

	// 重新计算内存
	// 取该内存的最后    64 KB(CFG_SMEM_SZ)内存为共享内存
	if ((NULL != vmem) && (0 != size)) {
		g_data->vir_address = g_data->vir_address + size - CFG_SMEM_SZ;
		g_data->phy_address = g_data->phy_address + size - CFG_SMEM_SZ;
		size = CFG_SMEM_SZ;
		vmem = (void*)g_data->vir_address;
	}

#else

#error Macro not defined, check definition in kernel of the SOC manufacturer and revise the code.
#error 定义的宏没有出现，请查看是否SOC厂商内核定义有变化，请修订代码或宏定义.

#endif

	if ((NULL == vmem) || (0 == size)) {
		ERR("memory malloc failed\n");
		return -ENOMEM;
	}

	// 设置全局变量
	g_data->size = (uint32_t)size;
	memset(vmem, 0, g_data->size);

	// 设置初始值
	psensor_smem_init((smem_head*)g_data->vir_address, g_data->size, g_def_sz);

	INF("memory malloc 0x%XByte at %016lX(physical: %016lX)\n",
		size, g_data->vir_address, g_data->phy_address);

	g_data->enable = 1;
	return count;
}

#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
#define DATA_BUFFER_SIZE    16
#define SMEM_PSEUDO_SENSOR_DEBUG_KIT_CH 14
static size_t sensor_debug_kit_update_data()
{
	char *p;
	size_t count = sizeof(g_data->debug_kit_data);
	smem_head *smem = (smem_head*)g_data->vir_address;

	if(g_data->vir_address == 0) {
		ERR("vir_address not bind yet, do bind first!\n");
		return -1;
	}
	p = (char*)g_data->vir_address;
	p += smem->bp[SMEM_PSEUDO_SENSOR_DEBUG_KIT_CH] << 2;

	memcpy((void *)p, (void *)&(g_data->debug_kit_data), count);
	smem->start[SMEM_PSEUDO_SENSOR_DEBUG_KIT_CH] = 1;
	INF("smem ready, test_type=%d, fault_code=%d, frequency=%d, repeat=%d!\n",
		g_data->debug_kit_data.test_type, g_data->debug_kit_data.fault_code,
		g_data->debug_kit_data.frequency, g_data->debug_kit_data.repeat);
	return count;
}

static ssize_t proc_fault_code_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[DATA_BUFFER_SIZE] = {0};
	int len = 0;

	if (*ppos > 0 || count < DATA_BUFFER_SIZE)
		return 0;

	len = snprintf(data, DATA_BUFFER_SIZE, "%d\n", g_data->debug_kit_data.fault_code);
	if(copy_to_user(buf, data, len))
		return -EFAULT;

	*ppos = len;
	return len;
}

static ssize_t proc_fault_code_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int cur_fault_code = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("fault code val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_fault_code)) {
		INF("fault code val input: %d\n", cur_fault_code);
	} else {
		ERR("fault code val input invalid\n");
		return -EINVAL;
	}

	if (cur_fault_code < 0 || cur_fault_code > 10002) {
		ERR("fault code val out range\n");
		return -EINVAL;
	}

	g_data->debug_kit_data.fault_code = cur_fault_code;
	sensor_debug_kit_update_data();
	return count;
}
static ssize_t proc_frequency_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[DATA_BUFFER_SIZE] = {0};
	int len = 0;

	if (*ppos > 0 || count < DATA_BUFFER_SIZE)
		return 0;

	len = snprintf(data, DATA_BUFFER_SIZE, "%d\n", g_data->debug_kit_data.frequency);
	if(copy_to_user(buf, data, len))
		return -EFAULT;

	*ppos = len;
	return len;
}
static ssize_t proc_frequency_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int cur_frequency = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("frequency val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_frequency)) {
		INF("frequency val input: %d\n", cur_frequency);
	} else {
		ERR("frequency val input invalid\n");
		return -EINVAL;
	}

	g_data->debug_kit_data.frequency = cur_frequency;
	sensor_debug_kit_update_data();
	return count;
}
static ssize_t proc_repeat_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[DATA_BUFFER_SIZE] = {0};
	int len = 0;

	if (*ppos > 0 || count < DATA_BUFFER_SIZE)
		return 0;

	len = snprintf(data, DATA_BUFFER_SIZE, "%d\n", g_data->debug_kit_data.repeat);
	if(copy_to_user(buf, data, len))
		return -EFAULT;

	*ppos = len;
	return len;
}
static ssize_t proc_repeat_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int cur_repeat = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("repeat val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_repeat)) {
		INF("repeat val input: %d\n", cur_repeat);
	} else {
		ERR("repeat val input invalid\n");
		return -EINVAL;
	}

	g_data->debug_kit_data.repeat = cur_repeat;
	sensor_debug_kit_update_data();
	return count;
}
static ssize_t proc_test_type_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[DATA_BUFFER_SIZE] = {0};
	int len = 0;

	if (*ppos > 0 || count < DATA_BUFFER_SIZE)
		return 0;

	len = snprintf(data, DATA_BUFFER_SIZE, "%d\n", g_data->debug_kit_data.test_type);
	if(copy_to_user(buf, data, len))
		return -EFAULT;

	*ppos = len;
	return len;
}
static ssize_t proc_test_type_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int cur_test_type = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("test type val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_test_type)) {
		INF("test type val input: %d\n", cur_test_type);
	} else {
		ERR("test type val input invalid\n");
		return -EINVAL;
	}

	g_data->debug_kit_data.test_type = cur_test_type;
	sensor_debug_kit_update_data();
	return count;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0))

static const struct file_operations proc_data_fops =
{
	.read = proc_data_r,
	.write = proc_data_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations proc_sizes_fops =
{
	.read = proc_sizes_r,
	.write = proc_sizes_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations proc_datareset_fops =
{
	.write = proc_datareset_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations proc_timeleft_fops =
{
	.owner = THIS_MODULE,
	.read = proc_timeleft_r,
	.write = proc_timeleft_w,
	.open = simple_open,
};


static const struct file_operations proc_sst_fops =
{
	.owner = THIS_MODULE,
	.read = proc_sst_r,
	.write = proc_sst_w,
	.open = simple_open,
};


static const struct file_operations proc_ecode_fops =
{
	.owner = THIS_MODULE,
	.read = proc_ecode_r,
	.write = proc_ecode_w,
	.open = simple_open,
};


static const struct file_operations proc_datatype_fops =
{
	.owner = THIS_MODULE,
	.read = proc_datatype_r,
	.write = proc_datatype_w,
	.open = simple_open,
};


static const struct file_operations proc_phy_fops =
{
	.owner = THIS_MODULE,
	.read = proc_phy_r,
	.open = simple_open,
};


static const struct file_operations proc_vir_fops =
{
	.owner = THIS_MODULE,
	.read = proc_vir_r,
	.open = simple_open,
};


static const struct file_operations proc_freemem_fops =
{
	.owner = THIS_MODULE,
	.read = proc_freemem_r,
	.open = simple_open,
};


static const struct file_operations proc_current_fops =
{
	.read = proc_current_r,
	.write = proc_current_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations proc_bind_fops =
{
	.read = proc_bind_r,
	.write = proc_bind_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
static const struct file_operations proc_fault_code_fops =
{
	.read       = proc_fault_code_r,
	.write      = proc_fault_code_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
static const struct file_operations proc_frequency_fops =
{
	.read       = proc_frequency_r,
	.write      = proc_frequency_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
static const struct file_operations proc_repeat_fops =
{
	.read       = proc_repeat_r,
	.write      = proc_repeat_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
static const struct file_operations proc_test_type_fops =
{
	.read       = proc_test_type_r,
	.write      = proc_test_type_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
#endif
#else

static const struct proc_ops proc_data_fops =
{
	.proc_read = proc_data_r,
	.proc_write = proc_data_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_sizes_fops =
{
	.proc_read = proc_sizes_r,
	.proc_write = proc_sizes_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_datareset_fops =
{
	.proc_write = proc_datareset_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_timeleft_fops =
{
	.proc_read = proc_timeleft_r,
	.proc_write = proc_timeleft_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_sst_fops =
{
	.proc_read = proc_sst_r,
	.proc_write = proc_sst_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_ecode_fops =
{
	.proc_read = proc_ecode_r,
	.proc_write = proc_ecode_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_datatype_fops =
{
	.proc_read = proc_datatype_r,
	.proc_write = proc_datatype_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_phy_fops =
{
	.proc_read = proc_phy_r,
	.proc_open = simple_open,
};
static const struct proc_ops proc_vir_fops =
{
	.proc_read = proc_vir_r,
	.proc_open = simple_open,
};


static const struct proc_ops proc_freemem_fops =
{
	.proc_read = proc_freemem_r,
	.proc_open = simple_open,
};


static const struct proc_ops proc_current_fops =
{
	.proc_read = proc_current_r,
	.proc_write = proc_current_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_bind_fops =
{
	.proc_read = proc_bind_r,
	.proc_write = proc_bind_w,
	.proc_open = simple_open,
};
#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
static const struct proc_ops proc_fault_code_fops =
{
	.proc_read       = proc_fault_code_r,
	.proc_write      = proc_fault_code_w,
	.proc_open       = simple_open,
};
static const struct proc_ops proc_frequency_fops =
{
	.proc_read       = proc_frequency_r,
	.proc_write      = proc_frequency_w,
	.proc_open       = simple_open,
};
static const struct proc_ops proc_repeat_fops =
{
	.proc_read       = proc_repeat_r,
	.proc_write      = proc_repeat_w,
	.proc_open       = simple_open,
};
static const struct proc_ops proc_test_type_fops =
{
	.proc_read       = proc_test_type_r,
	.proc_write      = proc_test_type_w,
	.proc_open       = simple_open,
};
#endif
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) && defined(OPLUS_FEATURE_SENSOR_DEBUG_KIT)
static int reserve_pseudo_sensor_share_mem()
{
	int rc = 0;
	rc = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, SMEM_PSEUDO_SENSOR_MEM_TYPE, SMEM_PSEUDO_SENSOR_SMEM_SIZE);
	if (rc < 0 && rc != -EEXIST) {
		pr_err("%s smem_alloc fail\n", __func__);
		rc = -EFAULT;
		return rc;
	}
	return 0;
}
#endif

static int __init pseduo_sensor_init(void)
{
	struct proc_dir_entry *dir;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) && defined(OPLUS_FEATURE_SENSOR_DEBUG_KIT)
	int err = 0;
	err = reserve_pseudo_sensor_share_mem();
	if(err) {
		pr_info("reserve_pseudo_sensor_share_mem failed\n");
		return -ENOMEM;
	}
#endif

	if (g_data != NULL) {
		ERR("driver already exist\n");
		return -EBUSY;
	} else {
		g_data = (struct psensor_data *)kmalloc(sizeof(struct psensor_data), GFP_KERNEL);
		if (NULL == g_data) {
			ERR("driver malloc memory failed\n");
			return -ENOMEM;
		}
	}

	memset(g_data, 0, sizeof(struct psensor_data));

	dir = proc_mkdir("pseudo-sensor", NULL);

	if (!dir) {
		ERR("mkdir /proc/psensor_dir failed\n");
		return -ENOMEM;
	}

	g_data->proc = dir;

	proc_create_data("bind", 0664, dir,  &proc_bind_fops, NULL);
	proc_create_data("sizes", 0664, dir,  &proc_sizes_fops, NULL);
	proc_create_data("current", 0664, dir,  &proc_current_fops, NULL);
	proc_create_data("data", 0664, dir,  &proc_data_fops, NULL);
	proc_create_data("datatype", 0664, dir,  &proc_datatype_fops, NULL);
	proc_create_data("datareset", 0220, dir,  &proc_datareset_fops, NULL);
	proc_create_data("phyaddr", 0444, dir,  &proc_phy_fops, NULL);
	proc_create_data("viraddr", 0444, dir,  &proc_vir_fops, NULL);
	proc_create_data("frees", 0444, dir,  &proc_freemem_fops, NULL);
	proc_create_data("timeleft", 0664, dir,  &proc_timeleft_fops, NULL);
	proc_create_data("sst", 0664, dir,  &proc_sst_fops, NULL);
	proc_create_data("ecode", 0664, dir,  &proc_ecode_fops, NULL);
#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
	proc_create_data("fault_code", 0664, dir, &proc_fault_code_fops, NULL);
	proc_create_data("frequency", 0664, dir, &proc_frequency_fops, NULL);
	proc_create_data("repeat", 0664, dir, &proc_repeat_fops, NULL);
	proc_create_data("test_type", 0664, dir, &proc_test_type_fops, NULL);
#endif
	// 初始化定时器
	g_timer.expires = 0;
	// 添加定时器，定时器开始生效
	add_timer(&g_timer);

	INF("pseduo sensor initialize successed\n");

	return 0;
}


static void __exit pseduo_sensor_exit(void)
{
	del_timer_sync(&g_timer);                           // 卸载模块时，删除定时器

	if (NULL == g_data) {
		ERR("malloc memory failed?\n");
		return;
	}

	proc_remove(g_data->proc);

	if (0 != g_data->vir_address) {

#if defined(CONFIG_ARCH_QCOM)

	// 高通的共享内存不是由linux分配
	// 不支持动态释放内存

#elif defined(CONFIG_MTK_PLATFORM)

	// MTK 的内存是静态分配并约定
	// 内存无法动态释放

#else

#error Macro not defined, check definition in kernel of the SOC manufacturer and revise the code.
#error 定义的宏没有出现，请查看是否SOC厂商内核定义有变化，请修订代码或宏定义.

#endif
	}

	kfree(g_data);
	g_data = NULL;
}


module_init(pseduo_sensor_init);
module_exit(pseduo_sensor_exit);


MODULE_AUTHOR("weng.yunfeng");
MODULE_DESCRIPTION("Pseudo Sensor Driver");
MODULE_LICENSE("GPL");
