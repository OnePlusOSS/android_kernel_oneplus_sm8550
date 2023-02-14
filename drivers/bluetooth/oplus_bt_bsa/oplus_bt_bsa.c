// SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 OPlus. All rights reserved.
 */
#include <linux/of.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/netfilter.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/version.h>
#include <linux/random.h>
#include <net/sock.h>
#include <linux/file.h>
#include <linux/netlink.h>
#include <linux/netfilter/xt_state.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_owner.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <net/genetlink.h>

#define BT_BSA_GPIO "oplus_bsa_gpio"
enum{
	OPLUS_BSA_UNDEFINE = 0,
	OPLUS_BSA_LOW,
	OPLUS_BSA_HIGH,
	OPLUS_BSA_NOTIFY_PID,
	__OPLUS_BSA_MSG_MAX,
};
#define OPLUS_BSA_MSG_MAX (__OPLUS_BSA_MSG_MAX - 1)
#define OPLUS_BT_BSA_CMD 1
static DEFINE_MUTEX(oplus_bt_bsa_nl_mutex);
static int oplus_bt_bsa_genl_rcv(struct sk_buff *skb, struct genl_info *info);
static long oplus_bsa_gpiolevel;

struct pinctrl *bsa_pinctl;
struct pinctrl_state *pinctrl_state_high;
struct pinctrl_state *pinctrl_state_low;
struct regulator *vdd_reg;
static u32 oplus_bt_bsa_nl_pid = 0;

static const struct genl_ops oplus_bt_bsa_genl_ops[] = {
	{
        .cmd = OPLUS_BT_BSA_CMD,
        .flags = 0,
        .doit = oplus_bt_bsa_genl_rcv,
        .dumpit = NULL,
	},
};

static struct genl_family oplus_bt_bsa_genl_family = {
	.id = 0,
	.hdrsize = 0,
	.name = BT_BSA_GPIO,
	.version = 1,
	.maxattr = OPLUS_BSA_MSG_MAX,
	.ops = oplus_bt_bsa_genl_ops,
	.n_ops = ARRAY_SIZE(oplus_bt_bsa_genl_ops),
};

static int oplus_switch_to_high() {
	if (!bsa_pinctl || !pinctrl_state_high) {
		printk(KERN_ERR "%s, oplus_bt_bsa, pinctrl is NULL\n",  __func__);
		return -ENOENT;
	}
	printk(KERN_INFO "%s, oplus_bt_bsa, is high\n",  __func__);
	/* toggle antenna switch to secondary antenna */
	pinctrl_select_state(bsa_pinctl, pinctrl_state_high);
	return 0;
}

static int oplus_switch_to_low() {
	if (!bsa_pinctl || !pinctrl_state_low) {
		printk(KERN_ERR "%s, oplus_bt_bsa, pinctrl is NULL\n",  __func__);
		return -ENOENT;
	}
	printk(KERN_INFO "%s, oplus_bt_bsa, is low\n",  __func__);
	/* toggle antenna switch to prime antenna */
	pinctrl_select_state(bsa_pinctl, pinctrl_state_low);
	return 0;
}

/*===platform device start===*/
/* register /sys/module/oplus_bt_bsa/parameters/oplus_bsa_param */
static int oplus_bsa_sysmodule_ops_set(const char *kmessage, const struct kernel_param *kp) {
	if (!kmessage) {
		printk(KERN_ERR "%s, oplus_bt_bsa, error: kmessage == null!\n",  __func__);
		return -1;
	}
	printk(KERN_INFO "%s, oplus_bt_bsa, %s\n",  __func__, kmessage);
	if (kstrtol(kmessage, 10, &oplus_bsa_gpiolevel) || (oplus_bsa_gpiolevel != 0 && oplus_bsa_gpiolevel != 1)) {
		printk(KERN_ERR "%s, oplus_bt_bsa, error: gpiolevel parsing error!\n",  __func__);
		return -1;
	}
	printk(KERN_INFO "%s, oplus_bt_bsa, level = %d", __func__, oplus_bsa_gpiolevel);
	if (!bsa_pinctl) {
		printk(KERN_INFO "%s, oplus_bt_bsa, pinctrl is NULL\n",  __func__);
		return -1;
	}
	if (oplus_bsa_gpiolevel == 0) {
		oplus_switch_to_high();
	} else {
		oplus_switch_to_low();
	}
	return 0;
}

static const struct kernel_param_ops oplus_bsa_sysmodule_ops = {
	.set = oplus_bsa_sysmodule_ops_set,
	.get = param_get_int,
};

module_param_cb(oplus_bsa_param, &oplus_bsa_sysmodule_ops, &oplus_bsa_gpiolevel, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

static int oplus_bsa_probe(struct platform_device *pdev) {
	int ret = 0;
	struct device *dev;
	printk(KERN_INFO "%s, oplus_bt_bsa, enter\n",  __func__);

	dev = &pdev->dev;
	if (dev->of_node == NULL) {
		printk(KERN_ERR "%s, oplus_bt_bsa, can't find compatible node in device tree\n",  __func__);
		return -ENOENT;
	}

	bsa_pinctl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(bsa_pinctl)) {
		ret = PTR_ERR(bsa_pinctl);
		printk(KERN_ERR "%s, oplus_bt_bsa, failed to get pinctrl, err = %d\n",  __func__, ret);
		bsa_pinctl = NULL;
		return ret;
	}

	pinctrl_state_high = pinctrl_lookup_state(bsa_pinctl, "bt_bsa_high");
	if (IS_ERR_OR_NULL(pinctrl_state_high)) {
		ret = PTR_ERR(pinctrl_state_high);
		printk(KERN_ERR "%s, oplus_bt_bsa, failed to get bsa pinctrl high state, err = %d\n",  __func__, ret);
		pinctrl_state_high = NULL;
		return ret;
	}

	pinctrl_state_low = pinctrl_lookup_state(bsa_pinctl, "bt_bsa_low");
	if (IS_ERR_OR_NULL(pinctrl_state_low)) {
		ret = PTR_ERR(pinctrl_state_low);
		printk(KERN_ERR "%s, oplus_bt_bsa, failed to get bsa pinctrl low state, err = %d\n",  __func__, ret);
		pinctrl_state_low = NULL;
		return ret;
	}

	if (of_find_property(dev->of_node, "vdd-supply", NULL)) {
		vdd_reg = devm_regulator_get(dev, "vdd");
		if (IS_ERR(vdd_reg)) {
			ret = PTR_ERR(vdd_reg);
			printk(KERN_ERR "%s, oplus_bt_bsa, failed to get vdd-supply, err = %d\n",  __func__, ret);
			return ret;
		}
	}

	if (vdd_reg) {
		ret = regulator_enable(vdd_reg);
		if (ret < 0) {
			printk(KERN_ERR "%s, oplus_bt_bsa, failed to enable vdd-supply, err = %d\n",  __func__, ret);
			return ret;
		}
	}
	printk(KERN_INFO "%s, oplus_bt_bsa, init as high state!\n",  __func__);
	pinctrl_select_state(bsa_pinctl, pinctrl_state_high);
	return 0;
}

static int oplus_bsa_remove(struct platform_device *pdev) {
	if (bsa_pinctl) {
		devm_pinctrl_put(bsa_pinctl);
	}
	if (vdd_reg) {
		int ret = regulator_disable(vdd_reg);
		if (ret < 0) {
			printk(KERN_ERR "%s, oplus_bt_bsa, fail to disable vdd-supply, ret = %d\n",  __func__, ret);
		}
	}
	return 0;
}

static const struct of_device_id oplus_bsa_dt_ids[] = {
	{ .compatible = "oplus-bt-bsa", },
	{},
};
MODULE_DEVICE_TABLE(of, oplus_bsa_dt_ids);

static struct platform_driver oplus_bsa_driver = {
	.probe = oplus_bsa_probe,
	.remove = oplus_bsa_remove,
	.driver = {
		.name = "oplus-bt-bsa",
		.of_match_table = of_match_ptr(oplus_bsa_dt_ids),
	},
};
/*===platform device end===*/

/*===netlink part start===*/
static int oplus_bt_bsa_genl_rcv_msg(struct sk_buff *skb, struct genl_info *info) {
	int ret = 0;
	struct nlmsghdr *nlhdr;
	struct genlmsghdr *genlhdr;
	struct nlattr *nla;
	u32 portid;

	nlhdr = nlmsg_hdr(skb);
	genlhdr = nlmsg_data(nlhdr);
	nla = genlmsg_data(genlhdr);

	portid = nlhdr->nlmsg_pid; /*NETLINK_CB(skb).portid;*/
	printk(KERN_INFO "%s, oplus_bt_bsa, the nla->nla_type = %u, len = %u, port id = %d\n", __func__, nla->nla_type, nla->nla_len, portid);

	if (nla->nla_type == OPLUS_BSA_NOTIFY_PID) {
		oplus_bt_bsa_nl_pid = portid;
		printk(KERN_INFO "%s, oplus_bt_bsa, oplus_bt_bsa_nl_pid port id=%d\n", __func__, oplus_bt_bsa_nl_pid);
	}
	/*only recv msg from target pid*/
	if (portid != oplus_bt_bsa_nl_pid) {
		return -1;
	}

	switch (nla->nla_type) {
	case OPLUS_BSA_HIGH:
		ret = oplus_switch_to_high();
		break;
	case OPLUS_BSA_LOW:
		ret = oplus_switch_to_low();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int oplus_bt_bsa_genl_rcv(struct sk_buff *skb, struct genl_info *info) {
	int ret = 0;
	/*mutex_lock(&oplus_bt_bsa_nl_mutex);*/
	ret = oplus_bt_bsa_genl_rcv_msg(skb, info);
	/*mutex_unlock(&oplus_bt_bsa_nl_mutex);*/
	printk(KERN_INFO "%s, oplus_bt_bsa, bt bsa genl rcv, result = %d\n", __func__, ret);
	return ret;
}

static int oplus_bsa_netlink_init(void) {
	int ret = 0;
	ret = genl_register_family(&oplus_bt_bsa_genl_family);
	if (ret) {
		return ret;
	} else {
		printk(KERN_INFO "%s, oplus_bt_bsa, genl_register_family complete, id = %d\n", __func__, oplus_bt_bsa_genl_family.id);
	}
	return ret;
}

static void oplus_bsa_netlink_deinit(void) {
	genl_unregister_family(&oplus_bt_bsa_genl_family);
}
/*===netlink part end===*/

/*===module part start===*/
static int __init oplus_bsa_init(void) {
	int ret = 0;
	printk(KERN_INFO "%s, oplus_bt_bsa, init\n",  __func__);
	ret = platform_driver_register(&oplus_bsa_driver);
	if (ret) {
		printk(KERN_ERR "%s, oplus_bt_bsa, cannot register driver\n",  __func__);
		return ret;
	}
	if (oplus_bsa_netlink_init()) {
		printk(KERN_ERR "%s, cannot init oplus bsa netlink\n",  __func__);
	}
	return ret;
}

static void __exit oplus_bsa_fini(void) {
	platform_driver_unregister(&oplus_bsa_driver);
	oplus_bsa_netlink_deinit();
}

module_init(oplus_bsa_init);
module_exit(oplus_bsa_fini);

MODULE_DESCRIPTION("OPLUS bt smart antenna");
MODULE_LICENSE("GPL v2");

