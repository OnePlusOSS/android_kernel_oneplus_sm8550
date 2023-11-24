/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 */
#ifndef FSA4480_I2C_H
#define FSA4480_I2C_H

#ifndef OPLUS_ARCH_EXTENDS
/* Add for OPLUS MACRO */
#define OPLUS_ARCH_EXTENDS
#endif /* OPLUS_ARCH_EXTENDS */

#include <linux/of.h>
#include <linux/notifier.h>

enum fsa_function {
	FSA_MIC_GND_SWAP,
	FSA_USBC_ORIENTATION_CC1,
	FSA_USBC_ORIENTATION_CC2,
	FSA_USBC_DISPLAYPORT_DISCONNECTED,
	#ifdef OPLUS_ARCH_EXTENDS
	/* Add DIO4480 support */
	FSA_CONNECT_LR,
	#endif /* OPLUS_ARCH_EXTENDS */
	FSA_EVENT_MAX,
};

#if IS_ENABLED(CONFIG_QCOM_FSA4480_I2C)
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event);
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node);
int fsa4480_unreg_notifier(struct notifier_block *nb,
			   struct device_node *node);

#ifdef OPLUS_ARCH_EXTENDS
/* Add DIO4480 support */
int fsa4480_get_chip_vendor(struct device_node *node);
int fsa4480_check_cross_conn(struct device_node *node);
#endif /* OPLUS_ARCH_EXTENDS */

#else
static inline int fsa4480_switch_event(struct device_node *node,
				       enum fsa_function event)
{
	return 0;
}

static inline int fsa4480_reg_notifier(struct notifier_block *nb,
				       struct device_node *node)
{
	return 0;
}

static inline int fsa4480_unreg_notifier(struct notifier_block *nb,
					 struct device_node *node)
{
	return 0;
}

#ifdef OPLUS_ARCH_EXTENDS
/* Add DIO4480 support */
static inline int fsa4480_get_chip_vendor(struct device_node *node)
{
    return 0;
}

static inline int fsa4480_check_cross_conn(struct device_node *node)
{
    return 0;
}
#endif /* OPLUS_ARCH_EXTENDS */
#endif /* CONFIG_QCOM_FSA4480_I2C */

#endif /* FSA4480_I2C_H */

