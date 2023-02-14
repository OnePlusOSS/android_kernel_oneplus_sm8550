/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _TOUCHPANEL_PROC_H_
#define _TOUCHPANEL_PROC_H_

int init_touchpanel_proc(struct syna_tcm *tcm, struct platform_device *pdev);
void remove_touchpanel_proc(struct syna_tcm *tcm);

#endif /*_TOUCHPANEL_PROC_H_*/
