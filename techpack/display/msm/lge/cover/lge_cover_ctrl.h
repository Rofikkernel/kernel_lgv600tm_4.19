/*
 * Copyright(c) 2019, LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LGE_COVER_CTRL_H_
#define _LGE_COVER_CTRL_H_
#include "../../dp/dp_gpio_hpd.h"
#include "../../dp/dp_hpd.h"
#include "../../dp/dp_display.h"

enum ds_state {
	DS_DISCONNECTED = 0,
	DS1_CONNECTED,
	DS1_HPD_CONNECTED,
	DS2_CONNECTED,
	DS2_HPD_CONNECTED,
};

enum ds_version {
	DS1 = 1,
	DS2,
};

enum ds_device_version {
	ONLY_DS1_SUPPORT = 1,
	ONLY_DS2_SUPPORT,
	DS1_DS2_SUPPORT
};

struct dp_hpd *dd_hpd_get(struct device *dev, struct dp_hpd_cb *cb);
void dd_hpd_put(struct dp_hpd *dd_hpd);
void dd_gpio_selection(int dd_hpd, int flip);
void dd_lattice_disable(void);
int lge_cover_ctrl_create_sysfs(struct device *panel_sysfs_dev);
int lge_cover_extcon_register(struct platform_device *pdev, struct lge_dp_display *lge_dp, int id);

#endif // _LGE_COVER_CTRL_H_

