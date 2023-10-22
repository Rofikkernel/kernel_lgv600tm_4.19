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

#ifndef _LGE_BACKLIGHT_COVER_H_
#define _LGE_BACKLIGHT_COVER_H_

#include "../brightness/lge_cover_brightness_def.h"

extern int lge_backlight_cover_setup(struct sde_connector *c_conn,	struct drm_device *dev);
extern void lge_backlight_cover_destroy(struct sde_connector *c_conn);
extern int lge_backlight_cover_device_update_status(struct backlight_device *bd);
extern int br_to_offset_br_ds1(struct dsi_panel *panel, int br, int max_lvl, enum cover_br_type type);

#endif // _LGE_BACKLIGHT_COVER_H_

