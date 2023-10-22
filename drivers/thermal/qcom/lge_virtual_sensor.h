/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 */

#ifdef CONFIG_LGE_VIRTUAL_SENSOR

int lge_virtual_sensor_register(struct device *dev);

#else

static inline int lge_virtual_sensor_register(struct device *dev)
{
	return -ENODEV;
}

#endif /* CONFIG_LGE_VIRTUAL_SENSOR */
