// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 */

#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include "lge_virtual_sensor.h"

#ifdef CONFIG_LGE_ONE_BINARY_SKU
#include <soc/qcom/lge/board_lge.h>
#endif

static const struct virtual_sensor_data sub6_lge_virtual_sensors[] = {
	/* 0.167*skin + 0.617*quiet + 5.444 */
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"skin-therm-usr", "quiet-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {167, 617},
		.avg_offset = 5444000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
};

static const struct virtual_sensor_data mmw_lge_virtual_sensors[] = {
	/* 0.159*skin + 0.645*quiet + 4.2 */
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"skin-therm-usr", "quiet-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {159, 645},
		.avg_offset = 4200000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},

	/* qtm0(north) : -1.314*quiet-therm + 2.096*qtm-n-therm + 3.914 */
	{
		.virt_zone_name = "qtm-0-vts-therm",
		.num_sensors = 2,
		.sensor_names = {"quiet-therm-usr", "qtm-n-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {-1314, 2096},
		.avg_offset = 3914000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* qtm1(west) : 0.71*quiet-therm + 0.139*qtm-w-therm + 3.266 */
	{
		.virt_zone_name = "qtm-1-vts-therm",
		.num_sensors = 2,
		.sensor_names = {"quiet-therm-usr", "qtm-w-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {710, 139},
		.avg_offset = 3266000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* qtm2(east) : 2.806*quiet-therm - 2.036*qtm-e-therm + 7.043 */
	{
		.virt_zone_name = "qtm-2-vts-therm",
		.num_sensors = 2,
		.sensor_names = {"quiet-therm-usr", "qtm-e-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {2806, -2036},
		.avg_offset = 7043000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* qtm-modem(smr) : -1.24*quiet-therm + 2.0*qtm-e-therm + 7.38 */
	{
		.virt_zone_name = "qtm-modem-vts-therm",
		.num_sensors = 2,
		.sensor_names = {"quiet-therm-usr", "qtm-e-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {-124, 200},
		.avg_offset = 738000,
		.avg_denominator = 100,
		.logic = VIRT_WEIGHTED_AVG,
	},
};


int lge_virtual_sensor_register(struct device *dev)
{
	int sens_ct = 0;
	static int idx;
	struct thermal_zone_device *tz;

#ifdef CONFIG_LGE_ONE_BINARY_SKU
	enum lge_sku_carrier_type sku_carrier = HW_SKU_MAX;
	sku_carrier = lge_get_sku_carrier();

	pr_info("operator is %s\n", sku_carrier == HW_SKU_NA_CDMA_VZW ? "VZW" : "Non-VZW");

	if (sku_carrier == HW_SKU_NA_CDMA_VZW){
		sens_ct = ARRAY_SIZE(mmw_lge_virtual_sensors);
		for (; idx < sens_ct; idx++) {
			tz = devm_thermal_of_virtual_sensor_register(dev,
					&mmw_lge_virtual_sensors[idx]);
			if (IS_ERR(tz))
				pr_err("%s: sensor:%s register error:%ld\n", __func__,
						mmw_lge_virtual_sensors[idx].virt_zone_name, PTR_ERR(tz));
			else
				pr_err("%s: sensor:%s register success\n", __func__,
					mmw_lge_virtual_sensors[idx].virt_zone_name);
		}
	} else {
		sens_ct = ARRAY_SIZE(sub6_lge_virtual_sensors);
		for (; idx < sens_ct; idx++) {
			tz = devm_thermal_of_virtual_sensor_register(dev,
					&sub6_lge_virtual_sensors[idx]);
			if (IS_ERR(tz))
				pr_err("%s: sensor:%s register error:%ld\n", __func__,
						sub6_lge_virtual_sensors[idx].virt_zone_name, PTR_ERR(tz));
			else
				pr_err("%s: sensor:%s register success\n", __func__,
						sub6_lge_virtual_sensors[idx].virt_zone_name);
		}
	}
#else
	sens_ct = ARRAY_SIZE(sub6_lge_virtual_sensors);
	for (; idx < sens_ct; idx++) {
		tz = devm_thermal_of_virtual_sensor_register(dev,
				&sub6_lge_virtual_sensors[idx]);
		if (IS_ERR(tz))
			pr_err("%s: sensor:%s register error:%ld\n", __func__,
					sub6_lge_virtual_sensors[idx].virt_zone_name, PTR_ERR(tz));
		else
			pr_err("%s: sensor:%s register success\n", __func__,
				sub6_lge_virtual_sensors[idx].virt_zone_name);
	}
#endif	//CONFIG_LGE_ONE_BINARY_SKU

	return 0;
}
