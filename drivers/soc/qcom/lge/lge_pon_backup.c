/*
 * driver/power/lge_pon_backup.c
 *
 * Copyright (C) 2019 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "PON_BACKUP: %s: " fmt, __func__

#include <linux/of.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/fs_struct.h>
#include <linux/platform_device.h>
#define MODULE_NAME "lge_pon_backup"

#define PON_DELAY_MS 10000
#define FTM_BLOCK_SIZE		4096
#define FTM_PON_BACKUP_OFFSET 227
#define FTM_PON_BACKUP_SIZE	488

#define PON_BACKUP_MAX_COUNT 10
#define PON_BACKUP_MAX_PMIC 5
#define PON_BACKUP_MAX_OCP 3
#define PON_BACKUP_MAX_REASON 3
#define FTM_PATH "/dev/block/bootdevice/by-name/ftm"

struct lge_pon_backup_data {
	struct delayed_work lge_pon_backup_dwork;
	u32 regulator_num[PON_BACKUP_MAX_PMIC * 3];
};

static const char * const qpnp_pon_reason_groups[] = {
	" PON=", " 0xC1=", " WR=", " 0xC3=", " ON=", " POFF=", " 0xC6=", " OFF="
};

static const char * const qpnp_pon_reasons[] = {
	"HR:", "SMPL:", "RTC:", "DC:", "USB:", "PON1:", "CBL:", "KPD:",
	"0:", "1:", "2:", "3:", "4:", "5:", "6:", "7:",
	"SOFT:", "PS_HOLD:", "PMIC_WD:", "GP1:", "GP2:", "KPD&RESIN:", "RESIN:", "KPD:",
	"0:", "1:", "2:", "3:", "4:", "5:", "6:", "7:",
	"0:", "1:", "2:", "3:", "4:", "5:", "WARM:", "PON:",
	"SOFT:", "PS_HOLD:", "PMIC_WD:", "GP1:", "GP2:", "KPD&RESIN:", "RESIN:", "KPD:",
	"0:", "1:", "2:", "3:", "4:", "5:", "6:", "7:",
	"0:", "1:", "RAW_XVDD:", "RAW_DVDD:", "IM_XVDD:", "S3_REST:", "FAULT:", "POFF:"
};

static const char * const qpnp_fault_reason_groups[] = {
	" FAULT1=", " FAULT2=", " S3=", " SOFT="
};

static const char * const qpnp_fault_reasons[] = {
	"GP_F0:", "GP_F1:", "GP_F2:", "GP_F3:", "MBG:", "OVLO:", "UVLO:", "AVDD:",
	"0:", "1:", "2:", "FAULT:", "PBS_WD:", "PBS_NACK:", "RESTART:", "OTST3:",
	"0:", "1:", "2:", "3:", "FAULT:", "PBS_WD:", "PBS_NACK:", "KPD&RESIN:",
	"SOFT:", "1:", "2:", "3:", "4:", "5:", "6:", "7:"
};

static const char * const qpnp_ocp_reasons[] = {
	" OCP=S", " OCP=LDO", " OCP=BOB"
};

typedef struct {
  uint32_t count;
  uint64_t pon_reg[PON_BACKUP_MAX_COUNT][PON_BACKUP_MAX_PMIC][PON_BACKUP_MAX_REASON];
} pon_backup_type;

static int lge_get_pon_backup(pon_backup_type *now_pon)
{
	struct file *fp;
	struct path root;
	int pon_count = 0, cnt = 0;

	mm_segment_t old_fs=get_fs();
	pr_info("lge_pon_backup_func start\n");
	set_fs(KERNEL_DS);

	memset(&root, 0, sizeof(struct path));
	task_lock(&init_task);
	get_fs_root(init_task.fs, &root);
	task_unlock(&init_task);
	fp = file_open_root(root.dentry, root.mnt, FTM_PATH, O_RDWR, 0);
	path_put(&root);

	if(IS_ERR(fp)){
		pr_err("Unable to open FTM (%d)\n", PTR_ERR(fp));
		pon_count = -1;
		goto err;
	}

	fp->f_pos = FTM_PON_BACKUP_OFFSET * FTM_BLOCK_SIZE; // LGFTM_PON_BACK_UP offset
	cnt = vfs_read(fp,(char*)now_pon, sizeof(*now_pon),&fp->f_pos);
	if(cnt != sizeof(*now_pon)){
		pr_err("Unable to read FTM (%d)\n", cnt);
		pon_count = -1;
		goto err;
	}

	if (now_pon->count >= PON_BACKUP_MAX_COUNT)
		pon_count = PON_BACKUP_MAX_COUNT;
	else
		pon_count = now_pon->count;

	pr_info("FTM Size %d, pon count %d", sizeof(*now_pon), now_pon->count);
	now_pon->count = 0;
	fp->f_pos = FTM_PON_BACKUP_OFFSET * FTM_BLOCK_SIZE; // LGFTM_PON_BACK_UP offset
	cnt = vfs_write(fp, (char*)now_pon, sizeof(*now_pon), &fp->f_pos);
	if(cnt != sizeof(*now_pon)) {
		pr_err("Unable to write FTM (%d)\n", cnt);
		goto err;
	}
err:
	if (!IS_ERR(fp))
		filp_close(fp, NULL);

	set_fs(old_fs);

	return pon_count;
}

#define PM_STATUS_MSG_LEN 300
static void lge_pon_backup_func(struct work_struct *w)
{
	struct lge_pon_backup_data *pon_backup = container_of(to_delayed_work(w),
		struct lge_pon_backup_data, lge_pon_backup_dwork);
	pon_backup_type now_pon = {0, };
	int i, j, count, index = 0, pre_index = -8, reg_index, ocp_index;
	char buf[PM_STATUS_MSG_LEN] = "";
	char num[10] = "";
	uint64_t pon_reg, fault_reg, ocp_reg;

	count = lge_get_pon_backup(&now_pon);
	if (count < -1)
		return;

	for (i = 0; i < count; i++)
		for (j = 0; j < PON_BACKUP_MAX_PMIC; j++) {
			pon_reg = now_pon.pon_reg[i][j][0];
			fault_reg = now_pon.pon_reg[i][j][1];
			ocp_reg = now_pon.pon_reg[i][j][2];
			pr_debug("Pon reg boot[%d] pm%d=0x%llx:0x%llx-0x%llx\n", i, j, pon_reg, fault_reg, ocp_reg);

			if (pon_reg == 0 && fault_reg == 0 && ocp_reg == 0)
				continue;

			while (pon_reg != 0 && index < sizeof(qpnp_pon_reasons)/sizeof(char*) ) {
				if (pon_reg & 1) {
					if (index/8 != pre_index/8)
						strlcat(buf, qpnp_pon_reason_groups[index/8], PM_STATUS_MSG_LEN);

					strlcat(buf, qpnp_pon_reasons[index], PM_STATUS_MSG_LEN);
					pre_index = index;
				}
				pon_reg = pon_reg >> 1;
				index++;
			}

			index = 0;
			pre_index = -8;
			while (fault_reg != 0 && index < sizeof(qpnp_fault_reasons)/sizeof(char*)) {
				if (fault_reg & 1) {
					if (index/8 != pre_index/8)
						strlcat(buf, qpnp_fault_reason_groups[index/8], PM_STATUS_MSG_LEN);

					strlcat(buf, qpnp_fault_reasons[index], PM_STATUS_MSG_LEN);
					pre_index = index;
				}
				fault_reg = fault_reg >> 1;
				index++;
			}

			index = 0;
			pre_index = 0;
			while (ocp_reg != 0) {
				if (ocp_reg & 1) {
					pre_index = 0;
					ocp_index = 0;
					for (reg_index = 0; reg_index < PON_BACKUP_MAX_OCP; reg_index++) {
						ocp_index += pon_backup->regulator_num[j * PON_BACKUP_MAX_OCP + reg_index];
						if (index < ocp_index)
							break;
						pre_index = ocp_index;
					}

					if (reg_index < PON_BACKUP_MAX_OCP) {
						strlcat(buf, qpnp_ocp_reasons[reg_index], PM_STATUS_MSG_LEN);
						snprintf(num, sizeof(num), "%d", index - pre_index + 1);
						strlcat(buf, num, PM_STATUS_MSG_LEN);
					}
				}
				ocp_reg = ocp_reg >> 1;
				index++;
			}

			pr_info("power on/off reason boot[%d] pm%d=%s\n", i, j, buf);
			snprintf(buf, sizeof(buf), "");
			index = 0;
			pre_index = -8;
		}

	return;
};

static int lge_pon_backup_probe(struct platform_device *pdev)
{
	struct lge_pon_backup_data *pon_backup;
	const struct device_node *node = pdev->dev.of_node;
	int rc = 0;

	pr_info("start Pon BackUp\n");
	pon_backup = kzalloc(sizeof(struct lge_pon_backup_data),
								GFP_KERNEL);
	if (!pon_backup) {
		pr_err("Fail to get alloc of pon\n");
		return -EIO;
	}

	rc = of_property_read_u32_array(node, "lge,regulator-table", pon_backup->regulator_num,
			PON_BACKUP_MAX_PMIC * 3);
	if (rc) {
		pr_err("Fail to get dt of regulator table.\n");
		return -EIO;
	}

	INIT_DELAYED_WORK(&pon_backup->lge_pon_backup_dwork,
		lge_pon_backup_func);
	schedule_delayed_work(&pon_backup->lge_pon_backup_dwork,
		msecs_to_jiffies(PON_DELAY_MS));

	return 0;
}

static struct of_device_id lge_pon_backup_match[] = {
	{ .compatible = "lge,pon-backup"},
	{}
};

static int lge_pon_backup_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver lge_pon_backup_driver = {
	.probe = lge_pon_backup_probe,
	.remove = lge_pon_backup_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lge_pon_backup_match,
	},
};

static int __init lge_pon_backup_init(void)
{
	return platform_driver_register(&lge_pon_backup_driver);
}

static void __exit lge_pon_backup_exit(void)
{
	platform_driver_unregister(&lge_pon_backup_driver);
}

module_init(lge_pon_backup_init);
module_exit(lge_pon_backup_exit);

MODULE_DESCRIPTION("LGE pon backup");
MODULE_LICENSE("GPL");