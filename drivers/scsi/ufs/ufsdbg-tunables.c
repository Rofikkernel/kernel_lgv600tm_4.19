/*
 * Copyright (c) 2017 -, Linux Foundation. All rights reserved.
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

struct ufsdbg_phy_tune {
	int de_emphasis_lvl;
	int pre_emphasis_lvl;
	int tx_drv_lvl;
	int rx_equ_gain;
};

struct ufsdbg_hba_tune {
	int gear_fix;
	int ufs_refclk_drv;

	void __iomem *tlmm_ref_clk_ctl;
};

struct ufsdbg_tunables {
	struct ufsdbg_phy_tune t_phy;
	struct ufsdbg_hba_tune t_hba;
};

#ifdef IMPORT_TO_UFSHCD
int ufsdbg_tunables_fix_gear(struct ufs_hba *hba, enum ufs_hs_gear_tag gear)
{
	int ret;
	struct ufs_pa_layer_attr new_pwr_info;
	bool scale_up = false;
	int old_gear = hba->pwr_info.gear_rx;

	if (hba->pwr_info.gear_rx == gear && hba->pwr_info.gear_tx == gear)
		return 0;

	if (gear > hba->pwr_info.gear_rx || gear > hba->pwr_info.gear_tx)
		scale_up = true;
	else
		scale_up = false;

	memcpy(&new_pwr_info, &hba->pwr_info, sizeof(struct ufs_pa_layer_attr));

	pm_runtime_get_sync(hba->dev);
	ufshcd_hold(hba, false);

	if (hba->clk_scaling.is_allowed) {
		cancel_work_sync(&hba->clk_scaling.suspend_work);
		cancel_work_sync(&hba->clk_scaling.resume_work);

		hba->clk_scaling.is_allowed = false;

		ufshcd_suspend_clkscaling(hba);
	}

	hba->ufs_stats.clk_hold.ctx = CLK_SCALE_WORK;
	ufshcd_hold_all(hba);

	ret = ufshcd_clock_scaling_prepare(hba);
	do {
		if (scale_up) {
			new_pwr_info.gear_rx ++;
			new_pwr_info.gear_tx ++;
		} else {
			new_pwr_info.gear_rx --;
			new_pwr_info.gear_tx --;
		}

		if (!scale_up) {
			ret = ufshcd_change_power_mode(hba, &new_pwr_info);
			if (ret) {
				dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_change_power_mode fail#1(%d)\n", ret);
				goto out;
			//ret = ufshcd_scale_gear(hba, false);
			} else {
				dev_info(hba->dev, "ufsdbg_fix_gear : change from gear-%d to gear-%d succeeded\n", old_gear, new_pwr_info.gear_rx);
				old_gear = new_pwr_info.gear_rx;
			}
		}
#if 0
		if (ufshcd_is_auto_hibern8_supported(hba)) {
			ret = ufshcd_uic_hibern8_enter(hba);
			if (ret) {
				dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_uic_hibern8_enter fail(%d)\n", ret);
				goto out;
			}
		}
#endif
		ret = ufshcd_scale_clks(hba, scale_up);
		if (ret) {
			dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_scale_clks fail(%d)\n", ret);
			goto out;
		} else {
			dev_info(hba->dev, "ufsdbg_fix_gear : scale_clks succeeded\n");
		}
#if 0
		if (ufshcd_is_auto_hibern8_supported(hba)) {
			ret = ufshcd_uic_hibern8_exit(hba);
			if (ret) {
				dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_uic_hibern8_exit fail(%d)\n", ret);
				goto out;
			}
		}
#endif
		if (scale_up) {
			ret = ufshcd_change_power_mode(hba, &new_pwr_info);
			if (ret) {
				dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_change_power_mode fail#2(%d)\n", ret);
				ufshcd_scale_clks(hba, false);
				goto out;
			} else {
				dev_info(hba->dev, "ufsdbg_fix_gear : change from gear-%d to gear-%d succeeded\n", old_gear, new_pwr_info.gear_rx);
				old_gear = new_pwr_info.gear_rx;
			}
		}

		if (!ret) {
			hba->clk_scaling.is_scaled_up = scale_up;
			if (scale_up)
				hba->clk_gating.delay_ms =
					hba->clk_gating.delay_ms_perf;
			else
				hba->clk_gating.delay_ms =
					hba->clk_gating.delay_ms_pwr_save;
		}

		msleep(1);

	} while (hba->pwr_info.gear_rx != gear || hba->pwr_info.gear_tx != gear);
out:
	ufshcd_clock_scaling_unprepare(hba);
	ufshcd_release_all(hba);
	hba->ufs_stats.clk_rel.ctx = CLK_SCALE_WORK;

	ufshcd_release(hba, false);
	pm_runtime_put_sync(hba->dev);

	return ret;
}

void ufsdbg_tunables_trigger_reset(struct ufs_hba *hba)
{
	unsigned long flags;

	pm_runtime_get_sync(hba->dev);
	ufshcd_hold(hba, false);

	spin_lock_irqsave(hba->host->host_lock, flags);
	/*
	* simulating a dummy error in order to "convince"
	* eh_work to actually reset the controller
	*/
	hba->saved_err |= INT_FATAL_ERRORS;
	//hba->silence_err_logs = true;
	schedule_work(&hba->eh_work);
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	flush_work(&hba->eh_work);

	ufshcd_release(hba, false);
	pm_runtime_put_sync(hba->dev);
}
// END IMPORT_TO_UFSHCD

#elif defined(IMPORT_TO_UFS_SYSFS)
extern int ufsdbg_tunables_fix_gear(struct ufs_hba *hba, enum ufs_hs_gear_tag gear);
static ssize_t gear_fix_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#if 0
	return sprintf(buf, "gear fix tunable is not working properly.\nIt's better to hard-coded debugging.\n\n");
#else
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;

	return sprintf(buf, "current expected gear = %d\n\n", tunable->t_hba.gear_fix);
#endif
}

static ssize_t gear_fix_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#if 0
	struct ufs_hba *hba = dev_get_drvdata(dev);
	dev_err(hba->dev, "%s:gear fix tunable is not working properly. It's better to hard-coded debugging.\n", __func__);
	return -ENOSYS;
#else
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	unsigned long value;

	if (!hba)
		return -EINVAL;

	if (kstrtoul(buf, 0, &value)) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (value == 0)
		return count;

	if (value > 0 && value <= 4) {
		tunable->t_hba.gear_fix = value;
	}
	else {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return count;
	}

	if (!ufsdbg_tunables_fix_gear(hba, tunable->t_hba.gear_fix))
		dev_info(hba->dev, "gear fix applied successfully\n");
	else
		dev_err(hba->dev, "gear fix failed\n");

	return count;
#endif
}

static ssize_t refclk_drv_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;

	return sprintf(buf, "current ufs_refclk_drv = %d\n\n", tunable->t_hba.ufs_refclk_drv);
}

static ssize_t refclk_drv_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	unsigned long value;

	if (!hba)
		return -EINVAL;

	if (kstrtoul(buf, 0, &value)) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (value>=0 && value<=7) {
		tunable->t_hba.ufs_refclk_drv = value;
	}
	else {
		return count;
	}

	return count;
}

static ssize_t de_emphasis_lvl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;

	return sprintf(buf, "current deemp_lvl = %d\n\n", tunable->t_phy.de_emphasis_lvl);
}

static ssize_t de_emphasis_lvl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	unsigned long value;

	if (!hba)
		return -EINVAL;

	if (kstrtoul(buf, 0, &value)) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (value>=0 && value<32) {
		tunable->t_phy.de_emphasis_lvl = value;
	}
	else {
		return count;
	}

	return count;
}

static ssize_t pre_emphasis_lvl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;

	return sprintf(buf, "current preemp_lvl = %d\n\n", tunable->t_phy.pre_emphasis_lvl);
}

static ssize_t pre_emphasis_lvl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	unsigned long value;

	if (!hba)
		return -EINVAL;

	if (kstrtoul(buf, 0, &value)) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (value>=0 && value<32) {
		tunable->t_phy.pre_emphasis_lvl = value;
	}
	else {
		return count;
	}

	return count;
}

static ssize_t tx_drv_lvl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;

	return sprintf(buf, "current tx_drv_lvl = %d\n\n", tunable->t_phy.tx_drv_lvl);
}

static ssize_t tx_drv_lvl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	unsigned long value;

	if (!hba)
		return -EINVAL;

	if (kstrtoul(buf, 0, &value)) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (value>=0 && value<32) {
		tunable->t_phy.tx_drv_lvl = value;
	}
	else {
		return count;
	}

	return count;
}

static ssize_t rx_equ_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;

	return sprintf(buf, "current rx_equ_gain = %d\n\n", tunable->t_phy.rx_equ_gain);
}

static ssize_t rx_equ_gain_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	unsigned long value;

	if (!hba)
		return -EINVAL;

	if (kstrtoul(buf, 0, &value)) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	// rx0,rx1 for each 0x00~0x0f
	if (value>=0 && value<=0xf) {
		tunable->t_phy.rx_equ_gain = value;
	}
	else {
		return count;
	}

	return count;
}

static ssize_t reset_control_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n%s\n",
		"echo 1 > /sys/devices/platform/soc/{dev_addr:ex.1d84000.ufshc}/tunables/reset_controller",
		"resets the UFS controller and restores its operational state");
}

extern void ufsdbg_tunables_trigger_reset(struct ufs_hba *hba);
static ssize_t reset_control_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	ufsdbg_tunables_trigger_reset(hba);

	return count;
}


static DEVICE_ATTR_RW(gear_fix);
static DEVICE_ATTR_RW(refclk_drv);
static DEVICE_ATTR_RW(de_emphasis_lvl);
static DEVICE_ATTR_RW(pre_emphasis_lvl);
static DEVICE_ATTR_RW(tx_drv_lvl);
static DEVICE_ATTR_RW(rx_equ_gain);
static DEVICE_ATTR_RW(reset_control);

static struct attribute *ufs_sysfs_ufshcd_tunables_attrs[] = {
	/* hba */
	&dev_attr_gear_fix.attr,
	&dev_attr_refclk_drv.attr,

	/* phy */
	&dev_attr_de_emphasis_lvl.attr,
	&dev_attr_pre_emphasis_lvl.attr,
	&dev_attr_tx_drv_lvl.attr,
	&dev_attr_rx_equ_gain.attr,

	/* ufs reset */
	&dev_attr_reset_control.attr,
	NULL
};

static const struct attribute_group ufs_sysfs_tunables_group = {
	.name = "tunables",
	.attrs = ufs_sysfs_ufshcd_tunables_attrs,
};
// END IMPORT_TO_UFS_SYSFS

#elif defined(IMPORT_TO_UFSQCOM)
#ifdef readl_poll_timeout
#undef readl_poll_timeout
#endif

//-----------------------------------------------------------------//
#if defined(CONFIG_ARCH_SM8150)
/* 80-PD867-2X. (HW register description documents) */
/* UFS	 Reference Clock Drive Strength : 0x03DB5000 */
#define TLMM_BASE								0x03000000
#define TLMM_SOUTH_REG_BASE						(TLMM_BASE + 0x00d00000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR			(TLMM_SOUTH_REG_BASE + 0x000b5000)
#define HWIO_TLMM_UFS1_REF_CLK_CTL_ADDR			(0x0) // not exists

	/*
		BIT[4:3] : UFS_REF_CLK_PULL
		BIT[2:0] : UFS_REF_CLK_HDRV
	*/
#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK			0x7

#elif defined(CONFIG_ARCH_KONA)
/* SM8250 */
/* 80-PL546-2X_C. (HW register description documents) */
/* 0x0F5B7000 TLMM_UFS_REF_CLK_CTL */
/* hwio_tlmm_ufs.h */
#define TLMM_BASE								0x0f000000
#define TLMM_SOUTH_REG_BASE						(TLMM_BASE + 0x00500000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR			(TLMM_SOUTH_REG_BASE + 0x000b7000)
#define HWIO_TLMM_UFS1_REF_CLK_CTL_ADDR			(0x0) //(0x0F9B9000) // not used

#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK			0x7

//-----------------------------------------------------------------//
#elif defined(CONFIG_ARCH_SDM845) || defined(CONFIG_ARCH_MSM8998)
/* 80-P6348-2X. (HW register description documents) */
/* UFS   Reference Clock Drive Strength : 0x03D9E000 */
/* UFS2 Reference CLock Drive Strength : 0x03DA0000 */

#define TLMM_BASE								0x03400000
#define TLMM_SOUTH_REG_BASE						(TLMM_BASE + 0x00900000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR			(TLMM_SOUTH_REG_BASE + 0x0009e000)
#define HWIO_TLMM_UFS1_REF_CLK_CTL_ADDR			(TLMM_SOUTH_REG_BASE + 0x000a0000)

/*
	BIT[4:3] : UFS_REF_CLK_PULL
	BIT[2:0] : UFS_REF_CLK_HDRV
*/
#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK			0x7
//-----------------------------------------------------------------//
#elif defined(CONFIG_ARCH_MSM8996)
/* UFS Reference Clock Drive Strength : 0x01146000 */
#define TLMM_BASE								0x01000000
#define TLMM_CSR_REG_BASE						(TLMM_BASE + 0x00010000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR			(TLMM_CSR_REG_BASE + 0x00136000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK			0x7
//-----------------------------------------------------------------//
#else
#define TLMM_BASE
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR
#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK
#endif
//-----------------------------------------------------------------//

extern void ufsdbg_tunables_phy_set_tunables(struct phy *generic_phy, void *tunables);

/* should be called after ufs qcom phy being initialized */
static int ufsdbg_tunables_init(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufsdbg_tunables *tunables = kzalloc(sizeof(struct ufsdbg_tunables), GFP_KERNEL);
	if(!tunables)
		return -ENOMEM;

	tunables->t_hba.gear_fix 		= -1;
	tunables->t_hba.ufs_refclk_drv 	= -1;
	tunables->t_phy.de_emphasis_lvl	= -1;
	tunables->t_phy.pre_emphasis_lvl	= -1;
	tunables->t_phy.tx_drv_lvl 		= -1;
	tunables->t_phy.rx_equ_gain 	= -1;

	dev_info(hba->dev, "%s : phy(%s)\n", __func__, dev_name(&host->generic_phy->dev));

	tunables->t_hba.tlmm_ref_clk_ctl = ioremap(HWIO_TLMM_UFS_REF_CLK_CTL_ADDR, 4);

	dev_info(hba->dev, "     tlmm_ref_clk_ctl(%p)\n", (void*)tunables->t_hba.tlmm_ref_clk_ctl);

	hba->ufsdbg_tunables = tunables;
	ufsdbg_tunables_phy_set_tunables(host->generic_phy, (void*)tunables);

	return 0;
}

static int ufsdbg_tunables_refclk_drv_apply(struct ufs_hba *hba)
{
	struct ufsdbg_tunables *tunables = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	// ref clk drive strength
	if (tunables->t_hba.ufs_refclk_drv>=0) {
		u32 read_clk_drv = readl_relaxed(tunables->t_hba.tlmm_ref_clk_ctl);
		u32 write_clk_drv = ((read_clk_drv & (unsigned int)(~HWIO_TLMM_UFS_REF_CLK_CTL_MSK)) | ((unsigned int)(tunables->t_hba.ufs_refclk_drv & HWIO_TLMM_UFS_REF_CLK_CTL_MSK)));
		dev_info(hba->dev, "read_clk_drv(0x%x), write_clk_drv(0x%x)\n", read_clk_drv, write_clk_drv);
		writel_relaxed( write_clk_drv, tunables->t_hba.tlmm_ref_clk_ctl);
		mb();
		read_clk_drv = readl_relaxed(tunables->t_hba.tlmm_ref_clk_ctl);
		dev_info(hba->dev, "UFS_REF_CLK_PULL(0x%x), UFS_REF_CLK_HDRV(0x%x)\n", read_clk_drv>>3, read_clk_drv & HWIO_TLMM_UFS_REF_CLK_CTL_MSK);
	} else {
		u32 read_clk_drv = readl_relaxed(tunables->t_hba.tlmm_ref_clk_ctl);
		dev_info(hba->dev, "UFS_REF_CLK_PULL(0x%x), UFS_REF_CLK_HDRV(0x%x)\n", read_clk_drv>>3, read_clk_drv & HWIO_TLMM_UFS_REF_CLK_CTL_MSK);
	}
	return 0;
}

static int ufsdbg_get_fix_gear(struct ufs_hba *hba)
{
	struct ufsdbg_tunables *tunables = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	return tunables->t_hba.gear_fix;
}

// END IMPORT_TO_UFSQCOM

#elif defined(IMPORT_TO_UFSMPHY)

//-----------------------------------------------------------------//
#if defined(CONFIG_ARCH_SM8150) || defined(CONFIG_ARCH_KONA)
/* SM8150 : 80-PD867-5D-B. (PHY tuning guideline) & 80-PD867-2X. (HW register description documents) */
/* SM8250(KONA) : 80-PL546-5D_A. & 80-PL546-2X_C. */
/* override or add to phy-qcom-ufs-qmp-v4.h */
/* UFS PHY TX registers */
/*
	5.3 Tx emphasis tuning parameters

	5.3.1 Precursor adjustment
	UFS_MEM_MPHY_UFS_QSERDES_TX0_PRE_EMPH: 0x01D87508
	UFS_MEM_MPHY_UFS_QSERDES_TX1_PRE_EMPH: 0x01D87908

	5.3.2 Postcursor adjustment
	UFS_MEM_MPHY_UFS_QSERDES_TX0_TX_EMP_POST1_LVL (0x01D8740C)
	UFS_MEM_MPHY_UFS_QSERDES_TX1_TX_EMP_POST1_LVL (0x01D8780C)
*/
#define QSERDES_TX0_PRE_EMPH	TX_OFF(0, 0x108)
#define QSERDES_TX1_PRE_EMPH	TX_OFF(1, 0x108)

#define QSERDES_TX0_TX_EMP_POST1_LVL	TX_OFF(0, 0xc)
#define QSERDES_TX1_TX_EMP_POST1_LVL	TX_OFF(1, 0xc)

/*
	5.2 Tx swing tuning parameters
	UFS_MEM_MPHY_UFS_QSERDES_TX0_TX_DRV_LVL (0x01D87414)
	UFS_MEM_MPHY_UFS_QSERDES_TX1_TX_DRV_LVL (0x01D87814)
*/
#define QSERDES_TX0_TX_DRV_LVL			TX_OFF(0, 0x14)
#define QSERDES_TX1_TX_DRV_LVL			TX_OFF(1, 0x14)

// ref. RX EQU default values
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2, 0x06),
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3, 0x04),
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4, 0x1D),

//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL2, 0x06),
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL3, 0x04),
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4, 0x1D),

//-----------------------------------------------------------------//
/* 80-P2169-5D. (PHY tuning guideline) & 80-P6348-2X. (HW register description documents) */
#elif defined(CONFIG_ARCH_SDM845) || defined(CONFIG_ARCH_MSM8998)
/* override or add to phy-qcom-ufs-qmp-v3.h */
/* UFS PHY TX registers */
#define QSERDES_TX0_TX_EMP_POST1_LVL	TX_OFF(0, 0xc)
#define QSERDES_TX1_TX_EMP_POST1_LVL	TX_OFF(1, 0xc)

#define QSERDES_TX0_TX_DRV_LVL			TX_OFF(0, 0x1c)
#define QSERDES_TX1_TX_DRV_LVL			TX_OFF(1, 0x1c)
// ref. RX EQU default values
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2, 0x06),	// BIT[4]==0 -> default to override CNTRL4 register
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4, 0x1D),	// BIT[4]==1 -> override BIT[0:3] : 1dB
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL2, 0x06),	// BIT[4]==0 -> default to override CNTRL4 register
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4, 0x1D),	// BIT[4]==1 -> override BIT[0:3] : 1dB

//-----------------------------------------------------------------//
#elif defined(CONFIG_ARCH_MSM8996)
/* override or add to phy-qcom-ufs-qmp-14nm.h */

//-----------------------------------------------------------------//
#else

#endif
//-----------------------------------------------------------------//
void ufsdbg_tunables_phy_set_tunables(struct phy *generic_phy, void *tunables)
{
	struct ufs_qcom_phy *ufs_qcom_phy = get_ufs_qcom_phy(generic_phy);
	ufs_qcom_phy->ufsdbg_tunables = tunables;
}
static int ufsdbg_tunables_mphy_apply(struct ufs_qcom_phy *ufs_qcom_phy)
{
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)ufs_qcom_phy->ufsdbg_tunables;

	dev_info(ufs_qcom_phy->dev, "%s : ufs_qcom_phy(%p)/mmio(%p)\n", __func__, ufs_qcom_phy, ufs_qcom_phy->mmio);
	// pre-emphasis_lvl
	if (tunable->t_phy.pre_emphasis_lvl >= 0) {
		dev_info(ufs_qcom_phy->dev, "pre_emphasis_lvl(%u)\n",tunable->t_phy.pre_emphasis_lvl);
		writel_relaxed(tunable->t_phy.pre_emphasis_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX0_PRE_EMPH);
		writel_relaxed(tunable->t_phy.pre_emphasis_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX1_PRE_EMPH);
		mb();
		dev_info(ufs_qcom_phy->dev, "written pre_emph lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_PRE_EMPH));
		dev_info(ufs_qcom_phy->dev, "written pre_emph lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_PRE_EMPH));
	} else {
		dev_info(ufs_qcom_phy->dev, "current pre_emph lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_PRE_EMPH));
		dev_info(ufs_qcom_phy->dev, "current pre_emph lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_PRE_EMPH));
	}

	// de-emphasis_lvl
	if (tunable->t_phy.de_emphasis_lvl >= 0) {
		dev_info(ufs_qcom_phy->dev, "de_emphasis_lvl(%u)\n",tunable->t_phy.de_emphasis_lvl);
		writel_relaxed(tunable->t_phy.de_emphasis_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX0_TX_EMP_POST1_LVL);
		writel_relaxed(tunable->t_phy.de_emphasis_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX1_TX_EMP_POST1_LVL);
		mb();
		dev_info(ufs_qcom_phy->dev, "written de_emph lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_TX_EMP_POST1_LVL));
		dev_info(ufs_qcom_phy->dev, "written de_emph lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_TX_EMP_POST1_LVL));
	} else {
		dev_info(ufs_qcom_phy->dev, "current de_emph lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_TX_EMP_POST1_LVL));
		dev_info(ufs_qcom_phy->dev, "current de_emph lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_TX_EMP_POST1_LVL));
	}

	// tx_drv_lvl
	if (tunable->t_phy.tx_drv_lvl >= 0) {
		dev_info(ufs_qcom_phy->dev, "tx_drv_lvl(%u)\n", tunable->t_phy.tx_drv_lvl);
		writel_relaxed(tunable->t_phy.tx_drv_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX0_TX_DRV_LVL);
		writel_relaxed(tunable->t_phy.tx_drv_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX1_TX_DRV_LVL);
		mb();
		dev_info(ufs_qcom_phy->dev, "current large_amp(0x%x)\n" , readl_relaxed(ufs_qcom_phy->mmio + UFS_PHY_TX_LARGE_AMP_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "        small_amp(0x%x)\n", readl_relaxed(ufs_qcom_phy->mmio + UFS_PHY_TX_SMALL_AMP_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "written tx_drv_lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_TX_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "written tx_drv_lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_TX_DRV_LVL));
	} else {
		dev_info(ufs_qcom_phy->dev, "current large_amp(0x%x)\n" , readl_relaxed(ufs_qcom_phy->mmio + UFS_PHY_TX_LARGE_AMP_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "        small_amp(0x%x)\n", readl_relaxed(ufs_qcom_phy->mmio + UFS_PHY_TX_SMALL_AMP_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "current tx_drv_lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_TX_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "current tx_drv_lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_TX_DRV_LVL));
	}

	// rx equ gain : QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4, QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4
	if (tunable->t_phy.rx_equ_gain >= 0) {
		dev_info(ufs_qcom_phy->dev, "rx_equ_gain(%u)\n", tunable->t_phy.rx_equ_gain);

#if defined(CONFIG_ARCH_SM8150) || defined(CONFIG_ARCH_KONA)
		/*
		Description: Equalizer gain adaptation must be turned off to apply manual settings. Use the
		following steps:
		1. Set RXn_EQU_ADAPTOR_CNTRL2 [6:4] to 0b101.
		2. Set RXn_EQU_ADAPTOR_CNTRL3 [2:0] to 0b100.
		3. Set RXn_EQU_ADAPTOR_CNTRL4 [6:5] to 0b11.
		4. Use RXn_EQU_ADAPTOR_CNTRL4 [3:0] to set EQ2 peaking.
		*/
		// RX0
		writel_relaxed( (5<<4), ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2);
		writel_relaxed( 4, ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3);
		writel_relaxed( (3<<5), ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
		writel_relaxed(tunable->t_phy.rx_equ_gain, ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
		mb();

		// RX1
		writel_relaxed( (5<<4), ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL2);
		writel_relaxed( 4, ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL3);
		writel_relaxed( (3<<5), ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4);
		writel_relaxed(tunable->t_phy.rx_equ_gain, ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4);
		mb();

#elif defined(CONFIG_ARCH_SDM845) || defined(CONFIG_ARCH_MSM8998)
		writel_relaxed(tunable->t_phy.rx_equ_gain, ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
		writel_relaxed(tunable->t_phy.rx_equ_gain, ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4);
		mb();
#endif
		dev_info(ufs_qcom_phy->dev, "written rx_equ_gain:(rx0:0x%x, rx1:0x%x)\n",
			readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4), readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4));
	} else {
		dev_info(ufs_qcom_phy->dev, "current rx_equ_gain - CTL2 : (RX0,RX1)\n");
		dev_info(ufs_qcom_phy->dev, "                            RX0 : 0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2));
		dev_info(ufs_qcom_phy->dev, "                            RX1 : 0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL2));
		dev_info(ufs_qcom_phy->dev, "                      CTL4 : (RX0,RX1)\n");
		dev_info(ufs_qcom_phy->dev, "                            RX0 : 0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4));
		dev_info(ufs_qcom_phy->dev, "                            RX1 : 0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4));
	}
	return 0;
}

#endif
