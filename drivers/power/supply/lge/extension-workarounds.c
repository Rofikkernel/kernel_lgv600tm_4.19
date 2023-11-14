/*
 * LGE workaround code list
 * - JUDY-12620     : Avoiding MBG fault on SBU pin
 * - JUDY-7463      : Detection of Standard HVDCP2
 * - JUDY-7843      : Rerun apsd for dcp charger
 * - TIME-3624      : Rerun apsd for abnormal sdp
 * - JUDY-7303      : Charging without CC
 * - MCCCC-11191    : Rerun apsd for unknown charger
 * - JUDY-6149      : Support for weak battery pack
 * - JUDY-7649      : Resuming Suspended USBIN
 * - JUDY-8481      : Charging with Rd-open charger.
 * - CAYMAN-2473    : Clear power role when charger is unpluged
 * - MCCCC-17560    : Clear DC Reverse Voltage status
 * - MCCCC-18127    : Rerun DC AICL
 * - MCCCC-16139    : Recovery vashdn during wireless charging
 * - MCCCC-16161    : Retry to enable vconn on vconn-oc
 * - MCCCC-19045    : Retry pd check
 * - TIME-3963      : Avoid Inrush current for USB Compliance test
 * - MCCCC-17740    : avoid over voltage by abnormal charger
 * - TIME-3557      : Fake USB type to SDP on Factory cable.
 * - TIME-3721      : Disable CP charging in battery fake mode.
 * - TIME-5647      : Disable hiccup of otg in compliance mode
 * - TIME-6306      : Control Vbus2 regulator
 * - TIME-2558      : Compensate charging power on CP QC3.0
 *                    input probation for cp setup.
 * - MSP-14434      : Recover & Fake CC status in factory mode.
 * - TIME-3625      : Retry APSD in factory mode.
 * - TIME-4164      : Faster try APSD for HVDCP test
 * - TIME-4164      : Faster try QC 3.0 for 2nd charger test
 * - MCISSUE-12913  : Charging for mcdodo
 * - TIME-3887      : Compensate pps ta output error.
 * - TIME-4626      : abnormal operation during PPS TA CP Charing.
 */

#define pr_fmt(fmt) "WA: %s: " fmt, __func__
#define pr_wa(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#define pr_dbg_wa(fmt, ...) pr_debug(fmt, ##__VA_ARGS__)

#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>

#include "../qcom/smb5-reg.h"
#include "../qcom/smb5-lib.h"
#ifdef CONFIG_LGE_PM_VENEER_PSY
#include "veneer-primitives.h"
#endif
#ifdef CONFIG_LGE_USB_SBU_SWITCH
#include <linux/usb/lge_sbu_switch.h>
#endif

////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Helper functions
////////////////////////////////////////////////////////////////////////////

extern void do_pd_hard_reset(void);

static struct smb_charger* wa_helper_chg(void)
{
	// getting smb_charger from air
	struct power_supply*	psy
		= power_supply_get_by_name("battery");
	struct smb_charger*	chg
		= psy ? power_supply_get_drvdata(psy) : NULL;
	if (psy)
		power_supply_put(psy);

	return chg;
}

static bool wa_command_apsd(/*@Nonnull*/ struct smb_charger* chg)
{
	bool	ret = false;
	int rc;

	rc = smblib_masked_write(chg, CMD_APSD_REG,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0) {
		pr_wa("Couldn't re-run APSD rc=%d\n", rc);
		goto failed;
	}
	ret = true;

failed:
	return ret;
}

bool wa_command_icl_override(/*@Nonnull*/ struct smb_charger* chg)
{
	if (smblib_masked_write(chg, USBIN_CMD_ICL_OVERRIDE_REG,
			ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT) < 0) {
		pr_wa("Couldn't icl override\n");
		return false;
	}

	return true;
}

static void wa_get_pmic_dump_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_get_pmic_dump_work);
	struct smb_charger *chg = ext_chg->chg;
	union power_supply_propval debug  = {-1, };

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	power_supply_set_property(chg->batt_psy,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &debug);
}

void wa_get_pmic_dump(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg;

	if (!chg) {
		chg = wa_helper_chg();
		if (!chg) {
			pr_wa("'chg' is not ready\n");
			return;
		}
	}
	ext_chg = chg->ext_chg;

	schedule_work(&ext_chg->wa_get_pmic_dump_work);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Avoiding MBG fault on SBU pin
////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_LGE_USB_SBU_SWITCH
// Rather than accessing pointer directly, Referring it as a singleton instance
static struct lge_sbu_switch_instance* wa_avoiding_mbg_fault_singleton(void)
{
	static struct lge_sbu_switch_desc 	wa_amf_description = {
		.flags  = LGE_SBU_SWITCH_FLAG_SBU_AUX
			| LGE_SBU_SWITCH_FLAG_SBU_USBID
			| LGE_SBU_SWITCH_FLAG_SBU_FACTORY_ID,
	};
	static struct lge_sbu_switch_instance*	wa_amf_instance;
	static DEFINE_MUTEX(wa_amf_mutex);
	struct smb_charger* chg;

	if (IS_ERR_OR_NULL(wa_amf_instance)) {
		mutex_lock(&wa_amf_mutex);
		chg = wa_helper_chg();
		if (IS_ERR_OR_NULL(wa_amf_instance) && chg) {
			wa_amf_instance
				= devm_lge_sbu_switch_instance_register(chg->dev, &wa_amf_description);
			if (IS_ERR_OR_NULL(wa_amf_instance))
				devm_lge_sbu_switch_instance_unregister(chg->dev, wa_amf_instance);
		}
		mutex_unlock(&wa_amf_mutex);
	}

	return IS_ERR_OR_NULL(wa_amf_instance) ? NULL : wa_amf_instance;
}

bool wa_avoiding_mbg_fault_uart(bool enable)
{
	// Preparing instance and checking validation of it.
	struct lge_sbu_switch_instance* instance
		= wa_avoiding_mbg_fault_singleton();
	if (!instance)
		return false;

	if (enable) {
		if (lge_sbu_switch_get_current_flag(instance) != LGE_SBU_SWITCH_FLAG_SBU_AUX)
			lge_sbu_switch_get(instance, LGE_SBU_SWITCH_FLAG_SBU_AUX);
	}
	else
		lge_sbu_switch_put(instance, LGE_SBU_SWITCH_FLAG_SBU_AUX);

	return true;
}

bool wa_avoiding_mbg_fault_usbid(bool enable)
{
	// Preparing instance and checking validation of it.
	struct lge_sbu_switch_instance* instance
		= wa_avoiding_mbg_fault_singleton();
	if (!instance)
		return false;

	if (enable)
		lge_sbu_switch_get(instance, LGE_SBU_SWITCH_FLAG_SBU_FACTORY_ID);
	else
		lge_sbu_switch_put(instance, LGE_SBU_SWITCH_FLAG_SBU_FACTORY_ID);

	return true;
}
#else
bool wa_avoiding_mbg_fault_uart(bool enable) { return false; };
bool wa_avoiding_mbg_fault_usbid(bool enable) { return false; };
#endif


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Detection of Standard HVDCP2
////////////////////////////////////////////////////////////////////////////

#define DSH_VOLTAGE_THRESHOLD  7000
#define HVDCP_MAX_COUNT        3
static void wa_detect_standard_hvdcp_main(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_detect_standard_hvdcp_work);
	struct smb_charger *chg = ext_chg->chg;
	union power_supply_propval	val = {0, };
	int rc, usb_vnow, count;

	ext_chg->wa_detect_standard_hvdcp_done = true;

	for (count = 0; count < HVDCP_MAX_COUNT; count++) {
		rc = smblib_dp_dm(chg, POWER_SUPPLY_DP_DM_FORCE_9V);
		if (rc < 0) {
			pr_wa("Couldn't force 9V rc=%d\n", rc);
			return;
		}

		msleep(200);
		usb_vnow = !smblib_get_prop_usb_voltage_now(chg, &val) ? val.intval/1000 : -1;
		if ( usb_vnow >= DSH_VOLTAGE_THRESHOLD) {
			ext_chg->wa_is_standard_hvdcp = true;
		}

		pr_wa("Check standard hvdcp. %d mV try %d\n", usb_vnow, count);
		rc = smblib_dp_dm(chg, POWER_SUPPLY_DP_DM_FORCE_5V);
		if (rc < 0) {
			pr_wa("Couldn't force 5v rc=%d\n", rc);
			return;
		}
		if (ext_chg->wa_is_standard_hvdcp)
			return;
		msleep(50);
	}
}

void wa_detect_standard_hvdcp_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	u8 stat;
	int rc;

	if (!ext_chg->enable_detect_standard_hvdcp)
		return;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read APSD_STATUS_REG rc=%d\n", rc);
		return;
	}

	pr_dbg_wa("apsd_status 0x%x, type %d\n", stat, chg->real_charger_type);
	if ((stat & QC_AUTH_DONE_STATUS_BIT)
			&& chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP
			&& !ext_chg->wa_detect_standard_hvdcp_done) {
		schedule_work(&ext_chg->wa_detect_standard_hvdcp_work);
	}
}

void wa_detect_standard_hvdcp_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_detect_standard_hvdcp)
		return;

	ext_chg->wa_is_standard_hvdcp = false;
	ext_chg->wa_detect_standard_hvdcp_done = false;
}

bool wa_detect_standard_hvdcp_check(void)
{
	struct smb_charger* chg = wa_helper_chg();
	struct ext_smb_charger *ext_chg;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return false;
	}

	ext_chg = chg->ext_chg;
	if (!ext_chg->enable_detect_standard_hvdcp)
		return false;

	return ext_chg->wa_is_standard_hvdcp;
}

void wa_detect_standard_hvdcp_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_detect_standard_hvdcp =
		of_property_read_bool(dnode, "lge,enable-detect-standard-hvdcp");

	if (!ext_chg->enable_detect_standard_hvdcp)
		return;

	ext_chg->wa_is_standard_hvdcp = false;
	ext_chg->wa_detect_standard_hvdcp_done = false;

	INIT_WORK(&ext_chg->wa_detect_standard_hvdcp_work,
			wa_detect_standard_hvdcp_main);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Rerun apsd for dcp charger
////////////////////////////////////////////////////////////////////////////

static void wa_rerun_apsd_for_dcp_main(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_rerun_apsd_for_dcp_dwork.work);
	struct smb_charger *chg = ext_chg->chg;

	if (chg->pd_active ||!ext_chg->wa_rerun_apsd_done) {
		pr_wa("stop apsd done. apsd(%d), pd(%d)\n",
				ext_chg->wa_rerun_apsd_done, chg->pd_active);
		return;
	}

	pr_wa("Rerun apsd\n");
	wa_command_apsd(chg);
}

void wa_rerun_apsd_for_dcp_triger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = { 0, };
	bool usb_type_dcp = chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP;
	bool usb_vbus_high
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val) ? !!val.intval : false;
	u8 stat;
	int rc;

	if (!ext_chg->enable_rerun_apsd_dcp)
		return;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read APSD_STATUS_REG rc=%d\n", rc);
		return;
	}

	pr_dbg_wa("legacy(%d), done(%d), TO(%d), DCP(%d), Vbus(%d)\n",
		chg->typec_legacy, ext_chg->wa_rerun_apsd_done, stat, usb_type_dcp, usb_vbus_high);

	if (chg->typec_legacy && !ext_chg->wa_rerun_apsd_done
			&& (stat & HVDCP_CHECK_TIMEOUT_BIT) && usb_type_dcp && usb_vbus_high) {
		ext_chg->wa_rerun_apsd_done = true;
		schedule_delayed_work(&ext_chg->wa_rerun_apsd_for_dcp_dwork,
			round_jiffies_relative(msecs_to_jiffies(APSD_RERUN_DELAY_MS)));
	}
}

void wa_rerun_apsd_for_dcp_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_rerun_apsd_dcp)
		return;

	ext_chg->wa_rerun_apsd_done = false;
}

void wa_rerun_apsd_for_dcp_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_rerun_apsd_dcp =
		of_property_read_bool(dnode, "lge,enable-rerun-apsd-dcp");

	if (!ext_chg->enable_rerun_apsd_dcp)
		return;

	ext_chg->wa_rerun_apsd_done = false;
	INIT_DELAYED_WORK(&ext_chg->wa_rerun_apsd_for_dcp_dwork,
			wa_rerun_apsd_for_dcp_main);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Rerun apsd for abnormal sdp
////////////////////////////////////////////////////////////////////////////

static void wa_rerun_apsd_for_sdp_main(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_rerun_apsd_for_sdp_dwork.work);
	struct smb_charger *chg = ext_chg->chg;

	if (!ext_chg->wa_rerun_apsd_done_with_sdp || ext_chg->is_usb_configured) {
		pr_wa("stop apsd done. apsd(%d), configured(%d)\n",
				ext_chg->wa_rerun_apsd_done_with_sdp, ext_chg->is_usb_configured);
		return;
	}

	pr_wa("Rerun apsd\n");
	wa_command_apsd(chg);
}

void wa_rerun_apsd_for_sdp_triger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = { 0, };
	bool usb_type_sdp = chg->real_charger_type == POWER_SUPPLY_TYPE_USB;
	bool usb_vbus_high
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val) ? !!val.intval : false;

	if (!ext_chg->enable_rerun_apsd_sdp)
		return;

	pr_dbg_wa("done(%d), SDP(%d), Vbus(%d)\n",
		ext_chg->wa_rerun_apsd_done_with_sdp, usb_type_sdp, usb_vbus_high);

	if (!ext_chg->wa_rerun_apsd_done_with_sdp
			&& usb_type_sdp && usb_vbus_high) {
		ext_chg->wa_rerun_apsd_done_with_sdp = true;
		schedule_delayed_work(&ext_chg->wa_rerun_apsd_for_sdp_dwork,
			round_jiffies_relative(msecs_to_jiffies(APSD_RERUN_DELAY_MS)));
	} else if (ext_chg->wa_rerun_apsd_done_with_sdp
			&& is_client_vote_enabled(chg->usb_icl_votable, USB_PSY_VOTER)
			&& !usb_type_sdp && usb_vbus_high) {
		vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	}
}

void wa_rerun_apsd_for_sdp_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_rerun_apsd_sdp)
		return;

	ext_chg->wa_rerun_apsd_done_with_sdp = false;
}

void wa_rerun_apsd_for_sdp_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_rerun_apsd_sdp =
		of_property_read_bool(dnode, "lge,enable-rerun-apsd-sdp");

	if (!ext_chg->enable_rerun_apsd_sdp)
		return;

	ext_chg->wa_rerun_apsd_done_with_sdp = false;
	INIT_DELAYED_WORK(&ext_chg->wa_rerun_apsd_for_sdp_dwork,
			wa_rerun_apsd_for_sdp_main);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Charging without CC
////////////////////////////////////////////////////////////////////////////

/* CWC has two works for APSD and HVDCP, and this implementation handles the
 * works independently with different delay.
 * but you can see that retrying HVDCP detection work is depends on rerunning
 * APSD. i.e, APSD work derives HVDCP work.
 */
#define CWC_DELAY_MS  1000
static bool wa_charging_without_cc_required(struct smb_charger *chg)
{
	union power_supply_propval val = { 0, };
	bool pd_hard_reset, usb_vbus_high, typec_mode_none,	src_mode, wa_required;
	struct ext_smb_charger *ext_chg;
	char buff [16] = { 0, };
	int fastpl = 0;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return false;
	}

	ext_chg = chg->ext_chg;
	if (unified_nodes_show("support_fastpl", buff))
		sscanf(buff, "%d", &fastpl);

	pd_hard_reset = chg->pd_hard_reset;
	usb_vbus_high = !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val)
		? !!val.intval : false;
	typec_mode_none = chg->typec_mode == POWER_SUPPLY_TYPEC_NONE;
	src_mode = chg->sink_src_mode != SRC_MODE;

	wa_required = !pd_hard_reset && usb_vbus_high && typec_mode_none && src_mode
				&& !fastpl && !ext_chg->wa_charging_without_cc_processed;
	if (!wa_required)
		pr_dbg_wa("Don't need CWC (pd_hard_reset:%d, usb_vbus_high:%d, typec_mode_none:%d, fastpl:%d)\n",
			pd_hard_reset, usb_vbus_high, typec_mode_none, fastpl, wa_required);

	return wa_required;
}

static void wa_charging_without_cc_main(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_charging_without_cc_dwork.work);
	struct smb_charger *chg = ext_chg->chg;

	if (wa_charging_without_cc_required(chg)) {
		pr_wa("CC line is not recovered until now, Start W/A\n");
		ext_chg->wa_charging_without_cc_processed = true;
		chg->typec_legacy = true;
		extension_hvdcp_detect_try_enable(chg, true);
		smblib_rerun_apsd_if_required(chg);
		wa_command_icl_override(chg);
	}
}

void wa_charging_without_cc_trigger(struct smb_charger *chg)
{
	// This may be triggered in the IRQ context of the USBIN rising.
	// So main function to start 'charging without cc', is deferred via delayed_work of kernel.
	// Just check and register (if needed) the work in this call.
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = { 0, };
	bool vbus = !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val)
		? !!val.intval : false;

	if (!ext_chg->enable_charging_without_cc)
		return;

	if (vbus && wa_charging_without_cc_required(chg)) {
		if (delayed_work_pending(&ext_chg->wa_charging_without_cc_dwork)) {
			pr_wa(" Cancel the pended trying apsd . . .\n");
			cancel_delayed_work(&ext_chg->wa_charging_without_cc_dwork);
		}

		schedule_delayed_work(&ext_chg->wa_charging_without_cc_dwork,
			msecs_to_jiffies(CWC_DELAY_MS));
	} else if (!vbus && ext_chg->wa_charging_without_cc_processed) {
		ext_chg->wa_charging_without_cc_processed = false;
		cancel_delayed_work(&ext_chg->wa_charging_without_cc_dwork);
		pr_wa("Call typec_removal by force\n");
		extension_typec_src_removal(chg);

		val.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		smblib_set_prop_typec_power_role(chg, &val);
	}
}

bool wa_charging_without_cc_is_running(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_charging_without_cc)
		return false;

	return ext_chg->wa_charging_without_cc_processed;
}

void wa_charging_without_cc_init(struct smb_charger* chg) {
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_charging_without_cc =
		of_property_read_bool(dnode, "lge,enable-charging-without-cc");

	if (!ext_chg->enable_charging_without_cc)
		return;

	ext_chg->wa_charging_without_cc_processed = false;
	INIT_DELAYED_WORK(&ext_chg->wa_charging_without_cc_dwork,
			wa_charging_without_cc_main);
}


#if 0
////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Rerun apsd for unknown charger
////////////////////////////////////////////////////////////////////////////
static void wa_charging_for_unknown_cable_main(struct work_struct *unused);
static DECLARE_DELAYED_WORK(wa_charging_for_unknown_cable_dwork, wa_charging_for_unknown_cable_main);

#define FLOAT_SETTING_DELAY_MS	1000
static void wa_charging_for_unknown_cable_main(struct work_struct *unused) {
	struct smb_charger*  chg = wa_helper_chg();
	struct power_supply* veneer = power_supply_get_by_name("veneer");
	union power_supply_propval floated
		= { .intval = POWER_SUPPLY_TYPE_USB_FLOAT, };
	union power_supply_propval val = { 0, };
	bool pd_hard_reset, usb_type_unknown, moisture_detected, usb_vbus_high,
			workaround_required, apsd_done, typec_mode_sink, ok_to_pd;
	bool vbus_valid = false;
	u8 stat;
	int rc;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		goto out_charging_for_unknown;
	}

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read APSD_STATUS_REG rc=%d\n", rc);
		goto out_charging_for_unknown;
	}
	apsd_done = (stat & APSD_DTC_STATUS_DONE_BIT);

	if (!(*chg->lpd_ux)) {
		if (chg->lpd_reason == LPD_MOISTURE_DETECTED) {
			if (*chg->lpd_dpdm_disable) {
				floated.intval = POWER_SUPPLY_TYPE_USB_DCP;
			} else {
				floated.intval = POWER_SUPPLY_TYPE_USB;
			}
		}
	}

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", rc);
		goto out_charging_for_unknown;
	}
	typec_mode_sink = (stat & SNK_SRC_MODE_BIT);

	pd_hard_reset = chg->pd_hard_reset;
	usb_type_unknown = chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN;
	moisture_detected
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_MOISTURE_DETECTED, &val)
		? (val.intval == POWER_SUPPLY_MOISTURE_DETECTED): false;
	usb_vbus_high
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val)
		? true : false;
	ok_to_pd = chg->ok_to_pd;

	workaround_required = !pd_hard_reset
				&& !typec_mode_sink
				&& usb_type_unknown
				&& usb_vbus_high
				&& !ok_to_pd;

	if (*chg->lpd_ux)
		workaround_required = workaround_required && !moisture_detected;

	if (!workaround_required) {
		pr_dbg_wa("check(!(pd_hard_reset:%d, MD:%d, typec_mode_sink:%d)"
			" usb_type_unknown:%d, usb_vbus_high:%d, ok_to_pd:%d,"
			" apsd_done:%d wa_charging_cc:%d, pending work:%d)\n",
			pd_hard_reset, moisture_detected, typec_mode_sink,
			usb_type_unknown, usb_vbus_high, ok_to_pd, apsd_done,
			wa_charging_without_cc_processed,
			delayed_work_pending(&wa_charging_without_cc_dwork));

		goto out_charging_for_unknown;
	}

	if (apsd_done && !delayed_work_pending(&wa_charging_without_cc_dwork)) {
		rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG,
				USBIN_ADAPTER_ALLOW_5V);
		if (rc < 0) {
			pr_wa("Couldn't write 0x%02x to"
					" USBIN_ADAPTER_ALLOW_CFG_REG rc=%d\n",
					USBIN_ADAPTER_ALLOW_CFG_REG, rc);
			goto out_charging_for_unknown;
		}
		vbus_valid = !power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_PRESENT, &val)
				? !!val.intval : false;
		if (vbus_valid) {
			pr_wa("Force setting cable as FLOAT if it is UNKNOWN after APSD\n");
			power_supply_set_property(veneer,
					POWER_SUPPLY_PROP_REAL_TYPE, &floated);
			power_supply_changed(veneer);
		}
		else {
			pr_wa("VBUS is not valid\n");
		}
	}
	else {
		schedule_delayed_work(&wa_charging_for_unknown_cable_dwork,
			round_jiffies_relative(msecs_to_jiffies(FLOAT_SETTING_DELAY_MS)));
	}

out_charging_for_unknown:
	power_supply_put(veneer);
}

void wa_charging_for_unknown_cable_trigger(struct smb_charger* chg) {
	union power_supply_propval val = { 0, };
	bool usb_vbus_high
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val)
		? true : false;
	if (usb_vbus_high
		&& (wa_charging_without_cc_processed ||
			delayed_work_pending(&wa_charging_without_cc_dwork))) {
		schedule_delayed_work(&wa_charging_for_unknown_cable_dwork,
			round_jiffies_relative(msecs_to_jiffies(FLOAT_SETTING_DELAY_MS)));
	}
}
#endif


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Support for weak battery pack
////////////////////////////////////////////////////////////////////////////

#define WEAK_SUPPLY_VOTER "WEAK_SUPPLY_VOTER"
#define WEAK_DELAY_MS		500
#define WEAK_DETECTION_COUNT	3
#define DEFAULT_WEAK_ICL_MA 1000

static void wa_support_weak_supply_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_support_weak_supply_dwork.work);
	struct smb_charger *chg = ext_chg->chg;
	u8 stat;

	if (!ext_chg->wa_support_weak_supply_running)
		return;

	if (chg && !smblib_read(chg, POWER_PATH_STATUS_REG, &stat)) {
		if ((stat & POWER_PATH_MASK) == POWER_PATH_USB) {
			ext_chg->wa_support_weak_supply_count++;
			pr_wa("wa_support_weak_supply_count = %d\n",
				ext_chg->wa_support_weak_supply_count);
			if (ext_chg->wa_support_weak_supply_count >= WEAK_DETECTION_COUNT) {
				pr_wa("Weak battery is detected, set ICL to 1A\n");
				vote(chg->usb_icl_votable, WEAK_SUPPLY_VOTER,
					true, DEFAULT_WEAK_ICL_MA*1000);
			}
		}
	}
	ext_chg->wa_support_weak_supply_running = false;
}

void wa_support_weak_supply_trigger(struct smb_charger* chg, u8 stat)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	bool trigger = !!(stat & USE_USBIN_BIT);

	if (!ext_chg->enable_support_weak_supply)
		return;

	if ((stat & POWER_PATH_MASK) != POWER_PATH_BATTERY)
		return;

	if (trigger) {
		if (!delayed_work_pending(&ext_chg->wa_support_weak_supply_dwork))
			schedule_delayed_work(&ext_chg->wa_support_weak_supply_dwork,
				round_jiffies_relative(msecs_to_jiffies(WEAK_DELAY_MS)));
	}
	else if (!!ext_chg->wa_support_weak_supply_count) {
		pr_wa("Clear wa_support_weak_supply_count\n");
		ext_chg->wa_support_weak_supply_count = 0;
		vote(chg->usb_icl_votable, WEAK_SUPPLY_VOTER, false, 0);
	}
	else
		; /* Do nothing */
}

void wa_support_weak_supply_check(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_support_weak_supply)
		return;

	if (delayed_work_pending(&ext_chg->wa_support_weak_supply_dwork)) {
		ext_chg->wa_support_weak_supply_running = true;
	}
}

void wa_support_weak_supply_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_support_weak_supply =
		of_property_read_bool(dnode, "lge,enable-support-weak-supply");

	if (!ext_chg->enable_support_weak_supply)
		return;

	ext_chg->wa_support_weak_supply_count = 0;
	ext_chg->wa_support_weak_supply_running = false;

	INIT_DELAYED_WORK(&ext_chg->wa_support_weak_supply_dwork,
			wa_support_weak_supply_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Resuming Suspended USBIN
////////////////////////////////////////////////////////////////////////////

#define RSU_MAX_ATTEMPTS	10
#define AICL_RERUN_DELAY_MS 3500
#define INITIAL_DELAY_MS 5000
static bool wa_resuming_suspended_usbin_required(struct smb_charger* chg)
{
	// Checking condition in prior to recover usbin suspending
	u8 reg_status_aicl, reg_status_powerpath, reg_status_rt;

	if (chg && (get_effective_result_locked(chg->usb_icl_votable) != 0)
			&& (smblib_read(chg, AICL_STATUS_REG, &reg_status_aicl) >= 0)
			&& (smblib_read(chg, POWER_PATH_STATUS_REG, &reg_status_powerpath) >= 0)
			&& (smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &reg_status_rt) >= 0) ) {
		pr_dbg_wa("AICL_STATUS_REG(0x%04x):0x%02x,"
			" POWER_PATH_STATUS_REG(0x%04x):0x%02x"
			" USB_INT_RT_STS(0x%04x):0x%02x\n",
			AICL_STATUS_REG, reg_status_aicl,
			POWER_PATH_STATUS_REG, reg_status_powerpath,
			USBIN_BASE + INT_RT_STS_OFFSET, reg_status_rt);

		if (reg_status_rt & USBIN_PLUGIN_RT_STS_BIT) {
			if ((reg_status_aicl & AICL_FAIL_BIT)
				|| (reg_status_powerpath & USBIN_SUSPEND_STS_BIT)) {
				pr_wa("AICL_FAIL:%d, USBIN_SUSPEND:%d\n",
					!!(reg_status_aicl & AICL_FAIL_BIT),
					!!(reg_status_powerpath & USBIN_SUSPEND_STS_BIT));
				return true;
			}
		}
		else
			pr_dbg_wa("[W/A] RSU-?) Skip because USB is not present\n");
	}

	return false;
}

static void wa_resuming_suspended_usbin_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_resuming_suspended_usbin_dwork.work);
	struct smb_charger *chg = ext_chg->chg;

// 0. Local variables
	// References for charger driver
	union power_supply_propval val = { 0, };
	bool vbus = false;
	bool usb_type_unknown = false;
	int 		    irq = (chg && chg->irq_info) ? chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq : 0;
	// Buffer to R/W PMI register
	int		    ret;
	u8		    buf;

	// Previous values to be restored
	int pre_usbin_collapse;
	int pre_aicl_rerun_time;

	if (!wa_resuming_suspended_usbin_required(chg)) {
		pr_wa("Exiting recovery for USBIN-suspend. (%p)\n", chg);
		return;
	}
	else if (ext_chg->wa_resuming_suspended_usbin_attempts >= RSU_MAX_ATTEMPTS) {
		pr_wa("Exiting resuming suspended usbin by over try\n");
		return;
	} else {
		ext_chg->wa_resuming_suspended_usbin_attempts++;
		pr_wa("Start resuming suspended usbin try %d\n",
				ext_chg->wa_resuming_suspended_usbin_attempts);
	}

// 1. W/A to prevent the IRQ 'usbin-icl-change' storm (CN#03165535) on SDM845
	// : Before recovery USBIN-suspend, be sure that IRQ 'usbin-icl-change' is enabled.
	//   If not, this recovery will not work well due to the disabled AICL notification.
	// : To prevent IRQ 'usbin-icl-change' storm, it might be disabled in its own ISR.
	//   Refer to the disabling IRQ condition in 'smblib_handle_icl_change()'
	ret = smblib_read(chg, POWER_PATH_STATUS_REG, &buf);
	if (irq && ret >= 0 && (buf & USBIN_SUSPEND_STS_BIT) && !chg->usb_icl_change_irq_enabled) {
		enable_irq(irq);
		chg->usb_icl_change_irq_enabled = true;
		pr_wa("USBIN_SUSPEND_STS_BIT = High, Enable ICL-CHANGE IRQ\n");
	}
	else {
		pr_wa("irq_number=%d, irq_enabled=%d, read_return=%d, read_register=%d\n",
			irq, chg->usb_icl_change_irq_enabled, ret, buf);
	}

// 2. Toggling USBIN_CMD_IL_REG
	pr_wa("Toggling USBIN_CMD_IL_REG(0x1340[0]) := 1\n");
	if (smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, USBIN_SUSPEND_BIT) < 0) {
		pr_wa("Couldn't write suspend to USBIN_SUSPEND_BIT\n");
		goto failed;
	}

	pr_wa("Toggling USBIN_CMD_IL_REG(0x1340[0]) := 0\n");
	if (smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0) < 0) {
		pr_wa("Couldn't write resume to USBIN_SUSPEND_BIT\n");
		goto failed;
	}

// 3. Save origial AICL configurations
	if (smblib_read(chg, USBIN_AICL_OPTIONS_CFG_REG /*0x1380*/, &buf) >= 0) {
		pre_usbin_collapse = buf & SUSPEND_ON_COLLAPSE_USBIN_BIT;
		pr_wa("USBIN_AICL_OPTIONS_CFG_REG=0x%02x, SUSPEND_ON_COLLAPSE_USBIN_BIT=0x%02x\n",
			buf, pre_usbin_collapse);
	}
	else {
		pr_wa("Couldn't read USBIN_AICL_OPTIONS_CFG_REG\n");
		goto failed;
	}

	if (smblib_read(chg, AICL_RERUN_TIME_CFG_REG /*0x1661*/, &buf) >= 0) {
		pre_aicl_rerun_time = buf & AICL_RERUN_TIME_MASK;
		pr_wa("AICL_RERUN_TIME_CFG_REG=0x%02x, AICL_RERUN_TIME_MASK=0x%02x\n",
			buf, pre_aicl_rerun_time);
	}
	else {
		pr_wa("Couldn't read AICL_RERUN_TIME_CFG_REG\n");
		goto failed;
	}

// 4. Set 0s to AICL configurationss
	pr_wa("Setting USBIN_AICL_OPTIONS(0x1380[7]) := 0x00\n");
	if (smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG, SUSPEND_ON_COLLAPSE_USBIN_BIT, 0) < 0) {
		pr_wa("Couldn't write USBIN_AICL_OPTIONS_CFG_REG\n");
		goto failed;
	}

	pr_wa("Setting AICL_RERUN_TIME_CFG_REG(0x1661[1:0]) := 0x00\n");
	if (smblib_masked_write(chg, AICL_RERUN_TIME_CFG_REG, AICL_RERUN_TIME_MASK, 0) < 0) {
		pr_wa("Couldn't write AICL_RERUN_TIME_CFG_REG\n");
		goto failed;
	}

// 5. Marginal delaying for AICL rerun
	pr_wa("Waiting more 3 secs . . .\n");
	msleep(AICL_RERUN_DELAY_MS);

// 6. Restore AICL configurations
	pr_wa("Restoring USBIN_AICL_OPTIONS(0x1380[7]) := 0x%02x\n", pre_usbin_collapse);
	if (smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG, SUSPEND_ON_COLLAPSE_USBIN_BIT,
			pre_usbin_collapse) < 0) {
		pr_wa("Couldn't write USBIN_AICL_OPTIONS_CFG_REG\n");
		goto failed;
	}

	pr_wa("Restoring AICL_RERUN_TIME_CFG_REG(0x1661[1:0]) := 0x%02x\n", pre_aicl_rerun_time);
	if (smblib_masked_write(chg, AICL_RERUN_TIME_CFG_REG, AICL_RERUN_TIME_MASK,
			pre_aicl_rerun_time) < 0) {
		pr_wa("Couldn't write AICL_RERUN_TIME_CFG_REG\n");
		goto failed;
	}

// 7. If USBIN suspend is not resumed even with rerunning AICL, recover it from APSD.
	msleep(APSD_RERUN_DELAY_MS);
	vbus = !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val) ? !!val.intval : false;
	usb_type_unknown = chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN;

	if (wa_resuming_suspended_usbin_required(chg) || (vbus && usb_type_unknown)) {
		pr_wa("Recover USBIN from APSD\n");
		wa_command_apsd(chg);
	}
	else
		pr_wa("Success resuming suspended usbin\n");

	return;
failed:
	pr_wa("Error on resuming suspended usbin\n");
}

void wa_resuming_suspended_usbin_trigger(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_resuming_suspended_usbin)
		return;

	if (!wa_resuming_suspended_usbin_required(chg)) {
		pr_wa(" Exiting recovery for USBIN-suspend.\n");
		return;
	}

	// Considering burst aicl-fail IRQs, previous wa works will be removed,
	// to make this trigger routine handle the latest aicl-fail
	if (delayed_work_pending(&ext_chg->wa_resuming_suspended_usbin_dwork)) {
		pr_wa("Cancel the pending resuming work . . .\n");
		cancel_delayed_work(&ext_chg->wa_resuming_suspended_usbin_dwork);
	}

	schedule_delayed_work(&ext_chg->wa_resuming_suspended_usbin_dwork,
		round_jiffies_relative(msecs_to_jiffies(INITIAL_DELAY_MS)));
}

void wa_resuming_suspended_usbin_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_resuming_suspended_usbin)
		return;

	cancel_delayed_work(&ext_chg->wa_resuming_suspended_usbin_dwork);
	ext_chg->wa_resuming_suspended_usbin_attempts = 0;
}

void wa_resuming_suspended_usbin_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_resuming_suspended_usbin =
		of_property_read_bool(dnode, "lge,enable-resuming-suspended-usbin");

	if (!ext_chg->enable_resuming_suspended_usbin)
		return;

	ext_chg->wa_resuming_suspended_usbin_attempts = 0;

	INIT_DELAYED_WORK(&ext_chg->wa_resuming_suspended_usbin_dwork,
			wa_resuming_suspended_usbin_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Charging with Rd-open charger.
////////////////////////////////////////////////////////////////////////////

void wa_charging_with_rd_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = { 0, };
	bool vbus = !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val) ? !!val.intval : false;
	bool non_src = (chg->sink_src_mode != SRC_MODE);
	bool cc_sink = (chg->typec_mode == POWER_SUPPLY_TYPEC_SINK);

	if (!ext_chg->enable_charging_with_rd)
		return;

	if (vbus && non_src && cc_sink) {
		ext_chg->wa_charging_with_rd_running = true;
		wa_command_icl_override(chg);
		pr_wa("Set icl override for rd open charger.\n");
	}
}

void wa_charging_with_rd_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval pval;

	if (!ext_chg->enable_charging_with_rd)
		return;

	ext_chg->wa_charging_with_rd_running = false;
	if (chg->typec_mode == POWER_SUPPLY_TYPEC_SINK ) {
		pr_wa("Set TYPEC_PR_DUAL for rd open charger \n");
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		// Clear cc status with Rd-open charger.
		smblib_set_prop_typec_power_role(chg, &pval);
	}
}

bool wa_charging_with_rd_is_running(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_charging_with_rd)
		return false;

	return ext_chg->wa_charging_with_rd_running;
}

void wa_charging_with_rd_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_charging_with_rd =
		of_property_read_bool(dnode, "lge,enable-charging-with-rd");

	if (!ext_chg->enable_charging_with_rd)
		return;

	ext_chg->wa_charging_with_rd_running = false;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Clear power role when charger is unpluged
////////////////////////////////////////////////////////////////////////////

static void wa_clear_pr_without_charger_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval pval = { .intval = 0, };

	if (!ext_chg->enable_clear_power_role)
		return;

	if (chg->typec_mode == POWER_SUPPLY_TYPEC_NONE
			&& chg->power_role != POWER_SUPPLY_TYPEC_PR_DUAL
			&& !chg->pd_hard_reset) {
		pr_wa("Set TYPEC_PR_DUAL for clear power role\n");
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		// Clear cc status without charger
		smblib_set_prop_typec_power_role(chg, &pval);
	}
}

static void wa_clear_pr_without_charger_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_clear_power_role =
		of_property_read_bool(dnode, "lge,enable-clear-power-role");

	if (!ext_chg->enable_clear_power_role)
		return;

}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Clear DC Reverse Voltage status
////////////////////////////////////////////////////////////////////////////

#define INT_ABNORMAL_OFFSET	0xe6
#define DC_IN_EN_OVERRIDE	BIT(1)
void wa_clear_dc_reverse_volt_trigger(bool enable)
{
	struct smb_charger* chg = wa_helper_chg();
	struct ext_smb_charger *ext_chg;
	u8 status;
	int rc = 0;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	ext_chg = chg->ext_chg;
	if (!ext_chg) {
		pr_wa("'ext_chg' is not ready\n");
		return;
	}

	if (!ext_chg->enable_clear_dc_reverse_volt)
		return;

	if (enable) {
		rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DC_IN_EN_OVERRIDE, 0);
		if (rc < 0) {
			pr_wa("Couldn't write to DCIN_CMD_IL_REG rc=%d\n", rc);
		}
	} else {
		rc = smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &status);
		if (!rc) {
			if (status == INT_ABNORMAL_OFFSET) {
				pr_wa("wa_clear_dc_reverse_volt_trigger write!\n");
				rc = smblib_masked_write(chg, DCIN_CMD_IL_REG,
					DC_IN_EN_OVERRIDE, DC_IN_EN_OVERRIDE);
				if (rc < 0) {
					pr_wa("Couldn't write to DCIN_CMD_IL_REG rc=%d\n", rc);
				}
			}
		}
	}
}

void wa_clear_dc_reverse_volt_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_clear_dc_reverse_volt =
		of_property_read_bool(dnode, "lge,enable-clear-dc-reverse-volt");

	if (!ext_chg->enable_clear_dc_reverse_volt)
		return;

}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Rerun DC AICL
////////////////////////////////////////////////////////////////////////////

#define AICL_DELAY_MS	5000
static void wa_dcin_rerun_aicl_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_dcin_rerun_aicl_dwork.work);
	struct smb_charger *chg = ext_chg->chg;
	int rc, icl, voting_icl, real_icl;

	rc = smblib_get_charge_param(chg, &chg->param.dc_icl, &icl);
	if (rc < 0) {
		pr_wa("Couldn't get dc_icl value, rc=%d\n", rc);
		return;
	}

	voting_icl = get_effective_result(chg->dc_icl_votable);
	real_icl = voting_icl - (voting_icl % chg->param.dc_icl.step_u);
	pr_wa("icl=%d, vote=%d, calculated=%d\n", icl, voting_icl, real_icl);

	if (icl < real_icl) {
		pr_wa("rerunning DCIN AICL\n");
		schedule_work(&chg->dcin_aicl_work);
	}
}

void wa_dcin_rerun_aicl_clear(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_dcin_rerun_aicl)
		return;

	if (delayed_work_pending(&ext_chg->wa_dcin_rerun_aicl_dwork))
		cancel_delayed_work(&ext_chg->wa_dcin_rerun_aicl_dwork);
}

void wa_dcin_rerun_aicl_trigger(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_dcin_rerun_aicl)
		return;

	schedule_delayed_work(&ext_chg->wa_dcin_rerun_aicl_dwork,
		round_jiffies_relative(msecs_to_jiffies(AICL_DELAY_MS)));
}

void wa_dcin_rerun_aicl_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_dcin_rerun_aicl =
		of_property_read_bool(dnode, "lge,enable-dcin-rerun-aicl");

	if (!ext_chg->enable_dcin_rerun_aicl)
		return;

	INIT_DELAYED_WORK(&ext_chg->wa_dcin_rerun_aicl_dwork,
			wa_dcin_rerun_aicl_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Recovery vashdn during wireless charging
////////////////////////////////////////////////////////////////////////////

#define VASHDN_DELAY_MS		3000
static void wa_recovery_vashdn_wireless_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_recovery_vashdn_wireless_dwork.work);
	struct smb_charger *chg = ext_chg->chg;
	struct power_supply* wireless_psy = power_supply_get_by_name("wireless");
	union power_supply_propval val = { .intval = 0, };
	u8 stat;

	if (!wireless_psy) {
		pr_wa("'wireless_psy' is not ready\n");
		return;
	}

	if (chg) {
		int dc_present = !smblib_get_prop_dc_present(chg, &val) ? val.intval : 0;
		int dc_online = !smblib_get_prop_dc_online(chg, &val) ? val.intval : 0;
		bool dc_vashdn = !smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat)
			? (stat & DCIN_VASHDN_RT_STS) : 0;
		bool dc_pause = !smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat)
			? (stat & PAUSE_CHARGE) : 0;

		if (dc_present && !dc_online && dc_vashdn && dc_pause) {
			pr_wa("detection Vashdn wireless charging stop!\n");
			val.intval = 2;
			power_supply_set_property(wireless_psy,
				POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);
		}
	}
	power_supply_put(wireless_psy);
}

void wa_recovery_vashdn_wireless_trigger(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_recovery_vashdn_wireless)
		return;

	if (delayed_work_pending(&ext_chg->wa_recovery_vashdn_wireless_dwork))
		cancel_delayed_work(&ext_chg->wa_recovery_vashdn_wireless_dwork);

	schedule_delayed_work(&ext_chg->wa_recovery_vashdn_wireless_dwork,
		round_jiffies_relative(msecs_to_jiffies(VASHDN_DELAY_MS)));
}

void wa_recovery_vashdn_wireless_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_recovery_vashdn_wireless =
		of_property_read_bool(dnode, "lge,enable-recovery-vashdn-wireless");

	if (!ext_chg->enable_recovery_vashdn_wireless)
		return;

	INIT_DELAYED_WORK(&ext_chg->wa_recovery_vashdn_wireless_dwork,
			wa_recovery_vashdn_wireless_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Retry to enable vconn on vconn-oc
////////////////////////////////////////////////////////////////////////////

#define VCONN_MAX_ATTEMPTS	3
#define MAX_OC_FALLING_TRIES 10
static void wa_retry_vconn_enable_on_vconn_oc_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_retry_vconn_enable_on_vconn_oc_work);
	struct smb_charger *chg = ext_chg->chg;
	static DEFINE_MUTEX(wa_vconn_oc_lock);
	int rc, i;
	u8 stat;

	pr_wa("over-current detected on VCONN\n");
	if (!chg || !chg->vconn_vreg || !chg->vconn_vreg->rdev)
		return;

	mutex_lock(&wa_vconn_oc_lock);
	rc = override_vconn_regulator_disable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		pr_wa("Couldn't disable VCONN rc=%d\n", rc);
		goto unlock;
	}

	if (++ext_chg->wa_vconn_attempts > VCONN_MAX_ATTEMPTS) {
		pr_wa("VCONN failed to enable after %d attempts\n",
			   ext_chg->wa_vconn_attempts - 1);
		ext_chg->wa_vconn_attempts = 0;
		goto unlock;
	}

	/*
	 * The real time status should go low within 10ms. Poll every 1-2ms to
	 * minimize the delay when re-enabling OTG.
	 */
	for (i = 0; i < MAX_OC_FALLING_TRIES; ++i) {
		usleep_range(1000, 2000);
		rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (rc >= 0 && !(stat & TYPEC_VCONN_OVERCURR_STATUS_BIT))
			break;
	}

	if (i >= MAX_OC_FALLING_TRIES) {
		pr_wa("VCONN OC did not fall after %dms\n",
						2 * MAX_OC_FALLING_TRIES);
		ext_chg->wa_vconn_attempts = 0;
		goto unlock;
	}
	pr_wa("VCONN OC fell after %dms\n", 2 * i + 1);

	rc = override_vconn_regulator_enable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		pr_wa("Couldn't enable VCONN rc=%d\n", rc);
		goto unlock;
	}

unlock:
	mutex_unlock(&wa_vconn_oc_lock);
}

void wa_retry_vconn_enable_on_vconn_oc_trigger(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	u8 stat;
	int rc;

	if (!ext_chg->enable_retry_vconn_with_oc)
		return;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", rc);
		return;
	}

	if (stat & TYPEC_VCONN_OVERCURR_STATUS_BIT)
		schedule_work(&ext_chg->wa_retry_vconn_enable_on_vconn_oc_work);
}

void wa_retry_vconn_enable_on_vconn_oc_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg;
	int rc;

	if (!chg || !chg->vconn_vreg->rdev) {
		pr_wa("chg or vconn_vreg is Null\n");
		return;
	}

	ext_chg = chg->ext_chg;
	if (!ext_chg->enable_retry_vconn_with_oc)
		return;

	if(smblib_vconn_regulator_is_enabled(chg->vconn_vreg->rdev)
			&& chg->typec_mode == POWER_SUPPLY_TYPEC_NONE
			&& ext_chg->wa_vconn_attempts != 0) {
		rc = override_vconn_regulator_disable(chg->vconn_vreg->rdev);
		if (rc < 0) {
			pr_wa("Couldn't disable VCONN rc=%d\n", rc);
			return;
		}
	}
	ext_chg->wa_vconn_attempts = 0;
}

void wa_retry_vconn_enable_on_vconn_oc_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_retry_vconn_with_oc =
		of_property_read_bool(dnode, "lge,enable-retry-vconn-with-oc");

	if (!ext_chg->enable_retry_vconn_with_oc)
		return;

	ext_chg->wa_vconn_attempts = 0;
	INIT_WORK(&ext_chg->wa_retry_vconn_enable_on_vconn_oc_work,
			wa_retry_vconn_enable_on_vconn_oc_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Retry pd check
////////////////////////////////////////////////////////////////////////////

void wa_retry_ok_to_pd_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	bool is_usb = chg->real_charger_type == POWER_SUPPLY_TYPE_USB
		|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT;
	bool is_high = chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_HIGH;

	if (!ext_chg->enable_retry_ok_to_pd)
		return;

	if (!ext_chg->wa_retry_ok_to_pd && is_usb && is_high) {
		chg->ok_to_pd = !(chg->pd_disabled) && !chg->pd_not_supported;
		ext_chg->wa_retry_ok_to_pd = true;
		pr_wa("retry ok_to_pd = %d\n", chg->ok_to_pd);
		power_supply_changed(chg->usb_psy);
	}
}

void wa_retry_ok_to_pd_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_retry_ok_to_pd)
		return;

	ext_chg->wa_retry_ok_to_pd = false;
}

void wa_retry_ok_to_pd_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_retry_ok_to_pd =
		of_property_read_bool(dnode, "lge,enable-retry-ok-to-pd");

	if (!ext_chg->enable_retry_ok_to_pd)
		return;

	ext_chg->wa_retry_ok_to_pd = false;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Avoid Inrush current for USB Compliance test
////////////////////////////////////////////////////////////////////////////

#define AVOID_INRUSH_DELAY_MS 100
#define AVOID_INRUSH_VOTER		"AVOID_INRUSH_VOTER"
static void wa_avoid_inrush_current_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_avoid_inrush_current_dwork.work);
	struct smb_charger *chg = ext_chg->chg;

	vote(chg->usb_icl_votable, AVOID_INRUSH_VOTER, false, 0);
}

void wa_avoid_inrush_current_triger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_avoid_inrush_current)
		return;

	if(unified_bootmode_fabproc())
		return;

	schedule_delayed_work(&ext_chg->wa_avoid_inrush_current_dwork,
			msecs_to_jiffies(AVOID_INRUSH_DELAY_MS));
	vote(chg->usb_icl_votable, AVOID_INRUSH_VOTER, true, 0);
}

void wa_avoid_inrush_current_with_compliance_triger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_avoid_inrush_current)
		return;

	if(unified_bootmode_fabproc())
		return;

	vote(chg->usb_icl_votable, AVOID_INRUSH_VOTER,
		ext_chg->is_usb_compliance_mode, 0);
}

void wa_avoid_inrush_current_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_avoid_inrush_current)
		return;

	vote(chg->usb_icl_votable, AVOID_INRUSH_VOTER,
		ext_chg->is_usb_compliance_mode, 0);
}

void wa_avoid_inrush_current_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_avoid_inrush_current =
		of_property_read_bool(dnode, "lge,enable-avoid-inrush-current");

	if (!ext_chg->enable_avoid_inrush_current)
		return;

	INIT_DELAYED_WORK(&ext_chg->wa_avoid_inrush_current_dwork,
			wa_avoid_inrush_current_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : avoid over voltage by abnormal charger
////////////////////////////////////////////////////////////////////////////

#define CHG_TERM_WA_ENTRY_DELAY_MS     300000    /* 5 min */
#define CHG_TERM_WA_EXIT_DELAY_MS      60000     /* 1 min */
int wa_protect_overcharging(struct smb_charger* chg, int input_present)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = { 0, };
	int batt_vol = -EINVAL, upper_border = -EINVAL, lower_border = -EINVAL;
	int delay = CHG_TERM_WA_ENTRY_DELAY_MS;
	u8 stat = 0;
	int rc = 0;

	if (!ext_chg->enable_protect_overcharging)
		return delay;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n", rc);
		return delay;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &val)) {
		upper_border = val.intval + ext_chg->prot_overchg_ent_dischg_off;
		lower_border = val.intval - ext_chg->prot_overchg_rel_off;
		if (stat == FULLON_CHARGE || stat == TAPER_CHARGE)
			upper_border = val.intval + ext_chg->prot_overchg_ent_chg_off;
	}

	if (!smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
		batt_vol = val.intval;

	if (batt_vol < lower_border) {
		vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
	} else if (batt_vol >= upper_border) {
		if (input_present & INPUT_PRESENT_USB)
			vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER,
					true, 0);
		if (input_present & INPUT_PRESENT_DC)
			vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER,
					true, 0);
	}

	if (is_client_vote_enabled(chg->usb_icl_votable, CHG_TERMINATION_VOTER)
		|| is_client_vote_enabled(chg->dc_suspend_votable, CHG_TERMINATION_VOTER))
		delay = CHG_TERM_WA_EXIT_DELAY_MS;
	else
		delay = CHG_TERM_WA_ENTRY_DELAY_MS;

	pr_wa("batt = %d (lower: %d, upper: %d), delay = %d min\n",
		batt_vol, lower_border, upper_border, delay/CHG_TERM_WA_EXIT_DELAY_MS);
	return delay;
}


void wa_protect_overcharging_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	int rc = 0;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_protect_overcharging =
		of_property_read_bool(dnode, "lge,enable-protect-overcharging");

	if (!ext_chg->enable_protect_overcharging)
		return;

 	rc = of_property_read_u32(node,	"lge,prot-overchg-ent-dischg-off-uv",
		&ext_chg->prot_overchg_ent_dischg_off);
	if (rc < 0) {
		pr_err("Fail to get prot-overchg-ent-dischg-off-uv rc = %d \n", rc);
		ext_chg->prot_overchg_ent_dischg_off = 10000;
	}

	rc = of_property_read_u32(node, "lge,prot-overchg-ent-chg-off-uv",
		&ext_chg->prot_overchg_ent_chg_off);
	if (rc < 0) {
		pr_err("Fail to get prot-overchg-ent-chg-off-uv rc = %d \n", rc);
		ext_chg->prot_overchg_ent_chg_off = 20000;
	}

	rc = of_property_read_u32(node, "lge,prot-overchg-rel-off-uv",
		&ext_chg->prot_overchg_rel_off);
	if (rc < 0) {
		pr_err("Fail to get prot-overchg-rel-off-uv rc = %d \n", rc);
		ext_chg->prot_overchg_rel_off = 50000;
	}
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Fake USB type to SDP on Factory cable.
////////////////////////////////////////////////////////////////////////////

void wa_fake_usb_type_with_factory_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = { 0, };
	int usb_id = 0;
	bool fabid = false;

	if (!ext_chg->enable_faked_usb_type)
		return;

	if (unified_bootmode_usermode())
		return;

	if (!power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_RESISTANCE_ID, &val))
		usb_id = val.intval;

	if (usb_id == CHARGER_USBID_56KOHM
			|| usb_id == CHARGER_USBID_130KOHM
			|| usb_id == CHARGER_USBID_910KOHM) {
		fabid = true;
	}

	if (fabid && chg->real_charger_type != POWER_SUPPLY_TYPE_USB
			&& chg->typec_mode == POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY) {
		ext_chg->faked_usb_type = true;
	} else {
		ext_chg->faked_usb_type = false;
	}
}

void wa_fake_usb_type_with_factory_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_faked_usb_type)
		return;

	ext_chg->faked_usb_type = false;
}

bool wa_fake_usb_type_with_factory_is_running(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_faked_usb_type)
		return false;

	return ext_chg->faked_usb_type;
}

void wa_fake_usb_type_with_factory_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_faked_usb_type =
		of_property_read_bool(dnode, "lge,enable-faked-usb-type");

	if (!ext_chg->enable_faked_usb_type)
		return;

	ext_chg->faked_usb_type = false;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Recover & Fake CC status in factory mode.
////////////////////////////////////////////////////////////////////////////

#define RECOVER_CC_DELAY_MS 100
#define MAX_RECOVER_CC_ATTEMPTS 3
static void wa_recover_cc_status_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_recover_cc_status_dwork.work);
	struct smb_charger *chg = ext_chg->chg;
	union power_supply_propval pval = {0, };

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	if (chg->typec_mode != POWER_SUPPLY_TYPEC_SINK) {
		pr_wa("Stop to recover cc status, because it isn't rd-open\n");
		ext_chg->wa_recover_cc_attempts = 0;
		return;
	}

	pr_wa("Recover CC status by factory cable's error\n");
	pval.intval = POWER_SUPPLY_TYPEC_PR_NONE;
	// Clear cc status with Rd-open charger.
	smblib_set_prop_typec_power_role(chg, &pval);
	msleep(10);

	pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
	// Clear cc status with Rd-open charger.
	smblib_set_prop_typec_power_role(chg, &pval);

	if (ext_chg->wa_recover_cc_attempts < MAX_RECOVER_CC_ATTEMPTS) {
		schedule_delayed_work(&ext_chg->wa_recover_cc_status_dwork,
				msecs_to_jiffies(RECOVER_CC_DELAY_MS));
		ext_chg->wa_recover_cc_attempts++;
	} else {
		ext_chg->wa_recover_cc_attempts = 0;
	}
}

void wa_recover_cc_status_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_recover_cc)
		return;

	if(!unified_bootmode_usermode()
			&& !delayed_work_pending(&ext_chg->wa_recover_cc_status_dwork))
		schedule_delayed_work(&ext_chg->wa_recover_cc_status_dwork,
				msecs_to_jiffies(RECOVER_CC_DELAY_MS));
}

bool wa_fake_cc_status_is_runnging(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval	val = {0, };
	bool ret = false;

	if (!ext_chg->enable_recover_cc)
		return ret;

	if (unified_bootmode_usermode() || chg->typec_mode != POWER_SUPPLY_TYPEC_SINK)
		return ret;

	if (!smblib_get_prop_usb_present(chg, &val) && val.intval)
		ret = true;

	return ret;
}

void wa_fake_cc_status_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_recover_cc =
		of_property_read_bool(dnode, "lge,enable-recover-cc");

	if (!ext_chg->enable_recover_cc)
		return;

	ext_chg->wa_recover_cc_attempts = 0;
	INIT_DELAYED_WORK(&ext_chg->wa_recover_cc_status_dwork,
			wa_recover_cc_status_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Retry APSD in factory mode.
////////////////////////////////////////////////////////////////////////////

#define MAX_RETRY_APSD_ATTEMPTS 3
void wa_retry_apsd_with_factory_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = { 0, };
	int fastpl = 0, typec_mode = chg->typec_mode;
	char buff [16] = { 0, };
	u8 stat;

	if (!ext_chg->enable_retry_apsd_with_factory)
		return;

	if(unified_bootmode_usermode())
		return;

	if (unified_nodes_show("support_fastpl", buff)
			&& sscanf(buff, "%d", &fastpl) && fastpl != 1)
		return;

	if (!power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_TYPEC_MODE, &val))
		typec_mode = val.intval;

	if (smblib_read(chg, APSD_STATUS_REG, &stat)) {
		pr_wa("Couldn't read APSD_STATUS_REG\n");
		return;
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT
			&& typec_mode != POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY
			&& (stat & APSD_DTC_STATUS_DONE_BIT)
			&& ext_chg->wa_apsd_apsd_attempts < MAX_RETRY_APSD_ATTEMPTS) {
		pr_wa("Retry apsd in factory mode\n");
		wa_command_apsd(chg);
		ext_chg->wa_apsd_apsd_attempts++;
	}
}

void wa_retry_apsd_with_factory_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_retry_apsd_with_factory)
		return;

	ext_chg->wa_apsd_apsd_attempts = 0;
}

void wa_retry_apsd_with_factory_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_retry_apsd_with_factory =
		of_property_read_bool(dnode, "lge,enable-retry-apsd-with-factory");

	if (!ext_chg->enable_retry_apsd_with_factory)
		return;

	ext_chg->wa_apsd_apsd_attempts = 0;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Faster try APSD for HVDCP test
////////////////////////////////////////////////////////////////////////////

static void wa_faster_try_apsd_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	char buff [16] = { 0, };
	int  test;

	if (!ext_chg->enable_faster_try_apsd)
		return;

	if(unified_bootmode_usermode())
		return;

	if (unified_nodes_show("support_fastpl", buff)
			&& sscanf(buff, "%d", &test) && test != 1)
		return;

	ext_chg->wa_faster_try_apsd_running = true;
	chg->typec_legacy = true;
	extension_hvdcp_detect_try_enable(chg, true);
	smblib_apsd_enable(chg, true);
	wa_command_icl_override(chg);
}

static void wa_faster_try_apsd_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = {0, };

	if (!ext_chg->enable_faster_try_apsd)
		return;

	if (ext_chg->wa_faster_try_apsd_running) {
		pr_wa("Call typec_removal by force\n");
		ext_chg->wa_faster_try_apsd_running = false;
		extension_typec_src_removal(chg);
		val.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		smblib_set_prop_typec_power_role(chg, &val);
	}
}

static void wa_faster_try_apsd_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_faster_try_apsd =
		of_property_read_bool(dnode, "lge,enable-faster-try-apsd");

	if (!ext_chg->enable_faster_try_apsd)
		return;

	ext_chg->wa_faster_try_apsd_running = false;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Faster try QC 3.0 for 2nd charger test
////////////////////////////////////////////////////////////////////////////

#define WA_COMP_CP_QC30_MARGIN_MV      200
#define WA_DEFAULT_VOLTAGE_MV          5000
#define WA_QC30_STEP_MV                200
#define FASTER_QC_VOTER        "FASTER_QC_VOTER"
#define MIN_CP_CURRENT_UA		2000000

static void wa_faster_try_cp_qc30_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	int vbat = 0, rc = 0;
	union power_supply_propval val = {0, };
	char buff [16] = { 0, };
	int  test;

	if (!ext_chg->enable_faster_try)
		return;

	if(unified_bootmode_usermode())
		return;

	if (unified_nodes_show("support_fastpl", buff)
			&& sscanf(buff, "%d", &test) && test != 1)
		return;

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_PD) {
		vote_override(chg->fcc_votable, FASTER_QC_VOTER, true, MIN_CP_CURRENT_UA);
		return;
	}

	if (chg->real_charger_type != POWER_SUPPLY_TYPE_USB_HVDCP_3)
		return;

	if (!ext_chg->wa_faster_try_running) {
		if (!chg->cp_disable_votable)
			chg->cp_disable_votable = find_votable("CP_DISABLE");
		if (chg->cp_disable_votable)
			vote_override(chg->cp_disable_votable, FASTER_QC_VOTER, true, 0);
		vote_override(chg->fcc_votable, FASTER_QC_VOTER, true, MIN_CP_CURRENT_UA);
		ext_chg->wa_faster_try_running = true;
		vbat = !power_supply_get_property(chg->bms_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val) ? val.intval/1000 : -1;

		ext_chg->wa_target_cnt = ((vbat * 2) + WA_COMP_CP_QC30_MARGIN_MV - WA_DEFAULT_VOLTAGE_MV) / WA_QC30_STEP_MV + 1;
		if (ext_chg->wa_target_cnt > 20)
			ext_chg->wa_target_cnt = 20;
		pr_info("set target = %d, vbat = %d\n",	ext_chg->wa_target_cnt, vbat);
	}

	if (ext_chg->wa_target_cnt > chg->pulse_cnt) {
		val.intval = POWER_SUPPLY_DP_DM_DP_PULSE;

		rc = power_supply_set_property(
			chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);
	} else {
		ext_chg->wa_target_cnt = 0;
	}
}

static void wa_faster_try_cp_qc30_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_faster_try)
		return;

	ext_chg->wa_faster_try_running = false;
	vote_override(chg->fcc_votable, FASTER_QC_VOTER, false, 0);

	if (chg->cp_disable_votable)
		vote_override(chg->cp_disable_votable, FASTER_QC_VOTER, false, 0);
}

static void wa_faster_try_cp_qc30_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_faster_try =
		of_property_read_bool(dnode, "lge,enable-faster-try");

	if (!ext_chg->enable_faster_try)
		return;

	ext_chg->wa_faster_try_running = false;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Charging for mcdodo
////////////////////////////////////////////////////////////////////////////

#define WA_MCDODO_VOTER			"WA_MCDODO_VOTER"
void wa_charging_for_mcdodo_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	static bool pre_floating = false;
	bool is_floating = (chg->lpd_reason == LPD_FLOATING_CABLE);
	bool vbus;
	union power_supply_propval val = { 0, };

	if (!ext_chg->enable_charging_for_mcdodo)
		return;

	if (pre_floating != is_floating) {
		vbus = !smblib_get_prop_usb_present(chg, &val) ? !!val.intval : false;
		if (is_floating && !vbus && chg->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
			vote(chg->usb_icl_votable, WA_MCDODO_VOTER, true, 0);
		} else {
			vote(chg->usb_icl_votable, WA_MCDODO_VOTER, false, 0);
		}
		pre_floating = is_floating;
	}
}

void wa_charging_for_mcdodo_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_charging_for_mcdodo)
		return;

	vote(chg->usb_icl_votable, WA_MCDODO_VOTER, false, 0);
}

static void wa_charging_for_mcdodo_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_charging_for_mcdodo =
		of_property_read_bool(dnode, "lge,enable-charging-for-mcdodo");

	if (!ext_chg->enable_charging_for_mcdodo)
		return;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Disable CP charging in battery fake mode
////////////////////////////////////////////////////////////////////////////

#define BAT_FAKE_VOTER			"BAT_FAKE_VOTER"
void wa_disable_cp_with_fake_mode_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	char buff [2] = { 0, };

	if (!ext_chg->enable_disable_cp_with_fake_mode)
		return;

	/* Updating fast charger status here */
	if (!unified_nodes_show("fake_battery", buff) || !chg->smb_override_votable) {
		pr_wa("Fail to get fake battery\n");
		return;
	}

	if (!strncmp(buff, "1", 1)
			&& !is_client_vote_enabled_locked(chg->smb_override_votable , BAT_FAKE_VOTER)) {
		vote(chg->smb_override_votable, BAT_FAKE_VOTER, true, 0);
	} else if ((!strncmp(buff, "0", 1)
			&& is_client_vote_enabled_locked(chg->smb_override_votable , BAT_FAKE_VOTER))) {
		vote(chg->smb_override_votable, BAT_FAKE_VOTER, false, 0);
	}
}

static void wa_disable_cp_with_fake_mode_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_disable_cp_with_fake_mode =
		of_property_read_bool(dnode, "lge,enable-disable-cp-with-fake-mode");

	if (!ext_chg->enable_disable_cp_with_fake_mode)
		return;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Disable hiccup of otg in compliance mode
////////////////////////////////////////////////////////////////////////////

#define WA_OTG_HICCUP_DELAY_MS	1000
static void wa_disable_otg_hiccup_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_disable_otg_hiccup_dwork.work);
	struct smb_charger *chg = ext_chg->chg;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	if (!ext_chg->enable_disable_otg_hiccup)
		return;

	if (chg->typec_mode != POWER_SUPPLY_TYPEC_NONE) {
		pr_info("Enable vbus regulator with otg-hiccup\n");
		override_vbus_regulator_enable(chg->vbus_vreg->rdev);
	}
}

void wa_disable_otg_hiccup_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_disable_otg_hiccup)
		return;

	if (!ext_chg->is_usb_compliance_mode)
		return;

	override_vbus_regulator_disable(chg->vbus_vreg->rdev);
	schedule_delayed_work(&ext_chg->wa_disable_otg_hiccup_dwork,
		msecs_to_jiffies(WA_OTG_HICCUP_DELAY_MS));
}

static void wa_disable_otg_hiccup_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_disable_otg_hiccup)
		return;

	if (!ext_chg->is_usb_compliance_mode)
		return;

	if (chg->typec_mode == POWER_SUPPLY_TYPEC_NONE)
		cancel_delayed_work(&ext_chg->wa_disable_otg_hiccup_dwork);
}

static void wa_disable_otg_hiccup_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	ext_chg->enable_disable_otg_hiccup =
		of_property_read_bool(dnode, "lge,enable-disable-otg-hiccup");

	if (!ext_chg->enable_disable_otg_hiccup)
		return;

	INIT_DELAYED_WORK(
		&ext_chg->wa_disable_otg_hiccup_dwork, wa_disable_otg_hiccup_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Control Vbus2 regulator
////////////////////////////////////////////////////////////////////////////

void wa_control_vbus2_regulator(struct smb_charger *chg, bool on)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	if (!ext_chg->enable_control_vbus2_regulator)
		return;

	if (on) {
		gpiod_set_value(ext_chg->ds_en_gpio, 1);
		gpiod_set_value(ext_chg->load_sw_on_gpio, 1);
	} else {
		gpiod_set_value(ext_chg->load_sw_on_gpio, 0);
		gpiod_set_value(ext_chg->ds_en_gpio, 0);
	}
}

void wa_control_vbus2_regulator_trigger(void)
{
	struct smb_charger* chg = wa_helper_chg();
	struct ext_smb_charger *ext_chg;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	ext_chg = chg->ext_chg;
	if (!ext_chg->enable_control_vbus2_regulator)
		return;

	if (smblib_vbus_regulator_is_enabled(chg->vbus_vreg->rdev)) {
		pr_wa("Disable Vbus2 by BCL.\n");
		gpiod_set_value(ext_chg->load_sw_on_gpio, 0);
		gpiod_set_value(ext_chg->ds_en_gpio, 0);
	}
}

static void wa_control_vbus2_regulator_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");
	char property [48];
	int i;

	// Building property name
	strncpy(property, "lge,enable-control-vbus2-regulator-", 48);
	strncat(property, unified_bootmode_operator(), 48);
	for (i=0; i<strlen(property); ++i)
		property[i] = tolower(property[i]);

	ext_chg->enable_control_vbus2_regulator =
		of_property_read_bool(dnode, "lge,enable-control-vbus2-regulator")
		|| of_property_read_bool(dnode, property);

	if (!ext_chg->enable_control_vbus2_regulator)
		return;
}

////////////////////////////////////////////////////////////////////////////
// LGE Workaround : compensate the power of cp qc3.0 TA
////////////////////////////////////////////////////////////////////////////
#define MAX_ILIM_UA                      3200000
#define CP_ILIM_STEP_MA                  100
#define CP_ILIM_STEP_UA                  100000

#define WA_SMB1390_MIN_CURR_MA           550		/* MIN ILIM + 50mA */
#define WA_SMB1390_DISABLE_CURR_MA       1800

#define WA_COMP_CP_QC30_TRIGGER_DELAY    15000
#define WA_COMP_CP_QC30_ILIM_DELAY       2000
#define WA_COMP_CP_QC30_DP_DM_DELAY      5000
#define WA_COMP_CP_QC30_MARGIN_MA        200
#define WA_COMP_CP_QC30_MSOC_MAX         188 /* msoc: 73.7%, ui: 76% */
#define WA_COMP_CP_QC30_BAD_ICP_MAX      3

static void wa_comp_pwr_cp_qc30_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = {0, };

	if (!ext_chg->enable_comp_cp_qc30)
		return;

	if (ext_chg->is_comp_cp_qc30) {
		ext_chg->bad_icp_cnt = 0;
		ext_chg->is_comp_cp_qc30 = false;
		veneer_voter_release(&ext_chg->cp_qc3_ibat);
		cancel_delayed_work(&ext_chg->wa_comp_cp_qc30_dwork);
		if (!chg->batt_psy)
			chg->batt_psy = power_supply_get_by_name("battery");
		if (chg->batt_psy) {
			val.intval = POWER_SUPPLY_DP_DM_LOCKED_RELEASE;
			power_supply_set_property(
				chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);
		}
		pr_info("clear done!!\n");
	}
}

static void wa_comp_pwr_cp_qc30_work_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work,
		struct ext_smb_charger, wa_comp_cp_qc30_dwork.work);
	struct smb_charger *chg = ext_chg->chg;
	union power_supply_propval val = {0, };
	int cp_enable = 0, cp_status1 = 0, cp_status2 = 0;
	int soc = 0, icp_ma = 0, vusb = 0, vbat = 0, fcc = 0, ibat = 0;
	int pulse_count = 0, rc = 0;
	int cp_ilim = 0, cp_ilim_comp = 0;
	int delay_ms = WA_COMP_CP_QC30_ILIM_DELAY;

	if (!chg->usb_psy)
		chg->usb_psy = power_supply_get_by_name("usb");
	if (!chg->bms_psy)
		chg->bms_psy = power_supply_get_by_name("bms");
	if (!chg->batt_psy)
		chg->batt_psy = power_supply_get_by_name("battery");
	if (!chg->cp_psy)
		chg->cp_psy = power_supply_get_by_name("charge_pump_master");

	if (!ext_chg->is_comp_cp_qc30 ||
		chg->cp_reason != POWER_SUPPLY_CP_HVDCP3) {
		wa_comp_pwr_cp_qc30_clear(chg);
		pr_info("clear reason -> NOT CP_HVDCP3, flag=%d, reason=%d\n",
			ext_chg->is_comp_cp_qc30, chg->cp_reason);
		return;
	}

	if (!chg->usb_psy || !chg->bms_psy || !chg->cp_psy || !chg->batt_psy) {
		schedule_delayed_work(
			&ext_chg->wa_comp_cp_qc30_dwork,
			msecs_to_jiffies(delay_ms));
		return;
	}

	ibat = !power_supply_get_property(chg->batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &val) ? val.intval/1000 : -1;
	vusb = !power_supply_get_property(chg->usb_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &val) ? val.intval/1000 : -1;
	vbat = !power_supply_get_property(chg->bms_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &val) ? val.intval/1000 : -1;
	fcc = !power_supply_get_property(chg->batt_psy,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val) ? val.intval/1000 : -1;
	soc = !power_supply_get_property(chg->bms_psy,
		POWER_SUPPLY_PROP_CAPACITY_RAW, &val) ? val.intval : -1;

	cp_enable = !power_supply_get_property(chg->cp_psy,
		POWER_SUPPLY_PROP_CP_ENABLE, &val) ? val.intval : 0;
	cp_status1 = !power_supply_get_property(chg->cp_psy,
		POWER_SUPPLY_PROP_CP_STATUS1, &val) ? val.intval : 0;
	cp_status2 = !power_supply_get_property(chg->cp_psy,
		POWER_SUPPLY_PROP_CP_STATUS2, &val) ? val.intval : 0;
	pulse_count = !power_supply_get_property(chg->batt_psy,
		POWER_SUPPLY_PROP_DP_DM, &val) ? val.intval : 0;
	icp_ma = !power_supply_get_property(chg->cp_psy,
		POWER_SUPPLY_PROP_CP_ISNS, &val) ? val.intval/1000 : -1;

	/* check point 1 : SOC */
	if (soc > WA_COMP_CP_QC30_MSOC_MAX) {
		val.intval = POWER_SUPPLY_DP_DM_LOCKED_RELEASE;
		power_supply_set_property(
			chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);

		veneer_voter_set(&ext_chg->cp_qc3_ibat, WA_SMB1390_DISABLE_CURR_MA);

		pr_info("clear reason -> OVER SOC(max: %d=%d), now soc: %d=%d\n",
			WA_COMP_CP_QC30_MSOC_MAX,
			WA_COMP_CP_QC30_MSOC_MAX*100/255, soc, soc*100/255);
		return;
	}

	/* check point 2 : cp status */
	if (!cp_enable ||
		!((cp_status1 == GENMASK(3, 2) ||
		   cp_status1 == BIT(2) ||
		   cp_status1 == (BIT(2)|BIT(5))) && cp_status2 == BIT(7))) {
		if (cp_enable && (cp_status2 & BIT(3))) {
			ext_chg->bad_icp_cnt++;
			if (ext_chg->bad_icp_cnt > WA_COMP_CP_QC30_BAD_ICP_MAX) {
				val.intval = POWER_SUPPLY_DP_DM_LOCKED_RELEASE;
				power_supply_set_property(
					chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);

				veneer_voter_set(&ext_chg->cp_qc3_ibat, WA_SMB1390_DISABLE_CURR_MA);

				pr_info("disable CP reason -> iCP status is bad, en=%d, "
						"sts1=0x%x, sts2=0x%x, bad_icp_cnt=%d\n",
						cp_enable, cp_status1, cp_status2, ext_chg->bad_icp_cnt);
				return;
			}
		}

		if (cp_enable && (cp_status1 & BIT(0))) { /* Over Voltage */
			delay_ms = WA_COMP_CP_QC30_DP_DM_DELAY;
			val.intval = POWER_SUPPLY_DP_DM_DM_PULSE_LOCKED;
			rc = power_supply_set_property(
				chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);
		} else if (cp_enable && (cp_status1 & BIT(1))) {  /* Under Voltage */
			delay_ms = WA_COMP_CP_QC30_DP_DM_DELAY;
			val.intval = POWER_SUPPLY_DP_DM_DP_PULSE_LOCKED;
			rc = power_supply_set_property(
				chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);
		}

		pr_info("retry reason -> CP STATUS, en=%d, OV=%d, UV=%d, "
				"sts1=0x%x, sts2=0x%x, pulse=%d, bad_icp_cnt=%d\n",
				cp_enable, cp_status1 & BIT(0), cp_status1 & BIT(1),
				cp_status1, cp_status2, pulse_count, ext_chg->bad_icp_cnt);

		schedule_delayed_work(
			&ext_chg->wa_comp_cp_qc30_dwork,
			msecs_to_jiffies(delay_ms));

		return;
	}

	/* check point 3: icp */
	if (icp_ma < WA_SMB1390_MIN_CURR_MA && pulse_count >= 20) {
		ext_chg->bad_icp_cnt++;
		if (ext_chg->bad_icp_cnt > WA_COMP_CP_QC30_BAD_ICP_MAX) {
			val.intval = POWER_SUPPLY_DP_DM_LOCKED_RELEASE;
			power_supply_set_property(
				chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);

			veneer_voter_set(&ext_chg->cp_qc3_ibat, WA_SMB1390_DISABLE_CURR_MA);

			pr_info("disable CP reason -> iCP is under 550mA.\n");
			return;
		}
	}
	else {
		ext_chg->bad_icp_cnt = 0;
	}

	/* check point 4: cp ilim */
	if ((ibat > (fcc + WA_COMP_CP_QC30_MARGIN_MA)) ||
		(cp_status1 == (BIT(2)|BIT(5)) && cp_status2 == BIT(7))) {
		cp_ilim = !power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_CP_ILIM, &val) ? val.intval : -1;

		cp_ilim_comp = (fcc - ibat) / 2 / CP_ILIM_STEP_MA * CP_ILIM_STEP_UA;

		if (cp_ilim_comp > CP_ILIM_STEP_UA)
			cp_ilim_comp = CP_ILIM_STEP_UA;
		else if (cp_ilim_comp < -CP_ILIM_STEP_UA)
			cp_ilim_comp = -CP_ILIM_STEP_UA;

		if (cp_ilim > 0 && fcc > 0 && ibat > 0 && cp_ilim_comp != 0) {
			val.intval = cp_ilim + cp_ilim_comp;
			if (val.intval > MAX_ILIM_UA)
				val.intval = MAX_ILIM_UA;
			power_supply_set_property(chg->cp_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &val);

			pr_info("CP ILIM %s -> new cp_ilim=%d, pulse=%d, status=0x%X,%X, "
					"vbus=%d, vbat=%d, icp=%d, fcc=%d, ibat=%d, cp_ilim=%d\n",
					cp_ilim_comp > 0 ? "up" : "down",
					val.intval / 1000, pulse_count, cp_status1, cp_status2,
					vusb, vbat, icp_ma,	fcc, ibat, cp_ilim / 1000);

			goto comp_cp_qc30_reschedule;
		}
	}

	/* check point 5: TA output voltage */
	if (vusb >= ((vbat * 2) + WA_COMP_CP_QC30_MARGIN_MV))
		goto comp_cp_qc30_reschedule;

	if (ibat < 0 ||
		(cp_status1 & BIT(0)) || (cp_status1 & BIT(5)) ||
		fcc <= (ibat + WA_COMP_CP_QC30_MARGIN_MA))
		goto comp_cp_qc30_reschedule;

	delay_ms = WA_COMP_CP_QC30_DP_DM_DELAY;
	val.intval = POWER_SUPPLY_DP_DM_DP_PULSE_LOCKED;
	rc = power_supply_set_property(
			chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);

	pr_info("TA voltage up -> pulse=%d->%d, status=0x%X,%X, "
			"vbus=%d, vbat=%d, icp=%d, fcc=%d, ibat=%d, bad_icp_cnt=%d\n",
			pulse_count, pulse_count + 1, cp_status1, cp_status2,
			vusb, vbat, icp_ma, fcc, ibat, ext_chg->bad_icp_cnt);

comp_cp_qc30_reschedule:

	schedule_delayed_work(
		&ext_chg->wa_comp_cp_qc30_dwork,
		msecs_to_jiffies(delay_ms));

	return;
}

static void wa_comp_pwr_cp_qc30_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval val = {0, };

	if (!ext_chg->enable_comp_cp_qc30)
		return;

	if (chg->cp_reason != POWER_SUPPLY_CP_HVDCP3)
		return;

	if (!ext_chg->is_comp_cp_qc30) {
		ext_chg->bad_icp_cnt = 0;
		ext_chg->is_comp_cp_qc30 = true;

		if (!chg->batt_psy)
			chg->batt_psy = power_supply_get_by_name("battery");
		if (chg->batt_psy) {
			val.intval = POWER_SUPPLY_DP_DM_LOCKED_RELEASE;
			power_supply_set_property(
				chg->batt_psy, POWER_SUPPLY_PROP_DP_DM, &val);
		}

		schedule_delayed_work(&ext_chg->wa_comp_cp_qc30_dwork,
			msecs_to_jiffies(WA_COMP_CP_QC30_TRIGGER_DELAY));

		pr_info("triggered..\n");
	}
}

static void wa_comp_pwr_cp_qc30_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	if (!dnode)
		ext_chg->enable_comp_cp_qc30 = false;
	else
		ext_chg->enable_comp_cp_qc30 =
			of_property_read_bool(dnode, "lge,enable-comp-cp-qc30");

	if (!ext_chg->enable_comp_cp_qc30)
		return;

	INIT_DELAYED_WORK(
		&ext_chg->wa_comp_cp_qc30_dwork, wa_comp_pwr_cp_qc30_work_func);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : input probation for cp setup.
////////////////////////////////////////////////////////////////////////////
#define WA_PROB_IBAT_1_DELAY        500
#define WA_PROB_IBAT_2_DELAY        500
#define WA_PROB_IBAT_DELAY_LIMIT    15000

static void wa_probate_ibat_voter_trigger(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_prob_ibat_voter)
		return;

	if (!ext_chg->is_prob_ibat_voter &&
		chg->pd_active == POWER_SUPPLY_PD_PPS_ACTIVE) {
		ext_chg->is_prob_ibat_voter = true;

		ext_chg->prob_ibat_limit = chg->batt_profile_fcc_ua / 10 * 6;
		vote_override(chg->fcc_votable, "WA_PROBATE_IBAT",
				true, ext_chg->prob_ibat_limit);

		schedule_delayed_work(&ext_chg->wa_probate_ibat_voter_dwork,
			msecs_to_jiffies(WA_PROB_IBAT_1_DELAY));

		pr_info("effective voter: %dmA\n", ext_chg->prob_ibat_limit / 1000);
	}
}

static void wa_probate_ibat_voter_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_prob_ibat_voter)
		return;

	if (ext_chg->is_prob_ibat_voter) {
		cancel_delayed_work(&ext_chg->wa_probate_ibat_voter_dwork);
		vote_override(chg->fcc_votable, "WA_PROBATE_IBAT", false, 0);
		ext_chg->is_prob_ibat_voter = false;
		pr_info("clear probate ibat\n");
	}
}

static void wa_probate_ibat_voter_reschedule(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	schedule_delayed_work(&ext_chg->wa_probate_ibat_voter_dwork,
		msecs_to_jiffies(WA_PROB_IBAT_2_DELAY));
}

static void wa_probate_ibat_voter_work_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_probate_ibat_voter_dwork.work);
	struct smb_charger *chg = ext_chg->chg;
	union power_supply_propval val = {0, };
	int cp_status1 = 0, cp_status2 = 0;
	static int work_count = 0;

	if (!chg->cp_psy)
		chg->cp_psy = power_supply_get_by_name("charge_pump_master");

	if (!chg->cp_psy) {
		wa_probate_ibat_voter_reschedule(chg);
		return;
	}

	cp_status1 = !power_supply_get_property(
		chg->cp_psy, POWER_SUPPLY_PROP_CP_STATUS1, &val) ? val.intval : 0;
	cp_status2 = !power_supply_get_property(
		chg->cp_psy, POWER_SUPPLY_PROP_CP_STATUS2, &val) ? val.intval : 0;

	pr_info("%d: cp_status1: 0x%x, cp_status2: 0x%x, flag=%d\n",
		work_count, cp_status1, cp_status2, ext_chg->is_prob_ibat_voter);

	if (!ext_chg->is_prob_ibat_voter) {
		work_count = 0;
		wa_probate_ibat_voter_clear(chg);
		return;
	}

	if ((cp_status1 == 0x4) && (cp_status2 == 0x80)) {
		work_count = 0;
		wa_probate_ibat_voter_clear(chg);
	}
	else {
		work_count++;
		if (WA_PROB_IBAT_DELAY_LIMIT
			< (WA_PROB_IBAT_1_DELAY + (WA_PROB_IBAT_2_DELAY * work_count))) {
			work_count = 0;
			wa_probate_ibat_voter_clear(chg);
		}
		else
			wa_probate_ibat_voter_reschedule(chg);
	}
}

static void wa_probate_ibat_voter_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	if (!dnode)
		ext_chg->enable_prob_ibat_voter = false;
	else
		ext_chg->enable_prob_ibat_voter =
			of_property_read_bool(dnode, "lge,enable-probate-ibat-voter");

	if (!ext_chg->enable_prob_ibat_voter)
		return;

	ext_chg->is_prob_ibat_voter = false;
	INIT_DELAYED_WORK(&ext_chg->wa_probate_ibat_voter_dwork,
			wa_probate_ibat_voter_work_func);
}

////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Compensate pps ta output error.
////////////////////////////////////////////////////////////////////////////
#define WA_COMP_PPS_PWR_MAX_ERROR         5       /*     500mA */
#define WA_COMP_PPS_PWR_IBAT_LOW_MAX      3       /* max count */
#define WA_COMP_PPS_PWR_LIMIT             2000    /*   2000 mA */
/* It takes 5sec to change PPS TA power */
#define WA_COMP_PPS_PWR_DELAY             5000    /*     5 sec */
#define WA_COMP_PPS_PWR_IBAT_LOW_DELAY    3000    /*     3 sec */
#define WA_COMP_PPS_PWR_MSOC_MAX          217     /* msoc: 85%, ui: 88% */

static void unified_nodes_store_int(const char* key, int val)
{
	char buf[12] = "0";
	snprintf(buf, sizeof(buf), "%d", val);
	unified_nodes_store("pps_ta_count", buf, strlen(buf));
}

static void wa_comp_pps_pwr_trigger(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_comp_pps_pwr)
		return;

	if (!ext_chg->is_comp_pps_pwr) {
		ext_chg->is_comp_pps_pwr = true;

		ext_chg->comp_pps_pwr_fcc = 0;
		ext_chg->comp_pps_pwr_count = 0;
		unified_nodes_store_int("pps_ta_count", 0);
		ext_chg->comp_pps_pwr_ibat_low_count = 0;
		ext_chg->comp_pps_pwr_settled = 0;

		ext_chg->comp_pps_pwr_effective_voter =
				(char*) get_effective_client(chg->fcc_votable);

		schedule_delayed_work(&ext_chg->wa_comp_pps_pwr_dwork,
				msecs_to_jiffies(WA_COMP_PPS_PWR_DELAY));
		pr_info("start. \n");
	}
}

static void wa_comp_pps_pwr_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_comp_pps_pwr)
		return;

	if (ext_chg->is_comp_pps_pwr) {
		ext_chg->is_comp_pps_pwr = false;

		ext_chg->comp_pps_pwr_fcc = 0;
		ext_chg->comp_pps_pwr_count = 0;
		unified_nodes_store_int("pps_ta_count", 0);
		ext_chg->comp_pps_pwr_ibat_low_count = 0;
		ext_chg->comp_pps_pwr_settled = 0;

		veneer_voter_release(&ext_chg->cp_pps_ibat);
		cancel_delayed_work(&ext_chg->wa_comp_pps_pwr_dwork);
		pr_info("clear done!!\n");
	}
}

static void wa_comp_pps_pwr_dwork_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work,
		struct ext_smb_charger, wa_comp_pps_pwr_dwork.work);
	struct smb_charger *chg = ext_chg->chg;
	union power_supply_propval val = {0, };
	char buff[2] = { 0, };
	char *client_voter = NULL;
	int cp_enable = 0, cp_status1 = 0, cp_status2 = 0;
	int ibat = 0, fcc = 0, original_fcc = 0;
	int new_error = 0, settled_error = 0;
	int icp_ma = 0, lcdon = 0, soc = 0;
	int cmp_size = 0;
	bool is_ibat_changed = false;

	if (!chg->batt_psy)
		chg->batt_psy = power_supply_get_by_name("battery");
	if (!chg->cp_psy)
		chg->cp_psy = power_supply_get_by_name("charge_pump_master");
	if (!chg->bms_psy)
		chg->bms_psy = power_supply_get_by_name("bms");

	if (!ext_chg->is_comp_pps_pwr ||
		chg->cp_reason != POWER_SUPPLY_CP_PPS) {
		wa_comp_pps_pwr_clear(chg);
		pr_info("clear reason -> NOT CP_PPS, flag=%d, reason=%d\n",
			ext_chg->is_comp_pps_pwr, chg->cp_reason);
		return;
	}

	if (!chg->cp_psy || !chg->batt_psy || !chg->bms_psy) {
		schedule_delayed_work(
			&ext_chg->wa_comp_pps_pwr_dwork,
			msecs_to_jiffies(WA_COMP_PPS_PWR_DELAY));
		return;
	}

	cp_enable = !power_supply_get_property(chg->cp_psy,
		POWER_SUPPLY_PROP_CP_ENABLE, &val) ? val.intval : 0;
	cp_status1 = !power_supply_get_property(chg->cp_psy,
		POWER_SUPPLY_PROP_CP_STATUS1, &val) ? val.intval : 0;
	cp_status2 = !power_supply_get_property(chg->cp_psy,
		POWER_SUPPLY_PROP_CP_STATUS2, &val) ? val.intval : 0;
	soc = !power_supply_get_property(chg->bms_psy,
		POWER_SUPPLY_PROP_CAPACITY_RAW, &val) ? val.intval : -1;

	if (!cp_enable ||
		!((cp_status1 == 0x0C || cp_status1 == 0x04) && cp_status2 == 0x80)) {

		if (soc < WA_COMP_PPS_PWR_MSOC_MAX) {
			pr_info("retry reason -> CP STATUS, "
					"soc=%d, en=%d, sts1=0x%x, sts2=0x%x\n",
				soc*100/255, cp_enable, cp_status1, cp_status2);

			schedule_delayed_work(
				&ext_chg->wa_comp_pps_pwr_dwork,
				msecs_to_jiffies(WA_COMP_PPS_PWR_DELAY));
		}
		else {
			wa_comp_pps_pwr_clear(chg);
			pr_info("clear reason -> OVER SOC & CP STATUS, "
					"soc=%d, en=%d, sts1=0x%x, sts2=0x%x\n",
				soc*100/255, cp_enable, cp_status1, cp_status2);
		}

		return;
	}

	if (unified_nodes_show("status_lcd", buff)) {
		sscanf(buff, "%d", &lcdon);
	}

	client_voter = (char*) get_effective_client(chg->fcc_votable);

	fcc = !power_supply_get_property(chg->batt_psy,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val) ? val.intval / 1000 : -1;

	original_fcc = fcc + (ext_chg->comp_pps_pwr_count * 100);

	cmp_size = min(strlen(client_voter),
					strlen(ext_chg->comp_pps_pwr_effective_voter));

	is_ibat_changed = veneer_voter_changed(VOTER_TYPE_IBAT) ||
						(original_fcc != ext_chg->comp_pps_pwr_fcc);
	if (is_ibat_changed ||
		strncmp(ext_chg->comp_pps_pwr_effective_voter, client_voter, cmp_size)) {
		/* if fcc is changed, previous offset should be cleared. */
		pr_info("retry reason -> new voter(%s) is appeared, "
				"ibat_changed=%d, total=%d, fcc(%dmA->%dmA(original=%dmA)), old=%s\n",
				client_voter,
				is_ibat_changed, ext_chg->comp_pps_pwr_count,
				ext_chg->comp_pps_pwr_fcc, fcc, original_fcc,
				ext_chg->comp_pps_pwr_effective_voter);

		ext_chg->comp_pps_pwr_count = 0;
		unified_nodes_store_int("pps_ta_count", 0);
		ext_chg->comp_pps_pwr_fcc = fcc;
		ext_chg->comp_pps_pwr_settled = 0;
		ext_chg->comp_pps_pwr_effective_voter = client_voter;

		veneer_voter_release(&ext_chg->cp_pps_ibat);

		schedule_delayed_work(
			&ext_chg->wa_comp_pps_pwr_dwork,
			msecs_to_jiffies(WA_COMP_PPS_PWR_DELAY));
		return;
	}

	ibat = !power_supply_get_property(chg->batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &val) ? val.intval / 1000 : -1;

	if ((ibat < WA_COMP_PPS_PWR_LIMIT) || (fcc <= 0)) {
		icp_ma = !power_supply_get_property(chg->cp_psy,
					POWER_SUPPLY_PROP_CP_ISNS, &val) ? val.intval/1000 : -1;

		if (icp_ma < WA_SMB1390_MIN_CURR_MA && (lcdon == 0))
			ext_chg->comp_pps_pwr_ibat_low_count++;

		if (ext_chg->comp_pps_pwr_ibat_low_count > WA_COMP_PPS_PWR_IBAT_LOW_MAX) {
			pr_info("clear reason -> ibat & icp is too low, "
					"ibat=%dmA, fcc=%dmA, original=%dmA, icp_ma=%dmA, count=%d\n",
					ibat, fcc, original_fcc, icp_ma,
					ext_chg->comp_pps_pwr_ibat_low_count);

			veneer_voter_set(
				&ext_chg->cp_pps_ibat, WA_SMB1390_DISABLE_CURR_MA);

			return;
		}

		pr_info("retry reason -> ibat is too low or fcc is under zero, "
				"ibat=%dmA, fcc=%dmA, original=%dmA, icp=%dmA, count=%d\n",
				ibat, fcc, original_fcc, icp_ma,
				ext_chg->comp_pps_pwr_ibat_low_count);

		schedule_delayed_work(
			&ext_chg->wa_comp_pps_pwr_dwork,
			msecs_to_jiffies(WA_COMP_PPS_PWR_IBAT_LOW_DELAY));

		return;
	}

	new_error = ((ibat - original_fcc) + 50 ) / 100;
	if (new_error <= 0) {
		pr_info("retry reason -> no need compensation, "
				"new=%d, total=%d, ibat=%dmA, fcc=%dmA, original=%dmA\n",
				new_error, ext_chg->comp_pps_pwr_count, ibat, fcc, original_fcc);

		schedule_delayed_work(
			&ext_chg->wa_comp_pps_pwr_dwork,
			msecs_to_jiffies(WA_COMP_PPS_PWR_DELAY));

		return;
	}

	if (new_error > 1 )
		new_error = 1;

	ext_chg->comp_pps_pwr_count += new_error;
	if (ext_chg->comp_pps_pwr_count > WA_COMP_PPS_PWR_MAX_ERROR) {
		new_error = 0;
		ext_chg->comp_pps_pwr_count = WA_COMP_PPS_PWR_MAX_ERROR;
	}
	unified_nodes_store_int("pps_ta_count", ext_chg->comp_pps_pwr_count);

	settled_error = original_fcc - (ext_chg->comp_pps_pwr_count * 100);
	if (settled_error < WA_COMP_PPS_PWR_LIMIT) {
		pr_info("retry reason -> compensated fcc is under 2A, "
				"ibat=%dmA, fcc %dmA -> %dmA(original=%dmA).\n",
			ibat, fcc, settled_error, original_fcc);

		schedule_delayed_work(
			&ext_chg->wa_comp_pps_pwr_dwork,
			msecs_to_jiffies(WA_COMP_PPS_PWR_DELAY));

		return;
	}

	if (ext_chg->comp_pps_pwr_settled != settled_error) {
		ext_chg->comp_pps_pwr_settled = settled_error;

		veneer_voter_set(&ext_chg->cp_pps_ibat, settled_error);

		ext_chg->comp_pps_pwr_effective_voter =
				(char*) get_effective_client(chg->fcc_votable);

		pr_info("compensated!!.. new=%d, total=%d, ibat=%dmA, "
				"down fcc from %dmA -> %dmA(original=%dmA).\n",
			 new_error, ext_chg->comp_pps_pwr_count, ibat,
			 fcc, settled_error, original_fcc);
	}

	ext_chg->comp_pps_pwr_ibat_low_count = 0;

	schedule_delayed_work(
		&ext_chg->wa_comp_pps_pwr_dwork,
		msecs_to_jiffies(WA_COMP_PPS_PWR_DELAY));
}

static void wa_comp_pps_pwr_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	if (!dnode)
		ext_chg->enable_comp_pps_pwr = false;
	else
		ext_chg->enable_comp_pps_pwr =
			of_property_read_bool(dnode, "lge,enable-comp-pps-pwr");

	if (!ext_chg->enable_comp_pps_pwr)
		return;

	ext_chg->comp_pps_pwr_fcc = 0;
	ext_chg->comp_pps_pwr_count = 0;
	unified_nodes_store_int("pps_ta_count", 0);
	ext_chg->comp_pps_pwr_ibat_low_count = 0;
	ext_chg->is_comp_pps_pwr = false;
	ext_chg->comp_pps_pwr_settled = 0;

	INIT_DELAYED_WORK(
		&ext_chg->wa_comp_pps_pwr_dwork, wa_comp_pps_pwr_dwork_func);
}

////////////////////////////////////////////////////////////////////////////
// LGE Workaround : abnormal operation during PPS TA CP Charing
////////////////////////////////////////////////////////////////////////////
#define WA_BAD_PPS_TA_DELAY            5000  /* 5sec           */

#if defined(CONFIG_MACH_KONA_TIMELM_DCM_JP) || \
	defined(CONFIG_MACH_KONA_TIMELM_SB_JP)
#define WA_BAD_PPS_TA_MAX_COUNT        3     /* it means 30sec */
#else
#define WA_BAD_PPS_TA_MAX_COUNT        6     /* it means 30sec */
#endif

#define WA_CP_MIN_CURRENT_MA           2000  /* 2A             */
#define WA_BAD_PPS_TA_MAX_VOLT_MV      9500  /* 9.5V           */
#define WA_BAD_PPS_TA_MIN_VOLT_MV      7500  /* 7.5V           */

bool is_bad_pps_detected(void){
	struct smb_charger* chg = wa_helper_chg();
	struct ext_smb_charger *ext_chg;
	bool ret = false;
	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return ret;
	}
	ext_chg = chg->ext_chg;
	if (ext_chg->bad_pps_ta_detected){
#if defined(CONFIG_MACH_KONA_TIMELM_DCM_JP) || \
	defined(CONFIG_MACH_KONA_TIMELM_SB_JP)
		ret =  true;
#endif
		pr_info("is_bad_pps_detected flag true..\n");
	}
	return ret;
}
EXPORT_SYMBOL(is_bad_pps_detected);

static void wa_bad_operation_pps_ta_trigger(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_bad_operation_pps_ta)
		return;

	if (chg->cp_disable_votable)
		vote(chg->cp_disable_votable, "BAD_PPS_WA", false, 0);
	if (chg->fcc_votable)
		vote(chg->fcc_votable, "BAD_PPS_WA", false, 0);

	if (!ext_chg->is_bad_operation_pps_ta) {
		ext_chg->is_bad_operation_pps_ta = true;
		ext_chg->bad_operation_pps_ta_count = 0;

		schedule_delayed_work(&ext_chg->wa_bad_operation_pps_ta_dwork,
			msecs_to_jiffies(WA_BAD_PPS_TA_DELAY));

		pr_info("trigger bad pps ta..\n");
	}
}

static void wa_bad_operation_pps_ta_clear(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	if (!ext_chg->enable_bad_operation_pps_ta)
		return;

	if (chg->cp_disable_votable)
		vote(chg->cp_disable_votable, "BAD_PPS_WA", false, 0);
	if (chg->fcc_votable)
		vote(chg->fcc_votable, "BAD_PPS_WA", false, 0);

	if (ext_chg->is_bad_operation_pps_ta) {
		ext_chg->is_bad_operation_pps_ta = false;
		ext_chg->bad_operation_pps_ta_count = 0;
		cancel_delayed_work(&ext_chg->wa_bad_operation_pps_ta_dwork);
#if defined(CONFIG_MACH_KONA_TIMELM_DCM_JP) || \
	defined(CONFIG_MACH_KONA_TIMELM_SB_JP)
		if (chg->cp_disable_votable)
			vote_override(chg->cp_disable_votable, "BAD_PPS_WA", false, 0);
#endif
		pr_info("clear bad pps ta..\n");
	}
}

static void wa_bad_operation_pps_ta_work_func(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						wa_bad_operation_pps_ta_dwork.work);
	struct smb_charger *chg = ext_chg->chg;
	union power_supply_propval val = {0, };
	int vusb_mv = 0, fcc_ma = 0, fcc_main_ma = 0, soc = 0;

	if (!chg->usb_psy)
		chg->usb_psy = power_supply_get_by_name("usb");
	if (!chg->bms_psy)
		chg->bms_psy = power_supply_get_by_name("bms");

	if (!chg->usb_psy || !chg->bms_psy || !chg->fcc_votable || !chg->fcc_main_votable) {
		schedule_delayed_work(
			&ext_chg->wa_bad_operation_pps_ta_dwork,
			msecs_to_jiffies(WA_BAD_PPS_TA_DELAY));
		return;
	}

	if (!ext_chg->is_bad_operation_pps_ta ||
		chg->cp_reason != POWER_SUPPLY_CP_PPS) {
		wa_bad_operation_pps_ta_clear(chg);
		pr_info("clear reason -> NOT CP_PPS, flag=%d, reason=%d\n",
			ext_chg->is_bad_operation_pps_ta, chg->cp_reason);
		return;
	}

	vusb_mv = !power_supply_get_property(
		chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val) ? val.intval/1000 : -1;
	fcc_ma = get_effective_result_locked(chg->fcc_votable) / 1000;
	fcc_main_ma = get_effective_result_locked(chg->fcc_main_votable) / 1000;
	soc = !power_supply_get_property(chg->bms_psy,
		POWER_SUPPLY_PROP_CAPACITY_RAW, &val) ? val.intval : -1;

	if (soc > WA_COMP_PPS_PWR_MSOC_MAX) {
		wa_bad_operation_pps_ta_clear(chg);
		pr_info("clear reason -> OVER SOC, soc=%d, flag=%d, reason=%d\n",
			soc*100/255, ext_chg->is_bad_operation_pps_ta, chg->cp_reason);
		return;
	}

	if ((vusb_mv > WA_BAD_PPS_TA_MAX_VOLT_MV) ||
		(vusb_mv > WA_BAD_PPS_TA_MIN_VOLT_MV &&
		 fcc_ma >= WA_CP_MIN_CURRENT_MA && fcc_ma == fcc_main_ma)) {
		ext_chg->bad_operation_pps_ta_count++;
		if (ext_chg->bad_operation_pps_ta_count >= WA_BAD_PPS_TA_MAX_COUNT) {
			pr_info("do pd hard reset..vusb=%dmV, fcc=%dmA, fcc_main=%dmA\n",
				vusb_mv, fcc_ma, fcc_main_ma);
			wa_bad_operation_pps_ta_clear(chg);
			ext_chg->bad_pps_ta_detected = true;
#if defined(CONFIG_MACH_KONA_TIMELM_DCM_JP) || \
	defined(CONFIG_MACH_KONA_TIMELM_SB_JP)
			if (chg->cp_disable_votable)
				vote_override(chg->cp_disable_votable, "BAD_PPS_WA", true, 0);
			if (chg->fcc_votable)
				vote_override(chg->fcc_votable, "BAD_PPS_WA", true, 3000000);
#else
			do_pd_hard_reset();
#endif
		}
		else {
			pr_info("one more check..count=%d, vusb=%dmV, fcc=%dmA, fcc_main=%dmA\n",
				ext_chg->bad_operation_pps_ta_count, vusb_mv, fcc_ma, fcc_main_ma);
			schedule_delayed_work(
				&ext_chg->wa_bad_operation_pps_ta_dwork,
				msecs_to_jiffies(WA_BAD_PPS_TA_DELAY));
		}
		return;
	}

	ext_chg->bad_operation_pps_ta_count = 0;
	schedule_delayed_work(
		&ext_chg->wa_bad_operation_pps_ta_dwork,
		msecs_to_jiffies(WA_BAD_PPS_TA_DELAY));
}

static void wa_bad_operation_pps_ta_init(struct smb_charger* chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *dnode =
		of_find_node_by_name(node, "veneer-workaround");

	if (!dnode)
		ext_chg->enable_bad_operation_pps_ta = false;
	else
		ext_chg->enable_bad_operation_pps_ta =
			of_property_read_bool(dnode, "lge,enable-bad-operation-pps-ta");

	if (!ext_chg->enable_bad_operation_pps_ta)
		return;

	ext_chg->is_bad_operation_pps_ta = false;
	ext_chg->bad_operation_pps_ta_count = 0;
	INIT_DELAYED_WORK(&ext_chg->wa_bad_operation_pps_ta_dwork,
			wa_bad_operation_pps_ta_work_func);
}

////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Helper functions init
////////////////////////////////////////////////////////////////////////////

/* update pmic register for complice test */
void wa_update_pmic_reg_with_complice_mode(struct smb_charger* chg)
{
#ifdef CONFIG_LGE_USB
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	int rc;

	if (ext_chg->is_usb_compliance_mode) {
		/*
		 * Check for VBUS at vSAFE0V before transitioning into
		 * ATTACHED.SRC state
		 */
		rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
				BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT,
				0);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set EXIT_STATE cfg rc=%d\n",
				rc);
			return;
		}

		/* disable crude sensors */
		rc = smblib_masked_write(chg, TYPE_C_CRUDE_SENSOR_CFG_REG,
				EN_SRC_CRUDE_SENSOR_BIT |
				EN_SNK_CRUDE_SENSOR_BIT,
				0);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't disable crude sensor rc=%d\n",
				rc);
			return;
		}

		/*
		 * Do not apply tPDdebounce on exit condition for
		 * attachedwait.SRC and attached.SRC
		 */
		rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
				BIT(1), 0);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set USE_TPD_FOR_EXITING_ATTACHSRC rc=%d\n",
				rc);
			return;
		}
	} else {
		/*
		 * Do not check for VBUS at vSAFE0V before transitioning into
		 * ATTACHED.SRC state
		 */
		rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
				BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT,
				BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set EXIT_STATE cfg rc=%d\n",
				rc);
			return;
		}

		/* enable crude sensors */
		rc = smblib_masked_write(chg, TYPE_C_CRUDE_SENSOR_CFG_REG,
				EN_SRC_CRUDE_SENSOR_BIT |
				EN_SNK_CRUDE_SENSOR_BIT,
				EN_SRC_CRUDE_SENSOR_BIT |
				EN_SNK_CRUDE_SENSOR_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't enable crude sensor rc=%d\n",
				rc);
			return;
		}
	}
#endif
}

/* global LGE workaround notification callback */
int wa_typec_state_change_nb(struct notifier_block *nb, unsigned long empty, void *v)
{
	struct smb_charger *chg = v;

	wa_charging_without_cc_trigger(chg);
	wa_charging_with_rd_trigger(chg);
	wa_retry_vconn_enable_on_vconn_oc_clear(chg);
	wa_disable_otg_hiccup_clear(chg);

	return NOTIFY_OK;
}

int wa_psy_change_nb(struct notifier_block *nb, unsigned long empty, void *v)
{
	struct smb_charger *chg = v;

	wa_charging_for_mcdodo_trigger(chg);
	wa_disable_cp_with_fake_mode_trigger(chg);

	return NOTIFY_OK;
}

int wa_usbin_plugin_nb(struct notifier_block *nb, unsigned long vbus, void *v)
{
	struct smb_charger *chg = v;

	wa_charging_without_cc_trigger(chg);
	wa_charging_for_mcdodo_clear(chg);
	if (vbus) { /* attached */
		wa_avoid_inrush_current_triger(chg);
		wa_faster_try_apsd_trigger(chg);
	}
	else { /* detached */
		wa_comp_pwr_cp_qc30_clear(chg);
		wa_detect_standard_hvdcp_clear(chg);
		wa_rerun_apsd_for_dcp_clear(chg);
		wa_rerun_apsd_for_sdp_clear(chg);
		wa_support_weak_supply_check(chg);
		wa_resuming_suspended_usbin_clear(chg);
		wa_charging_with_rd_clear(chg);
		wa_clear_pr_without_charger_trigger(chg);
		wa_retry_ok_to_pd_clear(chg);
		wa_avoid_inrush_current_clear(chg);
		wa_faster_try_apsd_clear(chg);
		wa_faster_try_cp_qc30_clear(chg);
		wa_fake_usb_type_with_factory_clear(chg);
		wa_retry_apsd_with_factory_clear(chg);
		wa_recover_cc_status_trigger(chg);
		wa_probate_ibat_voter_clear(chg);
		wa_comp_pps_pwr_clear(chg);
		wa_bad_operation_pps_ta_clear(chg);
		chg->ext_chg->bad_pps_ta_detected = false;
	}

	return NOTIFY_OK;
}

int wa_source_change_nb(struct notifier_block *nb, unsigned long empty, void *v)
{
	struct smb_charger *chg = v;

	wa_comp_pwr_cp_qc30_trigger(chg);
	wa_comp_pps_pwr_trigger(chg);
	wa_probate_ibat_voter_trigger(chg);
	wa_detect_standard_hvdcp_trigger(chg);
	wa_rerun_apsd_for_dcp_triger(chg);
	wa_rerun_apsd_for_sdp_triger(chg);
	wa_faster_try_cp_qc30_trigger(chg);
	wa_retry_ok_to_pd_trigger(chg);
	wa_fake_usb_type_with_factory_trigger(chg);
	wa_retry_apsd_with_factory_trigger(chg);
	wa_bad_operation_pps_ta_trigger(chg);

	return NOTIFY_OK;
}

void wa_update_usb_compliance_mode(bool mode)
{
	struct smb_charger* chg = wa_helper_chg();
	struct ext_smb_charger *ext_chg;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	ext_chg = chg->ext_chg;
	ext_chg->is_usb_compliance_mode = mode;

	wa_avoid_inrush_current_with_compliance_triger(chg);
	wa_update_pmic_reg_with_complice_mode(chg);
}

void wa_update_usb_configured(bool configured)
{
	struct smb_charger* chg = wa_helper_chg();
	struct ext_smb_charger *ext_chg;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}
	ext_chg = chg->ext_chg;

	ext_chg->is_usb_configured = configured;
}

void wa_update_hall_ic(bool hall_ic)
{
	struct smb_charger* chg = wa_helper_chg();
	struct ext_smb_charger *ext_chg;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}
	ext_chg = chg->ext_chg;

	ext_chg->is_hall_ic = hall_ic;
}

void wa_helper_init(struct smb_charger *chg)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	INIT_WORK(&ext_chg->wa_get_pmic_dump_work, wa_get_pmic_dump_func);

	wa_update_usb_compliance_mode(false);
	wa_update_usb_configured(false);
	wa_update_hall_ic(false);
	wa_detect_standard_hvdcp_init(chg);
	wa_rerun_apsd_for_dcp_init(chg);
	wa_rerun_apsd_for_sdp_init(chg);
	wa_charging_without_cc_init(chg);
	wa_support_weak_supply_init(chg);
	wa_resuming_suspended_usbin_init(chg);
	wa_charging_with_rd_init(chg);
	wa_clear_pr_without_charger_init(chg);
	wa_clear_dc_reverse_volt_init(chg);
	wa_dcin_rerun_aicl_init(chg);
	wa_recovery_vashdn_wireless_init(chg);
	wa_retry_vconn_enable_on_vconn_oc_init(chg);
	wa_retry_ok_to_pd_init(chg);
	wa_avoid_inrush_current_init(chg);
	wa_protect_overcharging_init(chg);
	wa_fake_usb_type_with_factory_init(chg);
	wa_fake_cc_status_init(chg);
	wa_faster_try_apsd_init(chg);
	wa_faster_try_cp_qc30_init(chg);
	wa_retry_apsd_with_factory_init(chg);
	wa_charging_for_mcdodo_init(chg);
	wa_disable_cp_with_fake_mode_init(chg);
	wa_disable_otg_hiccup_init(chg);
	wa_control_vbus2_regulator_init(chg);
	wa_comp_pwr_cp_qc30_init(chg);
	wa_probate_ibat_voter_init(chg);
	wa_comp_pps_pwr_init(chg);
	wa_bad_operation_pps_ta_init(chg);

	ext_chg->wa_source_change_nb.notifier_call = wa_source_change_nb;
	ext_chg->wa_usbin_plugin_nb.notifier_call = wa_usbin_plugin_nb;
	ext_chg->wa_typec_state_change_nb.notifier_call = wa_typec_state_change_nb;
	ext_chg->wa_psy_change_nb.notifier_call = wa_psy_change_nb;
	ext_chg->bad_pps_ta_detected = false;

	raw_notifier_chain_register(
		&ext_chg->usb_plugin_notifier,
		&ext_chg->wa_usbin_plugin_nb);
	raw_notifier_chain_register(
		&ext_chg->usb_source_change_notifier,
		&ext_chg->wa_source_change_nb);
	raw_notifier_chain_register(
		&ext_chg->typec_state_change_notifier,
		&ext_chg->wa_typec_state_change_nb);
	raw_notifier_chain_register(
		&ext_chg->psy_change_notifier,
		&ext_chg->wa_psy_change_nb);

	veneer_voter_register(
        &ext_chg->cp_qc3_ibat, "CP_QC3", VOTER_TYPE_IBAT, false);
	veneer_voter_register(
        &ext_chg->cp_pps_ibat, "CP_PPS", VOTER_TYPE_IBAT, false);
}
