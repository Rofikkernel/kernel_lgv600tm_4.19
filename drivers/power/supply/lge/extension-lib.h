
#ifndef __EXTENSION_LIB_H
#define __EXTENSION_LIB_H

#include "veneer-primitives.h"

#define APSD_RERUN_DELAY_MS 4000

#define USBIN_CMD_ICL_OVERRIDE_REG (USBIN_BASE + 0x42)
#define ICL_OVERRIDE_BIT BIT(0)

#define POWER_PATH_MASK		GENMASK(2, 1)
#define POWER_PATH_BATTERY	BIT(1)
#define POWER_PATH_USB		BIT(2)

#define AICL_RERUN_TIME_MASK	GENMASK(1, 0)
#define AICL_FAIL_BIT			BIT(1)

#define DCIN_VASHDN_RT_STS	BIT(1)

#define TYPEC_VCONN_OVERCURR_STATUS_BIT	BIT(2)

#define INPUT_PRESENT_USB	BIT(1)
#define INPUT_PRESENT_DC	BIT(2)

#define TYPE_C_DEBUG_ACCESS_SNK_CFG_REG			(TYPEC_BASE + 0x4A)
#define DAM_DIS_AICL_BIT				BIT(3)

#define VADP_DECR_TIMER_SEL_CFG_REG	(USBIN_BASE + 0x64)
#define HVDCP_DIS_NON_COMP_CFG_BIT	BIT(4)

#define SAFETY_TIMER_ENABLE_CFG_REG	(CHGR_BASE + 0xA0)
#define PRE_CHARGE_SAFETY_TIMER_EN	BIT(1)
#define FAST_CHARGE_SAFETY_TIMER_EN	BIT(0)

struct ext_smb_charger {
	struct smb_charger *chg;
	struct voter_entry cp_qc3_ibat;
	struct voter_entry cp_pps_ibat;

	struct gpio_desc	*vconn_boost_en_gpio;
	struct gpio_desc	*ds_en_gpio;
	struct gpio_desc	*load_sw_on_gpio;

	bool is_usb_configured;
	bool is_usb_compliance_mode;
	bool is_hall_ic;

	struct work_struct	wa_get_pmic_dump_work;

	struct work_struct	usb_plugin_work;
	struct work_struct	usb_source_change_work;
	struct work_struct	typec_state_change_work;
	struct work_struct	psy_change_work;
	struct notifier_block psy_change_nb;

	struct raw_notifier_head usb_plugin_notifier;
	struct raw_notifier_head usb_source_change_notifier;
	struct raw_notifier_head typec_state_change_notifier;
	struct raw_notifier_head psy_change_notifier;

	// Detection of Standard HVDCP2
	bool enable_detect_standard_hvdcp;
	bool wa_is_standard_hvdcp;
	bool wa_detect_standard_hvdcp_done;
	struct work_struct	wa_detect_standard_hvdcp_work;

	// Rerun apsd for dcp charger
	bool enable_rerun_apsd_dcp;
	bool wa_rerun_apsd_done;
	struct delayed_work	wa_rerun_apsd_for_dcp_dwork;

	// Rerun apsd for abnormal sdp
	bool enable_rerun_apsd_sdp;
	bool wa_rerun_apsd_done_with_sdp;
	struct delayed_work	wa_rerun_apsd_for_sdp_dwork;

	// Charging without CC
	bool enable_charging_without_cc;
	bool wa_charging_without_cc_processed;
	struct delayed_work	wa_charging_without_cc_dwork;

	// Support for weak battery pack
	bool enable_support_weak_supply;
	int	wa_support_weak_supply_count;
	bool wa_support_weak_supply_running;
	struct delayed_work	wa_support_weak_supply_dwork;

	// Resuming Suspended USBIN
	bool enable_resuming_suspended_usbin;
	int wa_resuming_suspended_usbin_attempts;
	struct delayed_work	wa_resuming_suspended_usbin_dwork;

	// Charging with Rd-open charger.
	bool enable_charging_with_rd;
	bool wa_charging_with_rd_running;

	// Clear power role when charger is unpluged
	bool enable_clear_power_role;

	// Clear DC Reverse Voltage status
	bool enable_clear_dc_reverse_volt;

	// Rerun DC AICL.
	bool enable_dcin_rerun_aicl;
	struct delayed_work	wa_dcin_rerun_aicl_dwork;

	// Recovery vashdn during wireless charging
	bool enable_recovery_vashdn_wireless;
	struct delayed_work	wa_recovery_vashdn_wireless_dwork;

	// Retry to enable vconn on vconn-oc
	bool enable_retry_vconn_with_oc;
	int wa_vconn_attempts;
	struct work_struct	wa_retry_vconn_enable_on_vconn_oc_work;

	// Retry pd check
	bool enable_retry_ok_to_pd;
	bool wa_retry_ok_to_pd;

	// Avoid Inrush current for USB Compliance test
	bool enable_avoid_inrush_current;
	struct delayed_work	wa_avoid_inrush_current_dwork;

	// avoid over voltage by abnormal charger
	bool enable_protect_overcharging;
	int prot_overchg_ent_dischg_off;
	int prot_overchg_ent_chg_off;
	int prot_overchg_rel_off;

	// Faster try APSD for HVDCP test
	bool enable_faster_try_apsd;
	bool wa_faster_try_apsd_running;

	// Faster try QC 3.0 for 2nd charger test
	bool enable_faster_try;
	bool wa_faster_try_running;
	int wa_target_cnt;

	// Fake USB type to SDP on Factory cable.
	bool enable_faked_usb_type;
	bool faked_usb_type;

	// Disable CP charging in battery fake mode
	bool enable_disable_cp_with_fake_mode;

	// Disable hiccup of otg in compliance mode
	bool enable_disable_otg_hiccup;
	struct delayed_work wa_disable_otg_hiccup_dwork;

	// Control Vbus2 regulator
	bool enable_control_vbus2_regulator;

	// Recover & Fake CC status in factory mode.
	bool enable_recover_cc;
	int  wa_recover_cc_attempts;
	struct delayed_work	wa_recover_cc_status_dwork;

	// Retry APSD in factory mode.
	bool enable_retry_apsd_with_factory;
	int  wa_apsd_apsd_attempts;

	// compensate the power of cp qc3.0 TA
	bool enable_comp_cp_qc30;
	bool is_comp_cp_qc30;
	int bad_icp_cnt;
	struct delayed_work wa_comp_cp_qc30_dwork;

	//input probation for cp setup.
	bool enable_prob_ibat_voter;
	bool is_prob_ibat_voter;
	int prob_ibat_limit;
	struct delayed_work wa_probate_ibat_voter_dwork;

	// Charging for mcdodo
	bool enable_charging_for_mcdodo;

	// Compensate pps ta output error.
	bool enable_comp_pps_pwr;
	bool is_comp_pps_pwr;
	int comp_pps_pwr_count;
	int comp_pps_pwr_ibat_low_count;
	int comp_pps_pwr_fcc;
	int comp_pps_pwr_settled;
	char *comp_pps_pwr_effective_voter;
	struct delayed_work wa_comp_pps_pwr_dwork;

	// abnormal operation during PPS TA CP Charing
	bool enable_bad_operation_pps_ta;
	bool is_bad_operation_pps_ta;
	bool bad_pps_ta_detected;
	int bad_operation_pps_ta_count;
	struct delayed_work wa_bad_operation_pps_ta_dwork;

	/* global LGE workaround notification callback */
	struct notifier_block wa_usbin_plugin_nb;
	struct notifier_block wa_source_change_nb;
	struct notifier_block wa_typec_state_change_nb;
	struct notifier_block wa_psy_change_nb;
};


static struct power_supply_desc batt_psy_desc_extension;
enum power_supply_property* extension_battery_properties(void);
size_t extension_battery_num_properties(void);
int extension_battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);
int extension_battery_set_property(struct power_supply *psy, enum power_supply_property psp, const union power_supply_propval *val);
int extension_battery_property_is_writeable(struct power_supply *psy, enum power_supply_property psp);

static struct power_supply_desc usb_psy_desc_extension;
enum power_supply_property* extension_usb_properties(void);
size_t extension_usb_num_properties(void);
int extension_usb_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);
int extension_usb_set_property(struct power_supply *psy, enum power_supply_property psp, const union power_supply_propval *val);
int extension_usb_property_is_writeable(struct power_supply *psy, enum power_supply_property psp);

static struct power_supply_desc usb_main_psy_desc_extension;
enum power_supply_property* extension_usb_port_properties(void);
size_t extension_usb_port_num_properties(void);
int extension_usb_port_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);
static struct power_supply_desc usb_port_psy_desc_extension;
int extension_usb_main_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);
int extension_usb_main_set_property(struct power_supply *psy, enum power_supply_property psp, const union power_supply_propval *val);

static struct power_supply_desc dc_psy_desc_extension;
enum power_supply_property* extension_dc_properties(void);
size_t extension_dc_num_properties(void);
int extension_dc_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);
int extension_dc_set_property(struct power_supply* psy, enum power_supply_property psp, const union power_supply_propval* val);

int extension_smb5_probe(struct smb_charger *chg);
int extension_get_apsd_result(struct smb_charger *chg);
void extension_typec_src_removal(struct smb_charger *chg);
void extension_smb5_determine_initial_status(struct smb_charger *chg, void *data);
void extension_hvdcp_detect_try_enable(struct smb_charger *chg, bool enable);

int override_vconn_regulator_enable(struct regulator_dev *rdev);
int override_vconn_regulator_disable(struct regulator_dev *rdev);
int override_vbus_regulator_enable(struct regulator_dev *rdev);
int override_vbus_regulator_disable(struct regulator_dev *rdev);

irqreturn_t override_usb_plugin_irq_handler(int irq, void *data);
irqreturn_t override_chg_state_change_irq_handler(int irq, void *data);
irqreturn_t override_typec_attach_detach_irq_handler(int irq, void *data);
irqreturn_t override_usb_source_change_irq_handler(int irq, void *data);
irqreturn_t override_usbin_uv_irq_handler(int irq, void *data);
irqreturn_t override_switcher_power_ok_irq_handler(int irq, void *data);
irqreturn_t override_typec_state_change_irq_handler(int irq, void *data);
irqreturn_t override_typec_or_rid_detection_change_irq_handler(int irq, void *data);
irqreturn_t override_aicl_fail_irq_handler(int irq, void *data);
irqreturn_t override_typec_vconn_oc_irq_handler(int irq, void *data);
irqreturn_t override_dcin_irq_handler(int irq, void *data);
irqreturn_t override_dc_plugin_irq_handler(int irq, void *data);
irqreturn_t override_dcin_uv_irq_handler(int irq, void *data);
irqreturn_t override_otg_oc_hiccup_irq_handler(int irq, void *data);
irqreturn_t override_otg_fault_irq_handler(int irq, void *data);

bool wa_detect_standard_hvdcp_check(void);
void wa_support_weak_supply_trigger(struct smb_charger* chg, u8 stat);
void wa_resuming_suspended_usbin_trigger(struct smb_charger* chg);
void wa_clear_dc_reverse_volt_trigger(bool enable);
void wa_dcin_rerun_aicl_trigger(struct smb_charger* chg);
void wa_dcin_rerun_aicl_clear(struct smb_charger* chg);
void wa_recovery_vashdn_wireless_trigger(struct smb_charger* chg);
bool wa_charging_without_cc_is_running(struct smb_charger* chg);
void wa_retry_vconn_enable_on_vconn_oc_trigger(struct smb_charger* chg);
bool wa_fake_cc_status_is_runnging(struct smb_charger *chg);
void wa_disable_otg_hiccup_trigger(struct smb_charger *chg);
bool wa_charging_with_rd_is_running(struct smb_charger* chg);
int wa_protect_overcharging(struct smb_charger* chg, int input_present);
bool wa_avoiding_mbg_fault_usbid(bool enable);
bool wa_fake_usb_type_with_factory_is_running(struct smb_charger *chg);
void wa_control_vbus2_regulator(struct smb_charger *chg, bool on);

void wa_get_pmic_dump(struct smb_charger *chg);
bool wa_command_icl_override(struct smb_charger* chg);
void wa_helper_init(struct smb_charger *chg);

extern bool unified_bootmode_fabproc(void);
extern bool unified_bootmode_usermode(void);
extern bool unified_bootmode_chargerlogo(void);
#endif /* __EXTENSION_LIB_H */
