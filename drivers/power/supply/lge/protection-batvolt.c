#define pr_fmt(fmt) "BVP: %s: " fmt, __func__
#define pr_batvolt(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_err(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

static int pr_debugmask;

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_batterydata.h>
#include <linux/workqueue.h>

#include "veneer-primitives.h"

#define BVP_NOTREADY	INT_MAX
#define BVP_VOTER	"BVP"

enum multi_fv_type {
	MFV_CC = 0,
	MFV_CV,
	MFV_MAX,
};

static void protection_batvolt_work(struct work_struct *work);

static struct protection_batvolt_struct {
	/* for protection behavoirs */
	bool (*bvp_get)(int* vnow_mv, int* icap_ma, int* inow_ma, int* chg_type);
	int (*get_veneer_param)(int id, int *val);
	int (*set_veneer_param)(int id, int val);
	struct voter_entry	bvp_voter;
	struct delayed_work	bvp_dwork;
	/* thresholds */
	int			threshold_vbat_limit;
	int			threshold_vbat_clear;
	int			threshold_ibat_rated;
	int			threshold_cv_ibat_rated;
	/* dt contents */
	int			step_ibat_ma;
	int			step_poll_ms;
	/* IR compensation Vbatt */
	bool irc_enabled;
	bool is_irc_full;  /* whether irc is supported with full range */
	int irc_resistance;
	int threshold_ibat_margin;
	/* multi - fv(float voltage) */
	bool multi_fv_enabled;
	int multi_fv_mvolt[MFV_MAX];
} bvp_me = {
	.bvp_get	= NULL,
	.bvp_voter	= { .type = VOTER_TYPE_INVALID },
	.bvp_dwork	= __DELAYED_WORK_INITIALIZER(bvp_me.bvp_dwork,
		protection_batvolt_work, 0),

	.threshold_vbat_limit		= BVP_NOTREADY,
	.threshold_vbat_clear		= BVP_NOTREADY,
	.threshold_ibat_rated		= BVP_NOTREADY,
	.threshold_cv_ibat_rated	= BVP_NOTREADY,

	.step_ibat_ma		= BVP_NOTREADY,
	.step_poll_ms		= BVP_NOTREADY,

	.irc_enabled = false,
	.is_irc_full = false,
	.irc_resistance = 0,
	.threshold_ibat_margin = 0,

	.multi_fv_enabled = false,
	.multi_fv_mvolt = {4450, 4430},
};

static bool set_multi_fv(int mode)
{
	int batt_profile_fv = 0;
	int charger = 0, mode_local = mode;
	int rc = 0;

	if (!bvp_me.multi_fv_enabled)
		return false;

	rc = bvp_me.get_veneer_param(
		VENEER_FEED_CHARGER_TYPE, &charger);
	if ((charger == CHARGING_SUPPLY_TYPE_UNKNOWN) ||
		(charger == CHARGING_SUPPLY_TYPE_FLOAT)   ||
		(charger == CHARGING_SUPPLY_TYPE_NONE)    ||
		(charger == CHARGING_SUPPLY_USB_2P0)      ||
		(charger == CHARGING_SUPPLY_USB_3PX)      ||
		(charger == CHARGING_SUPPLY_FACTORY_56K)  ||
		(charger == CHARGING_SUPPLY_FACTORY_130K) ||
		(charger == CHARGING_SUPPLY_FACTORY_910K) ||
		(charger == CHARGING_SUPPLY_WIRELESS_5W)  ||
		(charger == CHARGING_SUPPLY_WIRELESS_9W)  ){
        mode_local = MFV_CV;
    }

	rc = bvp_me.get_veneer_param(
		VENEER_FEED_BATT_PROFILE_FV_VOTER, &batt_profile_fv);
	if (!rc && batt_profile_fv > 0) {
		if (batt_profile_fv != bvp_me.multi_fv_mvolt[mode_local]) {
			rc = bvp_me.set_veneer_param(
				VENEER_FEED_BATT_PROFILE_FV_VOTER,
				bvp_me.multi_fv_mvolt[mode_local]);
			pr_batvolt(UPDATE, "%s multi-fv=%dmV...\n",
				(mode_local == MFV_CC) ? "initialize," : "enter CV mode,",
				bvp_me.multi_fv_mvolt[mode_local]);
		}
	}

	return !rc;
}

static void protection_batvolt_work(struct work_struct *work)
{
	// ibat_now: + is charging, - is discharging
	int vbat_now, icap_now, ibat_now, chg_now;
	int icap_new, virc_new = 0, ui_soc = 0;
	int irc_enabled = bvp_me.irc_enabled, irc_resistance = bvp_me.irc_resistance;
	int step_ibat = bvp_me.step_ibat_ma;

	if (!bvp_me.get_veneer_param(VENEER_FEED_IRC_ENABLED, &irc_enabled))
		bvp_me.irc_enabled = !!irc_enabled;
	if (!bvp_me.get_veneer_param(VENEER_FEED_IRC_RESISTANCE, &irc_resistance)) {
		bvp_me.is_irc_full = !!(irc_resistance / 1000);
		bvp_me.irc_resistance = irc_resistance % 1000;
	}
	if (bvp_me.get_veneer_param(VENEER_FEED_CAPACITY, &ui_soc))
		ui_soc = 0;

	if (bvp_me.bvp_get(&vbat_now, &icap_now, &ibat_now, &chg_now)) {
		if (irc_enabled
			&& chg_now == POWER_SUPPLY_CHARGE_TYPE_FAST
			&& bvp_me.irc_resistance > 0
			&& vbat_now <= MAX_IRC_VOLTAGE ) {
			virc_new = vbat_now - ((ibat_now * bvp_me.irc_resistance) / 1000);
			if (!bvp_me.is_irc_full && (ibat_now < bvp_me.threshold_ibat_rated))
				virc_new = vbat_now;

			pr_batvolt(UPDATE,
				"IR Compensated(res=%d, %d): "
				"virc=%d(-%d), vbatt=%d, ibat=%d, icap=%d, isix=%d\n",
				bvp_me.irc_resistance, irc_resistance, virc_new, vbat_now - virc_new, vbat_now,
				ibat_now, icap_now, bvp_me.threshold_ibat_rated);

		} else
			virc_new = vbat_now;

		if (virc_new <= bvp_me.threshold_vbat_limit) {
			pr_batvolt(VERBOSE, "Under voltage (%d)\n", virc_new);

			if (veneer_voter_enabled(&bvp_me.bvp_voter)
				&& virc_new <= bvp_me.threshold_vbat_clear) {
				pr_batvolt(UPDATE, "Clear batvolt protection\n");
				veneer_voter_release(&bvp_me.bvp_voter);
			}

			goto done;
		}

		if (chg_now != POWER_SUPPLY_CHARGE_TYPE_TAPER
			&& icap_now <=
				(bvp_me.threshold_ibat_rated - bvp_me.threshold_ibat_margin)) {
			pr_batvolt(VERBOSE, "Under C-rate (%d)\n", icap_now);
			goto done;
		}

		if (chg_now == POWER_SUPPLY_CHARGE_TYPE_TAPER
				&& icap_now <= bvp_me.threshold_cv_ibat_rated) {
			if (bvp_me.multi_fv_enabled) {
				set_multi_fv(MFV_CV);
			}
			else {
				pr_batvolt(VERBOSE, "Under C-rate (%d) on CV\n", icap_now);
			}
			goto done;
		}

		if (ibat_now > 3200)
			step_ibat = bvp_me.step_ibat_ma * 2;
	}
	else {
		pr_batvolt(UPDATE, "Host is not ready\n");
		goto done;
	}

	icap_new = (icap_now - step_ibat) / step_ibat * step_ibat;
	veneer_voter_set(&bvp_me.bvp_voter, icap_new);
	if (ui_soc >= 100)
		set_multi_fv(MFV_CV);
	else
		set_multi_fv(MFV_CC);

	pr_batvolt(UPDATE, "Condition : %dmv, %dma, chg type %d => Reduce IBAT to %d\n",
		virc_new, icap_now, chg_now, icap_new);
done:
	if (ui_soc >= 100)
		set_multi_fv(MFV_CV);
	schedule_delayed_work(to_delayed_work(work), msecs_to_jiffies(bvp_me.step_poll_ms));
	return;
}

void protection_batvolt_refresh(bool is_charging) {
	static bool is_started = false;

	bool is_ready = bvp_me.threshold_vbat_limit != BVP_NOTREADY
		&& bvp_me.threshold_ibat_rated != BVP_NOTREADY
		&& is_charging;

	if (is_ready) {
		if (!is_started) {
			schedule_delayed_work(&bvp_me.bvp_dwork, 0);
			is_started = true;
		}
		else
			; // Skip to handle BVP
	}
	else {
		veneer_voter_release(&bvp_me.bvp_voter);
		set_multi_fv(MFV_CC);
		cancel_delayed_work_sync(&bvp_me.bvp_dwork);
		is_started = false;
	}
}

bool protection_batvolt_create(struct device_node* dnode, int mincap,
	bool (*feed_protection_batvolt)(int* vnow_mv, int* icap_ma, int* inow_ma, int* chg_type),
	int (*get_veneer_param)(int id, int *val),
	int (*set_veneer_param)(int id, int val)
){
	int ret = 0, threshold_ibat_pct = 0, threshold_cv_ibat_pct = 0;
	int max_irc_mohm = 0;
	pr_debugmask = ERROR | UPDATE;

	/* Parse device tree */
	ret = of_property_read_s32(dnode, "lge,threshold-ibat-pct", &threshold_ibat_pct);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-ibat-pct' ret=%d\n", ret);
		goto destroy;
	}
	else
		bvp_me.threshold_ibat_rated = mincap * threshold_ibat_pct / 100;

	ret = of_property_read_s32(dnode, "lge,threshold-ibat-margin", &bvp_me.threshold_ibat_margin);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-ibat-margin' ret=%d\n", ret);
		bvp_me.threshold_ibat_margin = 0;
	}

	ret = of_property_read_s32(dnode, "lge,threshold-cv-ibat-pct", &threshold_cv_ibat_pct);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-cv-ibat-pct' ret=%d\n", ret);
		goto destroy;
	}
	else
		bvp_me.threshold_cv_ibat_rated = mincap * threshold_cv_ibat_pct / 100;

	ret = of_property_read_s32(dnode, "lge,threshold-vbat-limit", &bvp_me.threshold_vbat_limit);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-vbat-limit' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,threshold-vbat-clear", &bvp_me.threshold_vbat_clear);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-vbat-clear' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,step-ibat-ma", &bvp_me.step_ibat_ma);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,step-ibat-ma' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,step-poll-ms", &bvp_me.step_poll_ms);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,step-poll-ms' ret=%d\n", ret);
		goto destroy;
	}

	bvp_me.irc_enabled = false;
	if (of_property_read_bool(dnode, "lge,irc-enable")) {
		bvp_me.irc_enabled = true;
		ret = of_property_read_s32(dnode, "lge,irc-resist-mohm", &bvp_me.irc_resistance);
		if (ret < 0) {
			bvp_me.irc_resistance = 0;
		}

		if (bvp_me.irc_resistance) {
			bvp_me.is_irc_full = !!(bvp_me.irc_resistance / 1000);
			bvp_me.irc_resistance %=  1000;
			max_irc_mohm =
				(MAX_IRC_VOLTAGE - bvp_me.threshold_vbat_limit) * 1000 / mincap;
			pr_batvolt(UPDATE,
				"IR Compensation is enabled: %d mohm (dt=%d, max=%d, full=%d)\n",
				min(bvp_me.irc_resistance, max_irc_mohm), bvp_me.irc_resistance, max_irc_mohm, bvp_me.is_irc_full);
			bvp_me.irc_resistance = min(bvp_me.irc_resistance, max_irc_mohm);
		}
	}

	bvp_me.multi_fv_enabled = of_property_read_bool(dnode, "lge,mulit-fv-enable");
	ret = of_property_read_u32_array(dnode,
		"lge,multi-fv-mvolt", bvp_me.multi_fv_mvolt, MFV_MAX);
	if (ret < 0) {
		bvp_me.multi_fv_mvolt[0] = 4450;
		bvp_me.multi_fv_mvolt[1] = 4430;
	}

	/* Fill callback */
	if (!feed_protection_batvolt) {
		pr_batvolt(ERROR, "feed func(feed_protection_batvolt) should not be null\n");
		goto destroy;
	}
	else
		bvp_me.bvp_get = feed_protection_batvolt;

	if (!get_veneer_param) {
		pr_batvolt(ERROR, "feed func(get_veneer_param) should not be null\n");
		goto destroy;
	}
	else
		bvp_me.get_veneer_param = get_veneer_param;

	if (!set_veneer_param) {
		pr_batvolt(ERROR, "feed func(set_veneer_param) should not be null\n");
		goto destroy;
	}
	else
		bvp_me.set_veneer_param = set_veneer_param;

	/* Register voter */
	if (!veneer_voter_register(&bvp_me.bvp_voter, BVP_VOTER, VOTER_TYPE_IBAT, false)) {
		pr_batvolt(ERROR, "Failed to register the BVP voter\n");
		goto destroy;
	}

	pr_batvolt(UPDATE, "Complete to create, "
		"threshold_vbat_limit(%d), threshold_vbat_clear(%d), "
		"threshold_ibat_rated(%d, marging=%dmA), step_ibat_ma(%d), step_poll_ms(%d)\n",
		bvp_me.threshold_vbat_limit, bvp_me.threshold_vbat_clear,
		bvp_me.threshold_ibat_rated, bvp_me.threshold_ibat_margin,
		bvp_me.step_ibat_ma, bvp_me.step_poll_ms);

	return true;

destroy:
	protection_batvolt_destroy();
	return false;
}

void protection_batvolt_destroy(void) {
	cancel_delayed_work_sync(&bvp_me.bvp_dwork);
	veneer_voter_unregister(&bvp_me.bvp_voter);
	bvp_me.bvp_get = NULL;

	bvp_me.threshold_vbat_limit	= BVP_NOTREADY;
	bvp_me.threshold_vbat_clear	= BVP_NOTREADY;
	bvp_me.threshold_ibat_rated	= BVP_NOTREADY;
	bvp_me.threshold_cv_ibat_rated	= BVP_NOTREADY;

	bvp_me.step_ibat_ma		= BVP_NOTREADY;
	bvp_me.step_poll_ms		= BVP_NOTREADY;

	bvp_me.irc_enabled = false;
	bvp_me.is_irc_full = false;
	bvp_me.irc_resistance = 0;
}
