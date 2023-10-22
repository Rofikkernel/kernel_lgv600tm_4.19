/*
 * qns_system.c version 2.0
 * Qnovo QNS wrapper implementation. Compatible with kernel 4.14.
 * Copyright (C) 2014 Qnovo Corp
 * Miro Zmrzli <miro@qnovocorp.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)	"QNS: %s: " fmt, __func__
#define pr_qns(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_err(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/alarmtimer.h>

enum qns_logmask {
    PR_ERROR    = BIT(0),
    PR_INFO     = BIT(1),
    PR_DEBUG    = BIT(2),
};
static int pr_debugmask = PR_ERROR | PR_INFO;

// Set the sign (-1 or 1) so that /sys/class/qns/current_now returns negative
// values while discharging and positive while charging.
#define READ_CURRENT_SIGN	(1)

#define CHARGE_CURRENT_PROP	POWER_SUPPLY_PROP_CURRENT_QNOVO
#define CHARGE_VOLTAGE_PROP POWER_SUPPLY_PROP_VOLTAGE_QNOVO

#define IBATMANAME		"battery"

#define QNS_OK		0
#define QNS_ERROR	(-1)

struct qns_data
{
	struct power_supply * battery_psy;
	struct alarm alarm;
	bool alarm_inited;
	int alarm_value;
	struct wakeup_source *wakelock;
	bool wakelock_inited;
	bool wakelock_held;
	struct wakeup_source *charge_wakelock;
	bool charge_wakelock_inited;
	bool charge_wakelock_held;
	int options;
};

static struct qns_data data;
#ifdef CONFIG_QPNP_QNOVO5
/* if qni driver is existed, qna sysfs is registered on qni driver */
#else
static int qna_meter;
static int qna_safety;
static int qna_vtwarm;
static int qna_pcap;
#endif

static bool qns_has_psy(void)
{
	if (data.battery_psy == NULL) {
		data.battery_psy = power_supply_get_by_name(IBATMANAME);
		if (data.battery_psy == NULL) {
			pr_qns(PR_ERROR, "ERROR: unable to get battery\n");
			return false;
		}
	}
	return true;
}

static int fcc_ma = 0, fv_mv = 0;
static int qns_set_ibat(int ibatmA)
{
	union power_supply_propval propVal = {ibatmA * 1000,};

	if (!qns_has_psy())
		return QNS_ERROR;

	if (power_supply_set_property(data.battery_psy,
			CHARGE_CURRENT_PROP, &propVal) != 0) {
		pr_qns(PR_ERROR, "ERROR: set charging current..%dmA\n", ibatmA);
		return QNS_ERROR;
	}

	fcc_ma = ibatmA;
	pr_qns(PR_INFO, "write fcc: %dmA, fv:%dmV\n", fcc_ma, fv_mv);
	return QNS_OK;
}

static int qns_set_vbat(int vbatmV)
{
	union power_supply_propval propVal = {vbatmV * 1000,};

	if (!qns_has_psy())
		return QNS_ERROR;

	if (power_supply_set_property(data.battery_psy,
			CHARGE_VOLTAGE_PROP, &propVal) != 0) {
		pr_qns(PR_ERROR, "ERROR: set charging voltage...%dmV\n", vbatmV);
		return QNS_ERROR;
	}

	fv_mv = vbatmV;
	return QNS_OK;
}

static bool qns_is_charging(void)
{
	union power_supply_propval propVal = {0, };

	if (!qns_has_psy())
		return false;

	if (power_supply_get_property(data.battery_psy,
		POWER_SUPPLY_PROP_STATUS_RAW, &propVal) != 0) {
		pr_qns(PR_ERROR, "ERROR: unable to read charger properties\n");
		return false;
	}

	return (propVal.intval == POWER_SUPPLY_STATUS_CHARGING);
}

static int qns_get_scvt(int *soc, int *c, int *v, int *tx10)
{
	/*
	  	soc in %
		c in ma
		v in mv
		t in 0.1 deg c
	*/
	union power_supply_propval ret = {0,};
	int vbat = 0, ibat = 0;
	int retVal = QNS_OK;

	if (!qns_has_psy())
		retVal = QNS_ERROR;

	if (data.battery_psy) {
		if (c != NULL) {
			if (power_supply_get_property(data.battery_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &ret) != 0) {
				pr_qns(PR_ERROR, "ERROR: read POWER_SUPPLY_PROP_CURRENT_NOW\n");
				*c = 0;
				retVal = QNS_ERROR;
			}
			else {
				ibat = READ_CURRENT_SIGN * ret.intval / 1000;
				*c = ibat;
			}
		}

		if (v != NULL) {
			if (power_supply_get_property(data.battery_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret) != 0) {
				pr_qns(PR_ERROR, "ERROR: read POWER_SUPPLY_PROP_VOLTAGE_NOW\n");
				*v = 0;
				retVal = QNS_ERROR;
			}
			else {
				vbat = ret.intval / 1000;
				*v = vbat;
			}
		}

		if (tx10 != NULL) {
			if (power_supply_get_property(data.battery_psy,
					POWER_SUPPLY_PROP_TEMP, &ret) != 0) {
				pr_qns(PR_ERROR, "ERROR: read POWER_SUPPLY_PROP_TEMP\n");
				*tx10 = 0;
				retVal = QNS_ERROR;
			}
			else {
				*tx10 = ret.intval;
				pr_qns(PR_DEBUG, "read tx10: %d\n", ret.intval);
			}
		}

		if (soc != NULL) {
			if (power_supply_get_property(data.battery_psy,
					POWER_SUPPLY_PROP_CAPACITY_RAW, &ret) != 0) {
				pr_qns(PR_ERROR, "ERROR: read POWER_SUPPLY_PROP_CAPACITY_RAW\n");
				*soc = 0;
				retVal = QNS_ERROR;
			}
			else {
				*soc = ret.intval * 100 / 255;
				pr_qns(PR_DEBUG, "read soc: %d\n", ret.intval);
			}
		}
	} else {
		pr_qns(PR_ERROR, "battery power supply is not registered yet.\n");
		if (c != NULL) *c = 0;
		if (v != NULL) *v = 4000;
		if (tx10 != NULL) *tx10 = 250;
		if (soc != NULL) *soc = 50;

		retVal = QNS_ERROR;
	}

	return retVal;
}

static int qns_get_fcc_design(int *fcc, int *design)
{
	union power_supply_propval ret = {0,};
	int retVal = QNS_OK;

	if (!qns_has_psy())
		return QNS_ERROR;

	if (fcc != NULL) {
		if(power_supply_get_property(data.battery_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL, &ret) != 0) {
			pr_qns(PR_ERROR, "ERROR: read POWER_SUPPLY_PROP_CHARGE_FULL\n");
			*fcc = 0;
			retVal = QNS_ERROR;
		}
		else {
			*fcc = ret.intval / 1000;
			pr_qns(PR_INFO, "read fcc: %d mA\n", ret.intval / 1000);
		}
	}

	if (design != NULL) {
		if(power_supply_get_property(data.battery_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret) != 0) {
			pr_qns(PR_ERROR, "ERROR: read POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN\n");
			*design = 0;
			retVal = QNS_ERROR;
		}
		else {
			*design = ret.intval / 1000;
			pr_qns(PR_INFO, "read design capacity: %d mAh", ret.intval / 1000);
		}
	}

	return retVal;
}

static int qns_get_battery_type(const char **battery_type)
{
	union power_supply_propval ret = {0,};
	int retVal = QNS_OK;

	if (!qns_has_psy()) {
		*battery_type = "Unknown";
		return QNS_ERROR;
	}

	if (battery_type != NULL) {
		if (power_supply_get_property(data.battery_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &ret) != 0) {
			pr_qns(PR_ERROR, "ERROR: read POWER_SUPPLY_PROP_BATTERY_TYPE\n");
			*battery_type = "Unknown";
			retVal = QNS_ERROR;
		}
		else {
			*battery_type = ret.strval;
			pr_qns(PR_INFO, "read battery type: %s\n", ret.strval);
		}
	}

	return retVal;
}

/* charging_state handlers */
static ssize_t
charging_state_show(struct class *class, struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", qns_is_charging() ? 1 : 0);
}
static CLASS_ATTR_RO(charging_state);

/* current_now & voltage handlers */
static int c = 0, v = 0;
static ssize_t
current_now_show(struct class *class, struct class_attribute *attr, char *buf)
{
	qns_get_scvt(NULL, &c, &v, NULL);
	return scnprintf(buf, PAGE_SIZE, "%d\n", c);
}
static CLASS_ATTR_RO(current_now);

static ssize_t
voltage_show(struct class *class, struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static CLASS_ATTR_RO(voltage);

/* temp handlers */
static ssize_t
temp_show(struct class *class, struct class_attribute *attr, char *buf)
{
	int t;
	qns_get_scvt(NULL, NULL, NULL, &t);
	return scnprintf(buf, PAGE_SIZE, "%d\n", t);
}
static CLASS_ATTR_RO(temp);

/* fcc handlers */
static ssize_t
fcc_show(struct class *class, struct class_attribute *attr, char *buf)
{
	int fcc = 0;
	qns_get_fcc_design(&fcc, NULL);
	return scnprintf(buf, PAGE_SIZE, "%d\n", fcc);
}
static CLASS_ATTR_RO(fcc);

/* design handlers */
static ssize_t
design_show(struct class *class, struct class_attribute *attr, char *buf)
{
	int design = 0;
	qns_get_fcc_design(NULL, &design);
	return scnprintf(buf, PAGE_SIZE, "%d\n", design);
}
static CLASS_ATTR_RO(design);

/* soc handlers */
static ssize_t
soc_show(struct class *class, struct class_attribute *attr, char *buf)
{
	int soc = 0;
	qns_get_scvt(&soc, NULL, NULL, NULL);
	return scnprintf(buf, PAGE_SIZE, "%d\n", soc);
}
static CLASS_ATTR_RO(soc);

/* battery_type handlers */
static ssize_t
battery_type_show(struct class *class, struct class_attribute *attr, char *buf)
{
	const char *battery_type;
	qns_get_battery_type(&battery_type);
	return scnprintf(buf, PAGE_SIZE, "%s\n", battery_type);
}
static CLASS_ATTR_RO(battery_type);

/* charge_current handlers */
static ssize_t
charge_current_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	int val = 0, ret = -EINVAL;

	ret = kstrtoint(buf, 10, &val);

	if (!ret && (val > 0)) {
		qns_set_ibat(val);
		return count;
	}

	return -EINVAL;
}
static CLASS_ATTR_WO(charge_current);

/* charge_voltage handlers */
static ssize_t
charge_voltage_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	int val = 0, ret = -EINVAL;

	ret = kstrtoint(buf, 10, &val);

	if (!ret && (val > 0)) {
		qns_set_vbat(val);
		return count;
	}

	return -EINVAL;
}
static CLASS_ATTR_WO(charge_voltage);

/* options handlers */
static ssize_t
options_show(struct class *class, struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", data.options);
}

static ssize_t
options_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	int val = 0, ret = -EINVAL;

	ret = kstrtoint(buf, 10, &val);

	if (!ret && (val >= 0)) {
		data.options = val;
		return count;
	}

	return -EINVAL;
}
static CLASS_ATTR_RW(options);

/* alarm handlers */
static enum alarmtimer_restart
qns_alarm_handler(struct alarm * alarm, ktime_t now)
{
	pr_qns(PR_DEBUG, "ALARM! System wakeup!\n");
	__pm_stay_awake(data.wakelock);
	data.wakelock_held = true;
	data.alarm_value = 1;
	return ALARMTIMER_NORESTART;
}

enum alarm_values
{
	CHARGE_WAKELOCK = -4,
	CHARGE_WAKELOCK_RELEASE = -3,
	HANDLED = -2,
	CANCEL = -1,
	IMMEDIATE = 0,
};

static ssize_t
alarm_show(struct class *class, struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", data.alarm_value);
}

static ssize_t
alarm_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	int val = 0, ret = -EINVAL;
	ktime_t next_alarm;

	ret = kstrtoint(buf, 10, &val);

	if (!data.wakelock_inited) {
		data.wakelock = wakeup_source_register(NULL, "QnovoQNS");
		data.wakelock_inited = true;
	}

	if (!data.charge_wakelock_inited) {
		data.charge_wakelock = wakeup_source_register(NULL, "QnovoQNS");
		data.charge_wakelock_inited = true;
	}

	if (!ret) {
		if (val == CHARGE_WAKELOCK) {
			if (!data.charge_wakelock_held) {
				pr_qns(PR_DEBUG, "Alarm: acquiring charge_wakelock\n");
				__pm_stay_awake(data.charge_wakelock);
				data.charge_wakelock_held = true;
			}
		}
		else if (val == CHARGE_WAKELOCK_RELEASE) {
			if (data.charge_wakelock_held) {
				pr_qns(PR_DEBUG, "Alarm: releasing charge_wakelock\n");
				__pm_relax(data.charge_wakelock);
				data.charge_wakelock_held = false;
			}
		}
		else if (val == HANDLED) {
			if (data.wakelock_held) {
				pr_qns(PR_DEBUG, "Alarm: releasing wakelock via HANDLED\n");
				__pm_relax(data.wakelock);
			}
			data.alarm_value = 0;
			data.wakelock_held = false;
		}
		else if (val == CANCEL) {
			if (data.alarm_inited) {
				alarm_cancel(&data.alarm);
			}
			data.alarm_value = 0;
			if (data.wakelock_held) {
				pr_qns(PR_DEBUG, "Alarm: releasing wakelock via CANCEL\n");
				__pm_relax(data.wakelock);
			}
			data.wakelock_held = false;
		}
		else if (val == IMMEDIATE) {
			if (!data.wakelock_held) {
				pr_qns(PR_DEBUG, "Alarm: acquiring wakelock via IMMEDIATE\n");
				__pm_stay_awake(data.wakelock);
				data.wakelock_held = true;
			}
		}
		else if (val > 0) {
			if (!data.alarm_inited) {
				alarm_init(&data.alarm, ALARM_REALTIME, qns_alarm_handler);
				data.alarm_inited = true;
			}

			next_alarm = ktime_set(val, 0);
			alarm_start_relative(&data.alarm, next_alarm);

			if (data.wakelock_held) {
				pr_qns(PR_DEBUG, "Alarm: releasing wakelock via alarm>0\n");
				__pm_relax(data.wakelock);
			}
			data.alarm_value = 0;
			data.wakelock_held = false;
		}
		return count;
	}

	return -EINVAL;
}

static CLASS_ATTR_RW(alarm);

#ifdef CONFIG_QPNP_QNOVO5
/* if qni driver is existed, qna sysfs is registered on qni driver */
#else
static ssize_t
qna_meter_show(struct class *dev, struct class_attribute *attr, char *buf)
{
	ssize_t size = 0;

	pr_qns(PR_INFO, "QNA: qna_meter_show = %d\n", qna_meter);
	size = scnprintf(buf, PAGE_SIZE, "%d\n", qna_meter);

	return size;
}

static ssize_t
qna_meter_store(struct class *dev,
	struct class_attribute *attr, const char *buf, size_t count)
{

	int val, ret = -EINVAL;

	pr_qns(PR_INFO, "QNA: qns_meter_store = %d\n", qna_meter);

	ret = kstrtoint(buf, 10, &val);
	qna_meter = val;

	return count;
}

static ssize_t
qna_safety_show(struct class *dev, struct class_attribute *attr, char *buf)
{
	ssize_t size = 0;

	pr_qns(PR_INFO, "QNA: qna_safety_show = %d\n", qna_safety);
	size = scnprintf(buf, PAGE_SIZE, "%d\n", qna_safety);

	return size;
}

static ssize_t
qna_safety_store(struct class *dev,
	struct class_attribute *attr, const char *buf, size_t count)
{

	int val, ret = -EINVAL;

	pr_qns(PR_INFO, "QNA: qns_safety_store = %d\n", qna_safety);

	ret = kstrtoint(buf, 10, &val);
	qna_safety = val;

	return count;
}

static ssize_t
qna_vtwarn_show(struct class *dev, struct class_attribute *attr, char *buf)
{
	ssize_t size = 0;

	pr_qns(PR_INFO, "QNA: qna_vtwarm_show = %d\n", qna_vtwarm);
	size = scnprintf(buf, PAGE_SIZE, "%d\n", qna_vtwarm);

	return size;
}

static ssize_t
qna_vtwarn_store(struct class *dev,
	struct class_attribute *attr, const char *buf, size_t count)
{

	int val, ret = -EINVAL;

	pr_qns(PR_INFO, "QNA: qns_vtwarm_store = %d\n", qna_vtwarm);

	ret = kstrtoint(buf, 10, &val);
	qna_vtwarm = val;

	return count;
}

static ssize_t
qna_pcap_show(struct class *dev, struct class_attribute *attr, char *buf)
{
	ssize_t size = 0;

	pr_qns(PR_INFO, "QNA: qna_pcap_show = %d\n", qna_pcap);
	size = scnprintf(buf, PAGE_SIZE, "%d\n", qna_pcap);

	return size;
}

static ssize_t
qna_pcap_store(struct class *dev,
	struct class_attribute *attr, const char *buf, size_t count)
{

	int val, ret = -EINVAL;

	pr_qns(PR_INFO, "QNA: qns_pcap_store = %d\n", qna_pcap);

	ret = kstrtoint(buf, 10, &val);
	qna_pcap = val;

	return count;
}
#endif

static struct attribute *qns_class_attrs[] = {
	&class_attr_charging_state.attr,
	&class_attr_current_now.attr,
	&class_attr_voltage.attr,
	&class_attr_temp.attr,
	&class_attr_fcc.attr,
	&class_attr_design.attr,
	&class_attr_soc.attr,
	&class_attr_battery_type.attr,
	&class_attr_charge_current.attr,
	&class_attr_charge_voltage.attr,
	&class_attr_options.attr,
	&class_attr_alarm.attr,
	NULL,
};

ATTRIBUTE_GROUPS(qns_class);

#ifdef CONFIG_QPNP_QNOVO5
/* if qni driver is existed, qna sysfs is registered on qni driver */
#else
static struct class_attribute qna_attrs[] = {
	__ATTR(QNA_METER, 0660/*S_IWUGO*/, qna_meter_show, qna_meter_store),
	__ATTR(QNA_SAFETY, 0660/*S_IWUGO*/, qna_safety_show, qna_safety_store),
	__ATTR(QNA_VTWARN, 0660/*S_IWUGO*/, qna_vtwarn_show, qna_vtwarn_store),
	__ATTR(QNA_PCAP, 0660/*S_IWUGO*/, qna_pcap_show, qna_pcap_store),
	__ATTR_NULL,
};

enum
{
	QNA_METER_A,
	QNA_SAFETY_A,
	QNA_VTWARN_A,
	QNA_PCAP_A,
};

static struct attribute *qna_class_attrs[] = {
	[QNA_METER_A]			= &qna_attrs[QNA_METER_A].attr,
	[QNA_SAFETY_A]			= &qna_attrs[QNA_SAFETY_A].attr,
	[QNA_VTWARN_A]			= &qna_attrs[QNA_VTWARN_A].attr,
	[QNA_PCAP_A]			= &qna_attrs[QNA_PCAP_A].attr,
	NULL,
};
ATTRIBUTE_GROUPS(qna_class);
#endif

static struct class qns_class =
{
	.name = "qns",
	.owner = THIS_MODULE,
	.class_groups = qns_class_groups
};

#ifdef CONFIG_QPNP_QNOVO5
/* if qni driver is existed, qna sysfs is registered on qni driver */
#else
static struct class qna_class =
{
	.name = "qnovo",
	.owner = THIS_MODULE,
	//.class_attrs = qna_attrs
	.class_groups = qna_class_groups,
};
#endif

static int qns_init(void)
{
	memset(&data, 0, sizeof(data));
	data.options = -1;

	class_register(&qns_class);
#ifdef CONFIG_QPNP_QNOVO5
/* if qni driver is existed, qna sysfs is registered on qni driver */
#else
	class_register(&qna_class);
#endif
	pr_qns(PR_INFO, "START\n");
	return 0;
}

static void qns_exit(void)
{
	if(data.wakelock_held)
		__pm_relax(data.wakelock);

	data.wakelock_held = false;
	wakeup_source_unregister(data.wakelock);

	if(data.charge_wakelock_held)
		__pm_relax(data.charge_wakelock);

	data.charge_wakelock_held = false;
	wakeup_source_unregister(data.charge_wakelock);

	class_unregister(&qns_class);
#ifdef CONFIG_QPNP_QNOVO5
/* if qni driver is existed, qna sysfs is registered on qni driver */
#else
	class_unregister(&qna_class);
#endif
	pr_qns(PR_INFO, "EXIT\n");
}

module_init(qns_init);
module_exit(qns_exit);

MODULE_AUTHOR("Miro Zmrzli <miro@qnovocorp.com>");
MODULE_DESCRIPTION("QNS System Driver v2");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("QNS");
