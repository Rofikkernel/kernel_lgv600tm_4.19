/*
 * LGE USB DS3 driver
 *
 * Copyright (C) 2019 LG Electronics, Inc.
 * Author: Hansun Lee <hansun.lee@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define DEBUG
//#define VERBOSE_DEBUG

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/extcon.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/usb.h>
#include <linux/usb/usbpd.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#include <linux/lge_ds3.h>
#include <linux/hall_ic.h>
#include <linux/extcon-provider.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_LGE_PM_VENEER_PSY
#include <soc/qcom/lge/board_lge.h>
#include "../../power/supply/lge/veneer-primitives.h"
#endif
#ifdef CONFIG_LGE_USB_SBU_SWITCH
#include <linux/usb/lge_sbu_switch.h>
#endif

#include "usbpd.h"
#include "../../../techpack/display/msm/lge/dp/lge_dp_def.h"
extern bool lge_get_mfts_mode(void);

// FIXME : Temporarily disable 2nd USB
#if defined(CONFIG_MACH_KONA_TIMELM)
#define USE_2ND_USB
#endif

static bool usb_sudden_disconnect_check;
module_param(usb_sudden_disconnect_check, bool, 0644);

#ifdef USE_2ND_USB
static bool usb_2nd_host_test;
module_param(usb_2nd_host_test, bool, 0644);
#endif

static bool force_set_hallic;
module_param(force_set_hallic, bool, 0644);

static unsigned int ds_accid_reg_en_delay_ms = 50;
module_param(ds_accid_reg_en_delay_ms, uint, 0644);

static unsigned int ds_recheck_accid_ms = 2000;
module_param(ds_recheck_accid_ms, uint, 0644);

static unsigned int ds_usb_check_time_ms = 3000;
module_param(ds_usb_check_time_ms, uint, 0644);

static unsigned int ds_vconn_recovery_time_ms = 100;
module_param(ds_vconn_recovery_time_ms, uint, 0644);

static unsigned int ds_power_recovery_count = 5;
module_param(ds_power_recovery_count, uint, 0644);

static unsigned int usb_recovery_time_ms = 2000;
module_param(usb_recovery_time_ms, uint, 0644);

#define DS2_VID				0x1004
#define DS2_PID				0x637a

#define DS2_DLOAD_VID			0x0483
#define DS2_DLOAD_PID			0xdf11

#define DS2_PRODUCT_STR			"LMV515N"
#define DS3_PRODUCT_STR			"LMV600N"

#define IS_DS2_USB(udev) \
	((udev->descriptor.idVendor == DS2_VID) && \
	 (udev->descriptor.idProduct == DS2_PID) && \
	 (udev->product && \
	  (!strcmp(udev->product, DS2_PRODUCT_STR) || \
	   !strcmp(udev->product, "DS2"))))

#define IS_DS2_DLOAD_USB(udev) \
	((udev->descriptor.idVendor == DS2_DLOAD_VID) && \
	 (udev->descriptor.idProduct == DS2_DLOAD_PID) && \
	 (udev->product && !strcmp(udev->product, DS2_PRODUCT_STR)))

#define IS_DS3_USB(udev) \
	((udev->descriptor.idVendor == DS2_VID) && \
	 (udev->descriptor.idProduct == DS2_PID) && \
	 (udev->product && \
	  (!strcmp(udev->product, DS3_PRODUCT_STR) || \
	   !strcmp(udev->product, "DS3"))))

#define IS_DS3_DLOAD_USB(udev) \
	((udev->descriptor.idVendor == DS2_DLOAD_VID) && \
	 (udev->descriptor.idProduct == DS2_DLOAD_PID) && \
	 (udev->product && !strcmp(udev->product, DS3_PRODUCT_STR)))

#define IS_DS3_ANY_USB(udev) \
	(IS_DS3_USB(udev) || IS_DS3_DLOAD_USB(udev))

#define IS_DS2_ANY_USB(udev) \
	(IS_DS2_USB(udev) || IS_DS2_DLOAD_USB(udev))

#define PD_MAX_MSG_ID			7
#define PD_MAX_DATA_OBJ			7

#define PD_MSG_HDR(type, dr, pr, id, cnt, rev) \
	(((type) & 0x1F) | ((dr) << 5) | (rev << 6) | \
	 ((pr) << 8) | ((id) << 9) | ((cnt) << 12))
#define PD_MSG_HDR_COUNT(hdr)		(((hdr) >> 12) & 7)
#define PD_MSG_HDR_TYPE(hdr)		((hdr) & 0x1F)
#define PD_MSG_HDR_ID(hdr)		(((hdr) >> 9) & 7)
#define PD_MSG_HDR_REV(hdr)		(((hdr) >> 6) & 3)
#define PD_MSG_HDR_EXTENDED		BIT(15)
#define PD_MSG_HDR_IS_EXTENDED(hdr)	((hdr) & PD_MSG_HDR_EXTENDED)

#define PD_RDO_FIXED(obj, gb, mismatch, usb_comm, no_usb_susp, curr1, curr2) \
	(((obj) << 28) | ((gb) << 27) | ((mismatch) << 26) | \
	 ((usb_comm) << 25) | ((no_usb_susp) << 24) | \
	 ((curr1) << 10) | (curr2))

#define VDM_HDR_SVID(hdr)		((hdr) >> 16)
#define VDM_IS_SVDM(hdr)		((hdr) & 0x8000)
#define SVDM_HDR_VER(hdr)		(((hdr) >> 13) & 0x3)
#define SVDM_HDR_OBJ_POS(hdr)		(((hdr) >> 8) & 0x7)
#define SVDM_HDR_CMD_TYPE(hdr)		(((hdr) >> 6) & 0x3)
#define SVDM_HDR_CMD(hdr)		((hdr) & 0x1f)

#define SVDM_HDR(svid, ver, obj, cmd_type, cmd) \
		(((svid) << 16) | (1 << 15) | ((ver) << 13) \
		 | ((obj) << 8) | ((cmd_type) << 6) | (cmd))

#define IS_DATA(hdr, t) (!PD_MSG_HDR_IS_EXTENDED(hdr) && \
			 PD_MSG_HDR_COUNT(hdr) && \
			 (PD_MSG_HDR_TYPE(hdr) == (t)))
#define IS_CTRL(hdr, t) (!PD_MSG_HDR_COUNT(hdr) && \
			 (PD_MSG_HDR_TYPE(hdr) == (t)))

#define DP_USBPD_VDM_STATUS		0x10
#define DP_USBPD_VDM_CONFIGURE		0x11
#define USB_C_DP_SID			0xFF01
#define SENDER_RESPONSE_TIME            26

#ifdef CONFIG_LGE_USB_DS_DIRECT_SVID
#define CMD_DELAY   0x0
#define DP_VDM_STATUS                  0x10
#define DP_VDM_CONFIGURE               0x11

struct fake_rx_msg {
	u8				cmd;
	enum usbpd_svdm_cmd_type	cmd_type;
	int				len;
	u32				payload[7];
};

static struct fake_rx_msg packets[] = {
	{USBPD_SVDM_DISCOVER_MODES, SVDM_CMD_TYPE_RESP_ACK, 2, {0xff01a043, 0xc05}},
	{CMD_DELAY, 0, 40, {}},
	{USBPD_SVDM_ENTER_MODE, SVDM_CMD_TYPE_RESP_ACK, 1, {0xff01a144}},
	{CMD_DELAY, 0, 35, {}},
	{DP_VDM_STATUS, SVDM_CMD_TYPE_RESP_ACK, 2, {0xff01a150, 0x1a}},
	{CMD_DELAY, 0, 35, {}},
	{DP_VDM_CONFIGURE, SVDM_CMD_TYPE_RESP_ACK, 1, {0xff01a151}},
};
static struct fake_rx_msg attention[] = {
	{USBPD_SVDM_ATTENTION, SVDM_CMD_TYPE_INITIATOR, 2, {0xff01a106, 0x1a}},
	{USBPD_SVDM_ATTENTION, SVDM_CMD_TYPE_INITIATOR, 2, {0xff01a106, 0x9a}},
};


#endif

enum ds_state {
	STATE_UNKNOWN,
	STATE_DS_USB_WAIT,
	STATE_DS_STARTUP,
	STATE_DS_READY,
	STATE_DS_RECOVERY,
	STATE_DS_RECOVERY_POWER_OFF,
	STATE_DS_RECOVERY_POWER_ON,
	STATE_DS_RECOVERY_USB_WAIT,
	STATE_DS_DLOAD,
	STATE_NONE_DS_RECOVERY,
};

static const char * const ds_state_strings[] = {
	"Unknown",
	"DS_USB_Wait",
	"DS_Startup",
	"DS_Ready",
	"DS_Recovery",
	"DS_Recovery_Power_Off",
	"DS_Recovery_Power_On",
	"DS_Recovery_USB_Wait",
	"DS_Dload",
	"None_DS_Recovery",
};

enum ds3_usb {
	DS_USB_DISCONNECTED = 0,
	DS_USB_CONNECTED,
	DS_USB_DLOAD_CONNECTED,
};

struct ds3 {
	struct device			*dev;

	struct workqueue_struct		*wq;
	struct work_struct		sm_work;
	struct delayed_work		ds_acc_detect_work;
	struct hrtimer			acc_timer;
	struct hrtimer			timer;
	bool				sm_queued;
	enum ds_state			current_state;

	struct power_supply		*usb_psy;
	struct notifier_block		psy_nb;
	enum power_supply_typec_mode	typec_mode;

	struct regulator		*vconn;

	struct extcon_dev		*extcon;
	struct gpio_desc		*dd_sw_sel;
	struct gpio_desc		*load_sw_on;
	struct gpio_desc		*ds_en;
	struct notifier_block		nb;
	struct usb_device		*udev;

	bool				is_ds_connected;
	enum ds3_usb			is_ds_usb_connected;
	bool				is_ds_hal_ready;
	int				is_ds_recovery;
	bool				is_dp_configured;
	bool				is_dp_hpd_high;
	bool				is_accid_connected;

	bool				is_usb_connected;
	bool				is_usb_recovery;
	bool				vbus_present;
	bool				acc_det_vcomp;
	bool				acc_det_vadc;
	int				acc_det_count;
	int				pd_active;

	u8				tx_msgid;
	enum pd_spec_rev		spec_rev;
	enum data_role			current_dr;
	enum power_role			current_pr;
#ifdef CONFIG_LGE_USB_DS_DIRECT_SVID
	struct usbpd			*usbpd;
	struct usbpd_svid_handler	svid_handler;
#else
	struct completion		is_pd_msg_received;
#endif

#ifdef CONFIG_LGE_USB_SBU_SWITCH
	struct lge_sbu_switch_desc      sbu_desc;
	struct lge_sbu_switch_instance  *sbu_inst;
#endif
	int				acc_detect;
	struct regulator		*acc_det_pu;
	struct iio_channel		*channel;
};

enum pd_control_msg_type {
	MSG_RESERVED = 0,
	MSG_GOODCRC,
	MSG_GOTOMIN,
	MSG_ACCEPT,
	MSG_REJECT,
	MSG_PING,
	MSG_PS_RDY,
	MSG_GET_SOURCE_CAP,
	MSG_GET_SINK_CAP,
	MSG_DR_SWAP,
	MSG_PR_SWAP,
	MSG_VCONN_SWAP,
	MSG_WAIT,
	MSG_SOFT_RESET,
	MSG_NOT_SUPPORTED = 0x10,
};

enum usbpd_data_msg_type {
	MSG_SOURCE_CAPABILITIES = 1,
	MSG_REQUEST,
	MSG_BIST,
	MSG_SINK_CAPABILITIES,
	MSG_VDM = 0xF,
};

static struct ds3 *__ds3 = NULL;

static bool hallic_status = false;
static bool *ds3_connected = NULL;

#ifdef USE_2ND_USB
static const unsigned int ds_extcon_cable[] = {
	EXTCON_USB_HOST,
	EXTCON_NONE,
};
#endif

static void kick_sm(struct ds3 *ds3, int ms);
static void ds_set_state(struct ds3 *ds3, enum ds_state next_state);

extern struct hallic_dev luke_sdev;
extern void request_dualscreen_recovery(void);
extern struct lge_dp_display *get_lge_dp(void);
extern void call_disconnect_uevent(void);
static int check_ds3_accid(struct ds3 *ds3, bool enable, bool use_vadc);

void set_hallic_status(bool enable)
{
	struct ds3 *ds3 = __ds3;
	bool start_connect_ds = false;
	int accid_connected = 1;

	hallic_status = enable;

	if (!ds3) {
		pr_debug("%s: %d\n", __func__, enable);
		return;
	}

	dev_dbg(ds3->dev, "%s: %d\n", __func__, enable);

	if (ds3->acc_det_vcomp || ds3->acc_det_vadc) {
		accid_connected = check_ds3_accid(ds3, enable, ds3->acc_det_vadc ? true : false);
	} else {
		dev_info(ds3->dev, "%s: there is no acc_det_vcomp, use hallic only\n", __func__);
		accid_connected = enable ? 1 : 0;
		ds3->is_accid_connected = enable;
	}

#if defined(CONFIG_MACH_KONA_TIMELM)
	start_connect_ds = hallic_status && accid_connected;
#else
	start_connect_ds = ds3->pd_active == POWER_SUPPLY_PD_INACTIVE &&
			(ds3->typec_mode == POWER_SUPPLY_TYPEC_SINK ||
			ds3->typec_mode == POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE);
#endif

	if (enable) {
		dev_dbg(ds3->dev, "%s: typec:%d vbus:%d pd:%d ds3:%d hallic:%d accid:%d "
				" usb:%d ds3_usb:%d usb_recovery:%d ds3_recovery: %d\n",
			__func__,
			ds3->typec_mode,
			ds3->vbus_present,
			ds3->pd_active,
			ds3->is_ds_connected,
			hallic_status,
			ds3->is_accid_connected,
			ds3->is_usb_connected,
			ds3->is_ds_usb_connected,
			ds3->is_usb_recovery,
			ds3->is_ds_recovery);

		if (!ds3->is_ds_connected &&
		    !ds3->is_usb_connected &&
		    !ds3->is_usb_recovery &&
		    ds3->is_accid_connected &&
		    start_connect_ds)
			ds_set_state(ds3, STATE_DS_STARTUP);
	} else {
		/* ... */
		if (ds3->is_ds_connected &&
			!ds3->is_accid_connected)
			kick_sm(ds3, 0);
	}
}
EXPORT_SYMBOL(set_hallic_status);

int set_ds_extcon_state(unsigned int id, int state)
{
	int ret = 0;
	struct lge_dp_display *lge_dp = get_lge_dp();

	ret = extcon_set_state_sync(lge_dp->dd_extcon_sdev[0], id, state);

	return ret;
}

void hallic_state_notify(struct ds3 *ds3, struct hallic_dev *hdev, int state)
{
	char name_buf[40];
	char state_buf[40];
	char *uevent[3] = { name_buf, state_buf, NULL };

	if (!hdev || !hdev->dev) {
		dev_err(ds3->dev, "hallic_dev is NULL\n");
		return;
	}

	if (!lge_get_mfts_mode())
		hdev->state = state;

	snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", hdev->name);
	snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%d", state);

	kobject_uevent_env(&hdev->dev->kobj, KOBJ_CHANGE, uevent);
	dev_dbg(ds3->dev, "%s: %s\n", __func__, name_buf);
	dev_dbg(ds3->dev, "%s: %s\n", __func__, state_buf);
}

bool is_ds_connected(void)
{
	struct ds3 *ds3 = __ds3;
	bool ret = ds3_connected ? *ds3_connected : false;

	if (ds3)
		dev_dbg(ds3->dev, "%s: %d\n", __func__, ret);
	else
		pr_debug("%s: %d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(is_ds_connected);

int check_ds_connect_state(void)
{
	struct ds3 *ds3 = __ds3;

	if (!ds3)
		return 0;

	if (ds3->is_dp_hpd_high)
		return DS_STATE_HPD_ENABLED;
	else if (ds3->is_dp_configured)
		return DS_STATE_DP_CONNECTED;
	else if (ds3->is_accid_connected)
		return DS_STATE_ACC_ID_CONNECTED;
	else if (hallic_status)
		return DS_STATE_HALLIC_CONNECTED;

	return DS_STATE_DISCONNECTED;
}
EXPORT_SYMBOL(check_ds_connect_state);

#ifdef CONFIG_LGE_USB_DS_DIRECT_SVID
void handle_displayport_initialize(struct usbpd_svid_handler *handler){
	int i;

	// jump to dp driver connection callback function
	handler->connect(handler, false);
	mdelay(30);

	// send fake pd message to dp driver response callback function
	// handler->svdm_received(handler, cmd, cmd_type, vdos, num_vdos);
	// handler->svdm_received(handler, (u8)0x3, (int)1, (u32 *)0x1405, (int)1);

	for (i = 0; i < ARRAY_SIZE(packets) ; i++) {
		if (packets[i].cmd == CMD_DELAY) {
			mdelay(packets[i].len);
			continue;
		}
		handler->svdm_received(handler,
					packets[i].cmd,
					packets[i].cmd_type,
					&packets[i].payload[1],
					packets[i].len - 1);
	}
}

void handle_displayport_attention(struct usbpd_svid_handler *handler, int hpd){
	mdelay(30);
	handler->svdm_received(handler, attention[hpd].cmd,
			attention[hpd].cmd_type, &attention[hpd].payload[1],
			attention[hpd].len - 1);
}

static int ds_dp_config(struct ds3 *ds3, bool config)
{
	struct device *dev = ds3->dev;
	struct usbpd_svid_handler *handler;

	dev_info(dev, "%s: config:%d\n", __func__, config);

	handler = usbpd_find_dp_handler(ds3->usbpd);
	if (!handler) {
		dev_err(dev, "%s: No DP handler found\n", __func__);
		return -ENODEV;
	}
	if (config) {
		ds3->is_dp_configured = true;
		handle_displayport_initialize(handler);
		if (ds3->is_ds_usb_connected == DS_USB_CONNECTED) {
			set_ds_extcon_state(EXTCON_DISP_DS2, 1);
			dev_err(dev, "%s: current luke state = %d\n", __func__, luke_sdev.state);
			hallic_state_notify(ds3, &luke_sdev, 1);
		}
	} else {
		ds3->is_dp_configured = false;
		handler->disconnect(handler);
	}
#ifdef CONFIG_LGE_USB_SBU_SWITCH
	if (config)
		lge_sbu_switch_get(ds3->sbu_inst, LGE_SBU_SWITCH_FLAG_SBU_AUX);
	else
		lge_sbu_switch_put(ds3->sbu_inst, LGE_SBU_SWITCH_FLAG_SBU_AUX);
#endif

	return 0;
}

static int ds_dp_hpd_direct(struct ds3 *ds3, bool hpd)
{
	struct device *dev = ds3->dev;
	struct usbpd_svid_handler *handler;

	handler = usbpd_find_dp_handler(ds3->usbpd);
	if (!handler) {
		dev_err(dev, "%s: No DP handler found\n", __func__);
		return -ENODEV;
	}

	dev_info(dev, "%s: is_dp_hpd_high:%d hpd: %d\n", __func__,
			ds3->is_dp_hpd_high, hpd);

	if (ds3->is_dp_hpd_high == hpd) {
		dev_dbg(dev, "%s: duplicated value is set\n", __func__);
		return 0;
	}
	ds3->is_dp_hpd_high = hpd;
	handle_displayport_attention(handler, hpd);

	return 0;
}
#endif

#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
static int pd_sig_received(const void *emul, enum pd_sig_type sig)
{
	struct ds3 *ds3 = (struct ds3 *)emul;
	struct device *dev = ds3->dev;

	if (sig != HARD_RESET_SIG) {
		dev_err(dev, "invalid signal (%d) received\n", sig);
		return 0;
	}

	dev_err(dev, "%s: hard reset received\n", __func__);

	// Disable DisplayPort
	ds3->is_dp_configured = false;
	ds3->is_dp_hpd_high = false;

	return 0;
}

static int pd_send_msg(struct ds3 *ds3, u8 msg_type, const u32 *data,
		size_t num_data, enum pd_sop_type sop)
{
	struct device *dev = ds3->dev;
	u16 hdr = PD_MSG_HDR(msg_type, ds3->current_dr, ds3->current_pr,
			     ds3->tx_msgid, num_data, ds3->spec_rev);
	int ret;

	ret = pd_phy_emul_write(ds3, hdr, (u8 *)data, num_data * sizeof(u32),
				sop);
	if (ret) {
		dev_err(dev, "Error sending msg: %d\n", ret);
		return ret;
	}

	ds3->tx_msgid = (ds3->tx_msgid + 1) & PD_MAX_MSG_ID;
	return 0;
}

static int pd_send_svdm(struct ds3 *ds3, u16 svid, u8 cmd,
		enum usbpd_svdm_cmd_type cmd_type, int obj_pos,
		const u32 *vdos, int num_vdos)
{
	u32 svdm[PD_MAX_DATA_OBJ] = {
		[0] = SVDM_HDR(svid, (ds3->spec_rev == USBPD_REV_30) ? 1 : 0,
			       obj_pos, cmd_type, cmd),
	};

	memcpy(&svdm[1], vdos, num_vdos * sizeof(u32));

	return pd_send_msg(ds3, MSG_VDM, svdm, num_vdos + 1, SOP_MSG);
}

static int ds3_dp_hpd(struct ds3 *ds3, bool hpd)
{
	struct device *dev = ds3->dev;
	struct {
		uint32_t conn:2;
		uint32_t power_low:1;
		uint32_t adaptor_func:1;
		uint32_t multi_func:1;
		uint32_t usb_config:1;
		uint32_t exit_dp:1;
		uint32_t hpd_state:1;
		uint32_t irq_hpd:1;
		uint32_t reserved:23;
	} vdos = {
		.conn = 2, // UFP_D
		.adaptor_func = 1,
		.multi_func = 1,
		.hpd_state = hpd,
	};
	int ret;

	dev_dbg(dev, "%s: is_dp_hpd_high:%d hpd: %d\n", __func__,
			ds3->is_dp_hpd_high, hpd);

	if (ds3->is_dp_hpd_high == hpd) {
		dev_dbg(dev, "%s: duplicated value is set\n", __func__);
		return 0;
	}
	ds3->is_dp_hpd_high = hpd;

	ret = pd_send_svdm(ds3, USB_C_DP_SID, USBPD_SVDM_ATTENTION,
			   SVDM_CMD_TYPE_INITIATOR, 1,
			   (u32 *)&vdos, 1);
	if (ret) {
		dev_err(dev, "%s: error sending attention: %d\n", __func__,
			ret);
		return ret;
	}

	return 0;
}

static int pd_msg_received(const void *emul, enum pd_sop_type sop,
		u8 *buf, size_t len)
{
	struct ds3 *ds3 = (struct ds3 *)emul;
	struct device *dev = ds3->dev;
	u16 hdr;
	int ret;

	if (sop != SOP_MSG) {
		dev_dbg(dev, "%s: only SOP supported\n", __func__);
		return -EFAULT;
	}

	hdr = *((u16 *)buf);
	buf += sizeof(u16);
	len -= sizeof(u16);

	dev_dbg(dev, "%s: %s(%d)\n", __func__,
		PD_MSG_HDR_COUNT(hdr) ? "DATA" : "CTRL",
		PD_MSG_HDR_TYPE(hdr));

	if (IS_DATA(hdr, MSG_SOURCE_CAPABILITIES)) {
		u32 rdo = PD_RDO_FIXED(
			1, // Object position
			0, // GiveBack flag
			0, // Capability Mismatch
			// USB Communications Capable
#ifdef USE_2ND_USB
			0,
#else
			1,
#endif
			0, // No USB Suspend
			0, // Operating current in 10mA units
			0  // Maximum Operating Current 10mA units
		);

		dev_dbg(dev, "%s: MSG_SOURCE_CAPABILITIES\n", __func__);

		if (PD_MSG_HDR_REV(hdr) < ds3->spec_rev)
			ds3->spec_rev = PD_MSG_HDR_REV(hdr);

		ret = pd_send_msg(ds3, MSG_REQUEST, &rdo, 1, SOP_MSG);
		if (ret) {
			dev_err(dev, "%s: error sending RDO: %d\n", __func__,
				ret);
			return ret;
		}

	} else if (IS_CTRL(hdr, MSG_ACCEPT)) {
		dev_dbg(dev, "%s: MSG_ACCEPT\n", __func__);
		complete(&ds3->is_pd_msg_received);

	} else if (IS_CTRL(hdr, MSG_PS_RDY)) {
		dev_dbg(dev, "%s: MSG_PS_RDY\n", __func__);
		complete(&ds3->is_pd_msg_received);

	} else if (IS_DATA(hdr, MSG_VDM)) {
		u32 vdm_hdr = len >= sizeof(u32) ? ((u32 *)buf)[0] : 0;
		u8 cmd = SVDM_HDR_CMD(vdm_hdr);
		//u8 cmd_type = SVDM_HDR_CMD_TYPE(vdm_hdr);

		dev_dbg(dev, "%s: MSG_VDM\n", __func__);

		switch (cmd) {
		case USBPD_SVDM_DISCOVER_IDENTITY: {
			struct {
				/* ID Header */
				uint16_t vendor_id;
				uint16_t reserved1:10;
				uint16_t modal_opr:1;
				uint16_t product_type:3;
				uint16_t device_cap:1;
				uint16_t host_cap:1;
				/* Cert State */
				uint32_t xid;
				/* Product */
				uint16_t bcd_device;
				uint16_t product_id;
				/* AMA */
				uint32_t usb_ss:3;
				uint32_t vbus_req:1;
				uint32_t vconn_req:1;
				uint32_t vconn_power:3;
				uint32_t ssrx2:1; // USBPD_REV_20
				uint32_t ssrx1:1; // USBPD_REV_20
				uint32_t sstx2:1; // USBPD_REV_20
				uint32_t sstx1:1; // USBPD_REV_20
				uint32_t reserved2:9;
				uint32_t vdo_version:3; // USBPD_REV_30
				uint32_t fw_ver:4;
				uint32_t hw_ver:4;
			} vdos = {
				.vendor_id = DS2_VID,
				.modal_opr = 1,
				.product_type = 5, // AMA:5 VPD:6
				.device_cap = 1,
				.product_id = DS2_PID,
				.vconn_req = 1,
				.vconn_power = 6, // 6W
			};

			dev_dbg(dev, "%s: SVDM_DISCOVER_IDENTITY\n", __func__);

#ifdef USE_2ND_USB
			if (ds3->spec_rev == USBPD_REV_30) {
				struct {
					uint32_t chg_thr:1;
					uint32_t gnd_imp:6;
					uint32_t vbus_imp:6;
					uint32_t reserved1:2;
					uint32_t max_volt:2;
					uint32_t reserved2:4;
					uint32_t vdo_ver:3;
					uint32_t fw_ver:4;
					uint32_t hw_ver:4;
				} vpd_vdo = { 0, };

				vdos.product_type = 6; // VPD
				memcpy(&(((uint32_t *)&vdos)[1]),
				       &vpd_vdo,
				       sizeof(vpd_vdo));
			}
#endif

			ret = pd_send_svdm(ds3, USBPD_SID, cmd,
					   SVDM_CMD_TYPE_RESP_ACK, 0,
					   (u32 *)&vdos, 4);
			if (ret) {
				dev_err(dev, "%s: error sending discover_id: %d\n",
					__func__, ret);
				return ret;
			}
			break;
		}

		case USBPD_SVDM_DISCOVER_SVIDS: {
			struct {
				uint16_t sid;
				uint16_t vid;
				uint32_t reserved;
			} vdos = { USB_C_DP_SID, DS2_VID };

			dev_dbg(dev, "%s: SVDM_DISCOVER_SVIDS\n", __func__);

			ret = pd_send_svdm(ds3, USBPD_SID, cmd,
					   SVDM_CMD_TYPE_RESP_ACK, 0,
					   (u32 *)&vdos, 2);
			if (ret) {
				dev_err(dev, "%s: error sending discover_svids: %d\n",
					__func__, ret);
				return ret;
			}
			break;
		}

		case USBPD_SVDM_DISCOVER_MODES: {
			struct {
				uint8_t port_cap:2;
				uint8_t sig_supp:4;
				uint8_t recep_ind:1;
				uint8_t r2_0:1;
				uint8_t dfp_d_pins;
				uint8_t ufp_d_pins;
				uint8_t reserved;
			} vdos = {
				.port_cap = 1, // UFP_D
				.sig_supp = 1, // DP v1.3 signaling rate
				.dfp_d_pins = 0x0C, // Pin Assignments: C, D
			};

			dev_dbg(dev, "%s: SVDM_DISCOVER_MODES\n", __func__);

			ret = pd_send_svdm(ds3, USB_C_DP_SID, cmd,
					   SVDM_CMD_TYPE_RESP_ACK, 0,
					   (u32 *)&vdos, 1);
			if (ret) {
				dev_err(dev, "%s: error sending discover_modes: %d\n",
					__func__, ret);
				return ret;
			}
			break;
		}

		case USBPD_SVDM_ENTER_MODE:
			dev_dbg(dev, "%s: SVDM_ENTER_MODE\n", __func__);

			ret = pd_send_svdm(ds3, USB_C_DP_SID, cmd,
					   SVDM_CMD_TYPE_RESP_ACK, 1, NULL, 0);
			if (ret) {
				dev_err(dev, "%s: error sending enter_mode: %d\n",
					__func__, ret);
				return ret;
			}
			break;

		case USBPD_SVDM_EXIT_MODE:
			dev_dbg(dev, "%s: SVDM_EXIT_MODE\n", __func__);

			ret = pd_send_svdm(ds3, USB_C_DP_SID, cmd,
					   SVDM_CMD_TYPE_RESP_ACK, 1, NULL, 0);
			if (ret) {
				dev_err(dev, "%s: error sending enter_mode: %d\n",
					__func__, ret);
				return ret;
			}
			break;

		case USBPD_SVDM_ATTENTION:
			dev_dbg(dev, "%s: SVDM_ATTENTION\n", __func__);
			break;

		case DP_USBPD_VDM_STATUS: {
			struct {
				uint32_t conn:2;
				uint32_t power_low:1;
				uint32_t adaptor_func:2;
				uint32_t multi_func:1;
				uint32_t usb_config:1;
				uint32_t exit_dp:1;
				uint32_t hpd_state:1;
				uint32_t irq_hpd:1;
				uint32_t reserved:23;
			} vdos = {
				.conn = 2, // UFP_D
				.multi_func = 1,
				.adaptor_func = 1,
			};

			dev_dbg(dev, "%s: VDM_DP_STATUS\n", __func__);

			ret = pd_send_svdm(ds3, USB_C_DP_SID, cmd,
					   SVDM_CMD_TYPE_RESP_ACK, 1,
					   (u32 *)&vdos, 1);
			if (ret) {
				dev_err(dev, "%s: error sending dp_status: %d\n",
					__func__, ret);
				return ret;
			}
			break;
		}

		case DP_USBPD_VDM_CONFIGURE:
			dev_dbg(dev, "%s: VDM_DP_CONFIGURE\n", __func__);

			ret = pd_send_svdm(ds3, USB_C_DP_SID, cmd,
					   SVDM_CMD_TYPE_RESP_ACK, 1, NULL, 0);
			if (ret) {
				dev_err(dev, "%s: error sending dp_configure: %d\n",
					__func__, ret);
				return ret;
			}

			ds3->is_dp_configured = true;
			if (ds3->is_ds_usb_connected == DS_USB_CONNECTED &&
					ds3->is_ds_hal_ready) {
//				set_ds_extcon_state(EXTCON_DISP_DS2, 1);
				dev_err(dev, "%s: current luke state = %d\n", __func__, luke_sdev.state);
				hallic_state_notify(ds3, &luke_sdev, 1);
			}
			//ds3_dp_hpd(ds3, true);
			break;

		default:
			dev_err(dev, "%s: unknown svdm_cmd:%d\n", __func__,
				cmd);

			if (ds3->spec_rev == USBPD_REV_30) {
				ret = pd_send_msg(ds3, MSG_NOT_SUPPORTED,
						  NULL, 0, SOP_MSG);
				if (ret) {
					dev_err(dev, "%s: error sending not_supported: %d\n",
						__func__, ret);
					return ret;
				}
			}
			break;
		}

	} else {
		dev_err(dev, "%s: unknown msg_type:%d\n", __func__,
			PD_MSG_HDR_TYPE(hdr));

		if (ds3->spec_rev == USBPD_REV_30) {
			ret = pd_send_msg(ds3, MSG_NOT_SUPPORTED,
					  NULL, 0, SOP_MSG);
			if (ret) {
				dev_err(dev, "%s: error sending not_supported: %d\n",
					__func__, ret);
				return ret;
			}
		}
	}

	return 0;
}
#endif
#ifdef USE_2ND_USB
static void stop_2nd_usb_host(struct ds3 *ds3)
{
	struct device *dev = ds3->dev;

	if (extcon_get_state(ds3->extcon, EXTCON_USB_HOST) == 0)
		return;

	dev_dbg(dev, "%s\n", __func__);

	extcon_set_state_sync(ds3->extcon, EXTCON_USB_HOST, 0);
}

static void start_2nd_usb_host(struct ds3 *ds3)
{
	struct device *dev = ds3->dev;
	union extcon_property_value val;

	if (extcon_get_state(ds3->extcon, EXTCON_USB_HOST) == 1)
		return;

	dev_dbg(dev, "%s\n", __func__);

	val.intval = 0;
	extcon_set_property(ds3->extcon, EXTCON_USB_HOST,
			EXTCON_PROP_USB_SS, val);

	extcon_set_state_sync(ds3->extcon, EXTCON_USB_HOST, 1);
}
#endif

static int ds3_usb_notify(struct notifier_block *nb, unsigned long action,
			  void *data)
{
	struct ds3 *ds3 = container_of(nb, struct ds3, nb);
	struct device *dev = ds3->dev;
	struct usb_device *udev = data;

	dev_vdbg(dev, "%s: dev num:%d path:%s\n", __func__,
		udev->devnum, udev->devpath);
	dev_vdbg(dev, "%s: bus num:%d name:%s\n", __func__,
		udev->bus->busnum, udev->bus->bus_name);

#ifdef USE_2ND_USB
	if (usb_2nd_host_test)
		return NOTIFY_DONE;
#endif

	switch (action) {
	case USB_DEVICE_ADD:
		if (!udev->parent)
			return NOTIFY_DONE;

		dev_info(dev, "%s: USB_DEVICE_ADD: idVendor:%04x idProduct:%04x bcdDevice:%04x\n",
			__func__,
			udev->descriptor.idVendor,
			udev->descriptor.idProduct,
			udev->descriptor.bcdDevice);

		ds3->is_usb_connected = true;

		if (!IS_DS2_ANY_USB(udev) && !IS_DS3_ANY_USB(udev) )
			return NOTIFY_DONE;

		if (ds3->is_usb_recovery)
			dev_dbg(dev, "%s: DS3 connected during usb recovery\n",
				__func__);

		ds3->is_usb_recovery = false;
//		cancel_delayed_work_sync(&ds3->usb_check);
//		cancel_delayed_work_sync(&ds3->ds2_recovery);

#ifdef USE_2ND_USB
		// Secondary USB
		if (extcon_get_state(ds3->extcon, EXTCON_USB_HOST) == 0) {
			gpiod_direction_output(ds3->dd_sw_sel, 1);
			start_2nd_usb_host(ds3);
			return NOTIFY_OK;
		}
#endif

		set_ds_extcon_state(EXTCON_DISP_DS2, 1);

		// DS2 or DS3 USB Connected
		if (IS_DS2_USB(udev) || IS_DS3_USB(udev)) {
			dev_dbg(dev, "%s: FW_VER: %s-V%02u%c_XX\n", __func__,
				udev->product ? udev->product : IS_DS2_USB(udev) ? DS2_PRODUCT_STR : DS3_PRODUCT_STR,
				(udev->descriptor.bcdDevice >> 8) & 0xff,
				'a' + ((udev->descriptor.bcdDevice & 0xff) % 26/*a-z*/));

			ds3->is_ds_usb_connected = DS_USB_CONNECTED;

			if (!ds3->is_ds_connected)
				ds_set_state(ds3, STATE_DS_STARTUP);

		// DS2 Dload USB Connected
		} else if (IS_DS2_DLOAD_USB(udev) || IS_DS3_DLOAD_USB(udev)) {
			ds3->is_ds_usb_connected = DS_USB_DLOAD_CONNECTED;
			ds_set_state(ds3, STATE_DS_DLOAD);

			dev_err(dev, "%s: currunt luke state = %d\n",
				__func__, luke_sdev.state);
			call_disconnect_uevent();
			hallic_state_notify(ds3, &luke_sdev, 0);
		}

		return NOTIFY_OK;

	case USB_DEVICE_REMOVE:
		if (!udev->parent)
			return NOTIFY_DONE;

		dev_info(dev, "%s: USB_DEVICE_REMOVE: idVendor:%04x idProduct:%04x\n",
			__func__,
			udev->descriptor.idVendor,
			udev->descriptor.idProduct);

		ds3->is_usb_connected = false;

		if (!IS_DS2_ANY_USB(udev) && !IS_DS3_ANY_USB(udev))
			return NOTIFY_DONE;

		ds3->is_ds_usb_connected = DS_USB_DISCONNECTED;
		ds3->is_ds_hal_ready = false;

		// DS2 or DS3 USB Disconnected
		if (IS_DS2_USB(udev) || IS_DS3_USB(udev)) {

			BUG_ON(usb_sudden_disconnect_check &&
			       ds3->typec_mode != POWER_SUPPLY_TYPEC_NONE);

			if (ds3->is_ds_connected && ds3->is_ds_recovery <= 0) {
				ds3->is_ds_recovery = ds_power_recovery_count;
				ds_set_state(ds3, STATE_DS_RECOVERY);
			} else if (hallic_status && ds3->is_accid_connected) {
				set_hallic_status(true);
			}

		// DS2 or DS3 Dload USB Disconnected
		} else if (IS_DS2_DLOAD_USB(udev) || IS_DS3_DLOAD_USB(udev)) {

		}

		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static void ds3_sm(struct work_struct *w)
{
	struct ds3 *ds3 = container_of(w, struct ds3, sm_work);
	struct device *dev = ds3->dev;
#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
	struct pd_phy_emul_params emul_params = {
		.signal_cb = pd_sig_received,
		.msg_rx_cb = pd_msg_received,
	};
	union power_supply_propval val = { 0, };
#endif
	int ret = 0;

	hrtimer_cancel(&ds3->timer);
	ds3->sm_queued = false;

#ifdef USE_2ND_USB
	if (usb_2nd_host_test) {
		if (ds3->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
			stop_2nd_usb_host(ds3);
			gpiod_direction_output(ds3->dd_sw_sel, 0);
		} else {
			gpiod_direction_output(ds3->dd_sw_sel, 1);
			start_2nd_usb_host(ds3);
		}
		goto sm_done;
	}
#endif

	dev_info(dev, "%s: %s\n", __func__,
			ds_state_strings[ds3->current_state]);

#ifdef CONFIG_LGE_PM_VENEER_PSY
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO
			&& (!strcmp(unified_bootmode_region(), "CAN")
				|| !strcmp(unified_bootmode_region(), "USA"))) {
		dev_dbg(dev, "%s: Disable DS3 forcely in chargerlogo\n",
				__func__, unified_bootmode_region());
		goto sm_done;
	}
#endif

	// disconnect
#if defined(CONFIG_MACH_KONA_TIMELM)
	if (!hallic_status || !ds3->is_accid_connected) {
#else
	if (ds3->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
#endif
		if (!ds3->is_ds_connected) {
			dev_dbg(dev, "%s: DS2 or DS3 is already disconnected\n",
				__func__);
			goto  sm_done;
		}

		dev_info(dev, "%s: DS disconnect\n", __func__);
		ds3->is_ds_connected = false;

#ifdef USE_2ND_USB
		// Secondary USB
		stop_2nd_usb_host(ds3);
		gpiod_direction_output(ds3->dd_sw_sel, 0);

#endif


#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
		// Disable PowerDelivery
		pd_phy_register_emul(ds3, NULL);
#else
		ds_dp_config(ds3, false);
#endif
		// Disable DisplayPort
		dev_err(dev, "%s: currunt luke state = %d\n", __func__,
				luke_sdev.state);
		set_ds_extcon_state(EXTCON_DISP_DS2, 0);
		call_disconnect_uevent();
		hallic_state_notify(ds3, &luke_sdev, 0);

		if (ds3->ds_en) {
			gpiod_direction_output(ds3->ds_en, 0);
		} else if (ds3->vconn) {
			ret = regulator_disable(ds3->vconn);
			if (ret) {
				dev_err(dev, "Unable to disable vconn\n");
			}
		}
		gpiod_direction_output(ds3->load_sw_on, 0);
		ds3->is_dp_configured = false;
		ds3->is_dp_hpd_high = false;

		ds3->is_ds_recovery = 0;

		ds3->current_state = STATE_UNKNOWN;

		set_ds3_start(false);

		if (ds3->is_usb_recovery) {
			ds3->current_state = STATE_DS_RECOVERY_POWER_OFF;
			kick_sm(ds3, usb_recovery_time_ms);
		}

		goto sm_done;
	}

	switch (ds3->current_state) {
	case STATE_UNKNOWN:
	case STATE_DS_STARTUP:
		if (ds3->is_ds_connected) {
			dev_dbg(dev, "%s: DS2 is already connected\n",
				__func__);
			goto sm_done;
		}

		if (ds3->ds_en) {
			gpiod_direction_output(ds3->ds_en, 1);
		} else if (ds3->vconn) {
			ret = regulator_enable(ds3->vconn);
			if (ret) {
				dev_err(dev, "Unable to enable vconn\n");
			}
		}
		gpiod_direction_output(ds3->load_sw_on, 1);
		ds3->is_ds_connected = true;

#ifdef USE_2ND_USB
		// Secondary USB
		if (extcon_get_state(ds3->extcon, EXTCON_USB_HOST) == 0) {
			gpiod_direction_output(ds3->dd_sw_sel, 1);
			start_2nd_usb_host(ds3);
		}
#endif

#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
		// Activate PowerDelivery
		if (hallic_status) {
			val.intval = POWER_SUPPLY_PD_VPD_ACTIVE;
			power_supply_set_property(ds3->usb_psy,
						  POWER_SUPPLY_PROP_PD_ACTIVE,
						  &val);
		}
#else
		ret = ds_dp_config(ds3, true);
		if (ret) {
#ifdef USE_2ND_USB
			// Secondary USB
			stop_2nd_usb_host(ds3);
			gpiod_direction_output(ds3->dd_sw_sel, 0);
#endif
			ds3->is_ds_connected = false;
			ds_set_state(ds3, STATE_UNKNOWN);
			goto sm_done;
		}
#endif

		ds3->spec_rev = USBPD_REV_30;
		ds3->current_pr = PR_SINK;
		ds3->current_dr = DR_UFP;
		ds3->tx_msgid = 0;
		ds3->is_dp_hpd_high = false;
#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
		pd_phy_register_emul(ds3, &emul_params);
#endif
		set_ds3_start(true);

		if (!ds3->is_ds_usb_connected) {
			ds_set_state(ds3, STATE_DS_USB_WAIT);
			goto sm_done;
		}

		ds3->current_state = STATE_DS_STARTUP;
		goto sm_done;
		break;

	case STATE_DS_USB_WAIT:
		if (!ds3->is_ds_usb_connected)
			ds_set_state(ds3, STATE_DS_RECOVERY_POWER_OFF);
		break;

	case STATE_DS_READY:
		if (ds3->is_dp_configured) {
			dev_err(dev, "%s: current luke state = %d\n", __func__,
					luke_sdev.state);
			hallic_state_notify(ds3, &luke_sdev, 1);
		}
		break;

	case STATE_DS_RECOVERY:
		if (ds3->is_ds_usb_connected)
			break;

		dev_info(dev, "%s: %s %d\n", __func__,
				ds_state_strings[ds3->current_state],
				ds3->is_ds_recovery);

		ds3->is_ds_recovery--;
		ds_set_state(ds3, STATE_DS_RECOVERY_POWER_OFF);
		break;
		/* fall-through */

	case STATE_DS_RECOVERY_POWER_OFF:
		// 2nd USB off
		stop_2nd_usb_host(ds3);

#if 0
		/* blocks until USB host is completely stopped */
		ret = extcon_blocking_sync(ds3->extcon, EXTCON_USB_HOST, 0);
		if (ret) {
			dev_err(ds3->dev, "%s: err(%d) stopping host", ret);
			break;
		}
#endif
		// Power Off
		if (ds3->ds_en) {
			gpiod_direction_output(ds3->ds_en, 0);
		} else if (ds3->vconn) {
			ret = regulator_disable(ds3->vconn);
			if (ret) {
				dev_err(dev, "Unable to disable vconn\n");
			}
		}
		gpiod_direction_output(ds3->load_sw_on, 0);

#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
		reinit_completion(&ds3->is_pd_msg_received);

		ret= pd_send_msg(ds3, MSG_VCONN_SWAP, NULL, 0, SOP_MSG);
		if (ret) {
			dev_err(ds3->dev, "%s: error sending vcs(off) : %d\n",
					__func__, ret);
			break;
		}

		if (!wait_for_completion_timeout(&ds3->is_pd_msg_received,
					msecs_to_jiffies(SENDER_RESPONSE_TIME))) {
			dev_err(ds3->dev, "%s: timed out waiting accept(ofF)\n",
					__func__);
			break;
		}

		msleep(SENDER_RESPONSE_TIME);

		ret = pd_send_msg(ds3, MSG_PS_RDY, NULL, 0, SOP_MSG);
		if (ret) {
			dev_err(ds3->dev, "%s: error sending ps_rdy: %d\n",
					__func__, ret);
			break;
		}
#endif
		ds_set_state(ds3, STATE_DS_RECOVERY_POWER_ON);
		break;

	case STATE_DS_RECOVERY_POWER_ON:
		// 2nd USB on
		 start_2nd_usb_host(ds3);

#if 0
		 /* blocks until USB host is completely started */
		 ret = extcon_blocking_sync(ds3->extcon, EXTCON_USB_HOST, 0);
		 if (ret) {
			 dev_err(ds3->dev, "%s: err(%d) starting host", ret);
			 break;
		 }
#endif
		if (ds3->ds_en) {
			gpiod_direction_output(ds3->ds_en, 1);
		} else if (ds3->vconn) {
			ret = regulator_enable(ds3->vconn);
			if (ret) {
				dev_err(dev, "Unable to enable vconn\n");
			}
		}
		gpiod_direction_output(ds3->load_sw_on, 1);
#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
		 reinit_completion(&ds3->is_pd_msg_received);

		 ret = pd_send_msg(ds3, MSG_VCONN_SWAP, NULL, 0, SOP_MSG);
		 if (ret) {
			 dev_err(ds3->dev, "%s: error sending vcs(on) : %d\n",
					 __func__, ret);
			 break;
		 }

		 if (!wait_for_completion_timeout(&ds3->is_pd_msg_received,
					 msecs_to_jiffies(SENDER_RESPONSE_TIME))) {
			 dev_err(ds3->dev, "%s: timed out waiting accept (on)\n",
					 __func__);
			 break;
		 }

		 reinit_completion(&ds3->is_pd_msg_received);
		 if (!wait_for_completion_timout(&ds3->is_pd_msg_received,
					 msecs_to_jiffies(SENDER_RESPONSE_TIME)))
			 dev_dbg(ds3->dev, "%s: timed out waiting ps_rdy (on)\n",
					 __func__);
#endif
		 ds_set_state(ds3, STATE_DS_RECOVERY_USB_WAIT);
		 break;

	case STATE_DS_RECOVERY_USB_WAIT:
		 ds_set_state(ds3, STATE_DS_RECOVERY);
		 break;

	case STATE_DS_DLOAD:
		 break;

	case STATE_NONE_DS_RECOVERY:
		 set_hallic_status(true);

		 dev_dbg(dev, "%s: USB recovery is completed\n", __func__);
		 ds3->is_usb_recovery = false;
		 break;
	default:
		 dev_err(dev, "%s: Unhandled state %s\n", __func__,
				 ds_state_strings[ds3->current_state]);
		 break;
	}

sm_done:
	if (!ds3->sm_queued)
		pm_relax(ds3->dev);
}

static void ds_set_state(struct ds3 *ds3, enum ds_state next_state)
{
	struct device *dev = ds3->dev;
	dev_dbg(dev, "%s: %s -> %s\n", __func__,
			ds_state_strings[ds3->current_state],
			ds_state_strings[next_state]);

	ds3->current_state = next_state;

	switch (next_state) {
	case STATE_DS_USB_WAIT:
		kick_sm(ds3, ds_usb_check_time_ms);
		break;

	case STATE_DS_STARTUP:
		kick_sm(ds3, 0);
		break;

	case STATE_DS_READY:
		kick_sm(ds3, 0);
		break;

	case STATE_DS_RECOVERY:
		if (ds3->is_ds_recovery <= 0)
			break;

		kick_sm(ds3, ds_usb_check_time_ms);
		break;

	case STATE_DS_RECOVERY_POWER_OFF:
		kick_sm(ds3, 0);
		break;

	case STATE_DS_RECOVERY_POWER_ON:
		kick_sm(ds3, ds_vconn_recovery_time_ms);
		break;

	case STATE_DS_RECOVERY_USB_WAIT:
		kick_sm(ds3, ds_usb_check_time_ms);
		break;

	case STATE_DS_DLOAD:
		ds3->is_ds_recovery = 0;
#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
		ds3_dp_hpd(ds3, false);
#else
		ds_dp_hpd_direct(ds3, false);
#endif
		break;

	case STATE_NONE_DS_RECOVERY:
		ds3->is_usb_recovery = true;
		set_hallic_status(false);
		dev_err(dev, "%s: Recover because DS is not connected\n",
				__func__);
		break;

	default:
		dev_err(dev, "%s: No action for state %s\n", __func__,
				ds_state_strings[ds3->current_state]);
		break;
	}
}

static void kick_sm(struct ds3 *ds3, int ms)
{
	pm_stay_awake(ds3->dev);
	ds3->sm_queued = true;

	if (ms) {
		dev_dbg(ds3->dev, "delay %d ms", ms);
		hrtimer_start(&ds3->timer, ms_to_ktime(ms), HRTIMER_MODE_REL);
	} else {
		queue_work(ds3->wq, &ds3->sm_work);
	}
}

static enum hrtimer_restart ds_timeout(struct hrtimer *timer)
{
	struct ds3 *ds3 = container_of(timer, struct ds3, timer);

	queue_work(ds3->wq, &ds3->sm_work);

	return HRTIMER_NORESTART;
}

static void ds_acc_detect(struct work_struct *w)
{
	struct ds3 *ds3 = container_of(w, struct ds3, ds_acc_detect_work.work);
	hrtimer_cancel(&ds3->acc_timer);

	if (hallic_status && !ds3->is_accid_connected)
	{
		set_hallic_status(true);
	}
}

static enum hrtimer_restart ds_acc_timeout(struct hrtimer *timer)
{
	struct ds3 *ds3 = container_of(timer, struct ds3, acc_timer);

	schedule_delayed_work(&ds3->ds_acc_detect_work, 0);

	return HRTIMER_NORESTART;
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct ds3 *ds3 = container_of(nb, struct ds3, psy_nb);
	struct device *dev = ds3->dev;
	union power_supply_propval val;
	enum power_supply_typec_mode typec_mode;
	int ret;

	if (ptr != ds3->usb_psy || evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	ret = power_supply_get_property(ds3->usb_psy,
			POWER_SUPPLY_PROP_PD_ACTIVE, &val);
	if (ret) {
		dev_err(dev, "Unable to read PD_ACTIVE: %d\n", ret);
		return ret;
	}
	ds3->pd_active = val.intval;

	ret = power_supply_get_property(ds3->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	if (ret) {
		dev_err(dev, "Unable to read USB PRESENT: %d\n", ret);
		return ret;
	}
	ds3->vbus_present = val.intval;

	ret = power_supply_get_property(ds3->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_MODE, &val);
	if (ret < 0) {
		dev_err(dev, "Unable to read USB TYPEC_MODE: %d\n", __func__);
		return ret;
	}

	if (force_set_hallic)
		hallic_status = true;

	typec_mode = val.intval;

	dev_dbg(dev, "typec:%d vbus:%d pd:%d ds3:%d hallic:%d usb:%d"
			"ds3_usb:%d usb_recovery:%d ds_recovery:%d\n",
		typec_mode,
		ds3->vbus_present,
		ds3->pd_active,
		ds3->is_ds_connected,
		hallic_status,
		ds3->is_usb_connected,
		ds3->is_ds_usb_connected,
		ds3->is_usb_recovery,
		ds3->is_ds_recovery);

	if (typec_mode == ds3->typec_mode)
		return 0;

	ds3->typec_mode = typec_mode;

#ifndef CONFIG_MACH_KONA_TIMELM
	switch (typec_mode) {
	case POWER_SUPPLY_TYPEC_NONE:
#ifdef USE_2ND_USB
		if (usb_2nd_host_test) {
			kick_sm(ds3, 0);
			return 0;
		}
#endif

		// Disable DisplayPort
		ds3->is_dp_configured = false;
		ds3->is_dp_hpd_high = false;

		if (ds3->is_ds_connected)
			kick_sm(ds3, 0);
		break;

	case POWER_SUPPLY_TYPEC_SINK:
	case POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE:
#ifdef USE_2ND_USB
		if (usb_2nd_host_test) {
			ds_set_state(ds3, STATE_DS_STARTUP);
			return 0;
		}
#endif

		if (ds3->is_usb_recovery) {
			dev_dbg(dev, "Ignoring due to recovery\n");
			return 0;
		}

		if (hallic_status &&
		    !ds3->is_ds_connected &&
		    !ds3->is_usb_connected &&
		    ds3->pd_active == POWER_SUPPLY_PD_INACTIVE)
			ds_set_state(ds3, STATE_DS_STARTUP);
		break;

	default:
		break;
	}
#endif
	return 0;
}

static ssize_t ds2_hal_ready_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ds3 *ds3 = dev_get_drvdata(dev);
	bool ready;
	int ret;

	if (!ds3->is_ds_connected)
		return -ENODEV;

	ret = strtobool(buf, &ready);
	if (ret < 0)
		return ret;

	dev_info(ds3->dev, "%s: ready:%d, recovery:%d\n", __func__,
			ready, ds3->is_ds_recovery);

	if (!ready)
		return size;

	ds3->is_ds_hal_ready = true;

	if (ds3->is_ds_recovery || ds3->current_state == STATE_DS_RECOVERY) {
		ds3->is_ds_recovery = 0;
		request_dualscreen_recovery();
	}
	ds_set_state(ds3, STATE_DS_READY);

	return size;
}
static DEVICE_ATTR_WO(ds2_hal_ready);

static ssize_t ds2_pd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ds3 *ds3 = dev_get_drvdata(dev);
	dev_dbg(ds3->dev, "%s: hpd_high:%d\n", __func__, ds3->is_dp_hpd_high);
	return scnprintf(buf, PAGE_SIZE, "%d", ds3->is_dp_hpd_high);
}

static ssize_t ds2_pd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ds3 *ds3 = dev_get_drvdata(dev);
	int hpd_high, refresh_layer;

	if (sscanf(buf, "%d%d", &hpd_high, &refresh_layer) <= 0) {
		dev_err(ds3->dev, "%s: invalid agument: %s", __func__, buf);
		return -EINVAL;
	}

	dev_info(ds3->dev, "%s: hpd_high:%d refresh_layer:%d\n", __func__,
			hpd_high, refresh_layer);

	if (!ds3->is_dp_configured) {
		dev_info(ds3->dev, "%s: dp is not configured\n", __func__);
		return size;
	}

#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
	ds3_dp_hpd(ds3, hpd_high);
#else
	ds_dp_hpd_direct(ds3, hpd_high);
#endif

	return size;
}

static DEVICE_ATTR_RW(ds2_pd);

static ssize_t ds2_recovery_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ds3 *ds3 = dev_get_drvdata(dev);
	bool recovery;
	int ret;

	if (!ds3->is_ds_connected)
		return -ENODEV;

	ret = strtobool(buf, &recovery);
	if (ret < 0)
		return ret;

	dev_info(ds3->dev, "%s: recovery:%d\n", __func__, recovery);

	if (!recovery)
		return size;

	ds3->is_ds_recovery = ds_power_recovery_count;
	ds_set_state(ds3, STATE_DS_RECOVERY_POWER_OFF);

	return size;
}
// #define RECOVERY_PATH "/sys/class/dualscreen/ds2/ds2_recovery"
static DEVICE_ATTR_WO(ds2_recovery);


static void ds3_enable_acc_regulator(struct ds3 *ds3, bool enable)
{
	int ret = 0;

	if (enable) {
		if (ds3->acc_det_pu && !regulator_is_enabled(ds3->acc_det_pu)) {
			ret = regulator_enable(ds3->acc_det_pu);
			if (ret < 0)
				dev_err(ds3->dev, "%s: Can't Enable DS3 regulator, ret=%d\n",
						__func__, ret);
			else
				dev_info(ds3->dev, "%s: regulator setting to %d\n",
						__func__, !!enable);
		}
	} else {
		if (ds3->acc_det_pu && regulator_is_enabled(ds3->acc_det_pu)) {
			ret = regulator_disable(ds3->acc_det_pu);
			if (ret < 0)
				dev_err(ds3->dev, "%s: Can't disable DS3 regulator, ret=%d\n",
						__func__, ret);
			else
				dev_info(ds3->dev, "%s: regulator setting to %d\n",
						__func__, !!enable);
		}
	}
}

static int check_ds3_accid(struct ds3 *ds3, bool enable, bool use_vadc)
{
	int acc_det = 0;
	int i = 0;
	int acc_vadc_result = 0;

	if (enable) {
		ds3_enable_acc_regulator(ds3, true);
		msleep(ds_accid_reg_en_delay_ms);
		if (use_vadc) {
			for (i = 0; i < 3; i++) {
				if (!ds3->channel) {
					dev_err(ds3->dev, "IIO Channel doesn't exist\n");
					ds3->is_accid_connected = false;
					return 0;
				}
				iio_read_channel_processed(ds3->channel, &acc_vadc_result);
				dev_dbg(ds3->dev, "ACC_DETECT Vadc result=%d\n",
							acc_vadc_result);
				if (acc_vadc_result <= 850000 && acc_vadc_result >= 500000) {
					ds3->is_accid_connected = true;
					break;
				}
				msleep(20);
			}
		} else {
			for (i = 0; i < 3; i++) {
				acc_det = gpio_get_value(ds3->acc_detect);

				if (acc_det) {
				} else {
					ds3->is_accid_connected = true;
					break;
				}
				msleep(20);
			}
		}
		ds3_enable_acc_regulator(ds3, false);
		if (!ds3->is_accid_connected) {
			if (ds3->acc_det_count < 5) {
			dev_dbg(ds3->dev, "hallic detected but acc_id isn't. start timer\n");
				hrtimer_start(&ds3->acc_timer,
						ms_to_ktime(ds_recheck_accid_ms),
						HRTIMER_MODE_REL);
				ds3->acc_det_count++;
			} else {
				dev_info(ds3->dev, "acc_detect retry count exceeded\n");
			}
		} else {
			ds3->acc_det_count = 0;
			return 1;
		}
	} else {
		ds3->acc_det_count = 0;
		ds3_enable_acc_regulator(ds3, false);
		ds3->is_accid_connected = false;
		return 0;
	}

	return 0;
}

static int ds3_probe_vcomp(struct ds3 *ds3)
{
	struct device *dev = ds3->dev;
	int ret = 0;

	ds3->acc_detect = of_get_named_gpio(dev->of_node, "lge,acc-detect", 0);

	if (ds3->acc_detect < 0) {
		dev_err(dev, "Failed to get acc_detect: %d\n", ds3->acc_detect);
		return -ENODEV;
	}

	ret = gpio_request_one(ds3->acc_detect, GPIOF_DIR_IN, "acc_detect");
	if (ret < 0) {
		dev_err(dev, "Failed to request acc_detect, ret=%d\n", ret);
		return -ENXIO;
	}

	return 0;
}

static int ds3_probe_vadc(struct ds3 *ds3)
{
	struct iio_channel *channel = NULL;
	struct device *dev = ds3->dev;
	int ret = 0;
	channel = iio_channel_get(dev, "gpio4_pu3");
	if (PTR_ERR(channel) == -EPROBE_DEFER) {
		dev_err(dev, "channel probe defer\n");
		return -EPROBE_DEFER;
	}
	if (IS_ERR(channel)) {
		ret = PTR_ERR(channel);
		dev_err(dev, "Unable to get VADC dev\n");
		return ret;
	}
	ds3->channel = channel;
	ds3->acc_det_count = 0;

	return 0;
}

static int ds3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ds3 *ds3;
	static struct class *ds3_class;
	static struct device *ds3_dev;
#ifdef CONFIG_LGE_USB_DS_DIRECT_SVID
	struct usbpd *pd = NULL;
#endif
	int ret = 0;

	dev_info(dev, "%s\n", __func__);

#ifdef CONFIG_LGE_USB_DS_DIRECT_SVID
	pd = devm_usbpd_get_by_phandle(dev, "usbpd");
	if (IS_ERR(pd)) {
		dev_err(dev, "usbpd phandle failed (%ld)\n", PTR_ERR(pd));
		return (PTR_ERR(pd) == -EAGAIN) ? -EPROBE_DEFER : PTR_ERR(pd);
	}
#endif

	ds3 = devm_kzalloc(dev, sizeof(*ds3), GFP_KERNEL);
	if (!ds3) {
		dev_err(dev, "out of memory\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, ds3);
	ds3->dev = dev;
#ifdef CONFIG_LGE_USB_DS_DIRECT_SVID
	ds3->usbpd = pd;
#endif
	ds3->usb_psy = power_supply_get_by_name("usb");
	if (!ds3->usb_psy) {
		dev_err(dev, "couldn't get USB power_supply, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto err;
	}

	ds3->vconn = devm_regulator_get(dev, "vconn");
	if (IS_ERR(ds3->vconn)) {
		ret = PTR_ERR(ds3->vconn);
		dev_err(dev, "Unable to get vconn: %d\n", ret);
		goto err;
	}

	ds3->load_sw_on = devm_gpiod_get(dev, "lge,load-sw-on", GPIOD_OUT_LOW);
	if (IS_ERR(ds3->load_sw_on)) {
		ret = PTR_ERR(ds3->load_sw_on);
		dev_err(dev, "couldn't get load-sw-on: %d\n", ret);
		goto err;
	}

	ds3->ds_en = devm_gpiod_get(dev, "lge,ds-en", GPIOD_OUT_LOW);
	if (IS_ERR(ds3->ds_en)) {
		ret = PTR_ERR(ds3->ds_en);
		dev_err(dev, "Unable to get ds_en: %d\n", ret);
		ds3->ds_en = NULL;
	}

#ifdef USE_2ND_USB
	ds3->extcon = devm_extcon_dev_allocate(dev, ds_extcon_cable);
	if (IS_ERR(ds3->extcon)) {
		ret = PTR_ERR(ds3->extcon);
		dev_err(dev, "failed to allocate extcon device: %d\n", ret);
		goto err;
	}

	ret = devm_extcon_dev_register(dev, ds3->extcon);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device: %d\n", ret);
		goto err;
	}

	ds3->dd_sw_sel = devm_gpiod_get(dev, "lge,dd-sw-sel", GPIOD_OUT_LOW);
	if (IS_ERR(ds3->dd_sw_sel)) {
		ret = PTR_ERR(ds3->dd_sw_sel);
		dev_err(dev, "couldn't get dd-sw-sel gpio: %d\n", ret);
		goto err;
	}
#endif

	ret = device_init_wakeup(ds3->dev, true);
	if (ret < 0)
		goto err;

	ds3->wq = alloc_ordered_workqueue("ds_wq", WQ_HIGHPRI);
	if (!ds3->wq)
		return -ENOMEM;

	ds3->acc_det_vcomp = device_property_read_bool(dev,
				"lge,acc_det_vcomp");
	if (ds3->acc_det_vcomp) {
		ret = ds3_probe_vcomp(ds3);
		if (ret < 0) {
			dev_err(dev, "failed to register vcomp: %d\n", ret);
			goto err;
		}

	}

	ds3->acc_det_vadc = device_property_read_bool(dev,
				"lge,acc_det_vadc");
	if (ds3->acc_det_vadc) {
		ret = ds3_probe_vadc(ds3);
		if (ret < 0) {
			dev_err(dev, "failed to register vadc: %d\n", ret);
			goto err;
		}
	}

	if (ds3->acc_det_vadc || ds3->acc_det_vcomp) {
		ds3->acc_det_pu = regulator_get(dev, "lge,acc");
		if (IS_ERR(ds3->acc_det_pu)) {
			dev_err(dev, "Regulator get failed, errno=%d\n",
					PTR_ERR(ds3->acc_det_pu));
			return PTR_ERR(ds3->acc_det_pu);
		}
	}

	hrtimer_init(&ds3->acc_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ds3->acc_timer.function = ds_acc_timeout;

	hrtimer_init(&ds3->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ds3->timer.function = ds_timeout;

#ifndef CONFIG_LGE_USB_DS_DIRECT_SVID
	init_completion(&ds3->is_pd_msg_received);
#endif

#ifdef CONFIG_LGE_USB_SBU_SWITCH
	ds3->sbu_desc.flags = LGE_SBU_SWITCH_FLAG_SBU_AUX;
	ds3->sbu_inst = devm_lge_sbu_switch_instance_register(ds3->dev,
							&ds3->sbu_desc);
	if (IS_ERR_OR_NULL(ds3->sbu_inst)) {
		dev_err(dev, "Couldn't register lge_sbu_switch rc=%d\n",
				PTR_ERR(ds3->sbu_inst));
		return PTR_ERR(ds3->sbu_inst);
	}
#endif

	INIT_WORK(&ds3->sm_work, ds3_sm);
	INIT_DELAYED_WORK(&ds3->ds_acc_detect_work, ds_acc_detect);

	ds3->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&ds3->psy_nb);
	if (ret < 0)
		goto err;

	ds3_class = class_create(THIS_MODULE, "dualscreen");
	if (IS_ERR(ds3_class)) {
		ret = PTR_ERR(ds3_class);
		dev_err(dev, "failed to create dualscreen class: %d\n", ret);
		goto err_create_ds3_class;
	}

	ds3_dev = device_create(ds3_class, NULL, 0, ds3, "ds2");
	if (IS_ERR(ds3_dev)) {
		ret = PTR_ERR(ds3_dev);
		dev_err(dev, "failed to create device: %d\n", ret);
		goto err_create_ds3_dev;
	}

	ret = device_create_file(ds3_dev, &dev_attr_ds2_hal_ready);
	if (ret < 0) {
		dev_err(dev, "failed to create ds2_hal_ready node: %d\n", ret);
		goto err_create_ds3_hal_ready;
	}

	ret = device_create_file(ds3_dev, &dev_attr_ds2_pd);
	if (ret < 0) {
		dev_err(dev, "failed to create ds3_pd node: %d\n", ret);
		goto err_create_ds3_pd;
	}

	ret = device_create_file(ds3_dev, &dev_attr_ds2_recovery);
	if (ret < 0) {
		dev_err(dev, "failed to create ds2_recovery node: %d\n", ret);
		goto err_create_ds3_recovery;
	}
	ds3->nb.notifier_call = ds3_usb_notify;
	usb_register_notify(&ds3->nb);

	__ds3 = ds3;

	ds3_connected = &ds3->is_dp_configured;

	/* force read initial power_supply values */
	psy_changed(&ds3->psy_nb, PSY_EVENT_PROP_CHANGED, ds3->usb_psy);

	return 0;

err_create_ds3_recovery:
	device_remove_file(ds3_dev, &dev_attr_ds2_pd);
err_create_ds3_pd:
	device_remove_file(ds3_dev, &dev_attr_ds2_hal_ready);
err_create_ds3_hal_ready:
	device_unregister(ds3_dev);
err_create_ds3_dev:
	class_destroy(ds3_class);
err_create_ds3_class:
	power_supply_unreg_notifier(&ds3->psy_nb);
err:
	return ret;
}

static void ds3_shutdown(struct platform_device *pdev)
{
	struct ds3 *ds3 = platform_get_drvdata(pdev);

	power_supply_unreg_notifier(&ds3->psy_nb);
#if 0 // FIXME ...
	cancel_work(&ds3->work);
#endif
	hrtimer_cancel(&ds3->acc_timer);
	ds3->sm_queued = false;

#ifdef USE_2nd_USB
	// secondary USB
	stop_2nd_usb_host(ds3);
	gpiod_direction_output(ds3->dd_sw_sel, 0);
#endif
	if (ds3->acc_detect)
		gpio_free(ds3->acc_detect);

	if (ds3->acc_det_vcomp || ds3->acc_det_vadc)
		regulator_put(ds3->acc_det_pu);

}
static const struct of_device_id ds3_match_table[] = {
	{ .compatible = "lge,usb_ds3" },
	{ }
};
MODULE_DEVICE_TABLE(of, ds3_match_table);

static struct platform_driver ds3_driver = {
	.driver = {
		.name = "lge_usb_ds3",
		.of_match_table = ds3_match_table,
	},
	.probe = ds3_probe,
	.shutdown = ds3_shutdown,
};
module_platform_driver(ds3_driver);

MODULE_AUTHOR("Hansun Lee <hansun.lee@lge.com>");
MODULE_DESCRIPTION("LGE USB DS3 driver");
MODULE_LICENSE("GPL v2");
