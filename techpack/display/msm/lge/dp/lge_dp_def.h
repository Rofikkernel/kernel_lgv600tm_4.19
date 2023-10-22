#ifndef _H_LGE_DP_DEF_
#define _H_LGE_DP_DEF_
#include <linux/module.h>
#include <asm/atomic.h>
#if defined(CONFIG_LGE_DUAL_SCREEN)
#include <linux/extcon.h>
#define EXT_DD_MAX_COUNT 3
#endif

struct lge_dp_display {
	bool force_disconnection;
	int hpd_state;
	int skip_uevent;
	int dd_button_state;
	int led_status;
	atomic_t pending;
	int pending_status;
	atomic_t dd_uevent_switch;
	atomic_t dd_5v_power_state;
	struct completion recovery_off_comp;
	int real_disconnection;
	int block_state;
	unsigned int vid_pid;
	int tuning_enable;
	u32 tuning_swing_value;
	u32 tuning_pre_emp;
#if IS_ENABLED(CONFIG_LGE_DUAL_SCREEN)
	struct lge_cover_ops *cover_ops;
	struct mutex cover_lock;
	struct extcon_dev *dd_extcon_sdev[EXT_DD_MAX_COUNT];
	struct notifier_block dd_extcon_nb;
	int ds_connected;
	ktime_t last_recovery_time;
	int recovery_count;
	int need_to_recovery;
	bool need_to_wait_real_disconnect;
	int need_to_skip_real_disconnect;
#endif
};
#endif //_H_LGE_DP_DEF_
