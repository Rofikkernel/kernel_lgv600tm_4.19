#ifndef _LGE_DS3_H
#define _LGE_DS3_H

enum {
	DS_STATE_DISCONNECTED = 0,
	DS_STATE_HALLIC_CONNECTED,
	DS_STATE_ACC_ID_CONNECTED,
	DS_STATE_USB_CONNECTED,
	DS_STATE_DP_CONNECTED,
	DS_STATE_HPD_ENABLED,
};

#ifdef CONFIG_LGE_DUAL_SCREEN
bool is_ds_connected(void);
int check_ds_connect_state(void);
void set_hallic_status(bool enable);
#else
static inline bool is_ds_connected(void) { return false; };
static inline int check_ds_connect_state(void) { return false; };
static inline bool void set_hallic_status(bool enable) { return false; };
#endif

#endif // _LGE_DS3_H
