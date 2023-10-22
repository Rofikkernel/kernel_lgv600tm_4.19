#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/of_gpio.h>

#define MAX_FINGER_DEX		3

struct dex_active_area {
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
};

enum {
	DEX_IN = 1,
	DEX_OUT,
};

enum {
	DEX_PRESSED = 1,
	DEX_RELEASED,
	DEX_MOVED,
	DEX_NOT_SUPPORT,
};

struct touch_dex_ctrl {
	bool enable;
	struct dex_active_area area;
};

struct touch_dex_data {
	u8 pos;
	u8 status;
};
extern int dex_input_init(struct device *dev);
extern void dex_input_handler(struct device *dev, struct input_dev *input);
