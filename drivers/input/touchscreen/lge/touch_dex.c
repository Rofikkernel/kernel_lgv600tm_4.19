/* Driver for Touch DEX */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <touch_core.h>

static int touch_set_input_prop(struct touch_core_data *ts, struct input_dev *input)
{
	int ret;
	TOUCH_TRACE();
	input->phys = "devices/virtual/input";
	TOUCH_I("%s %d-%d-%d\n", __func__,
			ts->caps.max_x,
			ts->caps.max_y,
			MAX_FINGER_DEX);

	/* Common Set */
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	set_bit(BTN_TOOL_FINGER, input->keybit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			ts->caps.max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			ts->caps.max_y, 0, 0);
	set_bit(INPUT_PROP_POINTER, input->propbit);
	ret = input_mt_init_slots(input, MAX_FINGER_DEX, INPUT_PROP_POINTER);
	if (ret < 0) {
		TOUCH_E("failed to init slots (ret:%d)\n", ret);
		return -EAGAIN;
	}
	input_set_drvdata(input, ts);

	return 0;
}

int dex_input_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret;

	TOUCH_TRACE();

	ts->input_dex = input_allocate_device();
	if (!ts->input_dex) {
		TOUCH_E("failed to allocate memory for input dex\n");
		return -ENOMEM;
	}

	ts->input_dex->name = "touch_dex_dev";
	ret = touch_set_input_prop(ts, ts->input_dex);
	if (ret < 0) {
		goto error_register;
	}
	ret = input_register_device(ts->input_dex);
	if (ret < 0) {
		TOUCH_E("failed to register input dex(ret:%d)\n", ret);
		goto error_register;
	}
	return 0;

error_register:
	input_mt_destroy_slots(ts->input_dex);
	input_free_device(ts->input_dex);

	if (ts->input_dex) {
		input_mt_destroy_slots(ts->input_dex);
		input_free_device(ts->input_dex);
	}
	return ret;
}

static int dex_area_filter(struct device *dev, int id)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int x = 0;
	int y = 0;

	TOUCH_TRACE();

	x = ts->tdata[id].x;
	y = ts->tdata[id].y;

	if ((x >= ts->touch_dex.area.x1) && (x <= ts->touch_dex.area.x2)
		&& (y >= ts->touch_dex.area.y1) && (y <= ts->touch_dex.area.y2)) {
		return DEX_IN;
	} else {
		return DEX_OUT;
	}
}

/* Input Dex in Touch core 
 * This function is not support changing ts->mask
 * */
void dex_input_handler(struct device *dev, struct input_dev *input)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u16 old_mask = ts->old_mask;
	u16 new_mask = ts->new_mask;
	u16 press_mask = 0;
	u16 release_mask = 0;
	u16 change_mask = 0;
	int i;
	bool hide_lockscreen_coord =
		((atomic_read(&ts->state.lockscreen) == LOCKSCREEN_LOCK) &&
		 (ts->role.hide_coordinate));

	TOUCH_TRACE();

	change_mask = old_mask ^ new_mask;
	press_mask = new_mask & change_mask;
	release_mask = old_mask & change_mask;

	TOUCH_D(ABS, "mask [new: %04x, old: %04x]\n",
			new_mask, old_mask);
	TOUCH_D(ABS, "mask [change: %04x, press: %04x, release: %04x]\n",
			change_mask, press_mask, release_mask);

	/* Palm state - Report Pressure value 255 */
	if (ts->is_cancel) {
		touch_report_cancel_event(ts);
		ts->is_cancel = 0;
	}

	for (i = 0; i < MAX_FINGER_DEX; i++) {
		if (new_mask & (1 << i)) {
			ts->tdata[i].dex_data.pos = dex_area_filter(dev, i);

			if (press_mask & (1 << i)) {
				if (ts->tdata[i].dex_data.pos == DEX_OUT) {
					ts->tdata[i].dex_data.status = DEX_NOT_SUPPORT;
				} else {
					ts->tdata[i].dex_data.status = DEX_PRESSED;
					ts->dex_tcount++;
					if (hide_lockscreen_coord) {
						TOUCH_I("%d finger pressed dex:<%d>(xxxx,xxxx,xxxx)\n",
								ts->tcount, i);
					} else {
						TOUCH_I("%d finger pressed dex:<%d>(%4d,%4d)\n",
								ts->tcount,
								i,
								ts->tdata[i].x,
								ts->tdata[i].y);
					}
				}
			}

			if (ts->tdata[i].dex_data.status == DEX_PRESSED ||
					ts->tdata[i].dex_data.status == DEX_MOVED) {
				/* Force Released */
				if (ts->tdata[i].dex_data.pos == DEX_OUT) {
					input_mt_slot(input, i);
					input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
					ts->tdata[i].dex_data.status = DEX_RELEASED;
					if (ts->dex_tcount > 0)
						ts->dex_tcount--;

					if (hide_lockscreen_coord) {
						TOUCH_I(" finger released dex:<%d>(xxxx,xxxx,xxxx)\n",
								i);
					} else {
						TOUCH_I(" finger released dex:<%d>(%4d,%4d)\n",
								i,
								ts->tdata[i].x,
								ts->tdata[i].y);
					}
				} else {
					input_mt_slot(input, i);
					input_mt_report_slot_state(input, MT_TOOL_FINGER,
							true);
					input_report_key(input, BTN_TOUCH, 1);
					input_report_key(input, BTN_TOOL_FINGER, 1);
					input_report_abs(input, ABS_MT_POSITION_X,
							ts->tdata[i].x);
					input_report_abs(input, ABS_MT_POSITION_Y,
							ts->tdata[i].y);
					if (ts->tdata[i].dex_data.status == DEX_PRESSED) {
						input_sync(input);
						ts->tdata[i].dex_data.status = DEX_MOVED;
					}
				}
			} else {
				TOUCH_D(ABS,"Ignore Event:<%d>(%4d,%4d)\n",
						i,
						ts->tdata[i].x,
						ts->tdata[i].y);
			}

		} else if (release_mask & (1 << i)) {
			ts->tdata[i].dex_data.pos = dex_area_filter(dev, i);

			if (ts->tdata[i].dex_data.pos == DEX_IN
					&& ts->tdata[i].dex_data.status == DEX_MOVED) {
				input_mt_slot(input, i);
				input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
				ts->tdata[i].dex_data.status = DEX_RELEASED;
				if (ts->dex_tcount > 0)
					ts->dex_tcount--;
				if (hide_lockscreen_coord) {
					TOUCH_I(" finger released dex:<%d>(xxxx,xxxx,xxxx)\n",
							i);
				} else {
					TOUCH_I(" finger released dex:<%d>(%4d,%4d)\n",
							i,
							ts->tdata[i].x,
							ts->tdata[i].y);
				}

			} else {
				TOUCH_D(ABS,"Ignore Event:<%d>(%4d,%4d)\n",
						i,
						ts->tdata[i].x,
						ts->tdata[i].y);
			}
		}
	}

	if (!ts->dex_tcount) {
		input_report_key(input, BTN_TOUCH, 0);
		input_report_key(input, BTN_TOOL_FINGER, 0);
	}

	if (ts->tdata[i].dex_data.status != DEX_PRESSED) {
		input_sync(input);
	}
}

MODULE_AUTHOR("rangkast.jeong@lge.com");
MODULE_DESCRIPTION("LGE DEX driver v1");
MODULE_LICENSE("GPL");
