/*
 * touch_core.c
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/input/lge_touch_notify.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_common.h>

struct mutex touch_probe_lock;

u32 touch_debug_mask = BASE_INFO;
/* Debug mask value
 * usage: echo [debug_mask] > /sys/module/touch_core/parameters/debug_mask
 */
module_param_named(debug_mask, touch_debug_mask, int, 0664);

static void touch_suspend(struct device *dev);
static void touch_resume(struct device *dev);
#if defined(CONFIG_SECURE_TOUCH)
static irqreturn_t secure_touch_filter_interrupt(struct touch_core_data *ts);
#endif

void touch_report_cancel_event(struct touch_core_data *ts)
{
	u16 old_mask = ts->old_mask;
	int i = 0;

	TOUCH_TRACE();

	for (i = 0; i < MAX_FINGER; i++) {
		if (old_mask & (1 << i)) {
			input_mt_slot(ts->input, i);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER,
						   true);
			input_report_key(ts->input, BTN_TOUCH, 1);
			input_report_key(ts->input, BTN_TOOL_FINGER, 1);
			input_report_abs(ts->input, ABS_MT_PRESSURE,
					255);
			TOUCH_I("finger canceled:<%d>(%4d,%4d,%4d)\n",
					i,
					ts->tdata[i].x,
					ts->tdata[i].y,
					ts->tdata[i].pressure);
		}
	}

	input_sync(ts->input);
}

static void touch_report_event(struct touch_core_data *ts)
{
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

#if defined(CONFIG_MTK_PERFMGR_TOUCH_BOOST)
	if (new_mask)
		input_report_key(ts->input, BTN_TOUCH, 1);
	else
		input_report_key(ts->input, BTN_TOUCH, 0);
#endif

	/* Palm state - Report Pressure value 255 */
	if (ts->is_cancel) {
		touch_report_cancel_event(ts);
		ts->is_cancel = 0;
	}

	for (i = 0; i < MAX_FINGER; i++) {
		if (new_mask & (1 << i)) {
			input_mt_slot(ts->input, i);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER,
						   true);
			input_report_key(ts->input, BTN_TOUCH, 1);
			input_report_key(ts->input, BTN_TOOL_FINGER, 1);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
					ts->tdata[i].x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
					ts->tdata[i].y);
			input_report_abs(ts->input, ABS_MT_PRESSURE,
					ts->tdata[i].pressure);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
					ts->tdata[i].width_major);
			input_report_abs(ts->input, ABS_MT_WIDTH_MINOR,
					ts->tdata[i].width_minor);
			input_report_abs(ts->input, ABS_MT_ORIENTATION,
					ts->tdata[i].orientation);

			if (press_mask & (1 << i)) {
				if (hide_lockscreen_coord) {
					TOUCH_I("%d finger pressed:<%d>(xxxx,xxxx,xxxx)\n",
							ts->tcount, i);
				} else {
					TOUCH_I("%d finger pressed:<%d>(%4d,%4d,%4d)\n",
							ts->tcount,
							i,
							ts->tdata[i].x,
							ts->tdata[i].y,
							ts->tdata[i].pressure);
				}
			}
		} else if (release_mask & (1 << i)) {
			input_mt_slot(ts->input, i);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
			if (hide_lockscreen_coord) {
				TOUCH_I(" finger released:<%d>(xxxx,xxxx,xxxx)\n",
						i);
			} else {
				TOUCH_I(" finger released:<%d>(%4d,%4d,%4d)\n",
						i,
						ts->tdata[i].x,
						ts->tdata[i].y,
						ts->tdata[i].pressure);
			}
		}
	}

	if (!ts->tcount) {
		input_report_key(ts->input, BTN_TOUCH, 0);
		input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	}

	ts->old_mask = new_mask;
	input_sync(ts->input);
}

void touch_report_all_event(struct touch_core_data *ts)
{
	TOUCH_TRACE();

	ts->is_cancel = 1;
	if (ts->old_mask) {
		ts->new_mask = 0;
		touch_report_event(ts);
		ts->tcount = 0;
		memset(ts->tdata, 0, sizeof(struct touch_data) * MAX_FINGER);
	}
	ts->is_cancel = 0;
}

static void touch_core_initialize(struct touch_core_data *ts)
{
	/* lockscreen */
	touch_report_all_event(ts);
}

irqreturn_t touch_irq_handler(int irq, void *dev_id)
{
	struct touch_core_data *ts = (struct touch_core_data *) dev_id;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
		TOUCH_I("interrupt in suspend[%d]\n",
				atomic_read(&ts->state.pm));
		atomic_set(&ts->state.pm, DEV_PM_SUSPEND_IRQ);
		pm_wakeup_event(ts->dev, 1000);
		return IRQ_HANDLED;
	}
	return IRQ_WAKE_THREAD;
}

irqreturn_t touch_irq_thread(int irq, void *dev_id)
{
	struct touch_core_data *ts = (struct touch_core_data *) dev_id;
	int ret = 0;

	TOUCH_TRACE();

#if defined(CONFIG_SECURE_TOUCH)
	if (secure_touch_filter_interrupt(ts) == IRQ_HANDLED)
		return IRQ_HANDLED;
#endif
	mutex_lock(&ts->lock);

	ts->intr_status = 0;
	ret = ts->driver->irq_handler(ts->dev);

	if (ret >= 0) {
		if (ts->intr_status & TOUCH_IRQ_FINGER) {
#if defined(CONFIG_LGE_TOUCH_DEX)
			if (atomic_read(&ts->state.dex_mode)) {
				dex_input_handler(ts->dev, ts->input_dex);
			}
#endif
			touch_report_event(ts);
		}

		if (ts->intr_status & TOUCH_IRQ_KNOCK)
			touch_send_uevent(ts, TOUCH_UEVENT_KNOCK);

		if (ts->intr_status & TOUCH_IRQ_PASSWD)
			touch_send_uevent(ts, TOUCH_UEVENT_PASSWD);

		if (ts->intr_status & TOUCH_IRQ_SWIPE_DOWN)
			touch_send_uevent(ts, TOUCH_UEVENT_SWIPE_DOWN);

		if (ts->intr_status & TOUCH_IRQ_SWIPE_UP)
			touch_send_uevent(ts, TOUCH_UEVENT_SWIPE_UP);

		if (ts->intr_status & TOUCH_IRQ_SWIPE_LEFT)
			touch_send_uevent(ts, TOUCH_UEVENT_SWIPE_LEFT);

		if (ts->intr_status & TOUCH_IRQ_SWIPE_RIGHT)
			touch_send_uevent(ts, TOUCH_UEVENT_SWIPE_RIGHT);

		if (ts->intr_status & TOUCH_IRQ_WATER_MODE_ON)
			touch_send_uevent(ts, TOUCH_UEVENT_WATER_MODE_ON);

		if (ts->intr_status & TOUCH_IRQ_WATER_MODE_OFF)
			touch_send_uevent(ts, TOUCH_UEVENT_WATER_MODE_OFF);

		if (ts->intr_status & TOUCH_IRQ_AI_BUTTON)
			touch_send_uevent(ts, TOUCH_UEVENT_AI_BUTTON);

		if (ts->intr_status & TOUCH_IRQ_AI_PICK)
			touch_send_uevent(ts, TOUCH_UEVENT_AI_PICK);

		if (ts->intr_status & TOUCH_IRQ_SWIPE_LEFT2)
			touch_send_uevent(ts, TOUCH_UEVENT_SIDE_PAY);

		if (ts->intr_status & TOUCH_IRQ_SWIPE_RIGHT2)
			touch_send_uevent(ts, TOUCH_UEVENT_SIDE_PAY);

		if (ts->intr_status & TOUCH_IRQ_LPWG_LONGPRESS_DOWN)
			touch_send_uevent(ts, TOUCH_UEVENT_LPWG_LONGPRESS_DOWN);

		if (ts->intr_status & TOUCH_IRQ_LPWG_LONGPRESS_UP)
			touch_send_uevent(ts, TOUCH_UEVENT_LPWG_LONGPRESS_UP);

		if (ts->intr_status & TOUCH_IRQ_LPWG_SINGLE_WAKEUP)
			touch_send_uevent(ts, TOUCH_UEVENT_LPWG_SINGLE_WAKEUP);

		if (ts->intr_status & TOUCH_IRQ_PEN_WAKEUP)
			touch_send_uevent(ts, TOUCH_UEVENT_PEN_WAKEUP);

		if (ts->intr_status & TOUCH_IRQ_PEN_WAKEUP_BTN)
			touch_send_uevent(ts, TOUCH_UEVENT_PEN_WAKEUP_BTN);

	} else {
		if (ret == -EGLOBALRESET) {
			queue_delayed_work(ts->wq, &ts->panel_reset_work, 0);
		} else if (ret == -EHWRESET_ASYNC) {
			ts->driver->power(ts->dev, POWER_HW_RESET_ASYNC);
		} else if (ret == -EHWRESET_SYNC) {
			ts->driver->power(ts->dev, POWER_HW_RESET_SYNC);
		} else if (ret == -ESWRESET) {
			ts->driver->power(ts->dev, POWER_SW_RESET);
		} else if (ret == -EUPGRADE) {
			ts->force_fwup = 1;
			queue_delayed_work(ts->wq, &ts->upgrade_work, 0);
		}
	}

	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}

static void touch_init_work_func(struct work_struct *init_work)
{
	struct touch_core_data *ts =
		container_of(to_delayed_work(init_work),
				struct touch_core_data, init_work);
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("touch_ic_init Start\n");
	mutex_lock(&ts->lock);
	touch_core_initialize(ts);
	ret = ts->driver->init(ts->dev);
	if (ret < 0) {
		TOUCH_E("touch_ic_init failed %d\n", ret);
		mutex_unlock(&ts->lock);
		atomic_set(&ts->state.core, CORE_NORMAL);
		return;
	}
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		touch_restore_state(ts);
		queue_delayed_work(ts->wq, &ts->upgrade_work, 0);
		return;
	}

	atomic_set(&ts->state.core, CORE_NORMAL);
	TOUCH_I("touch_ic_init End\n");
}

static void touch_upgrade_work_func(struct work_struct *upgrade_work)
{
	struct touch_core_data *ts =
		container_of(to_delayed_work(upgrade_work),
				struct touch_core_data, upgrade_work);
	int ret;

	TOUCH_TRACE();

	atomic_set(&ts->state.core, CORE_UPGRADE);

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("%s: state.fb is not FB_RESUME\n", __func__);
		atomic_set(&ts->state.core, CORE_NORMAL);
		goto exit;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	ret = ts->driver->upgrade(ts->dev);

	if (ret < 0) {
		TOUCH_I("There is no need to reset (ret: %d)\n", ret);
		mutex_unlock(&ts->lock);
		mod_delayed_work(ts->wq, &ts->init_work, 0);
		goto exit;
	}

	ts->driver->power(ts->dev, POWER_OFF);
	ts->driver->power(ts->dev, POWER_ON);
	touch_msleep(ts->caps.hw_reset_delay);
	mutex_unlock(&ts->lock);

	mod_delayed_work(ts->wq, &ts->init_work, 0);

exit:
	/* init force_upgrade */
	ts->force_fwup = 0;
	ts->test_fwpath[0] = '\0';
}

#if defined(CONFIG_LGE_TOUCH_APP_FW_UPGRADE)
static void touch_app_upgrade_work_func(struct work_struct *app_upgrade_work)
{
	struct touch_core_data *ts =
		container_of(to_delayed_work(app_upgrade_work),
				struct touch_core_data, app_upgrade_work);
	int ret;

	TOUCH_TRACE();

	mutex_lock(&ts->lock);

	atomic_set(&ts->state.core, CORE_UPGRADE);

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("%s: state.fb is not FB_RESUME\n", __func__);
		goto exit;
	}

	if ((ts->app_fw_upgrade.offset == 0)
		|| (ts->app_fw_upgrade.offset > ts->app_fw_upgrade.max_data_size)
		|| (ts->app_fw_upgrade.data == NULL)) {
		TOUCH_E("invalid app_fw_upgrade info\n");
		goto exit;
	}

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	ret = ts->driver->app_upgrade(ts->dev);
	if (ret < 0) {
		TOUCH_I("%s: There is no need to reset (ret: %d)\n", __func__, ret);
		goto exit;
	}

	ts->driver->power(ts->dev, POWER_OFF);
	ts->driver->power(ts->dev, POWER_ON);
	touch_msleep(ts->caps.hw_reset_delay);
exit:
	/* init force_upgrade */
	ts->force_fwup = 0;
	/* free allocation */
	ts->app_fw_upgrade.status = APP_FW_UPGRADE_IDLE;
	ts->app_fw_upgrade.max_data_size = 0;
	ts->app_fw_upgrade.offset = 0;
	kfree(ts->app_fw_upgrade.data);
	ts->app_fw_upgrade.data = NULL;
	TOUCH_I("%s: clear app_fw_upgrade info\n", __func__);

	mutex_unlock(&ts->lock);

	mod_delayed_work(ts->wq, &ts->init_work, 0);
}
#endif

static void touch_fb_work_func(struct work_struct *fb_work)
{
	struct touch_core_data *ts =
			container_of(to_delayed_work(fb_work),
				struct touch_core_data, fb_work);

	TOUCH_TRACE();

	if (atomic_read(&ts->state.fb) == FB_SUSPEND)
		touch_suspend(ts->dev);
	else if (atomic_read(&ts->state.fb) == FB_RESUME)
		touch_resume(ts->dev);
}

static void touch_panel_reset_work_func(struct work_struct *panel_reset_work)
{
	struct touch_core_data *ts =
			container_of(to_delayed_work(panel_reset_work),
				struct touch_core_data, panel_reset_work);

	TOUCH_TRACE();

	TOUCH_I("Request panel reset !!!\n");
	ts->driver->esd_recovery(ts->dev);
}

static int touch_check_driver_function(struct touch_core_data *ts)
{
	if (ts->driver->probe &&
			ts->driver->remove &&
			ts->driver->shutdown &&
			ts->driver->suspend &&
			ts->driver->resume &&
			ts->driver->init &&
			ts->driver->irq_handler &&
			ts->driver->power &&
			ts->driver->upgrade &&
			ts->driver->esd_recovery &&
			ts->driver->lpwg &&
			ts->driver->swipe_enable &&
			ts->driver->notify &&
			ts->driver->init_pm &&
			ts->driver->register_sysfs &&
			ts->driver->set &&
			ts->driver->get)
		return 0;

	return -EPERM;
}

static int touch_init_platform_data(struct touch_core_data *ts)
{
	int ret;

	TOUCH_TRACE();

	ret = touch_check_driver_function(ts);

	if (ret < 0) {
		TOUCH_E("failed to check functions\n");
		return ret;
	}

	if (ts->dev->of_node)
		ret = touch_get_dts(ts);
	else
		ret = touch_get_platform_data(ts);

	if (ret < 0)
		TOUCH_E("failed to get platform data\n");

	return ret;
}

static void touch_init_locks(struct touch_core_data *ts)
{
	TOUCH_TRACE();

	mutex_init(&ts->lock);
	device_init_wakeup(ts->dev, true);
}

static int touch_init_input(struct touch_core_data *ts)
{
	struct input_dev *input;
	int ret;

	TOUCH_TRACE();

	input = input_allocate_device();

	if (!input) {
		TOUCH_E("failed to allocate memory for input\n");
		return -ENOMEM;
	}

	input->name = "touch_dev";
	input->phys = "devices/virtual/input";

	TOUCH_I("%s %d-%d-%d-%d-%d-%d-%d\n", __func__,
			ts->caps.max_x,
			ts->caps.max_y,
			ts->caps.max_pressure,
			ts->caps.max_width_major,
			ts->caps.max_width_minor,
			ts->caps.max_orientation,
			ts->caps.max_id);
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	set_bit(BTN_TOOL_FINGER, input->keybit);
	set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			ts->caps.max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			ts->caps.max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			ts->caps.max_pressure, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0,
			ts->caps.max_width_major, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR, 0,
			ts->caps.max_width_minor, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION, 0,
			ts->caps.max_orientation, 0, 0);

	ret = input_mt_init_slots(input, ts->caps.max_id, 0);

	if (ret < 0) {
		TOUCH_E("failed to initialize input device(ret:%d)\n", ret);
		goto error;
	}

	ret = input_register_device(input);

	if (ret < 0) {
		TOUCH_E("failed to register input device(ret:%d)\n", ret);
		goto error_register;
	}

	input_set_drvdata(input, ts);
	ts->input = input;

	return 0;

error_register:
	input_mt_destroy_slots(input);

error:
	input_free_device(input);

	return ret;
}

static void touch_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct module_data *md = to_module(plist->sub_dev[0]);
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s Start\n", __func__);
#if defined(CONFIG_SECURE_TOUCH)
	if (atomic_read(&ts->st_enabled))
		secure_touch_stop(ts, true);
#endif
	cancel_delayed_work_sync(&ts->init_work);
	cancel_delayed_work_sync(&ts->upgrade_work);
#if defined(CONFIG_LGE_TOUCH_APP_FW_UPGRADE)
	cancel_delayed_work_sync(&ts->app_upgrade_work);
#endif
	atomic_set(&ts->state.uevent, UEVENT_IDLE);
	mutex_lock(&ts->lock);
	touch_report_all_event(ts);
	atomic_set(&ts->state.fb, FB_SUSPEND);
	/* if need skip, return value is not 0 in pre_suspend */
	ret = ts->driver->suspend(dev);
	/*[TODO] if pdev->id == 1, call subdev suspend*/
	if (0)
		ret = md->m_driver.suspend(dev);
	mutex_unlock(&ts->lock);
	TOUCH_I("%s End\n", __func__);

	if (ret == 1)
		mod_delayed_work(ts->wq, &ts->init_work, 0);
}

static void touch_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct module_data *md = to_module(plist->sub_dev[0]);
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s Start\n", __func__);

	if (atomic_read(&ts->state.first_resume) == FIRST_RESUME_NOT_YET) {
		atomic_set(&ts->state.uevent, UEVENT_IDLE);
		atomic_set(&ts->state.first_resume, FIRST_RESUME_IS_OK);
		TOUCH_I("%s first resume! clear uevent state.\n", __func__);
	}

#if defined(CONFIG_SECURE_TOUCH)
	if (atomic_read(&ts->st_enabled))
		secure_touch_stop(ts, true);
#endif
	mutex_lock(&ts->lock);
	atomic_set(&ts->state.fb, FB_CHANGED);
	/* if need skip, return value is not 0 in pre_resume */
	ret = ts->driver->resume(dev);
	/*[TODO] if pdev->id == 1, call subdev resume*/
	if (0)
		ret = md->m_driver.resume(dev);
	atomic_set(&ts->state.fb, FB_RESUME);
	mutex_unlock(&ts->lock);
	TOUCH_I("%s End\n", __func__);

	if (ret == 0)
		mod_delayed_work(ts->wq, &ts->init_work, 0);
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void touch_early_suspend(struct early_suspend *h)
{
	struct touch_core_data *ts =
		container_of(h, struct touch_core_data, early_suspend);

	TOUCH_TRACE();

	touch_suspend(ts->dev);
}

static void touch_early_resume(struct early_suspend *h)
{
	struct touch_core_data *ts =
		container_of(h, struct touch_core_data, early_suspend);

	TOUCH_TRACE();

	touch_resume(ts->dev);
}

static int touch_init_pm(struct touch_core_data *ts)
{
	TOUCH_TRACE();

	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = touch_early_suspend;
	ts->early_suspend.resume = touch_early_resume;
	register_early_suspend(&ts->early_suspend);
	return 0;
}

#elif defined(CONFIG_DRM_MSM) && defined(CONFIG_FB)
static int touch_drm_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(self, struct touch_core_data, drm_notif);
	struct msm_drm_notifier *ev = (struct msm_drm_notifier *)data;

	if (ev && ev->data && event == MSM_DRM_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == MSM_DRM_BLANK_UNBLANK)
			touch_resume(ts->dev);
		else if (*blank == MSM_DRM_BLANK_POWERDOWN)
			touch_suspend(ts->dev);
	}

	return 0;
}

static int touch_init_pm(struct touch_core_data *ts)
{
	TOUCH_TRACE();

	ts->drm_notif.notifier_call = touch_drm_notifier_callback;
	ts->driver->init_pm(ts->dev);
	return msm_drm_register_client(&ts->drm_notif);
}

#elif defined(CONFIG_FB)
static int touch_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(self, struct touch_core_data, fb_notif);
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			touch_resume(ts->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			touch_suspend(ts->dev);
	}

	return 0;
}

static int touch_init_pm(struct touch_core_data *ts)
{
	TOUCH_TRACE();

	ts->fb_notif.notifier_call = touch_fb_notifier_callback;
	ts->driver->init_pm(ts->dev);
	return fb_register_client(&ts->fb_notif);
}
#endif

static struct bus_type touch_uevent_subsys = {
	.name = LGE_TOUCH_NAME,
	.dev_name = LGE_TOUCH_NAME,
};

static struct device device_uevent_touch = {
	.id    = 0,
	.bus   = &touch_uevent_subsys,
};

static int touch_init_uevent(struct touch_core_data *ts)
{
	int ret = 0;

	TOUCH_TRACE();

	atomic_set(&ts->state.first_resume, FIRST_RESUME_NOT_YET);

	ret = subsys_system_register(&touch_uevent_subsys, NULL);
	if (ret < 0)
		TOUCH_E("%s, bus is not registered, ret : %d\n",
				__func__, ret);
	ret = device_register(&device_uevent_touch);
	if (ret < 0)
		TOUCH_E("%s, device is not registered, ret : %d\n",
				__func__, ret);

	return ret;
}

char *uevent_str[TOUCH_UEVENT_SIZE][2] = {
	{"TOUCH_GESTURE_WAKEUP=WAKEUP", NULL},
	{"TOUCH_GESTURE_WAKEUP=PASSWORD", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWIPE_DOWN", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWIPE_UP", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWIPE_LEFT", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWIPE_RIGHT", NULL},
	{"TOUCH_GESTURE_WAKEUP=WATER_MODE_ON", NULL},
	{"TOUCH_GESTURE_WAKEUP=WATER_MODE_OFF", NULL},
	{"TOUCH_GESTURE_WAKEUP=AI_BUTTON", NULL},
	{"TOUCH_GESTURE_WAKEUP=AI_PICK", NULL},
	{"TOUCH_GESTURE_WAKEUP=LONG_DOWN", NULL},
	{"TOUCH_GESTURE_WAKEUP=LONG_UP", NULL},
	{"TOUCH_GESTURE_WAKEUP=SINGLE_WAKEUP", NULL},
	{"TOUCH_GESTURE_WAKEUP=PEN_WAKEUP", NULL},
	{"TOUCH_GESTURE_WAKEUP=PEN_WAKEUP_BTN", NULL},
	{"TOUCH_GESTURE_WAKEUP=PEN_DETECTION", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWITCH_AES_BOTH", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWITCH_AES_TO_1", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWITCH_AES_TO_2", NULL},
	{"TOUCH_GESTURE_WAKEUP=DS_UPDATE_STATE", NULL},
};

void touch_send_uevent(struct touch_core_data *ts, int type)
{
	TOUCH_TRACE();

	if (type == TOUCH_UEVENT_DS_UPDATE_STATE) {
		kobject_uevent_env(&device_uevent_touch.kobj,
				KOBJ_CHANGE, uevent_str[type]);
		TOUCH_I("%s\n",  uevent_str[type][0]);
	} else if (type == TOUCH_UEVENT_WATER_MODE_ON || type == TOUCH_UEVENT_WATER_MODE_OFF
			|| type == TOUCH_UEVENT_LPWG_LONGPRESS_UP) {
		kobject_uevent_env(&device_uevent_touch.kobj,
				KOBJ_CHANGE, uevent_str[type]);
		TOUCH_I("%s\n",  uevent_str[type][0]);
		touch_report_all_event(ts);
	} else {
		reinit_completion(&ts->uevent_complete);

		if (atomic_read(&ts->state.uevent) != UEVENT_IDLE &&
				touch_check_boot_mode(ts->dev) == TOUCH_NORMAL_BOOT) {
			TOUCH_I("%s is waiting", uevent_str[type][0]);
			wait_for_completion_timeout(&ts->uevent_complete, msecs_to_jiffies(UEVENT_TIMEOUT));
		}

		if (atomic_read(&ts->state.uevent) == UEVENT_IDLE ||
				touch_check_boot_mode(ts->dev) != TOUCH_NORMAL_BOOT) {
			pm_wakeup_event(ts->dev, 3000);
			atomic_set(&ts->state.uevent, UEVENT_BUSY);

			kobject_uevent_env(&device_uevent_touch.kobj,
					KOBJ_CHANGE, uevent_str[type]);
			TOUCH_I("%s\n",  uevent_str[type][0]);
			touch_report_all_event(ts);
		} else {
			TOUCH_I("%s  is not sent\n", uevent_str[type][0]);
		}
	}
}

static int connection;
static int wireless;
static int earjack;
static int fm_radio;

void touch_notify_connect(u32 type)
{
	connection = type;
	touch_atomic_notifier_call(NOTIFY_CONNECTION, &type);
}
EXPORT_SYMBOL(touch_notify_connect);

void touch_notify_wireless(u32 type)
{
	wireless = type;
	touch_atomic_notifier_call(NOTIFY_WIRELESS, &type);
}
EXPORT_SYMBOL(touch_notify_wireless);

void touch_notify_earjack(u32 type)
{
	earjack = type;
	touch_atomic_notifier_call(NOTIFY_EARJACK, &type);
}
EXPORT_SYMBOL(touch_notify_earjack);
void touch_notify_fm_radio(u32 type)
{
	fm_radio = type;
	touch_atomic_notifier_call(NOTIFY_FM_RADIO, &type);
}
EXPORT_SYMBOL(touch_notify_fm_radio);

void touch_restore_state(struct touch_core_data *ts) {

	TOUCH_TRACE();

	if (atomic_read(&ts->state.connect) != connection) {
		TOUCH_I("%s: need to restore connection (%d)\n", __func__, connection);
		touch_notify_connect(connection);
	}

	if (atomic_read(&ts->state.wireless) != wireless) {
		TOUCH_I("%s: need to restore wireless (%d)\n", __func__, wireless);
		touch_notify_wireless(wireless);
	}

	if (atomic_read(&ts->state.earjack) != earjack) {
		TOUCH_I("%s: need to restore earjack (%d)\n", __func__, earjack);
		touch_notify_earjack(earjack);
	}

	if (atomic_read(&ts->state.fm_radio) != fm_radio) {
		TOUCH_I("%s: need to restore fm_radio (%d)\n", __func__, fm_radio);
		touch_notify_fm_radio(fm_radio);
	}
}

static int touch_notify(struct touch_core_data *ts,
				   unsigned long event, void *data)
{
	int boot_mode = TOUCH_NORMAL_BOOT;
	int ret = 0;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(ts->dev);
	if (boot_mode == TOUCH_CHARGER_MODE
			|| boot_mode == TOUCH_LAF_MODE
			|| boot_mode == TOUCH_RECOVERY_MODE) {
		TOUCH_I("%s: boot_mode = %d\n", __func__, boot_mode);
		return 0;
	}

	if (ts->driver->notify) {
		mutex_lock(&ts->lock);
		switch (event) {
		case NOTIFY_TOUCH_RESET:
			ret = ts->driver->notify(ts->dev, event, data);
			break;

		case NOTIFY_CONNECTION:
			if (atomic_read(&ts->state.connect) != *(int *)data) {
				atomic_set(&ts->state.connect, *(int *)data);
				ret = ts->driver->notify(ts->dev, event, data);
			}
			break;

		case NOTIFY_WIRELESS:
			atomic_set(&ts->state.wireless, *(int *)data);
			ret = ts->driver->notify(ts->dev, event, data);
			break;

		case NOTIFY_FB:
			atomic_set(&ts->state.fb, *(int *)data);
			queue_delayed_work(ts->wq, &ts->fb_work, 0);
			break;

		case NOTIFY_EARJACK:
			atomic_set(&ts->state.earjack, *(int *)data);
			ret = ts->driver->notify(ts->dev, event, data);
			break;
		case NOTIFY_FM_RADIO:
			atomic_set(&ts->state.fm_radio, *(int *)data);
			ret = ts->driver->notify(ts->dev, event, data);
			break;

		default:
			ret = ts->driver->notify(ts->dev, event, data);
			break;
		}
		mutex_unlock(&ts->lock);
	}

	return ret;
}

#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
static int display_notify(struct touch_core_data *ts,
				   unsigned long event, void *data)
{
	int boot_mode = TOUCH_NORMAL_BOOT;
	int ret = 0;
	u32 display_notify_data = 0;
	struct lge_panel_notifier *panel_data = data;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(ts->dev);
	if (boot_mode == TOUCH_CHARGER_MODE
			|| boot_mode == TOUCH_LAF_MODE
			|| boot_mode == TOUCH_RECOVERY_MODE) {
		TOUCH_I("%s: boot_mode = %d\n", __func__, boot_mode);
		return 0;
	}

	if (atomic_read(&ts->state.core) == CORE_SHUTDOWN) {
		TOUCH_I("%s: skip in shutdown state\n", __func__);
		return 0;
	}

	if (ts->driver->notify) {
		mutex_lock(&ts->lock);
		switch (event) {
		case LGE_PANEL_EVENT_BLANK:
			TOUCH_D(TRACE, "EVENT_BLANK : %d %d", panel_data->display_id,
					panel_data->state);
			if (panel_data->state == LGE_PANEL_STATE_UNBLANK)
				display_notify_data = 3;
			else if (panel_data->state == LGE_PANEL_STATE_BLANK)
				display_notify_data = 0;
			else
				display_notify_data = panel_data->state;
			ret = ts->driver->notify(ts->dev, LCD_EVENT_LCD_MODE,
					&display_notify_data);
			break;
		case LGE_PANEL_EVENT_RESET:
			TOUCH_D(TRACE, "EVENT_RESET : %d %d", panel_data->display_id,
					panel_data->state);
			if (panel_data->state == LGE_PANEL_RESET_HIGH
					|| panel_data->state == LGE_PANEL_RESET_LOW)
				ret = ts->driver->notify(ts->dev, NOTIFY_TOUCH_RESET, data);
			break;
		case LGE_PANEL_EVENT_POWER:
			TOUCH_D(TRACE, "EVENT_POWER : %d %d", panel_data->display_id,
					panel_data->state);
			break;
		case LGE_PANEL_EVENT_RECOVERY:
			TOUCH_D(TRACE, "EVENT_RECOVERY : %d %d", panel_data->display_id,
					panel_data->state);
			if (panel_data->state == LGE_PANEL_RECOVERY_DEAD) {
				TOUCH_I("%s: Panel Recovery !!\n", __func__);
				touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			}
			break;
		default:
			TOUCH_D(TRACE, "unknwon type : %ld %d %d\n", event,
					panel_data->display_id,
					panel_data->state);
			break;
		}
		mutex_unlock(&ts->lock);
	}

	return ret;
}
#endif

static int touch_blocking_notifier_callback(struct notifier_block *this,
				   unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(this, struct touch_core_data, blocking_notif);

	TOUCH_TRACE();

	return touch_notify(ts, event, data);
}

static void touch_atomic_notifer_work_func(struct work_struct *notify_work)
{
	struct touch_core_data *ts =
		container_of(to_delayed_work(notify_work),
				struct touch_core_data, notify_work);
	struct atomic_notify_event *arr = ts->notify_event_arr;
	int i = 0;

	TOUCH_TRACE();

	for (i = 0; i < ATOMIC_NOTIFY_EVENT_SIZE; i++) {
		if (arr[i].event != NOTIFY_NO_EVENT) {
			touch_notify(ts, arr[i].event, &(arr[i].data));
			arr[i].event = NOTIFY_NO_EVENT;
		}
	}
}

static int touch_atomic_notifier_callback(struct notifier_block *this,
				   unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(this, struct touch_core_data, atomic_notif);
	struct atomic_notify_event *arr = ts->notify_event_arr;

	TOUCH_TRACE();

	switch (event) {
	case NOTIFY_CONNECTION:
		arr[ATOMIC_NOTIFY_CONNECTION].event = event;
		arr[ATOMIC_NOTIFY_CONNECTION].data = *(int *)data;
		break;
	case NOTIFY_WIRELESS:
		arr[ATOMIC_NOTIFY_WIRELESS].event = event;
		arr[ATOMIC_NOTIFY_WIRELESS].data = *(int *)data;
		break;
	case NOTIFY_EARJACK:
		arr[ATOMIC_NOTIFY_EARJACK].event = event;
		arr[ATOMIC_NOTIFY_EARJACK].data = *(int *)data;
		break;
	case NOTIFY_FM_RADIO:
		arr[ATOMIC_NOTIFY_FM_RADIO].event = event;
		arr[ATOMIC_NOTIFY_FM_RADIO].data = *(int *)data;
		break;
	default:
		TOUCH_I("%s: unknown event(%lu)\n", __func__, event);
		return 0;
	}

	queue_delayed_work(ts->wq, &ts->notify_work, 0);

	return 0;
}

#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
static int display_notifier_callback(struct notifier_block *this,
				   unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(this, struct touch_core_data, display_notif);

	TOUCH_TRACE();

	return display_notify(ts, event, data);
}
#endif

static int touch_init_notify(struct touch_core_data *ts)
{
	int ret = 0;

	TOUCH_TRACE();

	ts->blocking_notif.notifier_call = touch_blocking_notifier_callback;
	ret = touch_blocking_notifier_register(&ts->blocking_notif);

	if (ret < 0)
		TOUCH_E("failed to regiseter touch blocking_notify callback\n");

	ts->atomic_notif.notifier_call = touch_atomic_notifier_callback;
	ret = touch_atomic_notifier_register(&ts->atomic_notif);

	if (ret < 0)
		TOUCH_E("failed to regiseter touch atomic_notify callback\n");

#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
	ts->display_notif.notifier_call = display_notifier_callback;
	ret = lge_panel_notifier_register_client(&ts->display_notif);
#endif

	if (ret < 0)
		TOUCH_E("failed to regiseter lge_panel_notify callback\n");

	return ret;
}

static int touch_init_works(struct touch_core_data *ts)
{
	ts->wq = create_singlethread_workqueue("touch_wq");

	TOUCH_TRACE();

	if (!ts->wq) {
		TOUCH_E("failed to create workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&ts->init_work, touch_init_work_func);
	INIT_DELAYED_WORK(&ts->upgrade_work, touch_upgrade_work_func);
#if defined(CONFIG_LGE_TOUCH_APP_FW_UPGRADE)
	INIT_DELAYED_WORK(&ts->app_upgrade_work, touch_app_upgrade_work_func);
#endif
	INIT_DELAYED_WORK(&ts->notify_work, touch_atomic_notifer_work_func);
	INIT_DELAYED_WORK(&ts->fb_work, touch_fb_work_func);
	INIT_DELAYED_WORK(&ts->panel_reset_work, touch_panel_reset_work_func);
	init_completion(&ts->uevent_complete);

	return 0;
}

#if defined(CONFIG_SECURE_TOUCH)
void secure_touch_notify(struct touch_core_data *ts)
{
	TOUCH_I("secure_touch_notify\n");
	sysfs_notify(&ts->kobj, NULL, "secure_touch");
}

void secure_touch_init(struct touch_core_data *ts)
{
	ts->st_initialized = 0;
	init_completion(&ts->st_powerdown);
	init_completion(&ts->st_irq_processed);

	/* Get clocks */
	ts->core_clk = devm_clk_get(ts->pdev->dev.parent, "core_clk");
	if (IS_ERR(ts->core_clk)) {
		TOUCH_E("failed to get core_clk : %ld\n", PTR_ERR(ts->core_clk));
		ts->core_clk = NULL;
	}
	ts->iface_clk = devm_clk_get(ts->pdev->dev.parent, "iface_clk");
	if (IS_ERR(ts->iface_clk)) {
		TOUCH_E("failed to get iface_clk : %ld\n", PTR_ERR(ts->iface_clk));
		ts->iface_clk = NULL;
	}

	ts->st_initialized = 1;
}

static irqreturn_t secure_touch_filter_interrupt(struct touch_core_data *ts)
{
	if (atomic_read(&ts->st_enabled)) {
		if (atomic_cmpxchg(&ts->st_pending_irqs, 0, 1) == 0) {
			reinit_completion(&ts->st_irq_processed);
			secure_touch_notify(ts);
			wait_for_completion_interruptible(
					&ts->st_irq_processed);
		}
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/*
 *   'blocking' variable will have value 'true' when we want to prevent the driver
 *   from accessing the xPU/SMMU protected HW resources while the session is
 *   active.
 */
void secure_touch_stop(struct touch_core_data *ts, bool blocking)
{
	TOUCH_I("secure_touch_stop : %d", blocking);
	if (atomic_read(&ts->st_enabled)) {
		atomic_set(&ts->st_pending_irqs, -1);
		secure_touch_notify(ts);
		if (blocking && (atomic_read(&ts->state.core) != CORE_SHUTDOWN)) {
			wait_for_completion_interruptible(
					&ts->st_powerdown);
		}
	}
}
#endif

static int module_notify_func(int control, int data) {
	switch (control) {
		case MODULE_FIRMWARE_UPGRADE:
			TOUCH_I("%s: %d", __func__, data);
#ifdef CONFIG_LGE_DD_TEMP_BRINGUP
			notify_touch_version_check(data);
#endif
			break;
		case 2:
			break;
	};

	return 0;
}

struct pdev_list *plist;

static int touch_core_probe_normal(struct platform_device *pdev)
{
	struct touch_core_data *ts;
	int ret = 0;

	TOUCH_TRACE();

	////////////////////////////////////////////////
	TOUCH_I("pdev info name:%s id:%d", pdev->name,
			pdev->id);

	/* sub module */
	if (plist == NULL) {
		plist = kzalloc(sizeof(*plist), GFP_KERNEL);
		if (!plist) {
			TOUCH_E("Failed to allocate memory for plist\n");
			return -ENOMEM;
		}
	}
	/* check specific platform device num */
	if (pdev->id > 0) {
		struct module_data *md;
//		int data = 1;
//		char buf[100] = {0};

		/* check module IC */
		TOUCH_I("call from module to do someting\n");
		md = (struct module_data *) pdev->dev.platform_data;

		plist->sub_dev[pdev->id - 1] = pdev->dev.parent;

		ret = md->m_driver.match(md->dev);
		if (ret < 0) {
			TOUCH_I("Not found module IC. Skip module probe. %d\n", ret);
			return 0;
		}

		md->m_driver.notify = module_notify_func;
		md->m_driver.write_file = module_write_file;
		md->m_driver.log_file_size_check = module_log_file_size_check;
		md->m_driver.probe(md->dev);

		//md->m_driver.func(md->dev, data, buf);
		//TOUCH_I("%s\n", buf);

		if (md->wq != NULL) {
			queue_delayed_work(md->wq, &md->init_work, 0);
		}

		if (plist->touch_core_data != NULL) {
			touch_init_sysfs_module(md, plist->touch_core_data);
#ifdef CONFIG_LGE_TOUCH_PEN
			ts = (struct touch_core_data *) plist->touch_core_data;
			if (ts->driver->pen_func != NULL) {
				ts->driver->pen_func = md->m_driver.func;
				TOUCH_I("%s Pen Driver Handler Connected\n", __func__);
			}
#endif
		} else {
			TOUCH_I("%s Main PDev was not called", __func__);
		}

		return 0;
	}
	////////////////////////////////////////////////

	ts = (struct touch_core_data *) pdev->dev.platform_data;
	plist->touch_core_data = (void *)ts;

	atomic_set(&ts->state.core, CORE_PROBE);
	ret = touch_init_platform_data(ts);
	if (ret < 0) {
		TOUCH_E("failed to initialize platform_data\n");
		return -EINVAL;
	}

	ret = ts->driver->probe(ts->dev);
	if (ret < 0) {
		TOUCH_E("failed to device probe\n");
		return ret;
	}

	/* set defalut lpwg value because of AAT */
	ts->lpwg.screen = 1;
	ts->lpwg.sensor = PROX_FAR;

	ts->driver->power(ts->dev, POWER_ON);
	ts->driver->power(ts->dev, POWER_OFF);
	ts->driver->power(ts->dev, POWER_ON);

	touch_init_locks(ts);
	ret = touch_init_works(ts);
	if (ret < 0) {
		TOUCH_E("failed to initialize works\n");
		goto error_init_work;
	}

	ret = touch_init_input(ts);
	if (ret < 0) {
		TOUCH_E("failed to register input device(ret:%d)\n", ret);
		goto error_init_input;
	}
#if defined(CONFIG_LGE_TOUCH_DEX)
	ret = dex_input_init(ts->dev);
	if (ret < 0) {
		goto error_init_input;
	}
#endif
	ret = touch_request_irq(ts->irq, touch_irq_handler,
			touch_irq_thread, ts->irqflags | IRQF_ONESHOT,
			LGE_TOUCH_NAME, ts);
	if (ret < 0) {
		TOUCH_E("failed to request_thread_irq(irq:%d, ret:%d)\n",
				ts->irq, ret);
		goto error_request_irq;
	}

	touch_disable_irq(ts->irq);
	touch_init_pm(ts);

	touch_init_notify(ts);

	ret = touch_init_uevent(ts);
	ret = touch_init_sysfs(ts);

	TOUCH_I("hw_reset_delay : %d ms\n", ts->caps.hw_reset_delay);
	queue_delayed_work(ts->wq, &ts->init_work,
			msecs_to_jiffies(ts->caps.hw_reset_delay));

	return 0;
error_request_irq:
	touch_free_irq(ts->irq, ts->dev);
error_init_input:
	if (ts->input) {
		input_mt_destroy_slots(ts->input);
		input_free_device(ts->input);
	}
error_init_work:
	if (ts->wq)
		destroy_workqueue(ts->wq);

	return ret;
}

static int touch_core_probe_etc(struct platform_device *pdev)
{
	struct touch_core_data *ts;
	int ret = 0;

	TOUCH_TRACE();

	ts = (struct touch_core_data *) pdev->dev.platform_data;

	ret = touch_init_platform_data(ts);
	if (ret < 0) {
		TOUCH_E("failed to initialize platform_data\n");
		return -EINVAL;
	}

	ret = ts->driver->probe(ts->dev);
	if (ret < 0) {
		TOUCH_E("failed to device probe\n");
		return ret;
	}

	touch_init_locks(ts);
	ret = touch_init_works(ts);
	if (ret < 0) {
		TOUCH_E("failed to initialize works\n");
		return ret;
	}

	touch_init_pm(ts);
	touch_init_notify(ts);

	return ret;
}

static int touch_core_probe(struct platform_device *pdev)
{
	struct touch_core_data *ts;
	int boot_mode = TOUCH_NORMAL_BOOT;
	int ret = 0;

	TOUCH_TRACE();

	mutex_lock(&touch_probe_lock);

	if (pdev->id > 0) {
		TOUCH_I("pdev->id = %d, no need to check bootmode\n", pdev->id);
		ret = touch_core_probe_normal(pdev);
		goto out;
	}

	ts = (struct touch_core_data *) pdev->dev.platform_data;

	boot_mode = touch_check_boot_mode(ts->dev);
	if (boot_mode == TOUCH_CHARGER_MODE
			|| boot_mode == TOUCH_LAF_MODE
			|| boot_mode == TOUCH_RECOVERY_MODE) {
		TOUCH_I("%s: boot_mode = %d\n", __func__, boot_mode);
		ret = touch_core_probe_etc(pdev);
		goto out;
	}

	ret = touch_core_probe_normal(pdev);

out:
	mutex_unlock(&touch_probe_lock);
	return ret;
}

static int touch_core_remove(struct platform_device *pdev)
{
	struct touch_core_data *ts;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	///////////////////////////////////////////////////////////////////
	TOUCH_I("pdev->id:%d num_resources:%d" , pdev->id,
			pdev->num_resources);

	if (plist != NULL && pdev->id > 0) {
		struct module_data *md = to_module(plist->sub_dev[pdev->id - 1]);
		if (md != NULL)
			md->m_driver.remove(md->dev);

		plist->sub_dev[pdev->id - 1] = NULL;

		return 0;
	}
	////////////////////////////////////////////////////////////////////

	ts = (struct touch_core_data *) pdev->dev.platform_data;

	atomic_set(&ts->state.core, CORE_REMOVE);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_DRM_MSM) && defined(CONFIG_FB)
	msm_drm_unregister_client(&ts->drm_notif);
#elif defined(CONFIG_FB)
	fb_unregister_client(&ts->fb_notif);
#endif

	kobject_del(&ts->kobj);

	touch_blocking_notifier_unregister(&ts->blocking_notif);

	touch_atomic_notifier_unregister(&ts->atomic_notif);

	destroy_workqueue(ts->wq);
	device_init_wakeup(ts->dev, false);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	touch_free_irq(ts->irq, ts->dev);

	input_unregister_device(ts->input);

	ts->driver->remove(ts->dev);
	ts->driver->power(ts->dev, POWER_OFF);

	devm_kfree(ts->dev, ts);

	kfree(plist);

	return 0;
}

static void touch_core_shutdown(struct platform_device *pdev)
{
	struct touch_core_data *ts;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	TOUCH_I("pdev->id:%d num_resources:%d" , pdev->id,
			pdev->num_resources);
	if (pdev->id > 0) {
		TOUCH_I("%s : do not release platform data\n", __func__);
		return ;
	}

	ts = (struct touch_core_data *) pdev->dev.platform_data;

	atomic_set(&ts->state.core, CORE_SHUTDOWN);

#if defined(CONFIG_SECURE_TOUCH)
	if (atomic_read(&ts->st_enabled)) {
		secure_touch_stop(ts, false);
		complete(&ts->st_irq_processed);
	}
#endif

	if (ts->dev == NULL)
		return;

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	touch_free_irq(ts->irq, ts->dev);

	if (ts->driver->shutdown)
		ts->driver->shutdown(ts->dev);
}

static struct platform_driver touch_core_driver = {
	.driver = {
		.name = LGE_TOUCH_DRIVER_NAME,
		.owner = THIS_MODULE,
/*
		.of_match_table = touch_match_ids,
*/
	},
	.probe = touch_core_probe,
	.remove = touch_core_remove,
	.shutdown = touch_core_shutdown,
};

static void touch_core_async_init(void *data, async_cookie_t cookie)
{

	int ret = 0;

	TOUCH_TRACE();

	mutex_init(&touch_probe_lock);

	ret = platform_driver_register(&touch_core_driver);

	if (ret < 0)
		TOUCH_E("async_init failed\n");
}

static int __init touch_core_init(void)
{
	TOUCH_TRACE();
	async_schedule(touch_core_async_init, NULL);
	return 0;
}

static void __exit touch_core_exit(void)
{
	TOUCH_TRACE();
	platform_driver_unregister(&touch_core_driver);
}

late_initcall(touch_core_init);
module_exit(touch_core_exit);

MODULE_AUTHOR("hoyeon.jang@lge.com");
MODULE_DESCRIPTION("LGE touch driver v5");
MODULE_LICENSE("GPL");
