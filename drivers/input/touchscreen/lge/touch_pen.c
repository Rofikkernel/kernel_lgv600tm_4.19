/* Test driver for Active Pen */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <touch_pen.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <touch_core.h>

#define LGE_PEN_NAME		"lge_pen"
#define LGE_PEN_DRIVER_NAME "pen_driver"
#define PEN_DEVICE_COMPATIBLE_NAME "active,pen"
//struct g_pen_data g_data;
//struct touch_pen_data *tp;
struct pen_prediction_data *pp;
u32 pen_debug_mask = 0; //0xFFF;

static const char __used *pen_flow[11] = {
	[0] = "<NONE>      ",
	[1] = "<HoverEnter>",
	[2] = "<HoverExit> ",
	[3] = "<Press>     ",
	[4] = "<Release>   ",
	[5] = "<Button1_P> ",
	[6] = "<Button1_R> ",
	[7] = "<Button2_P> ",
	[8] = "<Button2_R> ",
	[9] = "<Debug(1)>  ",
	[10] = "<Debug(2)> ",
};

void prediction_filter(struct pen_data *pdata, struct touch_core_data *ts) {

	u32 DELTA_DEGREE_THRESHOLD = 0;
	u16 VELOCITY_THRESHOLD = 0;

	u16 K = 0;
	int x = pdata->x;
	int y = pdata->y;
	int dx = x - pp->prev_x;
	int dy = y - pp->prev_y;
	int distance = 0;
	int x_est = 0;
	int y_est = 0;
	int z_measured_x = 0;
	int z_measured_y =0;
	int currUnitX = 0;
	int currUnitY = 0;
	int deltaDegree = 0;
	int currVelocityX = dx*1000 / VSYNC_TIMING;
	int currVelocityY = dy*1000 / VSYNC_TIMING;

	if((pdata->in_range == 0) || (pdata->contact == 0)) {
		pp->prev_predict_x = 0;
		pp->prev_predict_y = 0;
		pp->predict_x = 0;
		pp->predict_y = 0;
		pp->prev_est_x = 0;
		pp->prev_est_y = 0;
		pp->prev_unit_x = 0;
		pp->prev_unit_y = 0;
		pp->prev_x = 0;
		pp->prev_y = 0;
		pp->move_cnt = 0;
		pp->predict_ratio = 0;
		return;
	}

	distance  = int_sqrt((dx*dx+dy*dy)*100); // *10
	if (distance != 0) {
		currUnitX = (dx*10000)/distance; // *1000
		currUnitY = (dy*10000)/distance; // *1000
	} else {
		currUnitX = 0;
		currUnitY = 0;
	}
	distance = distance*INCHTOMILLI/DPI;  // convert inch to mm / dpi => mm/s

	deltaDegree = (int)(currUnitX*pp->prev_unit_x + currUnitY*pp->prev_unit_y);

	//measure
	z_measured_x = currVelocityX;
	z_measured_y = currVelocityY;

	//correct
	K = 10;   // *100
	x_est = (pp->prev_est_x*100 + K * (z_measured_x - pp->prev_est_x))/100;
	y_est = (pp->prev_est_y*100 + K * (z_measured_y - pp->prev_est_y))/100;

	//prediction
	if ( MOVE_CHK_CNT < pp->move_cnt ) {
		DELTA_DEGREE_THRESHOLD = DELTA_DEGREE_LOW_THRESHOLD;
		VELOCITY_THRESHOLD = VELOCITY_LOW_THRESHOLD;
	} else {
		DELTA_DEGREE_THRESHOLD = DELTA_DEGREE_HIGH_THRESHOLD;
		VELOCITY_THRESHOLD = VELOCITY_HIGH_THRESHOLD;
	}

	if ((deltaDegree > DELTA_DEGREE_THRESHOLD) && VELOCITY_THRESHOLD < distance) {
		pp->move_cnt = pp->move_cnt + 1;
		if ( MOVE_CHK_CNT < pp->move_cnt ) {
			if (pp->predict_ratio <= 100) {
				pp->predict_ratio += 20;
			}
			pp->predict_x = (int)(VSYNC_TIMING * x_est / 1000 * pp->predict_ratio / 100);
			pp->predict_y = (int)(VSYNC_TIMING * y_est / 1000 * pp->predict_ratio / 100);
			pp->predict_x = (pp->prev_predict_x*100 + K * (pp->predict_x - pp->prev_predict_x))/100;
			pp->predict_y = (pp->prev_predict_y*100 + K * (pp->predict_y - pp->prev_predict_y))/100;
		} else if ((pp->predict_x != 0 || pp->predict_y != 0 ) && 2 <= pp->predict_ratio) {
			pp->predict_ratio -= 1;
			pp->predict_x = pp->predict_x * pp->predict_ratio / 100;
			pp->predict_y = pp->predict_y * pp->predict_ratio / 100;
			pp->predict_x = (pp->prev_predict_x*100 + K * (pp->predict_x - pp->prev_predict_x))/100;
			pp->predict_y = (pp->prev_predict_y*100 + K * (pp->predict_y - pp->prev_predict_y))/100;
			TOUCH_D(ABS, "Pen Predict blue1 [%4d,%4d] [%4d,%4d] [%4d,%4d] [%4d,%4d]\n", pp->prev_x, pp->prev_y,  x, y, pp->prev_predict_x,  pp->prev_predict_y, pp->predict_x,  pp->predict_y);
		} else {
			pp->predict_x = 0;
			pp->predict_y = 0;
			pp->predict_ratio = 0;
		}
	}
	else {
		if ((pp->predict_x != 0 || pp->predict_y != 0) && MOVE_CHK_CNT < pp->move_cnt) {
			pp->predict_x = (pp->prev_x + pp->predict_x) - x;
			pp->predict_y = (pp->prev_y + pp->predict_y) - y;
			pp->predict_ratio = 100;
			TOUCH_D(ABS, "Pen Predict black [%4d,%4d] [%4d,%4d] [%4d,%4d] [%4d,%4d]\n", pp->prev_x, pp->prev_y,  x, y, pp->prev_predict_x,  pp->prev_predict_y, pp->predict_x,  pp->predict_y);
		} else if ((pp->predict_x != 0 || pp->predict_y != 0 ) && 2 <= pp->predict_ratio) {
			TOUCH_D(ABS, "Pen Predict blue2 [%4d,%4d] [%4d,%4d] [%4d,%4d] [%4d,%4d]\n", pp->prev_x, pp->prev_y,  x, y, pp->prev_predict_x,  pp->prev_predict_y, pp->predict_x,  pp->predict_y);
			pp->predict_ratio -= 1;
			pp->predict_x = pp->predict_x * pp->predict_ratio / 100;
			pp->predict_y = pp->predict_y * pp->predict_ratio / 100;
			pp->predict_x = (pp->prev_predict_x*100 + K * (pp->predict_x - pp->prev_predict_x))/100;
			pp->predict_y = (pp->prev_predict_y*100 + K * (pp->predict_y - pp->prev_predict_y))/100;
		} else {
			pp->predict_x = 0;
			pp->predict_y = 0;
			pp->predict_ratio = 0;
		}
		x_est = 0;
		y_est = 0;
		pp->move_cnt = 0;
	}
//	if (pen_debug_mask)
		TOUCH_D(ABS, "Pen Predict [%4d,%4d] [%4d,%4d], est [%4d,%4d], [%4d,%8d], move_cnt [%d]\n",
				pdata->x,
				pdata->y,
				pp->predict_x,
				pp->predict_y,
				x_est,
				y_est,
				distance,
				deltaDegree,
				pp->move_cnt);
	x += pp->predict_x;
	y += pp->predict_y;

	if (x < 0)
		pdata->x = 0;
	else if (x > ts->caps.max_x)
		pdata->x = ts->caps.max_x;
	else
		pdata->x = x;

	if (y < 0)
		pdata->y = 0;
	else if (y > ts->caps.max_y)
		pdata->y = ts->caps.max_y;
	else
		pdata->y = y;

	//update
	pp->prev_est_x = x_est;
	pp->prev_est_y = y_est;
	pp->prev_x = x - pp->predict_x;
	pp->prev_y = y - pp->predict_y;
	pp->prev_unit_x = currUnitX;
	pp->prev_unit_y = currUnitY;
	pp->prev_predict_x = pp->predict_x;
	pp->prev_predict_y = pp->predict_y;
}

void pen_log(int pen_flow_num, u64 id, u16 in_range, u16 contact, u16 swit1, u16 swit2, u16 batt,
	u16 x, u16 y, u16 pressure, u16 tilt, u16 azimuth)
{
	int val = pen_debug_mask;

	if(!val)
		return;

	printk("[Touch] %s", pen_flow[pen_flow_num]);

	if(val & 0x1)
		printk("id:%llu", id);
	if((val>>1) & 0x1)
		printk("in_r:%2d cont:%d", in_range, contact);
	if((val>>2) & 0x1)
		printk("sw1:%2d sw2:%d", swit1, swit2);
	if((val>>3) & 0x1)
		printk("bat:%2d", batt);
	if((val>>4) & 0x1)
		printk("x:%4d y:%4d", x, y);
	if((val>>5) & 0x1)
		printk("z:%4d", pressure);
	if((val>>6) & 0x1)
		printk("tt:%2d", tilt);
	if((val>>7) & 0x1)
		printk("am:%2d", azimuth);
	printk("\n");
}

int pen_callback(struct device *dev, void *touch_core_data, void *data, int control)
{
	struct module_data *md = to_module(dev);
	struct input_dev *input = md->input;
	struct touch_core_data *ts = (struct touch_core_data *)touch_core_data;
	struct pen_data *pdata = (struct pen_data *)data;
	int pen_flow_num = PEN_NONE;
	int btn_status = PEN_NONE;
	int active_pen_state = 0;

	TOUCH_TRACE();

	active_pen_state = atomic_read(&ts->state.active_pen);
	if(control)
		prediction_filter(pdata, ts);


	/* btn status */
	if ((pdata->in_range == 1) && pdata->swit1) {
		input_report_key(input, BTN_STYLUS, 1);
		btn_status = SWITCH_1_P;
	} else {
		input_report_key(input, BTN_STYLUS, 0);
		if (pdata->old_btn_status == SWITCH_1_P) {
			btn_status = SWITCH_1_R;
		}
	}
	if ((pdata->in_range == 1) && pdata->swit2) {
		input_report_key(input, BTN_STYLUS2, 1);
		btn_status = SWITCH_2_P;
	} else {
		input_report_key(input, BTN_STYLUS2, 0);
		if (pdata->old_btn_status == SWITCH_2_P) {
			btn_status = SWITCH_2_R;
		}
	}

	/* sync btn status*/
	input_sync(input);

	if (pdata->old_btn_status != btn_status && active_pen_state) {
		if (btn_status != PEN_NONE)
			TOUCH_I("%s\n", pen_flow[btn_status]);
	}
	pdata->old_btn_status = btn_status;

	if ((pdata->in_range == 0) && (pdata->contact == 0)) {	//<release>
		pen_flow_num = HOVER_EXIT;
		input_report_abs(input, ABS_PRESSURE, pdata->pressure);
		input_report_abs(input, ABS_TILT_X, 0);
		input_report_abs(input, ABS_TILT_Y, 0);
		input_report_key(input, BTN_TOUCH, 0);
		if (pdata->old_pen_flow_num == PRESS) {
			input_sync(input);
		}
		input_report_abs(input, ABS_DISTANCE, 0);
		input_report_abs(input, ABS_X, pdata->x);
		input_report_abs(input, ABS_Y, pdata->y);
		input_report_key(input, BTN_TOOL_PEN, 0);
		pdata->state = 0;
		if (ts->lpwg.screen) {
			if (ts->aes_mode == 0 && atomic_read(&ts->state.dualscreen)
					&& pdata->uevent_handled) {
//				touch_send_uevent(ts, TOUCH_UEVENT_SWITCH_AES_BOTH);
				atomic_set(&ts->state.uevent, UEVENT_IDLE);
				pdata->uevent_handled = false;
			}
		}
	} else	{
		/* <Detect> */
		if((pdata->in_range == 1) && (pdata->contact == 1)) { //<Tip>
			pen_flow_num = PRESS;
			input_report_key(input, BTN_TOUCH, 1);
			input_report_abs(input, ABS_TILT_X, pdata->tilt);
			input_report_abs(input, ABS_TILT_Y, pdata->azimuth);
			if (ts->lpwg.screen) {
				if (!atomic_read(&ts->state.active_pen) && pdata->state == 0
							&& atomic_read(&ts->state.first_resume) == FIRST_RESUME_IS_OK) {
					touch_send_uevent(ts, TOUCH_UEVENT_PEN_DETECTION);
					pdata->state = 1;
				}
			}
		} else if((pdata->in_range == 1) && (pdata->contact == 0)) {
			if (pdata->old_pen_flow_num == PRESS) {
				input_report_key(input, BTN_TOUCH, 0);
				pen_flow_num = RELEASE;
			} else {
				pen_flow_num = HOVER_ENTER;
			}
		}

		if((pdata->in_range == 0) && (pdata->contact == 1)) {
			pen_flow_num = DEBUG_1;
		} else {
			input_report_key(input, BTN_TOOL_PEN, 1);	//<Tip> & <Hover>
			input_report_abs(input, ABS_X, pdata->x);
			input_report_abs(input, ABS_Y, pdata->y);
			input_report_abs(input, ABS_TILT_X, pdata->tilt);
			input_report_abs(input, ABS_TILT_Y, pdata->azimuth);
			if ((pdata->pressure == 0) && (pdata->contact == 1)) {
				TOUCH_I("abnormal pressure %d", pdata->pressure);
				if (pdata->prev_pressure == 0) {
					input_report_abs(input, ABS_PRESSURE, 1);
				} else {
					input_report_abs(input, ABS_PRESSURE, pdata->prev_pressure);
				}
			} else {
				input_report_abs(input, ABS_PRESSURE, pdata->pressure);
			}
			input_report_abs(input, ABS_DISTANCE, 1);
		}
	}
	if (pdata->old_pen_flow_num != pen_flow_num && active_pen_state) {
		TOUCH_I("%s X: %4d Y: %4d Batt: %2d P: %4d T: %3d A: %3d\n",
				pen_flow[pen_flow_num],
				pdata->x,
				pdata->y,
				pdata->batt,
				pdata->pressure,
				pdata->tilt,
				pdata->azimuth);
	}

	pdata->old_pen_flow_num = pen_flow_num;

	if (pen_debug_mask) {
		TOUCH_I("%s id:%llu id_r:%d cont:%d sw1:%d sw2:%d bat:%d x:%d y:%d z:%d prevz:%d tt:%d am:%d\n",
				pen_flow[pen_flow_num],
				(unsigned long long)pdata->id,
				pdata->in_range,
				pdata->contact,
				pdata->swit1,
				pdata->swit2,
				pdata->batt,
				pdata->x,
				pdata->y,
				pdata->pressure,
				pdata->prev_pressure,
				pdata->tilt,
				pdata->azimuth);
	}
	pdata->prev_pressure = pdata->pressure;
	input_sync(input);

	return 0;
}

static int pen_init_input(struct module_data *md)
{
	struct input_dev *input;
	int ret;

	TOUCH_I("%s ", __func__);
	input = input_allocate_device();

	if (!input) {
		TOUCH_E("failed to allocate memory for pen input\n");
		return -ENOMEM;
	}

	input->name = "pen_dev";
	input->phys = "devices/virtual/input";
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	set_bit(BTN_STYLUS, input->keybit);
	set_bit(BTN_STYLUS2, input->keybit);
	set_bit(BTN_TOOL_PEN, input->keybit);
	set_bit(INPUT_PROP_POINTER, input->propbit);
	input_set_abs_params(input, ABS_X, 0, 1080, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 2460, 0, 0);
	input_set_abs_params(input, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 4096, 0, 0);
	input_set_abs_params(input, ABS_TILT_X, 0, 180, 0, 0);
	input_set_abs_params(input, ABS_TILT_Y, 0, 180, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION, 0,
			90, 0, 0);
	ret = input_register_device(input);

	if (ret < 0) {
		TOUCH_E("failed to initialize input device(ret:%d)\n", ret);
		goto error;
	}

	input_set_drvdata(input, md);
	md->input = input;

	return 0;

error:
	input_free_device(input);

	return ret;
}

static ssize_t show_pen_log_ctl(struct device *dev, char *buf)
{
	int ret = 0;

	TOUCH_TRACE();

	ret += snprintf(buf + ret, PAGE_SIZE, "%d\n", pen_debug_mask);
	TOUCH_I("%s: pen_debug_mask = %d\n", __func__, pen_debug_mask);

	return ret;
}


static ssize_t store_pen_log_ctl(struct device *dev,
		const char *buf, size_t count)
{
	int id = 0;
	int contact_in_range = 0;
	int swit1_2 = 0;
	int batt = 0;
	int x_y = 0;
	int pressure = 0;
	int tilt = 0;
	int azimuth = 0;
	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%x", &value) <= 0)
		return count;

	pen_debug_mask = value;

	id = value & 0x1;
	contact_in_range = (value>>1) & 0x1;

	swit1_2 = (value>>2) & 0x1;

	batt = (value>>3) & 0x1;
	x_y = (value>>4) & 0x1;

	pressure = (value>>5) & 0x1;
	tilt = (value>>6) & 0x1;
	azimuth = (value>>7) & 0x1;

	TOUCH_I("(value:%x)(id:%d)(contact,in_range:%d)(swit1,2:%d)(batt:%d)\
(x,y:%d)(pressure:%d)(tilt:%d)(azimuth:%d)\n",
		value, id, contact_in_range, swit1_2, batt, x_y , pressure, tilt, azimuth);

	return count;
}


static PEN_ATTR(pen_log_ctl, show_pen_log_ctl, store_pen_log_ctl);

static struct attribute *pen_attribute_list[] = {
	&touch_attr_pen_log_ctl.attr,
	//&touch_attr_pen_log_ctl.attr,
	NULL,
};

static const struct attribute_group pen_attribute_group = {
	.attrs = pen_attribute_list,
	NULL,
};



static ssize_t pen_attr_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct module_data *md =
		container_of(kobj, struct module_data, kobj);
	struct touch_pen_attribute *priv =
		container_of(attr, struct touch_pen_attribute, attr);
	ssize_t ret = 0;

	if (priv->show)
		ret = priv->show(md->dev, buf);

	return ret;
}


static ssize_t pen_attr_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct module_data *md =
		container_of(kobj, struct module_data, kobj);
	struct touch_pen_attribute *priv =
		container_of(attr, struct touch_pen_attribute, attr);

	if (priv->store)
		count = priv->store(md->dev, buf, count);

	return count;
}

static const struct sysfs_ops pen_sysfs_ops = {
	.show	= pen_attr_show,
	.store	= pen_attr_store,
};
static struct kobj_type pen_kobj_type = {
	.sysfs_ops = &pen_sysfs_ops,
};
static int pen_register_sysfs(struct device *dev)
{
	struct module_data *md = to_module(dev);
	struct device *inputdev = &md->input->dev;
	int ret = 0;
	TOUCH_I("%s ", __func__);
	ret = kobject_init_and_add(&md->kobj, &pen_kobj_type,
			inputdev->kobj.parent, "%s", LGE_PEN_NAME);
	if (ret < 0) {
		TOUCH_E("failed to initialize kobject\n");
		return ret;
	}

	ret = sysfs_create_group(&md->kobj, &pen_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}
	return ret;
}
/*
static int pen_callback(struct device *dev, void *touch_core_data, void *data, int control)
{
	TOUCH_I("%s ", __func__);
	return 0;
}
*/
static int match_pen_ic(struct device *dev)
{
	TOUCH_I("%s ", __func__);
	return 0;
}
static int pen_probe(struct device *dev)
{
	TOUCH_I("%s ", __func__);
	return 0;
}
static int pen_remove(struct device *dev)
{
	struct module_data *md = to_module(dev);
	TOUCH_I("%s ", __func__);

	devm_kfree(dev,md);
	return 0;
}
static int pen_initialize(struct device *dev)
{
	TOUCH_I("%s ", __func__);
	return 0;
}

static int pen_resume(struct device *dev)
{
	TOUCH_I("%s ", __func__);
	return 0;
}

static int pen_suspend(struct device *dev)
{
	TOUCH_I("%s ", __func__);
	return 0;
}
static int pen_upgrade_force(struct device *dev)
{
	TOUCH_I("%s ", __func__);
	return 0;
}
static int pen_lpwg(struct device *dev, u32 code, void *param)
{
	TOUCH_I("%s ", __func__);
	return 0;
}
static void pen_dev_release(struct device *dev)
{
	TOUCH_I("%s ", __func__);
	if (dev->platform_data)
		dev->platform_data = NULL;
}
static int touch_pen_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct module_data *md;
	struct platform_device *pdev;

	TOUCH_I("%s ", __func__);

	TOUCH_I("i2c slave address : %x\n", i2c->addr);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	md = devm_kzalloc(&i2c->dev, sizeof(*md), GFP_KERNEL);
	pp = kzalloc(sizeof(struct pen_prediction_data), GFP_KERNEL);

	if (!md) {
		TOUCH_E("Failed to allocate memory for touch_core_data\n");
		return -ENOMEM;
	}
	md->dev = &i2c->dev;

	md->m_driver.match = match_pen_ic;
	md->m_driver.probe = pen_probe;
	md->m_driver.remove = pen_remove;
	md->m_driver.func = pen_callback;
	md->m_driver.init = pen_initialize;
	md->m_driver.suspend = pen_suspend;
	md->m_driver.resume = pen_resume;
	md->m_driver.upgrade = pen_upgrade_force;
	md->m_driver.register_sysfs = NULL;
	md->m_driver.lpwg = pen_lpwg;

	md->module_name = LGE_PEN_DRIVER_NAME;
	dev_set_drvdata(&i2c->dev, md);

	ret = pen_init_input(md);
	if (ret < 0) {
		TOUCH_E("init input failed\n");
		return -ENODEV;
	}
	ret = pen_register_sysfs(&i2c->dev);

	if (ret < 0) {
		TOUCH_E("register sysfs failed\n");
		return -ENODEV;
	}

	pdev = devm_kzalloc(&i2c->dev, sizeof(*pdev), GFP_KERNEL);
	if (!pdev) {
		TOUCH_E("Failed to allocate memory for module platform_devce\n");
		return -ENOMEM;
	}

	md->pdev = pdev;

	pdev->name = LGE_TOUCH_DRIVER_NAME;
	pdev->id = 4;
	pdev->num_resources = 0;
	pdev->dev.parent = &i2c->dev;
	pdev->dev.platform_data = md;
	pdev->dev.release = pen_dev_release;

	ret = platform_device_register(pdev);

	if (ret) {
		TOUCH_I("Module Touch Failed to allocate memory for touch platform_devce\n");
		return -ENODEV;
	}

	TOUCH_I("Module Touch platform device registered ...\n");

	return 0;
}
static int touch_pen_remove(struct i2c_client *i2c)
{
	struct module_data *md = to_module(&i2c->dev);
	if (md->input)
		input_unregister_device(md->input);
	kobject_del(&md->kobj);
	kfree(md);
	return 0;
}
static struct of_device_id match_table[] = {
	{ .compatible = PEN_DEVICE_COMPATIBLE_NAME,},
	{ },
};
static struct i2c_device_id tp_id[] = {
	{LGE_PEN_DRIVER_NAME, 0 },
};

MODULE_DEVICE_TABLE(i2c, tp_id);

static struct i2c_driver touch_pen_driver = {
	.probe = touch_pen_probe,
	.remove = touch_pen_remove,
	.id_table = tp_id,
	.driver = {
		.name = LGE_PEN_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
};

static void touch_pen_async_init(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&touch_pen_driver);

}

static int __init touch_pen_init(void)
{
	async_schedule(touch_pen_async_init, NULL);
	return 0;
}

static void __exit touch_pen_exit(void)
{
	i2c_del_driver(&touch_pen_driver);
}

late_initcall(touch_pen_init);
module_exit(touch_pen_exit);

MODULE_AUTHOR("rangkast.jeong@lge.com");
MODULE_DESCRIPTION("LGE pen driver v1");
MODULE_LICENSE("GPL");
