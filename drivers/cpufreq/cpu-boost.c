// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2015,2017,2019, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "cpu-boost: " fmt

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/time.h>
#include <linux/sysfs.h>
#include <linux/printk.h>

#define cpu_boost_attr_rw(_name)		\
static struct kobj_attribute _name##_attr =	\
__ATTR(_name, 0644, show_##_name, store_##_name)

#ifdef CONFIG_SCHED_CAS
#define cpu_boost_cas_attr_rw(_name)		\
static struct kobj_attribute _name##_attr =	\
__ATTR(_name, 0664, show_##_name, store_##_name)
#endif /* CONFIG_SCHED_CAS */


#define show_one(file_name)			\
static ssize_t show_##file_name			\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{								\
	return scnprintf(buf, PAGE_SIZE, "%u\n", file_name);	\
}

#define store_one(file_name)					\
static ssize_t store_##file_name				\
(struct kobject *kobj, struct kobj_attribute *attr,		\
const char *buf, size_t count)					\
{								\
								\
	sscanf(buf, "%u", &file_name);				\
	return count;						\
}

struct cpu_sync {
	int cpu;
	unsigned int input_boost_min;
	unsigned int input_boost_freq;
};

static DEFINE_PER_CPU(struct cpu_sync, sync_info);
static struct workqueue_struct *cpu_boost_wq;

static struct work_struct input_boost_work;

static bool input_boost_enabled;

static unsigned int input_boost_ms = 40;
show_one(input_boost_ms);
store_one(input_boost_ms);
cpu_boost_attr_rw(input_boost_ms);

#ifdef CONFIG_SCHED_CAS
static unsigned int cas_boost_status = 0;
static unsigned int cas_feature_enable = 1;
#define store_one_cas(file_name)								\
static ssize_t store_##file_name								\
(struct kobject *kobj, struct kobj_attribute *attr,				\
const char *buf, size_t count)									\
{																\
																\
	sscanf(buf, "%u", &file_name);								\
	if (cas_feature_enable) {									\
		if (cas_boost_status != 0) {						 	\
			schedtune_set_touch_boost(0);						\
		} else {												\
			schedtune_set_touch_boost(1);						\
		}														\
	}															\
																\
	return count;												\
}

#define store_one_cas_enable(file_name)							\
static ssize_t store_##file_name								\
(struct kobject *kobj, struct kobj_attribute *attr,				\
const char *buf, size_t count)									\
{																\
																\
	sscanf(buf, "%u", &file_name);								\
	if (cas_feature_enable == 0) {								\
		schedtune_set_touch_boost(0);								\
	} else {													\
		schedtune_set_touch_boost(1);								\
	}															\
																\
	return count;												\
}

show_one(cas_boost_status);
store_one_cas(cas_boost_status);
cpu_boost_cas_attr_rw(cas_boost_status);

show_one(cas_feature_enable);
store_one_cas_enable(cas_feature_enable);
cpu_boost_cas_attr_rw(cas_feature_enable);

#endif /* CONFIG_SCHED_CAS */

static unsigned int sched_boost_on_input;
show_one(sched_boost_on_input);
store_one(sched_boost_on_input);
cpu_boost_attr_rw(sched_boost_on_input);

static bool sched_boost_active;

static struct delayed_work input_boost_rem;
#ifdef CONFIG_SCHED_CAS
static unsigned int input_schedtune_boost_ms = 1500;
static struct delayed_work input_schedtune_boost;
#endif /* CONFIG_SCHED_CAS */
static u64 last_input_time;
#define MIN_INPUT_INTERVAL (150 * USEC_PER_MSEC)

static ssize_t store_input_boost_freq(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int i, ntokens = 0;
	unsigned int val, cpu;
	const char *cp = buf;
	bool enabled = false;

	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	/* single number: apply to all CPUs */
	if (!ntokens) {
		if (sscanf(buf, "%u\n", &val) != 1)
			return -EINVAL;
		for_each_possible_cpu(i)
			per_cpu(sync_info, i).input_boost_freq = val;
		goto check_enable;
	}

	/* CPU:value pair */
	if (!(ntokens % 2))
		return -EINVAL;

	cp = buf;
	for (i = 0; i < ntokens; i += 2) {
		if (sscanf(cp, "%u:%u", &cpu, &val) != 2)
			return -EINVAL;
		if (cpu >= num_possible_cpus())
			return -EINVAL;

		per_cpu(sync_info, cpu).input_boost_freq = val;
		cp = strnchr(cp, PAGE_SIZE - (cp - buf), ' ');
		cp++;
	}

check_enable:
	for_each_possible_cpu(i) {
		if (per_cpu(sync_info, i).input_boost_freq) {
			enabled = true;
			break;
		}
	}
	input_boost_enabled = enabled;

	return count;
}

static ssize_t show_input_boost_freq(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	int cnt = 0, cpu;
	struct cpu_sync *s;

	for_each_possible_cpu(cpu) {
		s = &per_cpu(sync_info, cpu);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%d:%u ", cpu, s->input_boost_freq);
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
	return cnt;
}

cpu_boost_attr_rw(input_boost_freq);

static DEFINE_PER_CPU(unsigned int, sub_boost_freq);

#define MIN_INPUT_INTERVAL_MS 40
#define MIN_INPUT_INTERVAL_US (MIN_INPUT_INTERVAL_MS * USEC_PER_MSEC)
#define MAX_PRECEDING_BOOST_TIME 200

static bool sub_boost_enabled = false;
static unsigned int prec_boost_ms = 0;
static unsigned int boost_step = 0;

static struct work_struct input_boost_multi_step_work;

static ssize_t store_sub_boost_freq(struct kobject *kobj,
                      struct kobj_attribute *attr,
                      const char *buf, size_t count)
{
	int i, ntokens = 0;
	unsigned int val, cpu;
	const char *cp = buf;
	bool enabled = false;

	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	/* single number: apply to all CPUs */
	if (!ntokens) {
		if (sscanf(buf, "%u\n", &val) != 1)
			return -EINVAL;
		for_each_possible_cpu(i)
			per_cpu(sub_boost_freq, i) = val;
		goto check_enable;
	}

	/* CPU:value pair */
	if (!(ntokens % 2))
		return -EINVAL;

	cp = buf;
	for (i = 0; i < ntokens; i += 2) {
		if (sscanf(cp, "%u:%u", &cpu, &val) != 2)
			return -EINVAL;
		if (cpu >= num_possible_cpus())
			return -EINVAL;

		per_cpu(sub_boost_freq, cpu) = val;
		cp = strchr(cp, ' ');
		cp++;
	}

check_enable:
	for_each_possible_cpu(i) {
		if (per_cpu(sub_boost_freq, i)) {
			enabled = true;
			break;
		}
	}
	sub_boost_enabled = enabled;

	return count;
}

static ssize_t show_sub_boost_freq(struct kobject *kobj,
                     struct kobj_attribute *attr, char *buf)
{
	int cnt = 0, cpu;
	unsigned int val;

	for_each_possible_cpu(cpu) {
		val = per_cpu(sub_boost_freq, cpu);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%d:%u ", cpu, val);
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
	return cnt;
}

cpu_boost_attr_rw(sub_boost_freq);

/*
 * The CPUFREQ_ADJUST notifier is used to override the current policy min to
 * make sure policy min >= boost_min. The cpufreq framework then does the job
 * of enforcing the new policy.
 */
static int boost_adjust_notify(struct notifier_block *nb, unsigned long val,
				void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int cpu = policy->cpu;
	struct cpu_sync *s = &per_cpu(sync_info, cpu);
	unsigned int ib_min = s->input_boost_min;

	switch (val) {
	case CPUFREQ_ADJUST:
		if (!ib_min)
			break;

		pr_debug("CPU%u policy min before boost: %u kHz\n",
			 cpu, policy->min);
		pr_debug("CPU%u boost min: %u kHz\n", cpu, ib_min);

		cpufreq_verify_within_limits(policy, ib_min, UINT_MAX);

		pr_debug("CPU%u policy min after boost: %u kHz\n",
			 cpu, policy->min);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block boost_adjust_nb = {
	.notifier_call = boost_adjust_notify,
};

static void update_policy_online(void)
{
	unsigned int i;

	/* Re-evaluate policy to trigger adjust notifier for online CPUs */
	get_online_cpus();
	for_each_online_cpu(i) {
		pr_debug("Updating policy for CPU%d\n", i);
		cpufreq_update_policy(i);
	}
	put_online_cpus();
}

#ifdef CONFIG_SCHED_CAS
static void do_input_schedtune_boost(struct work_struct *work){
	if (cas_feature_enable) {
		if (cas_boost_status != 0) {
			schedtune_set_touch_boost(0);
		} else {
			schedtune_set_touch_boost(1);
		}
	}
}
#endif /* CONFIG_SCHED_CAS */

static void do_input_boost_rem(struct work_struct *work)
{
	unsigned int i, ret;
	struct cpu_sync *i_sync_info;

	/* Reset the input_boost_min for all CPUs in the system */
	pr_debug("Resetting input boost min for all CPUs\n");
	for_each_possible_cpu(i) {
		i_sync_info = &per_cpu(sync_info, i);
		i_sync_info->input_boost_min = 0;
	}

	/* Update policies for all online CPUs */
	update_policy_online();

	if (sched_boost_active) {
		ret = sched_set_boost(0);
		if (ret)
			pr_err("cpu-boost: sched boost disable failed\n");
		sched_boost_active = false;
	}

	boost_step = 0;

#ifdef CONFIG_SCHED_CAS
	queue_delayed_work(cpu_boost_wq, &input_schedtune_boost,
					msecs_to_jiffies(input_schedtune_boost_ms));
#endif /* CONFIG_SCHED_CAS */

}

static void do_input_boost(struct work_struct *work)
{
	unsigned int i, ret;
	struct cpu_sync *i_sync_info;

	cancel_delayed_work_sync(&input_boost_rem);
	if (sched_boost_active) {
		sched_set_boost(0);
		sched_boost_active = false;
	}

	/* Set the input_boost_min for all CPUs in the system */
	pr_debug("Setting input boost min for all CPUs\n");
	for_each_possible_cpu(i) {
		i_sync_info = &per_cpu(sync_info, i);
		i_sync_info->input_boost_min = i_sync_info->input_boost_freq;
	}

	/* Update policies for all online CPUs */
	update_policy_online();

	/* Enable scheduler boost to migrate tasks to big cluster */
	if (sched_boost_on_input > 0) {
		ret = sched_set_boost(sched_boost_on_input);
		if (ret)
			pr_err("cpu-boost: sched boost enable failed\n");
		else
			sched_boost_active = true;
	}

	queue_delayed_work(cpu_boost_wq, &input_boost_rem,
					msecs_to_jiffies(input_boost_ms));
}

static void do_input_boost_multi_step(struct work_struct *work)
{
	unsigned int i, ret;
	struct cpu_sync *i_sync_info;

	cancel_delayed_work_sync(&input_boost_rem);

#ifdef CONFIG_SCHED_CAS
	if (cas_feature_enable) {
		if (cas_boost_status == 0 || cas_boost_status == 2) {
			schedtune_set_touch_boost(30);
		} else if (cas_boost_status == 1) {
			schedtune_set_touch_boost(1);
		} else {
			schedtune_set_touch_boost(0);
		}
	}
#endif

	if (boost_step == 0) {
		// step 1
		pr_debug("Multi step boost: step 1\n");
		if (sched_boost_active) {
			sched_set_boost(0);
			sched_boost_active = false;
		}

		for_each_possible_cpu(i) {
			i_sync_info = &per_cpu(sync_info, i);
			i_sync_info->input_boost_min = i_sync_info->input_boost_freq;
		}

		update_policy_online();

		if (sched_boost_on_input > 0) {
			ret = sched_set_boost(sched_boost_on_input);
			if (ret)
				pr_err("cpu-boost: HMP boost enable failed\n");
			else
				sched_boost_active = true;
		}

		if (sub_boost_enabled) {
			boost_step = 1;
			prec_boost_ms = 0;
		}
	} else if (boost_step == 1) {
		// step 2
		prec_boost_ms += MIN_INPUT_INTERVAL_MS;
		if (prec_boost_ms >= MAX_PRECEDING_BOOST_TIME) {
			pr_debug("Multi step boost: step 2\n");
			for_each_possible_cpu(i) {
				i_sync_info = &per_cpu(sync_info, i);
				i_sync_info->input_boost_min = per_cpu(sub_boost_freq, i);
			}

			update_policy_online();

			if (sched_boost_active) {
				ret = sched_set_boost(0);
				if (ret)
					pr_err("cpu-boost: HMP boost disable failed\n");
				sched_boost_active = false;
			}

			boost_step = 2;
		}
	}

	queue_delayed_work(cpu_boost_wq, &input_boost_rem,
					msecs_to_jiffies(input_boost_ms));
}

static void cpuboost_input_event(struct input_handle *handle,
		unsigned int type, unsigned int code, int value)
{
	u64 now;

	if (!input_boost_enabled)
		return;

	{ // multi-step boost
		now = ktime_to_us(ktime_get());
		if (now - last_input_time < MIN_INPUT_INTERVAL_US)
			return;

		if (work_pending(&input_boost_multi_step_work))
			return;

		queue_work(cpu_boost_wq, &input_boost_multi_step_work);
		last_input_time = ktime_to_us(ktime_get());
		return;
	}

	now = ktime_to_us(ktime_get());
	if (now - last_input_time < MIN_INPUT_INTERVAL)
		return;

	if (work_pending(&input_boost_work))
		return;

	queue_work(cpu_boost_wq, &input_boost_work);
	last_input_time = ktime_to_us(ktime_get());
}

static int cpuboost_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void cpuboost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpuboost_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler cpuboost_input_handler = {
	.event          = cpuboost_input_event,
	.connect        = cpuboost_input_connect,
	.disconnect     = cpuboost_input_disconnect,
	.name           = "cpu-boost",
	.id_table       = cpuboost_ids,
};

struct kobject *cpu_boost_kobj;
static int cpu_boost_init(void)
{
	int cpu, ret;
	struct cpu_sync *s;

	cpu_boost_wq = alloc_workqueue("cpuboost_wq", WQ_HIGHPRI, 0);
	if (!cpu_boost_wq)
		return -EFAULT;

	INIT_WORK(&input_boost_work, do_input_boost);
	INIT_DELAYED_WORK(&input_boost_rem, do_input_boost_rem);
	INIT_WORK(&input_boost_multi_step_work, do_input_boost_multi_step);

#ifdef CONFIG_SCHED_CAS
	INIT_DELAYED_WORK(&input_schedtune_boost, do_input_schedtune_boost);
#endif /* CONFIG_SCHED_CAS */

	for_each_possible_cpu(cpu) {
		s = &per_cpu(sync_info, cpu);
		s->cpu = cpu;
	}
	cpufreq_register_notifier(&boost_adjust_nb, CPUFREQ_POLICY_NOTIFIER);

	cpu_boost_kobj = kobject_create_and_add("cpu_boost",
						&cpu_subsys.dev_root->kobj);
	if (!cpu_boost_kobj)
		pr_err("Failed to initialize sysfs node for cpu_boost.\n");

	ret = sysfs_create_file(cpu_boost_kobj, &input_boost_ms_attr.attr);
	if (ret)
		pr_err("Failed to create input_boost_ms node: %d\n", ret);

	ret = sysfs_create_file(cpu_boost_kobj, &input_boost_freq_attr.attr);
	if (ret)
		pr_err("Failed to create input_boost_freq node: %d\n", ret);

	ret = sysfs_create_file(cpu_boost_kobj,
				&sched_boost_on_input_attr.attr);
	if (ret)
		pr_err("Failed to create sched_boost_on_input node: %d\n", ret);

    ret = sysfs_create_file(cpu_boost_kobj, &sub_boost_freq_attr.attr);
    if (ret)
        pr_err("Failed to create sub_boost_freq node: %d\n", ret);

#ifdef CONFIG_SCHED_CAS
    ret = sysfs_create_file(cpu_boost_kobj, &cas_boost_status_attr.attr);
    if (ret)
        pr_err("Failed to create cas_boost_status node: %d\n", ret);

    ret = sysfs_create_file(cpu_boost_kobj, &cas_feature_enable_attr.attr);
    if (ret)
        pr_err("Failed to create cas_feature_enable node: %d\n", ret);
#endif /* CONFIG_SCHED_CAS */

	ret = input_register_handler(&cpuboost_input_handler);
	return 0;
}
late_initcall(cpu_boost_init);
