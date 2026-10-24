// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens performance helpers.
 *
 * Small, self-contained helpers that steer frequency and task placement
 * through the standard freq_qos request API.  There is deliberately no private
 * cpufreq governor and no hook in the scheduler or the thermal core: a boost
 * only updates a QoS request, and the governor reacts exactly as it would to
 * any other client.  That keeps cpufreq_schedutil.c, the scheduler sources and
 * the thermal core byte-identical to the upstream tree.
 *
 * Every helper has its own runtime switch under /sys/kernel/grayravens/.
 *
 * Inspired by the approach used by SuiKernel; the implementation is ours.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/bitops.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/notifier.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "grayravens.h"

#define GR_MAX_POLICIES	8

struct gr_policy {
	struct cpufreq_policy *policy;
	struct freq_qos_request min_req;
};

static struct gr_policy gr_policies[GR_MAX_POLICIES];
static int gr_policy_count;
static DEFINE_MUTEX(gr_lock);

static bool gr_boot_boost = true;
static bool gr_touch_boost = true;
static unsigned int gr_boost_pct = CONFIG_GRAYRAVENS_BOOST_PCT;

static struct delayed_work gr_boost_off_work;

struct kobject *gr_kobj;

/* ------------------------------------------------------------------ */
/* freq_qos plumbing                                                   */

static void gr_register_policy(struct cpufreq_policy *policy)
{
	struct gr_policy *gp;
	int i;

	for (i = 0; i < gr_policy_count; i++)
		if (gr_policies[i].policy == policy)
			return;

	if (gr_policy_count >= GR_MAX_POLICIES)
		return;

	gp = &gr_policies[gr_policy_count];

	if (freq_qos_add_request(&policy->constraints, &gp->min_req,
				 FREQ_QOS_MIN, FREQ_QOS_MIN_DEFAULT_VALUE))
		return;

	gp->policy = policy;
	gr_policy_count++;
}

static void gr_scan_policies(void)
{
	int cpu;

	mutex_lock(&gr_lock);
	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (!policy)
			continue;
		gr_register_policy(policy);
		cpufreq_cpu_put(policy);
	}
	mutex_unlock(&gr_lock);
}

static s32 gr_scaled(struct gr_policy *gp, unsigned int pct)
{
	return (s32)((u64)gp->policy->cpuinfo.max_freq * pct / 100);
}

/*
 * Raise the floor on every policy to @pct of its maximum.  This is a QoS
 * request, so it composes with whatever the governor computes instead of
 * replacing it.
 */
static void __maybe_unused gr_boost_apply(unsigned int pct)
{
	int i;

	mutex_lock(&gr_lock);
	for (i = 0; i < gr_policy_count; i++) {
		struct gr_policy *gp = &gr_policies[i];

		freq_qos_update_request(&gp->min_req, gr_scaled(gp, pct));
	}
	mutex_unlock(&gr_lock);
}

static void gr_boost_release(void)
{
	int i;

	mutex_lock(&gr_lock);
	for (i = 0; i < gr_policy_count; i++)
		freq_qos_update_request(&gr_policies[i].min_req,
					FREQ_QOS_MIN_DEFAULT_VALUE);
	mutex_unlock(&gr_lock);
}

static void gr_boost_off(struct work_struct *work)
{
	gr_boost_release();
}

static void __maybe_unused gr_boost_for(unsigned int ms)
{
	mod_delayed_work(system_wq, &gr_boost_off_work, msecs_to_jiffies(ms));
}

#if IS_ENABLED(CONFIG_GRAYRAVENS_BOOT_BOOST)
static void gr_boot_boost_work(struct work_struct *work)
{
	if (!gr_boot_boost)
		return;

	gr_boost_apply(100);
	gr_boost_for(CONFIG_GRAYRAVENS_BOOT_HOLD_MS);
}
#endif

/* ------------------------------------------------------------------ */
/* Touch boost                                                         */

#if IS_ENABLED(CONFIG_GRAYRAVENS_TOUCH_BOOST)
static bool gr_is_touch(unsigned int type, unsigned int code, int value)
{
	if (type == EV_ABS)
		return code == ABS_MT_POSITION_X || code == ABS_MT_POSITION_Y ||
		       code == ABS_X || code == ABS_Y ||
		       code == ABS_MT_TRACKING_ID;
	if (type == EV_KEY && code == BTN_TOUCH && value)
		return true;

	return false;
}

static void gr_input_event(struct input_handle *handle, unsigned int type,
			   unsigned int code, int value)
{
	if (!gr_touch_boost || !gr_is_touch(type, code, value))
		return;

	gr_boost_apply(gr_boost_pct);
	gr_boost_for(CONFIG_GRAYRAVENS_TOUCH_HOLD_MS);
}

static int gr_input_connect(struct input_handler *handler,
			    struct input_dev *dev,
			    const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;

	error = input_register_handle(handle);
	if (error) {
		kfree(handle);
		return error;
	}

	return 0;
}

static void gr_input_disconnect(struct input_handle *handle)
{
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id gr_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
	},
	{ },
};

static struct input_handler gr_input_handler = {
	.event		= gr_input_event,
	.connect	= gr_input_connect,
	.disconnect	= gr_input_disconnect,
	.name		= "grayravens-boost",
	.id_table	= gr_ids,
};
#endif /* CONFIG_GRAYRAVENS_TOUCH_BOOST */

/* ------------------------------------------------------------------ */
/* Reclaim tuning                                                      */

#if IS_ENABLED(CONFIG_GRAYRAVENS_RECLAIM_TUNE)
/*
 * Return the first CPU of the policy with the highest maximum frequency, i.e.
 * a prime core on a DynamIQ cluster layout.
 */
static int gr_prime_cpu(void)
{
	unsigned int best_max = 0;
	int cpu, best = 0;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (!policy)
			continue;
		if (policy->cpuinfo.max_freq > best_max) {
			best_max = policy->cpuinfo.max_freq;
			best = cpumask_first(policy->related_cpus);
		}
		cpufreq_cpu_put(policy);
	}

	return best;
}

static void gr_tune_reclaim(void)
{
	int cpu = gr_prime_cpu();
	int nid;

	for_each_online_node(nid) {
		struct task_struct *kswapd = NODE_DATA(nid)->kswapd;

		if (!kswapd)
			continue;
		set_cpus_allowed_ptr(kswapd, cpumask_of(cpu));
	}

	vm_swappiness = 30;
}
#endif /* CONFIG_GRAYRAVENS_RECLAIM_TUNE */

/* ------------------------------------------------------------------ */
/* sysfs                                                               */

static ssize_t boost_pct_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%u\n", gr_boost_pct);
}

static ssize_t boost_pct_store(struct kobject *kobj, struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value > 100)
		return -EINVAL;

	gr_boost_pct = value;

	return count;
}

#define GR_BOOL_ATTR(_name, _var)					\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sprintf(buf, "%u\n", _var ? 1 : 0);			\
}									\
static ssize_t _name##_store(struct kobject *kobj,			\
			     struct kobj_attribute *attr,		\
			     const char *buf, size_t count)		\
{									\
	bool value;							\
									\
	if (kstrtobool(buf, &value))					\
		return -EINVAL;						\
	_var = value;							\
	return count;							\
}									\
static struct kobj_attribute _name##_attr =				\
	__ATTR(_name, 0644, _name##_show, _name##_store)

GR_BOOL_ATTR(boot_boost, gr_boot_boost);
GR_BOOL_ATTR(touch_boost, gr_touch_boost);

static struct kobj_attribute boost_pct_attr =
	__ATTR(boost_pct, 0644, boost_pct_show, boost_pct_store);

static struct attribute *gr_attrs[] = {
	&boost_pct_attr.attr,
	&boot_boost_attr.attr,
	&touch_boost_attr.attr,
	NULL,
};

static struct attribute_group gr_attr_group = {
	.attrs = gr_attrs,
};

/* ------------------------------------------------------------------ */
/* init                                                                */

static int gr_policy_notify(struct notifier_block *nb, unsigned long event,
			    void *data)
{
	if (event == CPUFREQ_CREATE_POLICY) {
		mutex_lock(&gr_lock);
		gr_register_policy(data);
		mutex_unlock(&gr_lock);
	}

	return NOTIFY_OK;
}

static struct notifier_block gr_policy_nb = {
	.notifier_call = gr_policy_notify,
};

static int __init gr_perf_init(void)
{
	INIT_DELAYED_WORK(&gr_boost_off_work, gr_boost_off);

	gr_kobj = kobject_create_and_add("grayravens", kernel_kobj);
	if (!gr_kobj)
		return -ENOMEM;

	if (sysfs_create_group(gr_kobj, &gr_attr_group)) {
		kobject_put(gr_kobj);
		return -ENOMEM;
	}

	gr_scan_policies();
	cpufreq_register_notifier(&gr_policy_nb, CPUFREQ_POLICY_NOTIFIER);

#if IS_ENABLED(CONFIG_GRAYRAVENS_BOOT_BOOST)
	{
		static struct delayed_work gr_boot_work;

		INIT_DELAYED_WORK(&gr_boot_work, gr_boot_boost_work);
		schedule_delayed_work(&gr_boot_work,
				      msecs_to_jiffies(CONFIG_GRAYRAVENS_BOOT_DELAY_MS));
	}
#endif

#if IS_ENABLED(CONFIG_GRAYRAVENS_TOUCH_BOOST)
	input_register_handler(&gr_input_handler);
#endif

#if IS_ENABLED(CONFIG_GRAYRAVENS_RECLAIM_TUNE)
	gr_tune_reclaim();
#endif

	pr_info("performance helpers ready (%d policies)\n", gr_policy_count);

	return 0;
}
late_initcall(gr_perf_init);
