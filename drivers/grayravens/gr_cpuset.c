// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens cpuset partition.
 *
 * Android's init writes /dev/cpuset/<group>/cpus from rc files, and a vendor
 * rc is free to concentrate every group on the little cluster or leave the
 * prime core in foreground.  Partition the groups from the real CPU
 * capacities once, then re-assert a bounded number of times so an init write
 * cannot undo it after the fact.
 *
 * Operates through filp_open() on the cpuset files, so unlike a cgroup hook
 * it cannot disturb any cgroup internal state and it degrades to a no-op on a
 * device without /dev/cpuset.
 *
 * Reimplemented from the behaviour of SuiKernel's airani_iofifteen_cpuset
 * (Kanagawa Yamada, GPL-3.0-only), which cannot be linked into a GPL-2.0-only
 * kernel.  No code is taken from it.  The delivery is a plain timer rather
 * than that tree's system-wide vfs_write kprobe.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sched/topology.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "grayravens.h"

enum gr_pool {
	GR_ALL,
	GR_NO_PRIME,
	GR_LITTLE,
};

/*
 * Groups AOSP ships.  Vendor-specific ones (foreground_window, oiface_*) are
 * deliberately absent: a path that does not exist simply fails to open.
 */
static const struct {
	const char *path;
	enum gr_pool pool;
} gr_targets[] = {
	{ "/dev/cpuset/top-app/cpus",		GR_ALL },
	{ "/dev/cpuset/display/cpus",		GR_ALL },
	{ "/dev/cpuset/sf/cpus",		GR_ALL },
	{ "/dev/cpuset/audio-app/cpus",		GR_ALL },
	{ "/dev/cpuset/camera-daemon/cpus",	GR_ALL },
	{ "/dev/cpuset/foreground/cpus",	GR_NO_PRIME },
	{ "/dev/cpuset/background/cpus",	GR_LITTLE },
	{ "/dev/cpuset/system-background/cpus",	GR_LITTLE },
	{ "/dev/cpuset/restricted/cpus",	GR_LITTLE },
	{ "/dev/cpuset/camera-background/cpus",	GR_LITTLE },
};

static bool gr_cpuset_enabled = true;
static unsigned int gr_cpuset_passes_left;
static unsigned int gr_cpuset_pass_total = 12;
static unsigned int gr_cpuset_interval_ms = 5000;

static struct cpumask gr_all, gr_no_prime, gr_little;
static struct delayed_work gr_cpuset_work;
static struct kobject *gr_cpuset_kobj;

static void gr_cpuset_build_masks(void)
{
	unsigned long max = 0, min = ULONG_MAX;
	int cpu;

	for_each_possible_cpu(cpu) {
		unsigned long cap = arch_scale_cpu_capacity(cpu);

		if (cap > max)
			max = cap;
		if (cap < min)
			min = cap;
	}

	cpumask_clear(&gr_all);
	cpumask_clear(&gr_no_prime);
	cpumask_clear(&gr_little);

	for_each_possible_cpu(cpu) {
		unsigned long cap = arch_scale_cpu_capacity(cpu);

		cpumask_set_cpu(cpu, &gr_all);

		if (cap == min)
			cpumask_set_cpu(cpu, &gr_little);

		/* Everything but the highest-capacity core. */
		if (cap < max || max == min)
			cpumask_set_cpu(cpu, &gr_no_prime);
	}
}

static const struct cpumask *gr_cpuset_pool(enum gr_pool pool)
{
	switch (pool) {
	case GR_NO_PRIME:
		return &gr_no_prime;
	case GR_LITTLE:
		return &gr_little;
	default:
		return &gr_all;
	}
}

static int gr_cpuset_write(const char *path, const struct cpumask *mask)
{
	char buf[64];
	struct file *f;
	loff_t pos = 0;
	int len;

	len = snprintf(buf, sizeof(buf), "%*pbl\n", cpumask_pr_args(mask));

	f = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	len = kernel_write(f, buf, len, &pos);
	filp_close(f, NULL);

	return len < 0 ? len : 0;
}

static void gr_cpuset_apply_one(void)
{
	int i, ret, applied = 0;

	gr_cpuset_build_masks();

	for (i = 0; i < ARRAY_SIZE(gr_targets); i++) {
		ret = gr_cpuset_write(gr_targets[i].path,
				      gr_cpuset_pool(gr_targets[i].pool));
		if (!ret)
			applied++;
	}

	pr_debug("cpuset: applied %d/%zu (all=%*pbl no-prime=%*pbl little=%*pbl)\n",
		 applied, ARRAY_SIZE(gr_targets),
		 cpumask_pr_args(&gr_all), cpumask_pr_args(&gr_no_prime),
		 cpumask_pr_args(&gr_little));
}

static void gr_cpuset_work_fn(struct work_struct *work)
{
	if (READ_ONCE(gr_cpuset_enabled))
		gr_cpuset_apply_one();

	if (gr_cpuset_passes_left)
		gr_cpuset_passes_left--;

	if (gr_cpuset_passes_left && READ_ONCE(gr_cpuset_enabled))
		schedule_delayed_work(&gr_cpuset_work,
			msecs_to_jiffies(gr_cpuset_interval_ms));
}

/* ------------------------------------------------------------------ */
/* Sysfs                                                              */
/* ------------------------------------------------------------------ */

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%u\n", gr_cpuset_enabled ? 1 : 0);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	bool value;

	if (kstrtobool(buf, &value))
		return -EINVAL;

	gr_cpuset_enabled = value;
	if (!value)
		gr_cpuset_passes_left = 0;

	return count;
}

static ssize_t apply_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	gr_cpuset_apply_one();

	return count;
}

static ssize_t interval_ms_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", gr_cpuset_interval_ms);
}

static ssize_t interval_ms_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int value;

	/* Below 1 s this starts competing with the work it is adjusting. */
	if (kstrtouint(buf, 0, &value) || value < 1000)
		return -EINVAL;

	gr_cpuset_interval_ms = value;

	return count;
}

static ssize_t passes_left_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", gr_cpuset_passes_left);
}

static ssize_t passes_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value > 720)
		return -EINVAL;

	gr_cpuset_pass_total = value;
	gr_cpuset_passes_left = value;

	if (value) {
		gr_cpuset_enabled = true;
		schedule_delayed_work(&gr_cpuset_work,
			msecs_to_jiffies(gr_cpuset_interval_ms));
	} else {
		cancel_delayed_work_sync(&gr_cpuset_work);
	}

	return count;
}

/* What the partition resolved to on this hardware. */
static ssize_t topology_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	gr_cpuset_build_masks();

	return sprintf(buf, "all=%*pbl\nno_prime=%*pbl\nlittle=%*pbl\n",
		       cpumask_pr_args(&gr_all), cpumask_pr_args(&gr_no_prime),
		       cpumask_pr_args(&gr_little));
}

static struct kobj_attribute enable_attr =
	__ATTR_RW(enable);
static struct kobj_attribute apply_attr =
	__ATTR(apply, 0200, NULL, apply_store);
static struct kobj_attribute interval_ms_attr =
	__ATTR_RW(interval_ms);
static struct kobj_attribute passes_left_attr =
	__ATTR_RO(passes_left);
static struct kobj_attribute passes_attr =
	__ATTR(passes, 0200, NULL, passes_store);
static struct kobj_attribute topology_attr =
	__ATTR_RO(topology);

static struct attribute *gr_cpuset_attrs[] = {
	&enable_attr.attr,
	&apply_attr.attr,
	&interval_ms_attr.attr,
	&passes_left_attr.attr,
	&passes_attr.attr,
	&topology_attr.attr,
	NULL,
};

static struct attribute_group gr_cpuset_attr_group = {
	.attrs = gr_cpuset_attrs,
};

static int __init gr_cpuset_init(void)
{
	if (!gr_kobj)
		return -ENODEV;

	gr_cpuset_kobj = kobject_create_and_add("cpuset", gr_kobj);
	if (!gr_cpuset_kobj)
		return -ENOMEM;

	if (sysfs_create_group(gr_cpuset_kobj, &gr_cpuset_attr_group))
		return -ENOMEM;

	INIT_DELAYED_WORK(&gr_cpuset_work, gr_cpuset_work_fn);

	/*
	 * /dev/cpuset is mounted by init, long after this runs, so the first
	 * passes exist to catch the mount and overwrite whatever init wrote.
	 */
	gr_cpuset_passes_left = gr_cpuset_pass_total;
	schedule_delayed_work(&gr_cpuset_work,
			      msecs_to_jiffies(gr_cpuset_interval_ms));

	pr_info("cpuset: partition armed, %u passes every %u ms\n",
		gr_cpuset_pass_total, gr_cpuset_interval_ms);

	return 0;
}
late_initcall(gr_cpuset_init);
