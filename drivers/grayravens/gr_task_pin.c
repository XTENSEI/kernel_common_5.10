// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens task pin helper.
 *
 * Pins a named thread group to a CPU list for as long as the caller keeps it
 * pinned, then restores each thread to the mask it actually had.  Threads can
 * carry different masks, so the masks are saved per thread rather than shared.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pid.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "grayravens.h"

#define GR_TP_MAX_THREADS 256
#define GR_TP_MASK_LEN 64

struct gr_tp_entry {
	struct task_struct *task;
	struct cpumask *saved;
};

static DEFINE_MUTEX(gr_tp_lock);
static struct gr_tp_entry *gr_tp_entries;
static int gr_tp_count;
static struct cpumask gr_tp_mask;
static char gr_tp_mask_str[GR_TP_MASK_LEN];
static struct kobject *gr_tp_kobj;

/* Caller holds gr_tp_lock and a reference on every saved thread. */
static void gr_tp_do_release(void)
{
	int i;

	for (i = 0; i < gr_tp_count; i++) {
		struct gr_tp_entry *e = &gr_tp_entries[i];

		/* Only undo our own change, another writer may have moved it. */
		if (cpumask_equal(&e->task->cpus_mask, &gr_tp_mask))
			set_cpus_allowed_ptr(e->task, e->saved);

		put_task_struct(e->task);
		kfree(e->saved);
	}

	kfree(gr_tp_entries);
	gr_tp_entries = NULL;
	gr_tp_count = 0;
}

static ssize_t boost_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	struct task_struct *task, *thread;
	struct gr_tp_entry *entries;
	int n = 0, pid, ret = 0;

	if (kstrtoint(buf, 0, &pid) || pid <= 0)
		return -EINVAL;

	if (!gr_tp_mask_str[0])
		return -EINVAL;

	if (bitmap_parselist(gr_tp_mask_str, cpumask_bits(&gr_tp_mask),
			     nr_cpu_ids))
		return -EINVAL;

	if (cpumask_empty(&gr_tp_mask) ||
	    !cpumask_subset(&gr_tp_mask, cpu_possible_mask))
		return -EINVAL;

	task = find_get_task_by_vpid(pid);
	if (!task)
		return -ESRCH;

	if (task->flags & PF_KTHREAD) {
		ret = -EINVAL;
		goto out_put;
	}

	entries = kcalloc(GR_TP_MAX_THREADS, sizeof(*entries), GFP_KERNEL);
	if (!entries) {
		ret = -ENOMEM;
		goto out_put;
	}

	rcu_read_lock();
	for_each_thread(task, thread) {
		struct cpumask *saved;

		if (n >= GR_TP_MAX_THREADS) {
			pr_warn_once("task_pin: thread group over %d, tail skipped\n",
				     GR_TP_MAX_THREADS);
			break;
		}

		saved = kmalloc(cpumask_size(), GFP_ATOMIC);
		if (!saved)
			break;

		cpumask_copy(saved, &thread->cpus_mask);
		get_task_struct(thread);

		entries[n].task = thread;
		entries[n].saved = saved;
		n++;
	}
	rcu_read_unlock();

	if (!n) {
		kfree(entries);
		ret = -ESRCH;
		goto out_put;
	}

	mutex_lock(&gr_tp_lock);
	if (gr_tp_count)
		gr_tp_do_release();
	gr_tp_entries = entries;
	gr_tp_count = n;
	mutex_unlock(&gr_tp_lock);

	{
		int i;

		for (i = 0; i < n; i++)
			set_cpus_allowed_ptr(entries[i].task, &gr_tp_mask);
	}

	pr_info("task_pin: pid %d pinned to %s (%d threads)\n",
		pid, gr_tp_mask_str, n);
out_put:
	put_task_struct(task);

	return ret ? ret : count;
}

static ssize_t release_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	mutex_lock(&gr_tp_lock);
	gr_tp_do_release();
	mutex_unlock(&gr_tp_lock);

	return count;
}

static ssize_t active_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_tp_lock);
	ret = sprintf(buf, "%d %s\n", gr_tp_count,
		      gr_tp_count ? gr_tp_mask_str : "-");
	mutex_unlock(&gr_tp_lock);

	return ret;
}

static ssize_t mask_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_tp_lock);
	ret = sprintf(buf, "%s\n", gr_tp_mask_str);
	mutex_unlock(&gr_tp_lock);

	return ret;
}

static ssize_t mask_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	struct cpumask tmp;
	char line[GR_TP_MASK_LEN];

	if (sscanf(buf, "%63s", line) != 1)
		return -EINVAL;

	if (bitmap_parselist(line, cpumask_bits(&tmp), nr_cpu_ids))
		return -EINVAL;

	if (cpumask_empty(&tmp) || !cpumask_subset(&tmp, cpu_possible_mask))
		return -EINVAL;

	mutex_lock(&gr_tp_lock);
	strscpy(gr_tp_mask_str, line, sizeof(gr_tp_mask_str));
	mutex_unlock(&gr_tp_lock);

	return count;
}

static ssize_t threads_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	int n;

	mutex_lock(&gr_tp_lock);
	n = gr_tp_count;
	mutex_unlock(&gr_tp_lock);

	return sprintf(buf, "%d\n", n);
}

static struct kobj_attribute boost_attr = __ATTR(boost, 0200, NULL, boost_store);
static struct kobj_attribute release_attr =
	__ATTR(release, 0200, NULL, release_store);
static struct kobj_attribute mask_attr = __ATTR(mask, 0644, mask_show, mask_store);
static struct kobj_attribute active_attr = __ATTR(active, 0444, active_show, NULL);
static struct kobj_attribute threads_attr =
	__ATTR(threads, 0444, threads_show, NULL);

static struct attribute *gr_tp_attrs[] = {
	&boost_attr.attr,
	&release_attr.attr,
	&mask_attr.attr,
	&active_attr.attr,
	&threads_attr.attr,
	NULL,
};

static struct attribute_group gr_tp_attr_group = {
	.attrs = gr_tp_attrs,
};

static int __init gr_task_pin_init(void)
{
	int ret;

	if (!gr_kobj)
		return -ENODEV;

	gr_tp_kobj = kobject_create_and_add("task_pin", gr_kobj);
	if (!gr_tp_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(gr_tp_kobj, &gr_tp_attr_group);
	if (ret)
		return ret;

	return 0;
}
late_initcall(gr_task_pin_init);
