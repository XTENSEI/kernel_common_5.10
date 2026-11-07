// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens foreground uclamp floor.
 *
 * Hikari's only actuator with a measurable effect was a uclamp_min boost on
 * the waking task.  The same floor can be requested through the sched_setattr
 * path from process context instead, so nothing runs on a wake-up, a tick or
 * a task switch.  Userspace names the task, the kernel lifts it and drops the
 * request when the hold window expires.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <uapi/linux/sched/types.h>

#include "grayravens.h"

static DEFINE_MUTEX(gr_uclamp_lock);
static struct task_struct *gr_uclamp_task;
static unsigned int gr_uclamp_pct = 80;
static unsigned int gr_uclamp_hold_ms = 2000;
static struct delayed_work gr_uclamp_release_work;
static struct kobject *gr_uclamp_kobj;

/* Requests go through __sched_setscheduler, never by writing task fields. */
static void gr_uclamp_set(struct task_struct *task, unsigned int pct)
{
	struct sched_attr attr = {
		.sched_flags = SCHED_FLAG_KEEP_POLICY | SCHED_FLAG_KEEP_PARAMS |
			       SCHED_FLAG_UTIL_CLAMP_MIN,
		.sched_util_min = pct * SCHED_CAPACITY_SCALE / 100,
	};

	if (sched_setattr_nocheck(task, &attr))
		pr_warn("uclamp: cannot set %u%% on %s (%d)\n",
			pct, task->comm, task->pid);
}

static void gr_uclamp_release(struct work_struct *work)
{
	struct task_struct *task;

	mutex_lock(&gr_uclamp_lock);
	task = gr_uclamp_task;
	gr_uclamp_task = NULL;
	mutex_unlock(&gr_uclamp_lock);

	if (!task)
		return;

	gr_uclamp_set(task, 0);
	put_task_struct(task);
}

static ssize_t boost_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	struct task_struct *task, *old;
	int pid;

	if (kstrtoint(buf, 0, &pid) || pid <= 0)
		return -EINVAL;

	task = find_get_task_by_vpid(pid);
	if (!task)
		return -ESRCH;

	mutex_lock(&gr_uclamp_lock);
	old = gr_uclamp_task;
	gr_uclamp_task = task;
	mutex_unlock(&gr_uclamp_lock);

	if (old) {
		gr_uclamp_set(old, 0);
		put_task_struct(old);
	}

	gr_uclamp_set(task, READ_ONCE(gr_uclamp_pct));
	mod_delayed_work(system_wq, &gr_uclamp_release_work,
			 msecs_to_jiffies(READ_ONCE(gr_uclamp_hold_ms)));

	return count;
}

static ssize_t release_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	cancel_delayed_work_sync(&gr_uclamp_release_work);
	gr_uclamp_release(NULL);

	return count;
}

static ssize_t pct_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", gr_uclamp_pct);
}

static ssize_t pct_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value < 1 || value > 100)
		return -EINVAL;

	gr_uclamp_pct = value;

	return count;
}

static ssize_t hold_ms_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%u\n", gr_uclamp_hold_ms);
}

static ssize_t hold_ms_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;

	gr_uclamp_hold_ms = value;

	return count;
}

static ssize_t active_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	int pid = 0;

	mutex_lock(&gr_uclamp_lock);
	if (gr_uclamp_task)
		pid = gr_uclamp_task->pid;
	mutex_unlock(&gr_uclamp_lock);

	return sprintf(buf, "%d\n", pid);
}

static struct kobj_attribute boost_attr =
	__ATTR(boost, 0200, NULL, boost_store);
static struct kobj_attribute release_attr =
	__ATTR(release, 0200, NULL, release_store);
static struct kobj_attribute pct_attr =
	__ATTR(pct, 0644, pct_show, pct_store);
static struct kobj_attribute hold_ms_attr =
	__ATTR(hold_ms, 0644, hold_ms_show, hold_ms_store);
static struct kobj_attribute active_attr =
	__ATTR(active, 0444, active_show, NULL);

static struct attribute *gr_uclamp_attrs[] = {
	&boost_attr.attr,
	&release_attr.attr,
	&pct_attr.attr,
	&hold_ms_attr.attr,
	&active_attr.attr,
	NULL,
};

static struct attribute_group gr_uclamp_attr_group = {
	.attrs = gr_uclamp_attrs,
};

static int __init gr_uclamp_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&gr_uclamp_release_work, gr_uclamp_release);

	if (!gr_kobj)
		return -ENODEV;

	gr_uclamp_kobj = kobject_create_and_add("uclamp", gr_kobj);
	if (!gr_uclamp_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(gr_uclamp_kobj, &gr_uclamp_attr_group);
	if (ret)
		return ret;

	return 0;
}
late_initcall(gr_uclamp_init);