// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens foreground I/O priority boost.
 *
 * Level loads on this class of device stall on storage, not on frequency.
 * Both schedulers in this tree honour ioprio: BFQ maps it to its own class
 * and mq-deadline maps it to DD_RT_PRIO, so raising the game's priority
 * moves its reads ahead of background work without touching the block
 * layer.  Userspace names the pid, the whole thread group is raised, and
 * release puts the group back on the nice-derived best-effort level.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/ioprio.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/sysfs.h>

#include "grayravens.h"

static DEFINE_MUTEX(gr_ioprio_lock);
static struct task_struct *gr_ioprio_task;
static unsigned int gr_ioprio_class = IOPRIO_CLASS_RT;
static unsigned int gr_ioprio_level = 6;
static struct kobject *gr_ioprio_kobj;

static int gr_ioprio_value(void)
{
	return IOPRIO_PRIO_VALUE(READ_ONCE(gr_ioprio_class),
				 READ_ONCE(gr_ioprio_level));
}

/* Reset one thread to what its nice value would have produced. */
static void gr_ioprio_reset_one(struct task_struct *task)
{
	set_task_ioprio(task, IOPRIO_PRIO_VALUE(IOPRIO_CLASS_BE,
						task_nice_ioprio(task)));
}

#define GR_IOPRIO_MAX_THREADS 256

/*
 * set_task_ioprio() can allocate an io_context, so the tasks have to be
 * collected under RCU and touched afterwards.  The cap keeps the stack
 * frame bounded; a game thread group is far below it.
 */
static void gr_ioprio_apply(struct task_struct *task, bool boost)
{
	struct task_struct *threads[GR_IOPRIO_MAX_THREADS];
	struct task_struct *thread;
	int n = 0, i;

	rcu_read_lock();
	for_each_thread(task, thread) {
		if (n >= GR_IOPRIO_MAX_THREADS) {
			pr_warn_once("ioprio: thread group over %d, tail skipped\n",
				     GR_IOPRIO_MAX_THREADS);
			break;
		}
		get_task_struct(thread);
		threads[n++] = thread;
	}
	rcu_read_unlock();

	for (i = 0; i < n; i++) {
		if (boost)
			set_task_ioprio(threads[i], gr_ioprio_value());
		else
			gr_ioprio_reset_one(threads[i]);
		put_task_struct(threads[i]);
	}
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

	mutex_lock(&gr_ioprio_lock);
	old = gr_ioprio_task;
	gr_ioprio_task = task;
	mutex_unlock(&gr_ioprio_lock);

	if (old) {
		gr_ioprio_apply(old, false);
		put_task_struct(old);
	}

	gr_ioprio_apply(task, true);

	return count;
}

static ssize_t release_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	struct task_struct *task;

	mutex_lock(&gr_ioprio_lock);
	task = gr_ioprio_task;
	gr_ioprio_task = NULL;
	mutex_unlock(&gr_ioprio_lock);

	if (!task)
		return count;

	gr_ioprio_apply(task, false);
	put_task_struct(task);

	return count;
}

static ssize_t active_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	int pid = 0;

	mutex_lock(&gr_ioprio_lock);
	if (gr_ioprio_task)
		pid = gr_ioprio_task->pid;
	mutex_unlock(&gr_ioprio_lock);

	return sysfs_emit(buf, "%d\n", pid);
}

static ssize_t class_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	return sysfs_emit(buf, "%u\n", gr_ioprio_class);
}

static ssize_t class_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	if (value != IOPRIO_CLASS_RT && value != IOPRIO_CLASS_BE)
		return -EINVAL;

	gr_ioprio_class = value;

	return count;
}

static ssize_t level_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	return sysfs_emit(buf, "%u\n", gr_ioprio_level);
}

static ssize_t level_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value >= IOPRIO_BE_NR)
		return -EINVAL;

	gr_ioprio_level = value;

	return count;
}

static struct kobj_attribute boost_attr =
	__ATTR(boost, 0200, NULL, boost_store);
static struct kobj_attribute release_attr =
	__ATTR(release, 0200, NULL, release_store);
static struct kobj_attribute active_attr =
	__ATTR_RO(active);
static struct kobj_attribute class_attr =
	__ATTR(class, 0644, class_show, class_store);
static struct kobj_attribute level_attr =
	__ATTR(level, 0644, level_show, level_store);

static struct attribute *gr_ioprio_attrs[] = {
	&boost_attr.attr,
	&release_attr.attr,
	&active_attr.attr,
	&class_attr.attr,
	&level_attr.attr,
	NULL,
};

static struct attribute_group gr_ioprio_attr_group = {
	.attrs = gr_ioprio_attrs,
};

static int __init gr_ioprio_init(void)
{
	if (!gr_kobj)
		return -ENODEV;

	gr_ioprio_kobj = kobject_create_and_add("ioprio", gr_kobj);
	if (!gr_ioprio_kobj)
		return -ENOMEM;

	return sysfs_create_group(gr_ioprio_kobj, &gr_ioprio_attr_group);
}
late_initcall(gr_ioprio_init);
