// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens audio thread priority.
 *
 * Audio dropouts usually come from a mixer thread losing its wake-up to other
 * runnable work on the same CPU.  This watches the scheduler's thread-rename
 * point for the known audio threads and, the first time one appears, lifts it
 * to a fixed SCHED_FIFO priority and requests a short CPU wake-up latency.
 *
 * The policy change runs from a work item, never from the probe itself:
 * sched_setscheduler_nocheck() takes runqueue locks and must not be called
 * with the probe's preemption disabled.  Threads that are already real-time
 * are left alone, so this can only ever lift a normal thread, never demote an
 * audio thread the vendor stack already boosted.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/kprobes.h>
#include <linux/pm_qos.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <uapi/linux/sched/types.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "grayravens.h"

/* Matched as prefixes, because the real names carry a suffix (AudioOut_2D). */
static const char * const gr_audio_names[] = {
	"audioserver",
	"AudioOut",
	"AudioIn",
	"FastMixer",
	"FastCapture",
	"AudioTrack",
	"AudioFlinger",
};

static bool gr_audio_rt = true;
static unsigned int gr_audio_prio = CONFIG_GRAYRAVENS_AUDIO_PRIO;
static unsigned int gr_audio_latency_us = CONFIG_GRAYRAVENS_AUDIO_LATENCY_US;

static DEFINE_SPINLOCK(gr_audio_lock);
static struct task_struct *gr_audio_pending;
static struct work_struct gr_audio_work;
static struct pm_qos_request gr_audio_latency;
static struct kobject *gr_audio_kobj;
static bool gr_audio_probing;

static bool gr_audio_match(const char *comm)
{
	size_t len;
	int i;

	for (i = 0; i < ARRAY_SIZE(gr_audio_names); i++) {
		len = strlen(gr_audio_names[i]);
		if (!strncmp(comm, gr_audio_names[i], len))
			return true;
	}

	return false;
}

static bool gr_audio_is_rt(struct task_struct *task)
{
	return task->policy == SCHED_FIFO || task->policy == SCHED_RR;
}

static void gr_audio_apply(struct work_struct *work)
{
	struct sched_param param = { .sched_priority = gr_audio_prio };
	struct task_struct *task;
	unsigned long flags;

	spin_lock_irqsave(&gr_audio_lock, flags);
	task = gr_audio_pending;
	gr_audio_pending = NULL;
	spin_unlock_irqrestore(&gr_audio_lock, flags);

	if (!task)
		return;

	if (!gr_audio_is_rt(task))
		sched_setscheduler_nocheck(task, SCHED_FIFO, &param);

	put_task_struct(task);
}

static int gr_audio_probe(struct kprobe *kp, struct pt_regs *regs)
{
	struct task_struct *task;
	const char *comm;
	unsigned long flags;

	if (!READ_ONCE(gr_audio_rt))
		return 0;

	task = (struct task_struct *)regs_get_kernel_argument(regs, 0);
	comm = (const char *)regs_get_kernel_argument(regs, 1);

	if (!task || !comm || (task->flags & PF_KTHREAD))
		return 0;

	if (!gr_audio_match(comm))
		return 0;

	spin_lock_irqsave(&gr_audio_lock, flags);
	if (!gr_audio_pending) {
		get_task_struct(task);
		gr_audio_pending = task;
		schedule_work(&gr_audio_work);
	}
	spin_unlock_irqrestore(&gr_audio_lock, flags);

	return 0;
}

static struct kprobe gr_audio_kp = {
	.symbol_name	= "__set_task_comm",
	.pre_handler	= gr_audio_probe,
};

static void gr_audio_latency_update(bool enabled)
{
	if (!gr_audio_probing)
		return;

	cpu_latency_qos_update_request(&gr_audio_latency,
			enabled ? (s32)gr_audio_latency_us : PM_QOS_DEFAULT_VALUE);
}

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%u\n", gr_audio_rt ? 1 : 0);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	bool value;

	if (kstrtobool(buf, &value))
		return -EINVAL;

	gr_audio_rt = value;
	gr_audio_latency_update(value);

	return count;
}

static ssize_t priority_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%u\n", gr_audio_prio);
}

static ssize_t priority_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || !value || value > 20)
		return -EINVAL;

	gr_audio_prio = value;

	return count;
}

static ssize_t latency_us_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", gr_audio_latency_us);
}

static ssize_t latency_us_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value > 1000)
		return -EINVAL;

	gr_audio_latency_us = value;
	gr_audio_latency_update(gr_audio_rt);

	return count;
}

static struct kobj_attribute enable_attr =
	__ATTR(enable, 0644, enable_show, enable_store);
static struct kobj_attribute priority_attr =
	__ATTR(priority, 0644, priority_show, priority_store);
static struct kobj_attribute latency_us_attr =
	__ATTR(latency_us, 0644, latency_us_show, latency_us_store);

static struct attribute *gr_audio_attrs[] = {
	&enable_attr.attr,
	&priority_attr.attr,
	&latency_us_attr.attr,
	NULL,
};

static struct attribute_group gr_audio_attr_group = {
	.attrs = gr_audio_attrs,
};

static int __init gr_audio_init(void)
{
	int ret;

	if (!gr_kobj)
		return -ENODEV;

	gr_audio_kobj = kobject_create_and_add("audio", gr_kobj);
	if (!gr_audio_kobj)
		return -ENOMEM;

	if (sysfs_create_group(gr_audio_kobj, &gr_audio_attr_group))
		return -ENOMEM;

	INIT_WORK(&gr_audio_work, gr_audio_apply);
	cpu_latency_qos_add_request(&gr_audio_latency, PM_QOS_DEFAULT_VALUE);

	ret = register_kprobe(&gr_audio_kp);
	if (ret) {
		pr_warn("audio: cannot probe __set_task_comm: %d\n", ret);
	} else {
		gr_audio_probing = true;
		if (gr_audio_rt)
			gr_audio_latency_update(true);
	}

	return 0;
}
late_initcall(gr_audio_init);
