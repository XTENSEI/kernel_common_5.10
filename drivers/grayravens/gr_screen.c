// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens screen-off frequency ceiling.
 *
 * While the display is blanked, nothing needs the top operating points: the
 * compositor is idle and the remaining work is background.  Cap every cpufreq
 * policy at a fraction of its maximum and lift the cap as soon as the display
 * comes back.
 *
 * The trigger is drm_atomic_connector_commit_dpms(), the connector-level
 * path every atomic DRM driver goes through when the DPMS property changes.
 * That is an edge-triggered hook, so unlike polling a backlight or DPMS sysfs
 * node there is no timer, no device-specific path, and no work at all while
 * the screen state is steady.
 *
 * The ceiling is a freq_qos maximum request: it can only lower what the
 * governor already chose, never raise it.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <drm/drm_modes.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/kprobes.h>
#include <linux/ptrace.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "grayravens.h"

static bool gr_screen_clamp = true;
static unsigned int gr_screen_pct = CONFIG_GRAYRAVENS_SCREEN_PCT;
static bool gr_screen_off;

static DEFINE_SPINLOCK(gr_screen_lock);
static bool gr_screen_pending;
static bool gr_screen_have_pending;
static struct work_struct gr_screen_work;
static struct kobject *gr_screen_kobj;

static void gr_screen_apply(struct work_struct *work)
{
	bool off;
	unsigned long flags;

	spin_lock_irqsave(&gr_screen_lock, flags);
	if (!gr_screen_have_pending) {
		spin_unlock_irqrestore(&gr_screen_lock, flags);
		return;
	}
	off = gr_screen_pending;
	gr_screen_have_pending = false;
	spin_unlock_irqrestore(&gr_screen_lock, flags);

	gr_screen_off = off;
	gr_set_freq_ceiling(off && gr_screen_clamp ? gr_screen_pct : 0);
}

static void gr_screen_queue(bool off)
{
	unsigned long flags;

	spin_lock_irqsave(&gr_screen_lock, flags);
	if (!gr_screen_have_pending || gr_screen_pending != off) {
		gr_screen_pending = off;
		gr_screen_have_pending = true;
		schedule_work(&gr_screen_work);
	}
	spin_unlock_irqrestore(&gr_screen_lock, flags);
}

static int gr_screen_probe(struct kprobe *kp, struct pt_regs *regs)
{
	int mode;

	if (!READ_ONCE(gr_screen_clamp))
		return 0;

	mode = (int)regs_get_kernel_argument(regs, 2);

	/* The core folds everything but ON into OFF before it commits. */
	gr_screen_queue(mode != DRM_MODE_DPMS_ON);

	return 0;
}

static struct kprobe gr_screen_kp = {
	.symbol_name	= "drm_atomic_connector_commit_dpms",
	.pre_handler	= gr_screen_probe,
};

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%u\n", gr_screen_clamp ? 1 : 0);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	bool value;

	if (kstrtobool(buf, &value))
		return -EINVAL;

	gr_screen_clamp = value;
	gr_set_freq_ceiling(value && gr_screen_off ? gr_screen_pct : 0);

	return count;
}

static ssize_t cap_pct_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%u\n", gr_screen_pct);
}

static ssize_t cap_pct_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value < 5 || value > 100)
		return -EINVAL;

	gr_screen_pct = value;
	if (gr_screen_clamp && gr_screen_off)
		gr_set_freq_ceiling(value);

	return count;
}

static ssize_t screen_off_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", gr_screen_off ? 1 : 0);
}

static struct kobj_attribute enable_attr =
	__ATTR(enable, 0644, enable_show, enable_store);
static struct kobj_attribute cap_pct_attr =
	__ATTR(cap_pct, 0644, cap_pct_show, cap_pct_store);
static struct kobj_attribute screen_off_attr =
	__ATTR_RO(screen_off);

static struct attribute *gr_screen_attrs[] = {
	&enable_attr.attr,
	&cap_pct_attr.attr,
	&screen_off_attr.attr,
	NULL,
};

static struct attribute_group gr_screen_attr_group = {
	.attrs = gr_screen_attrs,
};

static int __init gr_screen_init(void)
{
	int ret;

	if (!gr_kobj)
		return -ENODEV;

	gr_screen_kobj = kobject_create_and_add("screen", gr_kobj);
	if (!gr_screen_kobj)
		return -ENOMEM;

	if (sysfs_create_group(gr_screen_kobj, &gr_screen_attr_group))
		return -ENOMEM;

	INIT_WORK(&gr_screen_work, gr_screen_apply);

	ret = register_kprobe(&gr_screen_kp);
	if (ret) {
		pr_warn("screen: cannot probe DRM DPMS: %d\n", ret);
		return 0;
	}

	return 0;
}
late_initcall(gr_screen_init);
