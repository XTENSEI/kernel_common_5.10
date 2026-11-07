// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens cpuidle governor knob.
 *
 * Choosing menu or teo normally means rebuilding the kernel or editing a
 * file that is global and permanent.  This exposes the same switch with a
 * timed boost, so a scene can ask for teo and get menu back afterwards,
 * plus an explicit apply for when the change really is a preference.
 *
 * Nothing here touches cpuidle internals: the governor switch is the
 * upstream sysfs attribute, driven through the VFS.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "grayravens.h"

#define GR_IG_DIR	"/sys/devices/system/cpu/cpuidle/"
#define GR_IG_NAME	32
#define GR_IG_HOLD	30000	/* ms */

static DEFINE_MUTEX(gr_ig_lock);
static struct delayed_work gr_ig_restore_work;
static char gr_ig_saved[GR_IG_NAME];
static char gr_ig_target[GR_IG_NAME] = "teo";
static unsigned int gr_ig_hold_ms = GR_IG_HOLD;
static bool gr_ig_active;
static struct kobject *gr_ig_kobj;

static bool gr_ig_available(const char *name)
{
	char list[128], *tok, *p;
	bool found = false;

	if (gr_read_file(GR_IG_DIR "available_governors", list, sizeof(list)))
		return false;

	p = list;
	while ((tok = strsep(&p, " \t\n")) != NULL) {
		if (*tok && !strcmp(tok, name)) {
			found = true;
			break;
		}
	}

	return found;
}

static int gr_ig_current(char *buf, size_t size)
{
	return gr_read_file(GR_IG_DIR "current_governor_ro", buf, size);
}

static int gr_ig_switch(const char *name)
{
	char line[GR_IG_NAME + 2];
	int ret;

	if (!gr_ig_available(name))
		return -EINVAL;

	snprintf(line, sizeof(line), "%s\n", name);

	ret = gr_write_file(GR_IG_DIR "current_governor", line);
	if (ret)
		return ret;

	pr_info("idle_gov: governor is now %s\n", name);

	return 0;
}

/* Caller holds gr_ig_lock. */
static int gr_ig_do_restore(void)
{
	if (!gr_ig_active)
		return 0;

	gr_ig_active = false;

	if (!gr_ig_saved[0])
		return 0;

	return gr_ig_switch(gr_ig_saved);
}

static void gr_ig_restore_work_fn(struct work_struct *work)
{
	mutex_lock(&gr_ig_lock);
	gr_ig_do_restore();
	mutex_unlock(&gr_ig_lock);
}

static ssize_t boost_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	char name[GR_IG_NAME], cur[GR_IG_NAME];
	int ret;

	if (sscanf(buf, "%31s", name) != 1)
		return -EINVAL;

	mutex_lock(&gr_ig_lock);

	ret = gr_ig_current(cur, sizeof(cur));
	if (ret)
		goto out;

	/* Switching to the governor that is already running is a no-op. */
	if (strcmp(cur, name)) {
		ret = gr_ig_switch(name);
		if (ret)
			goto out;
	}

	strscpy(gr_ig_saved, cur, sizeof(gr_ig_saved));
	gr_ig_active = true;
	ret = count;

	mod_delayed_work(system_wq, &gr_ig_restore_work,
			 msecs_to_jiffies(READ_ONCE(gr_ig_hold_ms)));
out:
	mutex_unlock(&gr_ig_lock);

	return ret;
}

static ssize_t apply_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	char name[GR_IG_NAME];
	int ret;

	if (sscanf(buf, "%31s", name) != 1)
		return -EINVAL;

	cancel_delayed_work_sync(&gr_ig_restore_work);

	mutex_lock(&gr_ig_lock);
	gr_ig_active = false;
	gr_ig_saved[0] = '\0';
	ret = gr_ig_switch(name);
	mutex_unlock(&gr_ig_lock);

	return ret ? ret : count;
}

static ssize_t restore_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	int ret;

	cancel_delayed_work_sync(&gr_ig_restore_work);

	mutex_lock(&gr_ig_lock);
	ret = gr_ig_do_restore();
	mutex_unlock(&gr_ig_lock);

	return ret ? ret : count;
}

static ssize_t governor_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%s\n", gr_ig_target);
}

static ssize_t governor_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	char name[GR_IG_NAME];

	if (sscanf(buf, "%31s", name) != 1)
		return -EINVAL;

	if (!gr_ig_available(name))
		return -EINVAL;

	strscpy(gr_ig_target, name, sizeof(gr_ig_target));

	return count;
}

static ssize_t hold_ms_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(gr_ig_hold_ms));
}

static ssize_t hold_ms_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;

	WRITE_ONCE(gr_ig_hold_ms, value);

	return count;
}

static ssize_t available_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	int ret;

	ret = gr_read_file(GR_IG_DIR "available_governors", buf, PAGE_SIZE);
	if (ret)
		return ret;

	strcat(buf, "\n");

	return strlen(buf);
}

static ssize_t effective_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	int ret;

	ret = gr_ig_current(buf, PAGE_SIZE);
	if (ret)
		return ret;

	strcat(buf, "\n");

	return strlen(buf);
}

static ssize_t active_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_ig_lock);
	ret = sprintf(buf, "%d %s\n", gr_ig_active ? 1 : 0,
		      gr_ig_active ? gr_ig_saved : "-");
	mutex_unlock(&gr_ig_lock);

	return ret;
}

static struct kobj_attribute boost_attr = __ATTR(boost, 0200, NULL, boost_store);
static struct kobj_attribute apply_attr = __ATTR(apply, 0200, NULL, apply_store);
static struct kobj_attribute restore_attr =
	__ATTR(restore, 0200, NULL, restore_store);
static struct kobj_attribute governor_attr =
	__ATTR(governor, 0644, governor_show, governor_store);
static struct kobj_attribute hold_attr =
	__ATTR(hold_ms, 0644, hold_ms_show, hold_ms_store);
static struct kobj_attribute available_attr =
	__ATTR(available, 0444, available_show, NULL);
static struct kobj_attribute effective_attr =
	__ATTR(effective, 0444, effective_show, NULL);
static struct kobj_attribute active_attr = __ATTR(active, 0444, active_show, NULL);

static struct attribute *gr_ig_attrs[] = {
	&boost_attr.attr,
	&apply_attr.attr,
	&restore_attr.attr,
	&governor_attr.attr,
	&hold_attr.attr,
	&available_attr.attr,
	&effective_attr.attr,
	&active_attr.attr,
	NULL,
};

static struct attribute_group gr_ig_attr_group = {
	.attrs = gr_ig_attrs,
};

static int __init gr_idle_gov_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&gr_ig_restore_work, gr_ig_restore_work_fn);

	if (!gr_kobj)
		return -ENODEV;

	gr_ig_kobj = kobject_create_and_add("idle_gov", gr_kobj);
	if (!gr_ig_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(gr_ig_kobj, &gr_ig_attr_group);
	if (ret)
		return ret;

	return 0;
}
late_initcall(gr_idle_gov_init);
