// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens GPU frequency floor.
 *
 * The GPU devfreq node is named after its platform address on some SoCs and
 * after the driver on others, and it only exists once the vendor module has
 * probed.  So the node is discovered by pattern instead of being hardcoded,
 * and the caller can pin one explicitly when several nodes look alike.
 *
 * The floor is a write to the upstream min_freq attribute, which devfreq
 * already implements as a QoS request, so the governor keeps deciding above
 * it.  Locking the GPU to a frequency is avoided: only the floor moves.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "grayravens.h"

#define GR_GPU_DIR	"/sys/class/devfreq"
#define GR_GPU_PATTERNS	"mali,kgsl,ged,gpu,-3d"
#define GR_GPU_NAME	64
#define GR_GPU_PCT	70
#define GR_GPU_HOLD	30000	/* ms */

struct gr_gpu_scan {
	const char *patterns;
	char found[GR_GPU_NAME * 4];
	int count;
};

static DEFINE_MUTEX(gr_gpu_lock);
static struct delayed_work gr_gpu_release_work;
static char gr_gpu_device[GR_GPU_NAME];
static char gr_gpu_patterns[128] = GR_GPU_PATTERNS;
static char gr_gpu_available[GR_GPU_NAME * 4];
static unsigned int gr_gpu_pct = GR_GPU_PCT;
static unsigned int gr_gpu_hold_ms = GR_GPU_HOLD;
static unsigned long gr_gpu_saved_min;
static unsigned long gr_gpu_set_min;
static int gr_gpu_scan_count;
static bool gr_gpu_active;
static struct kobject *gr_gpu_kobj;

static int gr_gpu_visit(const char *name, int len, void *arg)
{
	struct gr_gpu_scan *s = arg;
	char entry[GR_GPU_NAME], path[128], devname[GR_GPU_NAME];

	if (len <= 0 || len >= sizeof(entry))
		return 0;

	memcpy(entry, name, len);
	entry[len] = '\0';

	snprintf(path, sizeof(path), GR_GPU_DIR "/%s/name", entry);
	if (gr_read_file(path, devname, sizeof(devname)))
		return 0;

	if (!gr_match_list(entry, s->patterns) &&
	    !gr_match_list(devname, s->patterns))
		return 0;

	if (strlen(s->found) + strlen(entry) + 2 < sizeof(s->found)) {
		strcat(s->found, entry);
		strcat(s->found, " ");
	}
	s->count++;

	return 0;
}

/* Caller holds gr_gpu_lock. */
static void gr_gpu_do_scan(void)
{
	struct gr_gpu_scan s = {
		.patterns = gr_gpu_patterns,
	};

	gr_gpu_available[0] = '\0';

	if (gr_scan_dir(GR_GPU_DIR, gr_gpu_visit, &s))
		return;

	strscpy(gr_gpu_available, s.found, sizeof(gr_gpu_available));
	gr_gpu_scan_count = s.count;
}

static int gr_gpu_resolve(char *out, size_t size)
{
	int ret;

	if (gr_gpu_device[0]) {
		strscpy(out, gr_gpu_device, size);
		return 0;
	}

	gr_gpu_do_scan();

	if (gr_gpu_scan_count != 1)
		return -ENODEV;

	/* The scan left exactly one name followed by a space. */
	ret = sscanf(gr_gpu_available, "%63s", out);

	return ret == 1 ? 0 : -ENODEV;
}

/* Caller holds gr_gpu_lock. */
static void gr_gpu_do_release(void)
{
	char path[128], buf[24];
	unsigned long cur = 0;

	if (!gr_gpu_active)
		return;

	gr_gpu_active = false;

	snprintf(path, sizeof(path), GR_GPU_DIR "/%s/min_freq", gr_gpu_device);

	if (gr_read_file_ulong(path, &cur) || cur != gr_gpu_set_min) {
		pr_info("gpu_floor: %s changed outside the helper, leaving it\n",
			path);
		return;
	}

	snprintf(buf, sizeof(buf), "%lu\n", gr_gpu_saved_min);
	gr_write_file(path, buf);
}

static void gr_gpu_release_work_fn(struct work_struct *work)
{
	mutex_lock(&gr_gpu_lock);
	gr_gpu_do_release();
	mutex_unlock(&gr_gpu_lock);
}

static ssize_t boost_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	char dev[GR_GPU_NAME], path[128], val[24];
	unsigned long max_freq, min_freq;
	int ret;

	mutex_lock(&gr_gpu_lock);

	ret = gr_gpu_resolve(dev, sizeof(dev));
	if (ret)
		goto out;

	snprintf(path, sizeof(path), GR_GPU_DIR "/%s/max_freq", dev);
	ret = gr_read_file_ulong(path, &max_freq);
	if (ret)
		goto out;

	snprintf(path, sizeof(path), GR_GPU_DIR "/%s/min_freq", dev);
	ret = gr_read_file_ulong(path, &min_freq);
	if (ret)
		goto out;

	if (gr_gpu_active)
		gr_gpu_do_release();

	strscpy(gr_gpu_device, dev, sizeof(gr_gpu_device));

	gr_gpu_saved_min = min_freq;
	gr_gpu_set_min = max_freq * READ_ONCE(gr_gpu_pct) / 100;
	if (gr_gpu_set_min < min_freq)
		gr_gpu_set_min = min_freq;

	snprintf(val, sizeof(val), "%lu\n", gr_gpu_set_min);
	ret = gr_write_file(path, val);
	if (ret)
		goto out;

	gr_gpu_active = true;
	ret = count;

	mod_delayed_work(system_wq, &gr_gpu_release_work,
			 msecs_to_jiffies(READ_ONCE(gr_gpu_hold_ms)));
out:
	mutex_unlock(&gr_gpu_lock);

	return ret;
}

static ssize_t release_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	cancel_delayed_work_sync(&gr_gpu_release_work);

	mutex_lock(&gr_gpu_lock);
	gr_gpu_do_release();
	mutex_unlock(&gr_gpu_lock);

	return count;
}

static ssize_t device_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_gpu_lock);
	ret = sprintf(buf, "%s\n", gr_gpu_device[0] ? gr_gpu_device : "auto");
	mutex_unlock(&gr_gpu_lock);

	return ret;
}

static ssize_t device_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	char name[GR_GPU_NAME], path[128], devname[GR_GPU_NAME];
	bool auto_sel;

	if (sscanf(buf, "%63s", name) != 1)
		return -EINVAL;

	auto_sel = !strcmp(name, "auto");

	if (!auto_sel) {
		snprintf(path, sizeof(path), GR_GPU_DIR "/%s/name", name);
		if (gr_read_file(path, devname, sizeof(devname)))
			return -ENODEV;
	}

	mutex_lock(&gr_gpu_lock);
	strscpy(gr_gpu_device, auto_sel ? "" : name, sizeof(gr_gpu_device));
	mutex_unlock(&gr_gpu_lock);

	return count;
}

static ssize_t patterns_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%s\n", gr_gpu_patterns);
}

static ssize_t patterns_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	char line[sizeof(gr_gpu_patterns)];

	if (sscanf(buf, "%127s", line) != 1)
		return -EINVAL;

	mutex_lock(&gr_gpu_lock);
	strscpy(gr_gpu_patterns, line, sizeof(gr_gpu_patterns));
	gr_gpu_do_scan();
	mutex_unlock(&gr_gpu_lock);

	return count;
}

static ssize_t scan_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	mutex_lock(&gr_gpu_lock);
	gr_gpu_do_scan();
	mutex_unlock(&gr_gpu_lock);

	return count;
}

static ssize_t available_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_gpu_lock);
	gr_gpu_do_scan();
	ret = sprintf(buf, "%s\n", gr_gpu_available[0] ? gr_gpu_available : "-");
	mutex_unlock(&gr_gpu_lock);

	return ret;
}

static ssize_t active_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_gpu_lock);
	if (gr_gpu_active)
		ret = sprintf(buf, "1 %s min=%lu\n", gr_gpu_device,
			      gr_gpu_set_min);
	else
		ret = sprintf(buf, "0 -\n");
	mutex_unlock(&gr_gpu_lock);

	return ret;
}

static ssize_t pct_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(gr_gpu_pct));
}

static ssize_t pct_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value < 1 || value > 100)
		return -EINVAL;

	WRITE_ONCE(gr_gpu_pct, value);

	return count;
}

static ssize_t hold_ms_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(gr_gpu_hold_ms));
}

static ssize_t hold_ms_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;

	WRITE_ONCE(gr_gpu_hold_ms, value);

	return count;
}

static struct kobj_attribute boost_attr = __ATTR(boost, 0200, NULL, boost_store);
static struct kobj_attribute release_attr =
	__ATTR(release, 0200, NULL, release_store);
static struct kobj_attribute device_attr =
	__ATTR(device, 0644, device_show, device_store);
static struct kobj_attribute patterns_attr =
	__ATTR(patterns, 0644, patterns_show, patterns_store);
static struct kobj_attribute scan_attr = __ATTR(scan, 0200, NULL, scan_store);
static struct kobj_attribute available_attr =
	__ATTR(available, 0444, available_show, NULL);
static struct kobj_attribute active_attr = __ATTR(active, 0444, active_show, NULL);
static struct kobj_attribute pct_attr = __ATTR(pct, 0644, pct_show, pct_store);
static struct kobj_attribute hold_attr =
	__ATTR(hold_ms, 0644, hold_ms_show, hold_ms_store);

static struct attribute *gr_gpu_attrs[] = {
	&boost_attr.attr,
	&release_attr.attr,
	&device_attr.attr,
	&patterns_attr.attr,
	&scan_attr.attr,
	&available_attr.attr,
	&active_attr.attr,
	&pct_attr.attr,
	&hold_attr.attr,
	NULL,
};

static struct attribute_group gr_gpu_attr_group = {
	.attrs = gr_gpu_attrs,
};

static int __init gr_gpu_floor_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&gr_gpu_release_work, gr_gpu_release_work_fn);

	if (!gr_kobj)
		return -ENODEV;

	gr_gpu_kobj = kobject_create_and_add("gpu", gr_kobj);
	if (!gr_gpu_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(gr_gpu_kobj, &gr_gpu_attr_group);
	if (ret)
		return ret;

	return 0;
}
late_initcall(gr_gpu_floor_init);
