// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens thermal frequency cap.
 *
 * Vendor thermal zones are named differently on every SoC, and new ones can
 * appear when a vendor module loads.  Zones are therefore discovered by
 * pattern over /sys/class/thermal rather than addressed by a fixed index, and
 * the caller can rescan and retarget at runtime.
 *
 * The cap is this driver's own freq_qos maximum request per policy, not the
 * one used by the screen clamp.  Two max requests compose the way the QoS
 * layer intends, so blanking the display and running hot at the same time
 * takes the lower of the two ceilings and releasing one leaves the other.
 *
 * The zones are only read from a periodic work item.  What must not happen,
 * and does not happen here, is a hook on thermal_zone_get_temp() or on any
 * scheduler path.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "grayravens.h"

#define GR_TC_DIR	"/sys/class/thermal"
#define GR_TC_PATTERNS	"skin,shell,batt,charger,gpu,soc,ap"
#define GR_TC_MAX_ZONES	8
#define GR_TC_MAX_POLICIES	8
#define GR_TC_NAME	32
#define GR_TC_THRESHOLD	45	/* degrees C */
#define GR_TC_PCT	30
#define GR_TC_INTERVAL	30000	/* ms */

struct gr_tc_policy {
	struct cpufreq_policy *policy;
	struct freq_qos_request max_req;
};

static DEFINE_MUTEX(gr_tc_lock);
static struct delayed_work gr_tc_work;
static struct gr_tc_policy gr_tc_policies[GR_TC_MAX_POLICIES];
static int gr_tc_policy_count;
static char gr_tc_patterns[128] = GR_TC_PATTERNS;
static char gr_tc_zones[GR_TC_MAX_ZONES][GR_TC_NAME];
static char gr_tc_types[GR_TC_MAX_ZONES][GR_TC_NAME];
static int gr_tc_zone_count;
static char gr_tc_zone_list[GR_TC_MAX_ZONES * GR_TC_NAME * 2];
static unsigned int gr_tc_threshold_c = GR_TC_THRESHOLD;
static unsigned int gr_tc_cap_pct = GR_TC_PCT;
static unsigned int gr_tc_interval_ms = GR_TC_INTERVAL;
static int gr_tc_hottest = -ENODATA;
static bool gr_tc_capped;
static bool gr_tc_enable;
static struct kobject *gr_tc_kobj;

struct gr_tc_scan {
	const char *patterns;
};

static int gr_tc_visit(const char *name, int len, void *arg)
{
	struct gr_tc_scan *s = arg;
	char entry[GR_TC_NAME], path[128], type[GR_TC_NAME];

	if (len <= 0 || len >= sizeof(entry))
		return 0;

	if (strncmp(name, "thermal_zone", 12) || len <= 12)
		return 0;

	memcpy(entry, name, len);
	entry[len] = '\0';

	snprintf(path, sizeof(path), GR_TC_DIR "/%s/type", entry);
	if (gr_read_file(path, type, sizeof(type)))
		return 0;

	if (!gr_match_list(type, s->patterns))
		return 0;

	if (gr_tc_zone_count >= GR_TC_MAX_ZONES)
		return 0;

	strscpy(gr_tc_zones[gr_tc_zone_count], entry, GR_TC_NAME);
	strscpy(gr_tc_types[gr_tc_zone_count], type, GR_TC_NAME);
	gr_tc_zone_count++;

	return 0;
}

/* Caller holds gr_tc_lock. */
static void gr_tc_do_scan(void)
{
	struct gr_tc_scan s = {
		.patterns = gr_tc_patterns,
	};

	gr_tc_zone_count = 0;
	gr_tc_zone_list[0] = '\0';

	if (gr_scan_dir(GR_TC_DIR, gr_tc_visit, &s))
		return;

	{
		int i;

		for (i = 0; i < gr_tc_zone_count; i++) {
			char item[GR_TC_NAME * 2 + 2];

			snprintf(item, sizeof(item), "%s:%s ",
				 gr_tc_zones[i], gr_tc_types[i]);

			if (strlen(gr_tc_zone_list) + strlen(item) + 1 <
			    sizeof(gr_tc_zone_list))
				strcat(gr_tc_zone_list, item);
		}
	}
}

static void gr_tc_register_policy(struct cpufreq_policy *policy)
{
	struct gr_tc_policy *tp;
	int i;

	for (i = 0; i < gr_tc_policy_count; i++)
		if (gr_tc_policies[i].policy == policy)
			return;

	if (gr_tc_policy_count >= ARRAY_SIZE(gr_tc_policies))
		return;

	tp = &gr_tc_policies[gr_tc_policy_count];

	if (freq_qos_add_request(&policy->constraints, &tp->max_req,
				 FREQ_QOS_MAX, FREQ_QOS_MAX_DEFAULT_VALUE))
		return;

	tp->policy = policy;
	gr_tc_policy_count++;
}

/* Caller holds gr_tc_lock. */
static void gr_tc_scan_policies(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (!policy)
			continue;
		gr_tc_register_policy(policy);
		cpufreq_cpu_put(policy);
	}
}

/* Caller holds gr_tc_lock. */
static void gr_tc_apply(bool capped)
{
	int i;

	gr_tc_scan_policies();

	for (i = 0; i < gr_tc_policy_count; i++) {
		struct gr_tc_policy *tp = &gr_tc_policies[i];
		s32 value = FREQ_QOS_MAX_DEFAULT_VALUE;

		if (capped)
			value = (s32)((u64)tp->policy->cpuinfo.max_freq *
				      gr_tc_cap_pct / 100);

		freq_qos_update_request(&tp->max_req, value);
	}

	gr_tc_capped = capped;
}

/*
 * Read through the zone's sysfs temp file rather than a cached zone pointer:
 * vendor modules come and go, types can repeat, and a stale pointer would be
 * a use-after-free for no gain.
 */
/* Caller holds gr_tc_lock. */
static int gr_tc_read_hottest(void)
{
	int hottest = -ENODATA;
	int i;

	for (i = 0; i < gr_tc_zone_count; i++) {
		char path[128];
		unsigned long milli = 0;
		int temp;

		snprintf(path, sizeof(path), GR_TC_DIR "/%s/temp", gr_tc_zones[i]);
		if (gr_read_file_ulong(path, &milli))
			continue;

		temp = (int)(milli / 1000);

		if (temp > hottest)
			hottest = temp;
	}

	return hottest;
}

static void gr_tc_work_fn(struct work_struct *work)
{
	mutex_lock(&gr_tc_lock);

	if (!gr_tc_enable || !gr_tc_zone_count)
		goto out;

	gr_tc_hottest = gr_tc_read_hottest();

	/*
	 * Act on a real reading only.  A sensor that failed to read must not
	 * release the cap, otherwise a hot SoC with one dead zone would come
	 * back to full frequency.
	 */
	if (gr_tc_hottest != -ENODATA) {
		bool capped = gr_tc_hottest >= (int)gr_tc_threshold_c;

		if (capped != gr_tc_capped)
			gr_tc_apply(capped);
	}
out:
	mutex_unlock(&gr_tc_lock);

	if (gr_tc_enable && gr_tc_zone_count)
		schedule_delayed_work(&gr_tc_work,
				      msecs_to_jiffies(gr_tc_interval_ms));
}

static void gr_tc_kick(void)
{
	cancel_delayed_work_sync(&gr_tc_work);

	if (gr_tc_enable && gr_tc_zone_count)
		schedule_delayed_work(&gr_tc_work,
				      msecs_to_jiffies(gr_tc_interval_ms));
}

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%d\n", gr_tc_enable ? 1 : 0);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	bool value;

	if (kstrtobool(buf, &value))
		return -EINVAL;

	mutex_lock(&gr_tc_lock);
	gr_tc_enable = value;

	if (!value) {
		if (gr_tc_capped)
			gr_tc_apply(false);
		gr_tc_hottest = -ENODATA;
	} else {
		gr_tc_do_scan();
	}
	mutex_unlock(&gr_tc_lock);

	gr_tc_kick();

	return count;
}

static ssize_t patterns_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%s\n", gr_tc_patterns);
}

static ssize_t patterns_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	char line[sizeof(gr_tc_patterns)];

	if (sscanf(buf, "%127s", line) != 1)
		return -EINVAL;

	mutex_lock(&gr_tc_lock);
	strscpy(gr_tc_patterns, line, sizeof(gr_tc_patterns));
	gr_tc_do_scan();
	mutex_unlock(&gr_tc_lock);

	gr_tc_kick();

	return count;
}

static ssize_t scan_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	mutex_lock(&gr_tc_lock);
	gr_tc_do_scan();
	mutex_unlock(&gr_tc_lock);

	gr_tc_kick();

	return count;
}

static ssize_t zones_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_tc_lock);
	gr_tc_do_scan();
	ret = sprintf(buf, "%s\n", gr_tc_zone_list[0] ? gr_tc_zone_list : "-");
	mutex_unlock(&gr_tc_lock);

	return ret;
}

static ssize_t state_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_tc_lock);
	ret = sprintf(buf, "capped=%d hottest=%d threshold=%u cap_pct=%u zones=%d\n",
		      gr_tc_capped ? 1 : 0, gr_tc_hottest,
		      gr_tc_threshold_c, gr_tc_cap_pct, gr_tc_zone_count);
	mutex_unlock(&gr_tc_lock);

	return ret;
}

static ssize_t threshold_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%u\n", gr_tc_threshold_c);
}

static ssize_t threshold_store(struct kobject *kobj, struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value < 20 || value > 110)
		return -EINVAL;

	mutex_lock(&gr_tc_lock);
	gr_tc_threshold_c = value;
	mutex_unlock(&gr_tc_lock);

	gr_tc_kick();

	return count;
}

static ssize_t cap_pct_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%u\n", gr_tc_cap_pct);
}

static ssize_t cap_pct_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value < 5 || value > 100)
		return -EINVAL;

	mutex_lock(&gr_tc_lock);
	gr_tc_cap_pct = value;
	mutex_unlock(&gr_tc_lock);

	gr_tc_kick();

	return count;
}

static ssize_t interval_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%u\n", gr_tc_interval_ms);
}

static ssize_t interval_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value < 1000 || value > 600000)
		return -EINVAL;

	mutex_lock(&gr_tc_lock);
	gr_tc_interval_ms = value;
	mutex_unlock(&gr_tc_lock);

	gr_tc_kick();

	return count;
}

static struct kobj_attribute enable_attr =
	__ATTR(enable, 0644, enable_show, enable_store);
static struct kobj_attribute patterns_attr =
	__ATTR(patterns, 0644, patterns_show, patterns_store);
static struct kobj_attribute zones_attr = __ATTR(zones, 0444, zones_show, NULL);
static struct kobj_attribute scan_attr = __ATTR(scan, 0200, NULL, scan_store);
static struct kobj_attribute state_attr = __ATTR(state, 0444, state_show, NULL);
static struct kobj_attribute threshold_attr =
	__ATTR(threshold_c, 0644, threshold_show, threshold_store);
static struct kobj_attribute cap_pct_attr =
	__ATTR(cap_pct, 0644, cap_pct_show, cap_pct_store);
static struct kobj_attribute interval_attr =
	__ATTR(interval_ms, 0644, interval_show, interval_store);

static struct attribute *gr_tc_attrs[] = {
	&enable_attr.attr,
	&patterns_attr.attr,
	&zones_attr.attr,
	&scan_attr.attr,
	&state_attr.attr,
	&threshold_attr.attr,
	&cap_pct_attr.attr,
	&interval_attr.attr,
	NULL,
};

static struct attribute_group gr_tc_attr_group = {
	.attrs = gr_tc_attrs,
};

static int __init gr_thermal_cap_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&gr_tc_work, gr_tc_work_fn);

	if (!gr_kobj)
		return -ENODEV;

	gr_tc_kobj = kobject_create_and_add("thermal", gr_kobj);
	if (!gr_tc_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(gr_tc_kobj, &gr_tc_attr_group);
	if (ret)
		return ret;

	/* Off until a zone matches, so an idle device pays no timer. */
	mutex_lock(&gr_tc_lock);
	gr_tc_do_scan();
	mutex_unlock(&gr_tc_lock);

	return 0;
}
late_initcall(gr_thermal_cap_init);
