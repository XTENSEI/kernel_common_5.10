// SPDX-License-Identifier: GPL-2.0
/*
 * thermal_charger_guard (霞ガード) -- auto-suppress Kasumi and Iyashi
 * while an external charger is online.  -- XTENSEI
 *
 * Background:
 *   Some Transsion userspace battery managers read post-Kasumi
 *   dampened thermal_zone temperatures and refuse to start charging
 *   until that reading falls inside an expected band.  Because Kasumi
 *   lags the real temperature on purpose, the band check spins and
 *   the device throws "won't charge until normal temps" warnings on a
 *   battery that is otherwise healthy.  Iyashi can produce a similar
 *   surprise for any userspace that watches cooling-device cur_state.
 *
 *   Pragmatic fix: whenever any MAINS / USB-family / WIRELESS power
 *   supply reports ONLINE, suppress both subsystems for the duration
 *   of the charging session.  Restore on unplug.
 *
 * Enhanced features:
 *   - Battery temperature awareness: when the battery is too hot,
 *     suppression is skipped so thermal protection stays active.
 *   - Debounce time is tunable via sysfs (debounce_ms).
 *   - Stats counters track suppression on/off and battery-temp skips.
 *
 * Layering:
 *   This module does NOT touch /sys/kernel/{kasumi,iyashi}/enabled.
 *   It carries an orthogonal "charger_suppressed" flag on each side.
 *   The effective active state is (enabled && !charger_suppressed).
 *   Userspace can still toggle `enabled` at will; the suppression
 *   flag is visible read-only at /sys/kernel/{kasumi,iyashi}/
 *   charger_suppressed for observability.
 *
 * Debouncing:
 *   PSY_EVENT_PROP_CHANGED fires often (battery SOC updates, ICL
 *   re-negotiation, temp polling) so the notifier itself just kicks
 *   a delayed_work with a configurable debounce.  The worker then walks
 *   every registered power_supply once and OR-reduces ONLINE across
 *   charger-typed psys to derive the aggregate state.  Battery-only events
 *   are ignored unless a charger is online or the battery-hot latch is set,
 *   because those events are the only way to notice that the safety gate
 *   should trip or clear while the cable stays plugged in.
 *
 * Battery temperature awareness:
 *   If the battery temperature exceeds batt_temp_thresh_decicelsius
 *   (default 500 = 50.0 °C), suppression is skipped for that refresh
 *   cycle.  This prevents the charger guard from disabling thermal
 *   protection when the battery itself is overheating — Kasumi/Iyashi
 *   are left active so the thermal framework can throttle.
 *
 * Boot ordering:
 *   We register at late_initcall, the same level as kasumi/iyashi
 *   sysfs init.  The setters are no-ops while the target's static
 *   storage is still all-zero, so an early notifier callback before
 *   the target's init runs is harmless -- it just writes 0 to a 0.
 *   An initial refresh is kicked from init() to pick up devices
 *   already plugged in when the kernel finishes booting.
 */

#define pr_fmt(fmt) "thermal_charger_guard: " fmt

#include <linux/device.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "iyashi.h"
#include "kasumi.h"

/* ---- Tunables ---- */

/*
 * Debounce interval in milliseconds.  Every PSY_PROP_CHANGED event
 * re-arms a delayed_work by this amount.  Writable via
 * /sys/module/thermal_charger_guard/parameters/debounce_ms.
 * Range 50..5000, default 250.
 */
static unsigned int debounce_ms = 250;
module_param(debounce_ms, uint, 0644);
MODULE_PARM_DESC(debounce_ms,
		 "Debounce interval (ms) for charger refresh; 50..5000");

/*
 * Battery temperature threshold in deci-Celsius.  When the battery
 * temperature exceeds this value, suppression is skipped (Kasumi and
 * Iyashi remain active so the thermal framework can protect the
 * hardware).  0 disables battery temperature awareness entirely
 * (suppression always applies when a charger is online).
 * Default 500 = 50.0 °C.
 */
static unsigned int batt_temp_thresh_decicelsius = 500;
module_param(batt_temp_thresh_decicelsius, uint, 0644);
MODULE_PARM_DESC(batt_temp_thresh_decicelsius,
		 "Battery temp threshold (deci-Celsius); "
		 "0 = disabled, default 500 = 50.0 C");

/*
 * Battery temperature hysteresis in deci-Celsius.  Once the battery
 * exceeds batt_temp_thresh_decicelsius, the temperature must fall
 * below (thresh - hyst) before suppression is re-enabled.  This
 * prevents rapid on/off flapping near the threshold.
 * Default 50 = 5.0 °C.
 */
static unsigned int batt_temp_hyst_decicelsius = 50;
module_param(batt_temp_hyst_decicelsius, uint, 0644);
MODULE_PARM_DESC(batt_temp_hyst_decicelsius,
		 "Battery temp hysteresis (deci-Celsius); default 50 = 5.0 C");

/* ---- Internal state ---- */

/*
 * Suppression tier, by charger type:
 *   0 = offline (no suppression)
 *   1 = standard charger (suppress Kasumi only -- fixes Transsion
 *       battery-manager band check on USB/DCP/MAINS)
 *   2 = fast / wireless charger (suppress both Kasumi and Iyashi)
 */
static unsigned int charger_raw_tier __read_mostly;
static unsigned int charger_tier __read_mostly;
static DEFINE_SPINLOCK(charger_state_lock);

/* Battery hot flag: true when battery temp exceeded threshold */
static bool batt_hot __read_mostly;

static struct delayed_work charger_refresh_work;

/* ---- Stats counters ---- */
static struct {
	unsigned long suppression_on;
	unsigned long suppression_off;
	unsigned long batt_temp_skip;
} stats;
static DEFINE_SPINLOCK(stats_lock);

/* ---- Helpers ---- */

struct charger_batt_walk {
	int temp;
	bool found;
};

static int charger_batt_temp_check(struct device *dev, void *data)
{
	struct charger_batt_walk *w = data;
	struct power_supply *psy = dev_get_drvdata(dev);
	union power_supply_propval val;

	if (!psy || !psy->desc ||
	    psy->desc->type != POWER_SUPPLY_TYPE_BATTERY)
		return 0;

	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val))
		return 0;

	w->temp = val.intval;
	w->found = true;
	return 1;
}

/*
 * Read the battery temperature in deci-Celsius from the first registered
 * BATTERY-type power supply that exposes POWER_SUPPLY_PROP_TEMP.  Returns
 * 0 on error or if the property is not available.
 */
static bool charger_read_batt_temp_decicelsius(int *temp)
{
	struct charger_batt_walk w = { };

	if (!temp || !power_supply_class)
		return false;

	class_for_each_device(power_supply_class, NULL, &w,
			      charger_batt_temp_check);
	if (!w.found)
		return false;

	*temp = w.temp; /* already in deci-Celsius */
	return true;
}

/*
 * Check whether the battery is too hot for suppression to be safe.
 * Returns true if suppression should be SKIPPED (i.e. keep Kasumi/
 * Iyashi active).
 */
static bool charger_batt_hot_check(void)
{
	int temp;
	unsigned int thresh = READ_ONCE(batt_temp_thresh_decicelsius);

	if (!thresh)
		return false; /* battery temp awareness disabled */

	if (!charger_read_batt_temp_decicelsius(&temp))
		return READ_ONCE(batt_hot); /* can't read; stay safe */

	if (READ_ONCE(batt_hot)) {
		/* Already hot: check if we've cooled below hyst */
		unsigned int hyst = READ_ONCE(batt_temp_hyst_decicelsius);
		int cool_thresh = (hyst >= thresh) ? 0 : (int)(thresh - hyst);

		if (temp < cool_thresh) {
			WRITE_ONCE(batt_hot, false);
			return false;
		}
		return true;
	}

	/* Not hot yet: check if we crossed the threshold */
	if (temp >= (int)thresh) {
		WRITE_ONCE(batt_hot, true);
		return true;
	}

	return false;
}

/*
 * Map a power_supply type to its suppression tier.
 * Non-charger types return 0 (ignored).
 */
static unsigned int psy_charger_tier(struct power_supply *psy)
{
	if (!psy || !psy->desc)
		return 0;

	switch (psy->desc->type) {
	/* Tier 2: fast charging / wireless */
	case POWER_SUPPLY_TYPE_USB_PD:
	case POWER_SUPPLY_TYPE_USB_PD_DRP:
	case POWER_SUPPLY_TYPE_WIRELESS:
		return 2;

	/* Tier 1: standard chargers (MAINS, USB, legacy) */
	case POWER_SUPPLY_TYPE_MAINS:
	case POWER_SUPPLY_TYPE_USB:
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB_ACA:
	case POWER_SUPPLY_TYPE_USB_TYPE_C:
	case POWER_SUPPLY_TYPE_APPLE_BRICK_ID:
		return 1;

	default:
		return 0;
	}
}

struct charger_walk {
	unsigned int max_tier;
};

static int charger_psy_check(struct device *dev, void *data)
{
	struct charger_walk *w = data;
	struct power_supply *psy = dev_get_drvdata(dev);
	union power_supply_propval val;
	unsigned int tier;

	tier = psy_charger_tier(psy);
	if (!tier)
		return 0;

	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val))
		return 0;

	if (val.intval && tier > w->max_tier)
		w->max_tier = tier;

	return 0;
}

static void charger_refresh(struct work_struct *work)
{
	struct charger_walk w = { .max_tier = 0 };
	unsigned int prev_tier;
	unsigned int tier;
	bool skip = false;

	if (class_for_each_device(power_supply_class, NULL, &w,
				  charger_psy_check))
		return;

	/*
	 * Check battery temperature before applying suppression.
	 * If the battery is too hot, we skip suppression so the
	 * thermal framework can still protect the hardware.
	 */
	if (w.max_tier > 0 && charger_batt_hot_check()) {
		skip = true;
		spin_lock(&stats_lock);
		stats.batt_temp_skip++;
		spin_unlock(&stats_lock);
	}

	spin_lock(&charger_state_lock);
	prev_tier = READ_ONCE(charger_tier);
	tier = skip ? 0 : w.max_tier;
	WRITE_ONCE(charger_raw_tier, w.max_tier);
	WRITE_ONCE(charger_tier, tier);
	spin_unlock(&charger_state_lock);

	if (prev_tier == tier)
		return; /* no change in effective state */

	/*
	 * Tier 1: suppress Kasumi only (fixes Transsion battery-manager
	 *          band check on USB/DCP/MAINS).
	 * Tier 2: suppress both Kasumi and Iyashi.
	 * Tier 0: restore both.
	 */
	if (tier > 0) {
		kasumi_set_charger_suppressed(true);
		iyashi_set_charger_suppressed(tier >= 2);
		spin_lock(&stats_lock);
		stats.suppression_on++;
		spin_unlock(&stats_lock);
		pr_info("charger tier %u active -> kasumi suppressed%s\n",
			tier, tier >= 2 ? ", iyashi suppressed" : "");
	} else {
		kasumi_set_charger_suppressed(false);
		iyashi_set_charger_suppressed(false);
		spin_lock(&stats_lock);
		stats.suppression_off++;
		spin_unlock(&stats_lock);
		pr_info("charger guard inactive -> kasumi restored, iyashi restored%s\n",
			skip ? " (battery temp threshold exceeded)" : "");
	}
}

static int charger_psy_notify(struct notifier_block *nb, unsigned long event,
			      void *data)
{
	struct power_supply *psy = data;

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_DONE;

	/*
	 * Battery-only psys never change our aggregate charger state,
	 * but their SOC / temperature props churn the most.  Filter
	 * them out to keep this off the hot path entirely.  Note: we
	 * still need battery temp reads for batt_hot_check, but those
	 * happen in the worker, not here.
	 */
	if (psy && psy->desc &&
	    psy->desc->type == POWER_SUPPLY_TYPE_BATTERY) {
		if (!READ_ONCE(batt_temp_thresh_decicelsius) ||
		    (!READ_ONCE(charger_raw_tier) && !READ_ONCE(batt_hot)))
			return NOTIFY_DONE;
	}

	mod_delayed_work(system_wq, &charger_refresh_work,
			 msecs_to_jiffies(READ_ONCE(debounce_ms)));
	return NOTIFY_OK;
}

static struct notifier_block charger_psy_nb = {
	.notifier_call = charger_psy_notify,
};

static int __init thermal_charger_guard_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&charger_refresh_work, charger_refresh);

	ret = power_supply_reg_notifier(&charger_psy_nb);
	if (ret) {
		pr_err("power_supply_reg_notifier failed: %d\n", ret);
		return ret;
	}

	/*
	 * Initial sweep: catch the case where the device was plugged in
	 * before boot finished, so the suppression latches correctly
	 * without waiting for the next prop change.
	 */
	mod_delayed_work(system_wq, &charger_refresh_work, 0);

	pr_info("registered (debounce=%ums batt_temp_thresh=%u.%uC hyst=%u.%uC)\n",
		READ_ONCE(debounce_ms),
		READ_ONCE(batt_temp_thresh_decicelsius) / 10,
		READ_ONCE(batt_temp_thresh_decicelsius) % 10,
		READ_ONCE(batt_temp_hyst_decicelsius) / 10,
		READ_ONCE(batt_temp_hyst_decicelsius) % 10);
	return 0;
}
late_initcall(thermal_charger_guard_init);

/* ---- sysfs interface (/sys/kernel/thermal_charger_guard/) ---- */

static ssize_t tier_show(struct kobject *kobj,
			 struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(charger_tier));
}

static ssize_t debounce_ms_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(debounce_ms));
}

static ssize_t debounce_ms_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 50 || val > 5000)
		return -EINVAL;

	WRITE_ONCE(debounce_ms, val);
	return count;
}

static ssize_t batt_temp_thresh_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(batt_temp_thresh_decicelsius));
}

static ssize_t batt_temp_thresh_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	/* 0 = disabled; 100..1000 deci-Celsius (10.0C..100.0C) */
	if (val != 0 && (val < 100 || val > 1000))
		return -EINVAL;

	WRITE_ONCE(batt_temp_thresh_decicelsius, val);
	mod_delayed_work(system_wq, &charger_refresh_work, 0);
	return count;
}

static ssize_t batt_temp_hyst_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(batt_temp_hyst_decicelsius));
}

static ssize_t batt_temp_hyst_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val > 200) /* max 20.0 C hysteresis */
		return -EINVAL;

	WRITE_ONCE(batt_temp_hyst_decicelsius, val);
	mod_delayed_work(system_wq, &charger_refresh_work, 0);
	return count;
}

static ssize_t stats_show(struct kobject *kobj,
			  struct kobj_attribute *attr, char *buf)
{
	unsigned long on, off, skip;

	spin_lock(&stats_lock);
	on  = stats.suppression_on;
	off = stats.suppression_off;
	skip = stats.batt_temp_skip;
	spin_unlock(&stats_lock);

	return sysfs_emit(buf,
			  "suppression_on   %lu\n"
			  "suppression_off  %lu\n"
			  "batt_temp_skip   %lu\n",
			  on, off, skip);
}

static struct kobj_attribute thermal_charger_guard_tier_attr =
	__ATTR(tier, 0444, tier_show, NULL);

static struct kobj_attribute thermal_charger_guard_debounce_attr =
	__ATTR(debounce_ms, 0644, debounce_ms_show, debounce_ms_store);

static struct kobj_attribute thermal_charger_guard_batt_temp_thresh_attr =
	__ATTR(batt_temp_thresh, 0644,
	       batt_temp_thresh_show, batt_temp_thresh_store);

static struct kobj_attribute thermal_charger_guard_batt_temp_hyst_attr =
	__ATTR(batt_temp_hyst, 0644,
	       batt_temp_hyst_show, batt_temp_hyst_store);

static struct kobj_attribute thermal_charger_guard_stats_attr =
	__ATTR(stats, 0444, stats_show, NULL);

static struct attribute *thermal_charger_guard_attrs[] = {
	&thermal_charger_guard_tier_attr.attr,
	&thermal_charger_guard_debounce_attr.attr,
	&thermal_charger_guard_batt_temp_thresh_attr.attr,
	&thermal_charger_guard_batt_temp_hyst_attr.attr,
	&thermal_charger_guard_stats_attr.attr,
	NULL,
};

static struct attribute_group thermal_charger_guard_attr_group = {
	.attrs = thermal_charger_guard_attrs,
};

static struct kobject *thermal_charger_guard_kobj;

static int __init thermal_charger_guard_sysfs_init(void)
{
	int ret;

	thermal_charger_guard_kobj =
		kobject_create_and_add("thermal_charger_guard", kernel_kobj);
	if (!thermal_charger_guard_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(thermal_charger_guard_kobj,
				 &thermal_charger_guard_attr_group);
	if (ret) {
		kobject_put(thermal_charger_guard_kobj);
		thermal_charger_guard_kobj = NULL;
	}

	return ret;
}

/* later_initcall_sync to guarantee kasumi/iyashi sysfs init ran first */
late_initcall_sync(thermal_charger_guard_sysfs_init);
