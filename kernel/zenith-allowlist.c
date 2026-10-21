// SPDX-License-Identifier: GPL-2.0
/*
 * kernel/zenith-allowlist.c — GrayRavens Device Boot Allowlist
 *
 * Only allows the kernel to boot on explicitly allowlisted devices.
 * On unlisted devices, the kernel panics early in boot with a clear
 * message.  This prevents the kernel from being flashed on unsupported
 * hardware where drivers, DTBs, or power profiles may be incompatible.
 *
 * The device match is done by reading the serialno property from the
 * device tree (/firmware/android/serialno), which is a unique per-device
 * string set by the bootloader.  The match is case-insensitive and
 * supports substring matching.
 *
 * Inspired by Kanagawa Yamada's "Yamada is Angry" blocklist in SuiKernel
 * (drivers/Yamada/yamada_is_angry.c).  Turned into an allowlist to match
 * the GrayRavens use-case: one kernel image per target device.
 *
 * Sysfs: /sys/kernel/zenith-allowlist/
 *   device        (RO) — configured allowlisted device string
 *   enabled       (RW) — 0/1 toggle (default 1; set to 0 to allow all)
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/string.h>

/* ------------------------------------------------------------------ */
/* Configuration — set via Kconfig or override here                    */
/* ------------------------------------------------------------------ */

#define ZENITH_ALLOWLIST_DEVICE		CONFIG_ZENITH_ALLOWLIST_DEVICE_STRING

/* ------------------------------------------------------------------ */
/* Sysfs interface                                                     */
/* ------------------------------------------------------------------ */

static bool allowlist_enabled __read_mostly = true;

static ssize_t device_show(struct kobject *kobj,
			   struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, ZENITH_ALLOWLIST_DEVICE "\n");
}

static ssize_t enabled_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", allowlist_enabled);
}

static ssize_t enabled_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val) < 0)
		return -EINVAL;

	allowlist_enabled = !!val;
	return count;
}

static struct kobj_attribute device_attr = __ATTR_RO(device);
static struct kobj_attribute enabled_attr = __ATTR_RW(enabled);

static struct attribute *zenith_allowlist_attrs[] = {
	&device_attr.attr,
	&enabled_attr.attr,
	NULL,
};

static struct attribute_group zenith_allowlist_attr_group = {
	.attrs = zenith_allowlist_attrs,
};

static struct kobject *zenith_allowlist_kobj;

/* ------------------------------------------------------------------ */
/* Helper — case-insensitive substring search                         */
/* ------------------------------------------------------------------ */

static const char *stristr(const char *haystack, const char *needle)
{
	size_t hl, nl;

	if (!haystack || !needle)
		return NULL;

	nl = strlen(needle);
	if (!nl)
		return haystack;

	hl = strlen(haystack);
	while (hl >= nl) {
		if (!strncasecmp(haystack, needle, nl))
			return haystack;
		haystack++;
		hl--;
	}
	return NULL;
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

static int __init zenith_allowlist_init(void)
{
	const char *serialno = NULL;
	struct device_node *np;
	int ret;

	/*
	 * Sysfs — only create if kernel_kobj is available (it may not be
	 * at early_initcall level since kernel_init_freeable() sets it up
	 * as another early_initcall with undefined ordering).  The panic
	 * check below runs unconditionally regardless of sysfs state.
	 */
	zenith_allowlist_kobj = NULL;
	if (kernel_kobj) {
		zenith_allowlist_kobj = kobject_create_and_add("zenith-allowlist",
							       kernel_kobj);
		if (zenith_allowlist_kobj) {
			ret = sysfs_create_group(zenith_allowlist_kobj,
						 &zenith_allowlist_attr_group);
			if (ret) {
				kobject_put(zenith_allowlist_kobj);
				zenith_allowlist_kobj = NULL;
				pr_warn("zenith-allowlist: sysfs creation failed\n");
			}
		} else {
			pr_warn("zenith-allowlist: sysfs kobject creation failed\n");
		}
	} else {
		pr_info("zenith-allowlist: sysfs deferred (kernel_kobj not ready)\n");
	}

	/* If disabled via sysfs (or Kconfig), skip check */
	if (!allowlist_enabled) {
		pr_info("zenith-allowlist: disabled — allowing all devices\n");
		return 0;
	}

	/* Read serialno from device tree (/firmware/android/serialno) */
	np = of_find_node_by_path("/firmware/android");
	if (np) {
		of_property_read_string(np, "serialno", &serialno);
		of_node_put(np);
	}

	if (!serialno) {
		pr_warn("zenith-allowlist: no serialno found in DT; "
			"allowing boot (module loaded after DT init)\n");
		return 0;
	}

	pr_info("zenith-allowlist: device serialno: %s\n", serialno);
	pr_info("zenith-allowlist: allowlisted device: %s\n",
		ZENITH_ALLOWLIST_DEVICE);

	/* Check if device is allowlisted */
	if (stristr(serialno, ZENITH_ALLOWLIST_DEVICE)) {
		pr_info("zenith-allowlist: device matched — boot allowed\n");
	} else {
		pr_emerg("============================================\n");
		pr_emerg(" GrayRavens Kernel is ALLOWLISTED!\n");
		pr_emerg(" This kernel only boots on devices matching:\n");
		pr_emerg("   \"%s\"\n", ZENITH_ALLOWLIST_DEVICE);
		pr_emerg(" Current device serialno: %s\n", serialno);
		pr_emerg("============================================\n");
		panic("zenith-allowlist: device not in allowlist");
	}

	return 0;
}

early_initcall(zenith_allowlist_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens Team");
MODULE_DESCRIPTION("GrayRavens Device Boot Allowlist");
