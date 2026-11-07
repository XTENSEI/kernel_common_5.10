// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens readahead boost.
 *
 * These loads stall on storage rather than on frequency, so a named app gets
 * a larger readahead window on the device that backs its executable while it
 * is starting up or streaming, and the previous value comes back afterwards.
 *
 * The block device is resolved from the task itself, so sda, sdb, mmcblk0
 * and dm-N are all handled without a table and without a per-platform path.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/blkdev.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/pid.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "grayravens.h"

#define GR_RA_TARGET_DEFAULT	512	/* KiB */
#define GR_RA_HOLD_DEFAULT	10000	/* ms */
#define GR_RA_DISK_NAME		32

static DEFINE_MUTEX(gr_ra_lock);
static struct delayed_work gr_ra_release_work;
static char gr_ra_disk[GR_RA_DISK_NAME];
static unsigned int gr_ra_saved_kb;
static unsigned int gr_ra_target_kb = GR_RA_TARGET_DEFAULT;
static unsigned int gr_ra_hold_ms = GR_RA_HOLD_DEFAULT;
static bool gr_ra_active;
static struct kobject *gr_ra_kobj;

static void gr_ra_queue_path(char *path, size_t size, const char *suffix)
{
	snprintf(path, size, "/sys/block/%s/queue/%s", gr_ra_disk, suffix);
}

/* Caller holds gr_ra_lock. */
static void gr_ra_do_release(void)
{
	char path[128], buf[16];

	if (!gr_ra_active)
		return;

	gr_ra_active = false;

	gr_ra_queue_path(path, sizeof(path), "read_ahead_kb");
	{
		unsigned long cur = 0;

		if (gr_read_file_ulong(path, &cur) || cur != gr_ra_target_kb) {
			/* Someone else owns the knob now, do not fight them. */
			pr_info("readahead: %s changed outside the helper, leaving it\n",
				path);
			goto out;
		}
	}

	snprintf(buf, sizeof(buf), "%u\n", gr_ra_saved_kb);
	gr_write_file(path, buf);
	pr_info("readahead: %s restored to %u KiB\n", gr_ra_disk, gr_ra_saved_kb);
out:
	gr_ra_disk[0] = '\0';
}

static void gr_ra_release_work_fn(struct work_struct *work)
{
	mutex_lock(&gr_ra_lock);
	gr_ra_do_release();
	mutex_unlock(&gr_ra_lock);
}

static int gr_ra_disk_of(struct task_struct *task, char *disk, size_t size)
{
	struct mm_struct *mm;
	struct file *exe;
	struct block_device *bdev;
	int ret = -ENODEV;

	mm = get_task_mm(task);
	if (!mm)
		return -ENODEV;

	mmap_read_lock(mm);
	exe = mm->exe_file;
	if (exe)
		get_file(exe);
	mmap_read_unlock(mm);
	mmput(mm);

	if (!exe)
		return -ENODEV;

	bdev = file_inode(exe)->i_sb->s_bdev;
	if (bdev && bdev->bd_disk &&
	    strscpy(disk, bdev->bd_disk->disk_name, size) > 0)
		ret = 0;

	fput(exe);

	return ret;
}

static ssize_t boost_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	struct task_struct *task;
	char disk[GR_RA_DISK_NAME];
	char path[128], val[16];
	unsigned long saved;
	unsigned int target;
	int ret, pid;

	if (kstrtoint(buf, 0, &pid) || pid <= 0)
		return -EINVAL;

	task = find_get_task_by_vpid(pid);
	if (!task)
		return -ESRCH;

	ret = gr_ra_disk_of(task, disk, sizeof(disk));
	put_task_struct(task);
	if (ret)
		return ret;

	mutex_lock(&gr_ra_lock);
	target = READ_ONCE(gr_ra_target_kb);

	if (gr_ra_active)
		gr_ra_do_release();

	strscpy(gr_ra_disk, disk, sizeof(gr_ra_disk));

	gr_ra_queue_path(path, sizeof(path), "read_ahead_kb");
	ret = gr_read_file_ulong(path, &saved);
	if (ret) {
		gr_ra_disk[0] = '\0';
		mutex_unlock(&gr_ra_lock);
		return ret;
	}

	snprintf(val, sizeof(val), "%u\n", target);
	ret = gr_write_file(path, val);
	if (ret) {
		gr_ra_disk[0] = '\0';
		mutex_unlock(&gr_ra_lock);
		return ret;
	}

	gr_ra_saved_kb = saved;
	gr_ra_active = true;

	mod_delayed_work(system_wq, &gr_ra_release_work,
			 msecs_to_jiffies(READ_ONCE(gr_ra_hold_ms)));
	mutex_unlock(&gr_ra_lock);

	return count;
}

static ssize_t release_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	cancel_delayed_work_sync(&gr_ra_release_work);

	mutex_lock(&gr_ra_lock);
	gr_ra_do_release();
	mutex_unlock(&gr_ra_lock);

	return count;
}

static ssize_t active_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	ssize_t ret;

	mutex_lock(&gr_ra_lock);
	ret = sprintf(buf, "%d %s\n", gr_ra_active ? 1 : 0,
		      gr_ra_active ? gr_ra_disk : "-");
	mutex_unlock(&gr_ra_lock);

	return ret;
}

static ssize_t target_kb_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(gr_ra_target_kb));
}

static ssize_t target_kb_store(struct kobject *kobj, struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value > 4096)
		return -EINVAL;

	WRITE_ONCE(gr_ra_target_kb, value);

	return count;
}

static ssize_t hold_ms_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(gr_ra_hold_ms));
}

static ssize_t hold_ms_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;

	WRITE_ONCE(gr_ra_hold_ms, value);

	return count;
}

static struct kobj_attribute boost_attr = __ATTR(boost, 0200, NULL, boost_store);
static struct kobj_attribute release_attr =
	__ATTR(release, 0200, NULL, release_store);
static struct kobj_attribute active_attr = __ATTR(active, 0444, active_show, NULL);
static struct kobj_attribute target_attr =
	__ATTR(target_kb, 0644, target_kb_show, target_kb_store);
static struct kobj_attribute hold_attr =
	__ATTR(hold_ms, 0644, hold_ms_show, hold_ms_store);

static struct attribute *gr_ra_attrs[] = {
	&boost_attr.attr,
	&release_attr.attr,
	&active_attr.attr,
	&target_attr.attr,
	&hold_attr.attr,
	NULL,
};

static struct attribute_group gr_ra_attr_group = {
	.attrs = gr_ra_attrs,
};

static int __init gr_readahead_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&gr_ra_release_work, gr_ra_release_work_fn);

	if (!gr_kobj)
		return -ENODEV;

	gr_ra_kobj = kobject_create_and_add("readahead", gr_kobj);
	if (!gr_ra_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(gr_ra_kobj, &gr_ra_attr_group);
	if (ret)
		return ret;

	return 0;
}
late_initcall(gr_readahead_init);
