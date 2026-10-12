// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 * Adapted for Linux 5.10 GKI by RapliVx
 *
 * Implements the /proc/oplus_afs_config interface required by
 * ColorOS Athena and SystemUI for application anti-freeze configuration.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>

#define AFS_MAX_CONFIG_SIZE (64 * 1024)

static struct proc_dir_entry *afs_dir;
static bool afs_enabled = true;
static DEFINE_MUTEX(afs_lock);

static char *afs_config_buf;
static size_t afs_config_len;

/* /proc/oplus_afs_config/afs_enable */
static int afs_enable_show(struct seq_file *m, void *v)
{
	mutex_lock(&afs_lock);
	seq_printf(m, "%s\n", afs_enabled ? "true" : "false");
	mutex_unlock(&afs_lock);
	return 0;
}

static int afs_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, afs_enable_show, NULL);
}

static ssize_t afs_enable_write(struct file *filp, const char __user *ubuf,
				size_t cnt, loff_t *ppos)
{
	char kbuf[16];

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	mutex_lock(&afs_lock);
	if (sysfs_streq(kbuf, "1") || sysfs_streq(kbuf, "true"))
		afs_enabled = true;
	else if (sysfs_streq(kbuf, "0") || sysfs_streq(kbuf, "false"))
		afs_enabled = false;
	mutex_unlock(&afs_lock);

	return cnt;
}

static const struct proc_ops afs_enable_proc_ops = {
	.proc_open    = afs_enable_open,
	.proc_read    = seq_read,
	.proc_write   = afs_enable_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_afs_config/afs_config */
static ssize_t afs_config_read(struct file *file, char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	ssize_t ret;

	mutex_lock(&afs_lock);
	if (!afs_config_buf || afs_config_len == 0) {
		mutex_unlock(&afs_lock);
		return 0;
	}

	ret = simple_read_from_buffer(ubuf, count, ppos, afs_config_buf, afs_config_len);
	mutex_unlock(&afs_lock);
	return ret;
}

static ssize_t afs_config_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	if (count > AFS_MAX_CONFIG_SIZE)
		return -EFBIG;

	mutex_lock(&afs_lock);
	if (!afs_config_buf) {
		afs_config_buf = kzalloc(AFS_MAX_CONFIG_SIZE, GFP_KERNEL);
		if (!afs_config_buf) {
			mutex_unlock(&afs_lock);
			return -ENOMEM;
		}
	}

	if (copy_from_user(afs_config_buf, ubuf, count)) {
		mutex_unlock(&afs_lock);
		return -EFAULT;
	}

	afs_config_len = count;
	*ppos += count;
	mutex_unlock(&afs_lock);

	return count;
}

static const struct proc_ops afs_config_proc_ops = {
	.proc_read    = afs_config_read,
	.proc_write   = afs_config_write,
	.proc_lseek   = default_llseek,
};

static int __init oplus_afs_config_init(void)
{
	afs_dir = proc_mkdir("oplus_afs_config", NULL);
	if (!afs_dir) {
		pr_err("oplus_afs_config: Failed to create /proc/oplus_afs_config\n");
		return -ENOMEM;
	}

	proc_create("afs_enable", 0644, afs_dir, &afs_enable_proc_ops);
	proc_create("afs_config", 0644, afs_dir, &afs_config_proc_ops);

	pr_info("oplus_afs_config: Anti-freeze interfaces initialized\n");
	return 0;
}

static void __exit oplus_afs_config_exit(void)
{
	if (afs_dir) {
		remove_proc_entry("afs_config", afs_dir);
		remove_proc_entry("afs_enable", afs_dir);
		remove_proc_entry("oplus_afs_config", NULL);
	}

	mutex_lock(&afs_lock);
	kfree(afs_config_buf);
	afs_config_buf = NULL;
	afs_config_len = 0;
	mutex_unlock(&afs_lock);
}

module_init(oplus_afs_config_init);
module_exit(oplus_afs_config_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RapliVx <raffikun1203@gmail.com>");
MODULE_DESCRIPTION("OPlus Application Anti-Freeze Service Config Driver");
MODULE_VERSION("1.0.0");
MODULE_ALIAS("oplus_afs_config");
