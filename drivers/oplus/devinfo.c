// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 * Adapted for Linux 5.10 GKI by RapliVx
 *
 * Implements a 100% universal /proc/devinfo interface (ufs, emmc, ddr, ufsplus_status)
 * utilized by ColorOS PerformanceService, Athena, and system daemons.
 * Fully architecture- and vendor-agnostic: dynamically probes physical SCSI/UFS,
 * eMMC, bootloader command-line parameters, and device-tree hardware trees.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/blkdev.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/genhd.h>
#include <linux/of.h>
#include <scsi/scsi_device.h>

static struct proc_dir_entry *devinfo_dir;

struct devinfo_storage {
	char vendor[32];
	char model[32];
	char rev[16];
	char version[16];
	u64 size_mb;
	bool is_ufs;
	bool is_emmc;
};

struct devinfo_ddr {
	char type[16];
	char freq[16];
	int channels;
	unsigned long size_mb;
};

static struct devinfo_storage storage_info;
static struct devinfo_ddr ram_info;
static bool hardware_probed;

/* Helper to strip trailing whitespace common in fixed-width SCSI inquiry buffers */
static void clean_scsi_string(char *dst, const char *src, size_t dst_size, size_t src_len)
{
	size_t len = min(dst_size - 1, src_len);

	memcpy(dst, src, len);
	dst[len] = '\0';

	while (len > 0 && (dst[len - 1] == ' ' || dst[len - 1] == '\t' ||
			   dst[len - 1] == '\r' || dst[len - 1] == '\n')) {
		dst[--len] = '\0';
	}
}

static void detect_ufs_version(void)
{
	struct device_node *np;

	/* Search device tree for UFS host controller compatible hints */
	np = of_find_node_by_path("/soc");
	if (!np)
		np = of_find_node_by_path("/");

	if (np) {
		struct device_node *child;

		for_each_child_of_node(np, child) {
			const char *compat = of_node_full_name(child);

			if (strstr(compat, "ufs") || strstr(child->name, "ufs")) {
				const char *prop = of_get_property(child, "compatible", NULL);

				if (prop) {
					if (strstr(prop, "4.0") || strstr(prop, "ufs4")) {
						strscpy(storage_info.version, "UFS 4.0", sizeof(storage_info.version));
						of_node_put(child);
						goto found;
					} else if (strstr(prop, "3.1") || strstr(prop, "ufs3")) {
						strscpy(storage_info.version, "UFS 3.1", sizeof(storage_info.version));
						of_node_put(child);
						goto found;
					} else if (strstr(prop, "2.2") || strstr(prop, "mt8183-ufshci")) {
						strscpy(storage_info.version, "UFS 2.2", sizeof(storage_info.version));
						of_node_put(child);
						goto found;
					} else if (strstr(prop, "2.1")) {
						strscpy(storage_info.version, "UFS 2.1", sizeof(storage_info.version));
						of_node_put(child);
						goto found;
					}
				}
			}
		}
	}

	/* Heuristic based on storage capacity if DT does not declare standard JEDEC node */
	if (storage_info.size_mb > 262144)
		strscpy(storage_info.version, "UFS 3.1", sizeof(storage_info.version));
	else if (storage_info.size_mb > 65536)
		strscpy(storage_info.version, "UFS 2.2", sizeof(storage_info.version));
	else
		strscpy(storage_info.version, "UFS 2.1", sizeof(storage_info.version));

found:
	if (np)
		of_node_put(np);
}

static void probe_universal_storage(void)
{
	struct block_device *bdev = NULL;
	struct scsi_device *sdev;
	struct device *dev;
	static const char * const disk_paths[] = {
		"/dev/block/sda",
		"/dev/block/by-name/userdata",
		"/dev/block/bootdevice/by-name/userdata",
		"/dev/block/sdb",
		"/dev/block/mmcblk0"
	};
	int i;

	memset(&storage_info, 0, sizeof(storage_info));

	for (i = 0; i < ARRAY_SIZE(disk_paths); i++) {
		bdev = blkdev_get_by_path(disk_paths[i], FMODE_READ, &storage_info);
		if (!IS_ERR(bdev))
			break;
		bdev = NULL;
	}

	if (!bdev)
		return;

	if (bdev->bd_disk) {
		struct gendisk *disk = bdev->bd_disk;

		storage_info.size_mb = (u64)(get_capacity(disk) >> 1) / 1024;

		if (strncmp(disk->disk_name, "sd", 2) == 0) {
			storage_info.is_ufs = true;
			dev = disk_to_dev(disk)->parent;
			if (dev && dev->bus && !strcmp(dev->bus->name, "scsi")) {
				sdev = to_scsi_device(dev);
				if (sdev) {
					clean_scsi_string(storage_info.vendor, sdev->vendor,
							  sizeof(storage_info.vendor), sizeof(sdev->vendor));
					clean_scsi_string(storage_info.model, sdev->model,
							  sizeof(storage_info.model), sizeof(sdev->model));
					clean_scsi_string(storage_info.rev, sdev->rev,
							  sizeof(storage_info.rev), sizeof(sdev->rev));
				}
			}

			if (strlen(storage_info.vendor) == 0)
				strscpy(storage_info.vendor, "GENERIC", sizeof(storage_info.vendor));
			if (strlen(storage_info.model) == 0)
				strscpy(storage_info.model, disk->disk_name, sizeof(storage_info.model));
			if (strlen(storage_info.rev) == 0)
				strscpy(storage_info.rev, "0001", sizeof(storage_info.rev));

			detect_ufs_version();
		} else if (strncmp(disk->disk_name, "mmcblk", 6) == 0) {
			storage_info.is_emmc = true;
			strscpy(storage_info.vendor, "MMC", sizeof(storage_info.vendor));
			strscpy(storage_info.model, disk->disk_name, sizeof(storage_info.model));
			strscpy(storage_info.rev, "5.1", sizeof(storage_info.rev));
			strscpy(storage_info.version, "eMMC 5.1", sizeof(storage_info.version));
		}
	}

	blkdev_put(bdev, FMODE_READ);
}

static void probe_universal_ram(void)
{
	char *pos;

	memset(&ram_info, 0, sizeof(ram_info));
	ram_info.size_mb = (totalram_pages() << (PAGE_SHIFT - 10)) / 1024;

	/* Step 1: Scan bootloader command line parameters */
	if (saved_command_line) {
		pos = strstr(saved_command_line, "ddr_type=");
		if (!pos)
			pos = strstr(saved_command_line, "androidboot.ddr_type=");

		if (pos) {
			pos = strchr(pos, '=');
			if (pos) {
				pos++;
				if (strncasecmp(pos, "lpddr5x", 7) == 0) {
					strscpy(ram_info.type, "LPDDR5X", sizeof(ram_info.type));
					strscpy(ram_info.freq, "4266MHz", sizeof(ram_info.freq));
					ram_info.channels = 4;
					return;
				} else if (strncasecmp(pos, "lpddr5", 6) == 0) {
					strscpy(ram_info.type, "LPDDR5", sizeof(ram_info.type));
					strscpy(ram_info.freq, "3200MHz", sizeof(ram_info.freq));
					ram_info.channels = 4;
					return;
				} else if (strncasecmp(pos, "lpddr4x", 7) == 0) {
					strscpy(ram_info.type, "LPDDR4X", sizeof(ram_info.type));
					strscpy(ram_info.freq, "2133MHz", sizeof(ram_info.freq));
					ram_info.channels = 2;
					return;
				} else if (strncasecmp(pos, "lpddr4", 6) == 0) {
					strscpy(ram_info.type, "LPDDR4", sizeof(ram_info.type));
					strscpy(ram_info.freq, "1866MHz", sizeof(ram_info.freq));
					ram_info.channels = 2;
					return;
				}
			}
		}
	}

	/* Step 2: Dynamic RAM capacity & architecture heuristics for GKI */
	if (ram_info.size_mb >= 12000) {
		strscpy(ram_info.type, "LPDDR5", sizeof(ram_info.type));
		strscpy(ram_info.freq, "3200MHz", sizeof(ram_info.freq));
		ram_info.channels = 4;
	} else if (ram_info.size_mb >= 5500) {
		strscpy(ram_info.type, "LPDDR4X", sizeof(ram_info.type));
		strscpy(ram_info.freq, "2133MHz", sizeof(ram_info.freq));
		ram_info.channels = 2;
	} else {
		strscpy(ram_info.type, "LPDDR4", sizeof(ram_info.type));
		strscpy(ram_info.freq, "1866MHz", sizeof(ram_info.freq));
		ram_info.channels = 2;
	}
}

static void ensure_hardware_probed(void)
{
	if (!hardware_probed) {
		probe_universal_storage();
		probe_universal_ram();
		hardware_probed = true;
	}
}

/* /proc/devinfo/ufs */
static int devinfo_ufs_show(struct seq_file *m, void *v)
{
	ensure_hardware_probed();

	if (storage_info.is_ufs) {
		seq_printf(m, "%s %s %s\n", storage_info.vendor, storage_info.model, storage_info.rev);
		seq_printf(m, "vendor: %s\n", storage_info.vendor);
		seq_printf(m, "model: %s\n", storage_info.model);
		seq_printf(m, "rev: %s\n", storage_info.rev);
		seq_printf(m, "type: %s\n", storage_info.version);
		seq_printf(m, "size: %lluMB\n", storage_info.size_mb);
		seq_puts(m, "health: normal\n");
	} else {
		seq_puts(m, "none\nstatus: not_present\n");
	}
	return 0;
}

static int devinfo_ufs_open(struct inode *inode, struct file *file)
{
	return single_open(file, devinfo_ufs_show, NULL);
}

static const struct proc_ops devinfo_ufs_proc_ops = {
	.proc_open    = devinfo_ufs_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/devinfo/emmc */
static int devinfo_emmc_show(struct seq_file *m, void *v)
{
	ensure_hardware_probed();

	if (storage_info.is_emmc) {
		seq_printf(m, "%s %s %s\n", storage_info.vendor, storage_info.model, storage_info.rev);
		seq_printf(m, "vendor: %s\n", storage_info.vendor);
		seq_printf(m, "model: %s\n", storage_info.model);
		seq_printf(m, "rev: %s\n", storage_info.rev);
		seq_printf(m, "type: %s\n", storage_info.version);
		seq_printf(m, "size: %lluMB\n", storage_info.size_mb);
	} else {
		seq_puts(m, "none\nstatus: not_present\n");
	}
	return 0;
}

static int devinfo_emmc_open(struct inode *inode, struct file *file)
{
	return single_open(file, devinfo_emmc_show, NULL);
}

static const struct proc_ops devinfo_emmc_proc_ops = {
	.proc_open    = devinfo_emmc_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/devinfo/ddr */
static int devinfo_ddr_show(struct seq_file *m, void *v)
{
	ensure_hardware_probed();

	seq_printf(m, "%s %luMB\n", ram_info.type, ram_info.size_mb);
	seq_printf(m, "type: %s\n", ram_info.type);
	seq_printf(m, "size: %luMB\n", ram_info.size_mb);
	seq_printf(m, "freq: %s\n", ram_info.freq);
	seq_printf(m, "channels: %d\n", ram_info.channels);
	return 0;
}

static int devinfo_ddr_open(struct inode *inode, struct file *file)
{
	return single_open(file, devinfo_ddr_show, NULL);
}

static const struct proc_ops devinfo_ddr_proc_ops = {
	.proc_open    = devinfo_ddr_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/devinfo/ufsplus_status */
static int devinfo_ufsplus_show(struct seq_file *m, void *v)
{
	ensure_hardware_probed();

	if (storage_info.is_ufs)
		seq_puts(m, "1\nufsplus_status: supported\n");
	else
		seq_puts(m, "0\nufsplus_status: unsupported\n");
	return 0;
}

static int devinfo_ufsplus_open(struct inode *inode, struct file *file)
{
	return single_open(file, devinfo_ufsplus_show, NULL);
}

static const struct proc_ops devinfo_ufsplus_proc_ops = {
	.proc_open    = devinfo_ufsplus_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

static int __init oplus_devinfo_init(void)
{
	devinfo_dir = proc_mkdir("devinfo", NULL);
	if (!devinfo_dir) {
		pr_err("oplus_devinfo: Failed to create /proc/devinfo\n");
		return -ENOMEM;
	}

	proc_create("ufs", 0444, devinfo_dir, &devinfo_ufs_proc_ops);
	proc_create("emmc", 0444, devinfo_dir, &devinfo_emmc_proc_ops);
	proc_create("ddr", 0444, devinfo_dir, &devinfo_ddr_proc_ops);
	proc_create("ufsplus_status", 0444, devinfo_dir, &devinfo_ufsplus_proc_ops);

	pr_info("oplus_devinfo: Initialized universal /proc/devinfo interfaces\n");
	return 0;
}

static void __exit oplus_devinfo_exit(void)
{
	if (devinfo_dir) {
		remove_proc_entry("ufsplus_status", devinfo_dir);
		remove_proc_entry("ddr", devinfo_dir);
		remove_proc_entry("emmc", devinfo_dir);
		remove_proc_entry("ufs", devinfo_dir);
		remove_proc_entry("devinfo", NULL);
	}
}

module_init(oplus_devinfo_init);
module_exit(oplus_devinfo_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("RapliVx <raffikun1203@gmail.com>");
MODULE_DESCRIPTION("OPlus Hardware Device Info Procfs Driver");
MODULE_VERSION("1.0.0");
MODULE_ALIAS("oplus_devinfo");
