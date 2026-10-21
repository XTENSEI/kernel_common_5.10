// SPDX-License-Identifier: GPL-2.0
/*
 * koakuma — GrayRavens Game Preloader
 *
 * Listens to selene's notifier chain.  When a game starts, walks the
 * game process's memory mappings and preloads file-backed pages into
 * the page cache using page_cache_sync_readahead().  This reduces
 * game loading time by warming the cache before the game actually
 * needs those pages.
 *
 * The preload runs asynchronously via a workqueue so the game launch
 * is not blocked.
 *
 * Sysfs: /sys/kernel/koakuma/
 *   enabled           (RW) — 0/1 master switch (default 1)
 *   max_preload_mb    (RW) — max. MB to preload per game session (default 64)
 *   last_preload_mb   (RO) — how many MB were preloaded last time
 */
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/sched/mm.h>
#include <linux/sched/signal.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/selene.h>

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static DEFINE_MUTEX(koakuma_lock);
static bool koakuma_enabled = true;
static unsigned int koakuma_max_preload_mb = 64;
static unsigned long koakuma_last_preloaded_bytes;

static struct work_struct koakuma_preload_work;
static struct task_struct *koakuma_game_task;
static char koakuma_game_name[128];

/* ------------------------------------------------------------------ */
/* Notifier — we listen to selene events                              */
/* ------------------------------------------------------------------ */
static void koakuma_preload_work_fn(struct work_struct *work)
{
	struct task_struct *task;
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	unsigned long total_preloaded = 0;
	unsigned long max_bytes = koakuma_max_preload_mb * 1024UL * 1024UL;

	if (!koakuma_enabled)
		return;

	/* Take a ref under the mutex so koakuma_game_task doesn't vanish */
	mutex_lock(&koakuma_lock);
	task = koakuma_game_task;
	if (task)
		get_task_struct(task);
	mutex_unlock(&koakuma_lock);

	if (!task)
		return;

	mm = get_task_mm(task);
	if (!mm) {
		/* Game task already exited -- release the ref we took. */
		put_task_struct(task);
		return;
	}

	mmap_read_lock(mm);

	for (vma = mm->mmap; vma && total_preloaded < max_bytes;
	     vma = vma->vm_next) {
		struct file *file = vma->vm_file;
		pgoff_t start_index;
		unsigned long nr_pages, bytes;
		struct file_ra_state ra;

		if (!file || !file->f_mapping)
			continue;

		/* Only preload executable and read-only data mappings */
		if (!(vma->vm_flags & (VM_EXEC | VM_READ)))
			continue;

		/* Skip writable private mappings (they're COW copies) */
		if (vma->vm_flags & VM_WRITE)
			continue;

		bytes = vma->vm_end - vma->vm_start;
		if (bytes == 0)
			continue;

		/*
		 * Skip tiny mappings (<64 KB).  Games often have thousands
		 * of small guard-page / vvar / vdso-ish VMAs that are not
		 * worth preloading -- each one triggers a fresh readahead
		 * call and adds I/O queue pressure without meaningful benefit.
		 * The 64 KB threshold matches a typical huge-page size.
		 */
		if (bytes < SZ_64K)
			continue;

		/* Cap to remaining budget */
		if (total_preloaded + bytes > max_bytes)
			bytes = max_bytes - total_preloaded;

		start_index = vma->vm_pgoff;
		nr_pages = bytes >> PAGE_SHIFT;

		/* Preload with fresh ra_state */
		file_ra_state_init(&ra, file->f_mapping);
		page_cache_sync_readahead(file->f_mapping, &ra, file,
					  start_index, nr_pages);

		total_preloaded += bytes;
	}

	mmap_read_unlock(mm);
	mmput(mm);

	koakuma_last_preloaded_bytes = total_preloaded;

	pr_info("GrayRavens: koakuma: preloaded %lu KB for '%s'\n",
		total_preloaded / 1024, koakuma_game_name);

	pr_debug("koakuma: preloaded %lu KB for %s\n",
		 total_preloaded / 1024, koakuma_game_name);

	put_task_struct(task);
}

static int koakuma_selene_notifier(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	if (!koakuma_enabled)
		return NOTIFY_OK;

	switch (event) {
	case SELENE_EVENT_GAME_START: {
		const char *name = data ? (const char *)data : "";
		struct task_struct *p;
		bool found = false;

		/* Find the game process by comm/cmdline match */
		rcu_read_lock();
		for_each_process(p) {
			char comm[TASK_COMM_LEN];
			get_task_comm(comm, p);
			if (strstr(name, comm) ||
			    strstr(comm, name)) {
				get_task_struct(p);
				found = true;
				break;
			}
		}
		rcu_read_unlock();

		if (!found)
			break;

		mutex_lock(&koakuma_lock);
		if (koakuma_game_task)
			put_task_struct(koakuma_game_task);
		koakuma_game_task = p;
		strscpy(koakuma_game_name, name,
			sizeof(koakuma_game_name));
		mutex_unlock(&koakuma_lock);

		schedule_work(&koakuma_preload_work);
		pr_info("GrayRavens: koakuma: GAME_START '%s': preload queued (max %u MB)\n",
			name, koakuma_max_preload_mb);
		break;
	}
	case SELENE_EVENT_GAME_STOP:
		mutex_lock(&koakuma_lock);
		if (koakuma_game_task) {
			put_task_struct(koakuma_game_task);
			koakuma_game_task = NULL;
		}
		koakuma_game_name[0] = '\0';
		mutex_unlock(&koakuma_lock);
		pr_info("GrayRavens: koakuma: GAME_STOP: preload cancelled\n");
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block koakuma_nb = {
	.notifier_call = koakuma_selene_notifier,
};

/* ------------------------------------------------------------------ */
/* Sysfs                                                               */
/* ------------------------------------------------------------------ */
static struct kobject *koakuma_kobj;

static ssize_t enabled_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", koakuma_enabled);
}

static ssize_t enabled_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	koakuma_enabled = !!val;
	return count;
}
static struct kobj_attribute enabled_attr =
	__ATTR_RW(enabled);

static ssize_t max_preload_mb_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", koakuma_max_preload_mb);
}

static ssize_t max_preload_mb_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (val > 1024)
		return -ERANGE;
	koakuma_max_preload_mb = val;
	return count;
}
static struct kobj_attribute max_preload_mb_attr =
	__ATTR_RW(max_preload_mb);

static ssize_t last_preload_mb_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%lu\n",
			  koakuma_last_preloaded_bytes / (1024 * 1024));
}
static struct kobj_attribute last_preload_mb_attr =
	__ATTR_RO(last_preload_mb);

static struct attribute *koakuma_attrs[] = {
	&enabled_attr.attr,
	&max_preload_mb_attr.attr,
	&last_preload_mb_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(koakuma);

/* ------------------------------------------------------------------ */
/* Init / Exit                                                         */
/* ------------------------------------------------------------------ */
static int __init koakuma_init(void)
{
	int ret;

	koakuma_kobj = kobject_create_and_add("koakuma", kernel_kobj);
	if (!koakuma_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(koakuma_kobj, koakuma_groups);
	if (ret) {
		kobject_put(koakuma_kobj);
		return ret;
	}

	INIT_WORK(&koakuma_preload_work, koakuma_preload_work_fn);

	{
		int sret = selene_register_notifier(&koakuma_nb);

		if (sret)
			pr_warn("koakuma: selene notifier register failed (%d)\n", sret);
		else
			pr_info("koakuma: subscribed to selene game detection chain\n");
	}

	pr_info("koakuma: loaded (max preload %u MB)\n",
		koakuma_max_preload_mb);
	return 0;
}

static void __exit koakuma_exit(void)
{
	selene_unregister_notifier(&koakuma_nb);
	cancel_work_sync(&koakuma_preload_work);

	mutex_lock(&koakuma_lock);
	if (koakuma_game_task) {
		put_task_struct(koakuma_game_task);
		koakuma_game_task = NULL;
	}
	mutex_unlock(&koakuma_lock);

	sysfs_remove_groups(koakuma_kobj, koakuma_groups);
	kobject_put(koakuma_kobj);

	pr_info("koakuma: unloaded\n");
}

module_init(koakuma_init);
module_exit(koakuma_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Koakuma — GrayRavens Game Preloader");
MODULE_AUTHOR("GrayRavens");
