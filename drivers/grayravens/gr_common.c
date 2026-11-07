// SPDX-License-Identifier: GPL-2.0-only
/*
 * GrayRavens shared helpers.
 *
 * The helpers below write upstream sysfs knobs through the VFS instead of
 * touching the subsystems behind them.  A knob that does not exist is an
 * error the caller can ignore, which is how every per-SoC difference is
 * handled here: nothing is hardcoded to one platform's naming.
 */

#define pr_fmt(fmt) "grayravens: " fmt

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "grayravens.h"

int gr_read_file(const char *path, char *buf, size_t size)
{
	struct file *f;
	loff_t pos = 0;
	ssize_t n;

	if (!size)
		return -EINVAL;

	f = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	n = kernel_read(f, buf, size - 1, &pos);
	filp_close(f, NULL);

	if (n < 0)
		return n;

	buf[n] = '\0';
	strim(buf);

	return 0;
}

int gr_read_file_ulong(const char *path, unsigned long *val)
{
	char buf[64];
	int ret;

	ret = gr_read_file(path, buf, sizeof(buf));
	if (ret)
		return ret;

	return kstrtoul(buf, 0, val);
}

int gr_write_file(const char *path, const char *data)
{
	struct file *f;
	loff_t pos = 0;
	ssize_t n;

	f = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	n = kernel_write(f, data, strlen(data), &pos);
	filp_close(f, NULL);

	return n < 0 ? n : 0;
}

struct gr_dir_ctx {
	struct dir_context ctx;
	int (*visit)(const char *name, int len, void *arg);
	void *arg;
};

static int gr_dir_actor(struct dir_context *ctx, const char *name, int len,
			loff_t offset, u64 ino, unsigned int d_type)
{
	struct gr_dir_ctx *c = container_of(ctx, struct gr_dir_ctx, ctx);

	if (len <= 0 || (name[0] == '.' &&
			 (len == 1 || (len == 2 && name[1] == '.'))))
		return 0;

	if (c->visit)
		c->visit(name, len, c->arg);

	return 0;
}

int gr_scan_dir(const char *path,
		int (*visit)(const char *name, int len, void *arg), void *arg)
{
	struct gr_dir_ctx c = {
		.ctx.actor = gr_dir_actor,
		.visit = visit,
		.arg = arg,
	};
	struct file *f;
	int ret;

	f = filp_open(path, O_RDONLY | O_DIRECTORY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	ret = iterate_dir(f, &c.ctx);
	filp_close(f, NULL);

	return ret;
}

bool gr_match_list(const char *name, const char *patterns)
{
	const char *p = patterns;
	size_t nlen;

	if (!name || !patterns || !*patterns)
		return false;

	nlen = strlen(name);

	while (*p) {
		const char *end = strchr(p, ',');
		size_t len = end ? (size_t)(end - p) : strlen(p);
		char pat[32];
		size_t i;

		if (len && len < sizeof(pat)) {
			memcpy(pat, p, len);
			pat[len] = '\0';
			strim(pat);

			len = strlen(pat);
			for (i = 0; len && i + len <= nlen; i++) {
				if (!strncasecmp(name + i, pat, len))
					return true;
			}
		}

		if (!end)
			break;
		p = end + 1;
	}

	return false;
}
