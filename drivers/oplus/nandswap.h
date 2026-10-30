/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#ifndef _NAND_SWAP_H
#define _NAND_SWAP_H

#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/pagewalk.h>

/* must check if this flag is repeat in <include/linux/mm.h> */
#define VM_NANDSWAP	0x8000000000UL	/* swapin mark */

#define AID_APP		10000		/* first app user */
#define AID_USER_OFFSET	100000		/* offset for uid ranges for each user */

enum {
	NS_OUT_STANDBY,
	NS_OUT_QUEUE,
	NS_OUT_CACHE,
	NS_OUT_DONE,
	NS_IN_QUEUE,
};

enum {
	NS_TYPE_FG,
	NS_TYPE_NAND_ACT,
	NS_TYPE_NAND_ALL,
	NS_TYPE_DROP,
	NS_TYPE_RATIO,
	NS_TYPE_END
};

extern int swapin_walk_pmd_entry(pmd_t *pmd, unsigned long start,
				 unsigned long end, struct mm_walk *walk);
extern unsigned long nswap_reclaim_page_list(struct list_head *page_list,
					     struct vm_area_struct *vma, bool scan);
extern int isolate_lru_page(struct page *page);
extern void putback_lru_page(struct page *page);
#endif /* _NAND_SWAP_H */