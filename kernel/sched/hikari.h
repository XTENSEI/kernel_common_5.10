/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Hikari wake-latency vruntime shift -- internal scheduler header.
 *
 * Pulled in from kernel/sched/sched.h so the fair-class hook site has the
 * prototype available without needing local forward declarations.
 *
 * See Documentation/scheduler/sched-hikari.rst for the design rationale
 * and the tunable surface (kernel.sched_hikari_shift,
 * kernel.sched_hikari_ewma_shift).
 */
#ifndef _KERNEL_SCHED_HIKARI_H
#define _KERNEL_SCHED_HIKARI_H

struct cfs_rq;
struct sched_entity;

#ifdef CONFIG_SCHED_HIKARI

extern unsigned int sysctl_sched_hikari_shift;
extern unsigned int sysctl_sched_hikari_ewma_shift;
extern unsigned int sysctl_sched_hikari_topapp_boost;

void hikari_apply_wake_shift(struct cfs_rq *cfs_rq, struct sched_entity *se);

#else /* !CONFIG_SCHED_HIKARI */

static inline void
hikari_apply_wake_shift(struct cfs_rq *cfs_rq, struct sched_entity *se) { }

#endif /* CONFIG_SCHED_HIKARI */

#endif /* _KERNEL_SCHED_HIKARI_H */
