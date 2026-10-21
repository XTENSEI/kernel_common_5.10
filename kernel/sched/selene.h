/* SPDX-License-Identifier: GPL-2.0 */
/*
 * selene.h — Internal header for GrayRavens Game Detection Engine
 *
 * Peer drivers (lucid, koakuma, etc.) include this header to
 * register for game start/stop notifications.  No EXPORT_SYMBOL
 * is needed because everything is built into the kernel image (=y).
 */
#ifndef _SELENE_H
#define _SELENE_H

#include <linux/notifier.h>
#include <linux/types.h>

/* Notifier event values */
#define SELENE_EVENT_GAME_START		1
#define SELENE_EVENT_GAME_STOP		0

/**
 * selene_register_notifier - Register for game state change events
 * @nb: notifier block to register
 *
 * Returns 0 on success, negative errno on failure.
 */
int selene_register_notifier(struct notifier_block *nb);

/**
 * selene_unregister_notifier - Unregister from game state change events
 * @nb: notifier block to unregister
 *
 * Returns 0 on success, negative errno on failure.
 */
int selene_unregister_notifier(struct notifier_block *nb);

#endif /* _SELENE_H */
