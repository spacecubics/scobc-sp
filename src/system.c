/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "system.h"

#include <errno.h>
#include <stddef.h>
#include <stdio.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define SC_SYSTEM_NODE DT_NODELABEL(system_register)
BUILD_ASSERT(DT_NODE_HAS_STATUS(SC_SYSTEM_NODE, okay),
	     "system_register node must be enabled in devicetree");

#define SC_SYSTEM_REG_BASE DT_REG_ADDR(SC_SYSTEM_NODE)

#define SC_SYSTEM_REG_IP_VERSION		0x00UL
#define SC_SYSTEM_REG_GIT_HASH			0x04UL
#define SC_SYSTEM_REG_SCRATCH_PAD		0x08UL
#define SC_SYSTEM_REG_BOARD_ID			0x10UL
#define SC_SYSTEM_REG_LED_CONTROL		0x20UL
#define SC_SYSTEM_REG_KEEP_ALIVE_CONTROL	0x24UL
#define SC_SYSTEM_REG_BOOT_MEMORY_SELECT	0x28UL

#define SC_SYSTEM_IP_VERSION_MAJOR_MASK		0xff000000UL
#define SC_SYSTEM_IP_VERSION_MAJOR_SHIFT	24
#define SC_SYSTEM_IP_VERSION_MINOR_MASK		0x00ff0000UL
#define SC_SYSTEM_IP_VERSION_MINOR_SHIFT	16
#define SC_SYSTEM_IP_VERSION_PATCH_MASK		0x0000ffffUL
#define SC_SYSTEM_IP_VERSION_PATCH_SHIFT	0

#define SC_SYSTEM_IP_HASH_HASH_MASK		0xffffff00UL
#define SC_SYSTEM_IP_HASH_HASH_SHIFT		8
#define SC_SYSTEM_IP_HASH_STATUS_MASK		0x000000ffUL
#define SC_SYSTEM_IP_HASH_STATUS_SHIFT		0
#define SC_SYSTEM_IP_HASH_STATUS_CLEAN		0x00UL
#define SC_SYSTEM_IP_HASH_STATUS_DIRTY		0xffUL

#define SC_SYSTEM_BOARD_ID_BOARD_REVISION_MASK	0x00000002UL
#define SC_SYSTEM_BOARD_ID_BOARD_REVISION_SHIFT	1
#define SC_SYSTEM_BOARD_ID_BOARD_GRADE_MASK	0x00000001UL
#define SC_SYSTEM_BOARD_ID_BOARD_GRADE_SHIFT	0
#define SC_SYSTEM_BOARD_ID_BOARD_REVISION_A	0x00UL
#define SC_SYSTEM_BOARD_ID_BOARD_REVISION_B	0x01UL
#define SC_SYSTEM_BOARD_ID_BOARD_GRADE_SPACE	0x00UL
#define SC_SYSTEM_BOARD_ID_BOARD_GRADE_DEV	0x01UL

#define SC_SYSTEM_LED_CONTROL_RED_MASK		0x00000002UL
#define SC_SYSTEM_LED_CONTROL_RED_SHIFT		1
#define SC_SYSTEM_LED_CONTROL_GREEN_MASK		0x00000001UL
#define SC_SYSTEM_LED_CONTROL_GREEN_SHIFT	0
#define SC_SYSTEM_LED_CONTROL_OFF		0x00UL
#define SC_SYSTEM_LED_CONTROL_ON			0x01UL

#define SC_SYSTEM_KEEP_ALIVE_CONTROL_MASK	0x00000001UL

#define SC_SYSTEM_BOOT_MEMORY_SELECT_MASK	0x00000001UL
#define SC_SYSTEM_BOOT_MEMORY_SELECT_PORT0	0x00UL
#define SC_SYSTEM_BOOT_MEMORY_SELECT_PORT1	0x01UL

#define SC_SYSTEM_KEEP_ALIVE_STACK_SIZE 1024
#define SC_SYSTEM_KEEP_ALIVE_PRIORITY     K_PRIO_PREEMPT(1)
#define SC_SYSTEM_KEEP_ALIVE_INTERVAL     K_MSEC(1000)

static void sc_system_write(uint32_t reg, uint32_t val)
{
	sys_write32(val, SC_SYSTEM_REG_BASE + reg);
}

static uint32_t sc_system_read(uint32_t reg)
{
	return sys_read32(SC_SYSTEM_REG_BASE + reg);
}

static void sc_system_keep_alive_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		sc_system_update_keep_alive();
		k_sleep(SC_SYSTEM_KEEP_ALIVE_INTERVAL);
	}
}

K_THREAD_DEFINE(sc_system_keep_alive_tid,
		SC_SYSTEM_KEEP_ALIVE_STACK_SIZE,
		sc_system_keep_alive_thread,
		NULL, NULL, NULL,
		SC_SYSTEM_KEEP_ALIVE_PRIORITY,
		0,
		0);

static uint32_t sc_system_get_field(uint32_t val, uint32_t mask, uint32_t shift)
{
	return (val & mask) >> shift;
}

int sc_system_get_ip_version(char *version, size_t version_size)
{
	const char *status_str;
	uint32_t git_hash;
	uint32_t hash;
	uint32_t ip_version;
	uint32_t status;

	if (version == NULL) {
		return -EINVAL;
	}

	ip_version = sc_system_read(SC_SYSTEM_REG_IP_VERSION);
	git_hash = sc_system_read(SC_SYSTEM_REG_GIT_HASH);
	hash = sc_system_get_field(git_hash, SC_SYSTEM_IP_HASH_HASH_MASK,
				   SC_SYSTEM_IP_HASH_HASH_SHIFT);
	status = sc_system_get_field(git_hash, SC_SYSTEM_IP_HASH_STATUS_MASK,
				     SC_SYSTEM_IP_HASH_STATUS_SHIFT);

	switch (status) {
	case SC_SYSTEM_IP_HASH_STATUS_CLEAN:
		status_str = "";
		break;
	case SC_SYSTEM_IP_HASH_STATUS_DIRTY:
		status_str = "-dirty";
		break;
	default:
		status_str = "-unknown";
		break;
	}

	snprintf(version, version_size, "v%lu.%lu.%lu-g%06lx%s",
		 (unsigned long)sc_system_get_field(ip_version,
						      SC_SYSTEM_IP_VERSION_MAJOR_MASK,
						      SC_SYSTEM_IP_VERSION_MAJOR_SHIFT),
		 (unsigned long)sc_system_get_field(ip_version,
						      SC_SYSTEM_IP_VERSION_MINOR_MASK,
						      SC_SYSTEM_IP_VERSION_MINOR_SHIFT),
		 (unsigned long)sc_system_get_field(ip_version,
						      SC_SYSTEM_IP_VERSION_PATCH_MASK,
						      SC_SYSTEM_IP_VERSION_PATCH_SHIFT),
		 (unsigned long)hash, status_str);

	return 0;
}

int sc_system_get_board_grade(char *grade, size_t grade_size)
{
	uint32_t board_id;
	uint32_t grade_id;

	if (grade == NULL) {
		return -EINVAL;
	}

	board_id = sc_system_read(SC_SYSTEM_REG_BOARD_ID);
	grade_id = sc_system_get_field(board_id,
					SC_SYSTEM_BOARD_ID_BOARD_GRADE_MASK,
					SC_SYSTEM_BOARD_ID_BOARD_GRADE_SHIFT);

	switch (grade_id) {
	case SC_SYSTEM_BOARD_ID_BOARD_GRADE_SPACE:
		snprintf(grade, grade_size, "Space Grade");
		break;
	case SC_SYSTEM_BOARD_ID_BOARD_GRADE_DEV:
		snprintf(grade, grade_size, "Developer Grade");
		break;
	default:
		snprintf(grade, grade_size, "Unknown");
		break;
	}

	return 0;
}

int sc_system_get_board_revision(char *revision, size_t revision_size)
{
	uint32_t board_id;
	uint32_t revision_id;

	if (revision == NULL) {
		return -EINVAL;
	}

	board_id = sc_system_read(SC_SYSTEM_REG_BOARD_ID);
	revision_id = sc_system_get_field(board_id,
					   SC_SYSTEM_BOARD_ID_BOARD_REVISION_MASK,
					   SC_SYSTEM_BOARD_ID_BOARD_REVISION_SHIFT);

	switch (revision_id) {
	case SC_SYSTEM_BOARD_ID_BOARD_REVISION_A:
		snprintf(revision, revision_size, "Rev.A");
		break;
	case SC_SYSTEM_BOARD_ID_BOARD_REVISION_B:
		snprintf(revision, revision_size, "Rev.B");
		break;
	default:
		snprintf(revision, revision_size, "Unknown");
		break;
	}

	return 0;
}

void sc_system_set_led_red(bool on)
{
	uint32_t led_control;

	led_control = sc_system_read(SC_SYSTEM_REG_LED_CONTROL);
	led_control &= ~SC_SYSTEM_LED_CONTROL_RED_MASK;
	led_control |= (on ? SC_SYSTEM_LED_CONTROL_ON : SC_SYSTEM_LED_CONTROL_OFF)
		       << SC_SYSTEM_LED_CONTROL_RED_SHIFT;
	sc_system_write(SC_SYSTEM_REG_LED_CONTROL, led_control);
}

void sc_system_set_led_green(bool on)
{
	uint32_t led_control;

	led_control = sc_system_read(SC_SYSTEM_REG_LED_CONTROL);
	led_control &= ~SC_SYSTEM_LED_CONTROL_GREEN_MASK;
	led_control |= (on ? SC_SYSTEM_LED_CONTROL_ON : SC_SYSTEM_LED_CONTROL_OFF)
		       << SC_SYSTEM_LED_CONTROL_GREEN_SHIFT;
	sc_system_write(SC_SYSTEM_REG_LED_CONTROL, led_control);
}

bool sc_system_get_led_red(void)
{
	uint32_t led_control;

	led_control = sc_system_read(SC_SYSTEM_REG_LED_CONTROL);

	return sc_system_get_field(led_control, SC_SYSTEM_LED_CONTROL_RED_MASK,
				   SC_SYSTEM_LED_CONTROL_RED_SHIFT) ==
		SC_SYSTEM_LED_CONTROL_ON;
}

bool sc_system_get_led_green(void)
{
	uint32_t led_control;

	led_control = sc_system_read(SC_SYSTEM_REG_LED_CONTROL);

	return sc_system_get_field(led_control, SC_SYSTEM_LED_CONTROL_GREEN_MASK,
				   SC_SYSTEM_LED_CONTROL_GREEN_SHIFT) ==
		SC_SYSTEM_LED_CONTROL_ON;
}

void sc_system_update_keep_alive(void)
{
	uint32_t val;

	val = sc_system_read(SC_SYSTEM_REG_KEEP_ALIVE_CONTROL);
	val ^= SC_SYSTEM_KEEP_ALIVE_CONTROL_MASK;
	sc_system_write(SC_SYSTEM_REG_KEEP_ALIVE_CONTROL, val);
}

int sc_system_set_boot_memory_port(uint32_t port)
{
	uint32_t val;

	if (port > 1U) {
		return -EINVAL;
	}

	val = sc_system_read(SC_SYSTEM_REG_BOOT_MEMORY_SELECT);
	val &= ~SC_SYSTEM_BOOT_MEMORY_SELECT_MASK;
	val |= port == 0 ? SC_SYSTEM_BOOT_MEMORY_SELECT_PORT0 :
			 SC_SYSTEM_BOOT_MEMORY_SELECT_PORT1;
	sc_system_write(SC_SYSTEM_REG_BOOT_MEMORY_SELECT, val);

	return 0;
}

uint32_t sc_system_get_boot_memory_port(void)
{
	uint32_t val;

	val = sc_system_read(SC_SYSTEM_REG_BOOT_MEMORY_SELECT);

	return val & SC_SYSTEM_BOOT_MEMORY_SELECT_MASK;
}
