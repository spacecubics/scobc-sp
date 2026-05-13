/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/sys/printk.h>

#include "system.h"
#include "mppc.h"
#include "version.h"

static void print_system_info(void)
{
	char ip_version[32];
	char grade[32];
	char revision[16];

	sc_system_get_board_grade(grade, sizeof(grade));
	sc_system_get_board_revision(revision, sizeof(revision));
	sc_system_get_ip_version(ip_version, sizeof(ip_version));

	printk("SC-OBC Safety Processor Firmware " SCOBC_SP_FW_VERSION "\n");
	printk("Board: %s %s\n", grade, revision);
	printk("IP version: %s\n", ip_version);
}

int main(void)
{
	print_system_info();

	sc_mppc_main_power_on();

	return 0;
}
