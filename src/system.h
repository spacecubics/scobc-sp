/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SC_SYSTEM_H_
#define SC_SYSTEM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int sc_system_get_ip_version(char *version, size_t version_size);
int sc_system_get_board_grade(char *grade, size_t grade_size);
int sc_system_get_board_revision(char *revision, size_t revision_size);
void sc_system_set_led_red(bool on);
void sc_system_set_led_green(bool on);
bool sc_system_get_led_red(void);
bool sc_system_get_led_green(void);
void sc_system_update_keep_alive(void);
int sc_system_set_boot_memory_port(uint32_t port);
uint32_t sc_system_get_boot_memory_port(void);

#endif /* SC_SYSTEM_H_ */
