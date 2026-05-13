/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SC_MPPC_H_
#define SC_MPPC_H_

#include <stdbool.h>
#include <stdint.h>

uint32_t sc_mppc_get_power_status(void);
int sc_mppc_main_power_on(void);
int sc_mppc_main_power_off(void);
void sc_mppc_main_power_cycle(void);

#endif /* SC_MPPC_H_ */
