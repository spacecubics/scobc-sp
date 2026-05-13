/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SC_IPC_H_
#define SC_IPC_H_

#include <stdbool.h>
#include <stdint.h>

void sc_ipc_update_response_cache(void);

#define SC_IPC_COMMAND_FIELD_SIZE 4U
#define SC_IPC_DATA_CRC_SIZE      2U
#define SC_IPC_RESPONSE_SEQ_SIZE  1U
#define SC_IPC_MAX_DATA_SIZE      8U

#define SC_IPC_HEADER_SEQ_NUM 0U
#define SC_IPC_HEADER_CMD0    1U
#define SC_IPC_HEADER_CMD1    2U
#define SC_IPC_HEADER_CRC     3U
#define SC_IPC_COMMAND_CRC_INPUT_SIZE \
	(SC_IPC_COMMAND_FIELD_SIZE - 1U)

#define SC_IPC_CMD_CATEGORY_SHIFT   14U
#define SC_IPC_CMD_NUMBER_SHIFT     8U
#define SC_IPC_CMD_SUBCOMMAND_SHIFT 5U
#define SC_IPC_CMD_TYPE_SHIFT       2U

#define SC_IPC_CMD_CATEGORY_MASK   0x3U
#define SC_IPC_CMD_NUMBER_MASK     0x3fU
#define SC_IPC_CMD_SUBCOMMAND_MASK 0x7U
#define SC_IPC_CMD_TYPE_MASK       0x7U
#define SC_IPC_CMD_SIZE_MASK       0x3U

#define SC_IPC_CMD(category, number, subcommand, type, data_size) \
	((uint16_t)((((category) & SC_IPC_CMD_CATEGORY_MASK) << SC_IPC_CMD_CATEGORY_SHIFT) | \
		    (((number) & SC_IPC_CMD_NUMBER_MASK) << SC_IPC_CMD_NUMBER_SHIFT) | \
		    (((subcommand) & SC_IPC_CMD_SUBCOMMAND_MASK) << SC_IPC_CMD_SUBCOMMAND_SHIFT) | \
		    (((type) & SC_IPC_CMD_TYPE_MASK) << SC_IPC_CMD_TYPE_SHIFT) | \
		    ((data_size) & SC_IPC_CMD_SIZE_MASK)))

enum sc_ipc_cmd_category {
	SC_IPC_CMD_CATEGORY_SYSTEM = 0U,
	SC_IPC_CMD_CATEGORY_BOARD_MANAGEMENT = 1U,
	SC_IPC_CMD_CATEGORY_MAIN_PROCESSOR_CONTROL = 2U,
};

enum sc_ipc_cmd_type {
	SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ = 1U,
	SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_WRITE = 2U,
};

enum sc_ipc_cmd_data_size_code {
	SC_IPC_CMD_DATA_SIZE_0 = 0U,
	SC_IPC_CMD_DATA_SIZE_2 = 1U,
	SC_IPC_CMD_DATA_SIZE_4 = 2U,
	SC_IPC_CMD_DATA_SIZE_8 = 3U,
};

static inline int sc_ipc_cmd_data_size(uint16_t cmd)
{
	switch (cmd & SC_IPC_CMD_SIZE_MASK) {
	case SC_IPC_CMD_DATA_SIZE_0:
		return 0;
	case SC_IPC_CMD_DATA_SIZE_2:
		return 2;
	case SC_IPC_CMD_DATA_SIZE_4:
		return 4;
	case SC_IPC_CMD_DATA_SIZE_8:
		return 8;
	default:
		return -1;
	}
}

static inline bool sc_ipc_cmd_is_read(uint16_t cmd)
{
	return ((cmd >> SC_IPC_CMD_TYPE_SHIFT) & SC_IPC_CMD_TYPE_MASK) ==
	       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ;
}

static inline bool sc_ipc_cmd_is_write(uint16_t cmd)
{
	return ((cmd >> SC_IPC_CMD_TYPE_SHIFT) & SC_IPC_CMD_TYPE_MASK) ==
	       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_WRITE;
}

static inline uint8_t sc_ipc_header_seq(
	const uint8_t header[SC_IPC_COMMAND_FIELD_SIZE])
{
	return header[SC_IPC_HEADER_SEQ_NUM];
}

static inline uint16_t sc_ipc_header_cmd(
	const uint8_t header[SC_IPC_COMMAND_FIELD_SIZE])
{
	return ((uint16_t)header[SC_IPC_HEADER_CMD0] << 8) |
	       (uint16_t)header[SC_IPC_HEADER_CMD1];
}

static inline uint8_t sc_ipc_header_crc(
	const uint8_t header[SC_IPC_COMMAND_FIELD_SIZE])
{
	return header[SC_IPC_HEADER_CRC];
}

#define SC_IPC_CMD_READ_API_VERSION \
	SC_IPC_CMD(SC_IPC_CMD_CATEGORY_SYSTEM, 0x01U, 0U, \
		       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ, \
		       SC_IPC_CMD_DATA_SIZE_4)

#define SC_IPC_CMD_GET_TEMP_AVERAGE \
	SC_IPC_CMD(SC_IPC_CMD_CATEGORY_BOARD_MANAGEMENT, 0x01U, 0U, \
		       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ, \
		       SC_IPC_CMD_DATA_SIZE_2)

#define SC_IPC_CMD_GET_CURRENT_TOTAL \
	SC_IPC_CMD(SC_IPC_CMD_CATEGORY_BOARD_MANAGEMENT, 0x02U, 0U, \
		       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ, \
		       SC_IPC_CMD_DATA_SIZE_2)
#define SC_IPC_CMD_GET_CURRENT_SYS \
	SC_IPC_CMD(SC_IPC_CMD_CATEGORY_BOARD_MANAGEMENT, 0x02U, 1U, \
		       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ, \
		       SC_IPC_CMD_DATA_SIZE_2)
#define SC_IPC_CMD_GET_CURRENT_PS \
	SC_IPC_CMD(SC_IPC_CMD_CATEGORY_BOARD_MANAGEMENT, 0x02U, 2U, \
		       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ, \
		       SC_IPC_CMD_DATA_SIZE_2)
#define SC_IPC_CMD_GET_CURRENT_PL \
	SC_IPC_CMD(SC_IPC_CMD_CATEGORY_BOARD_MANAGEMENT, 0x02U, 3U, \
		       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ, \
		       SC_IPC_CMD_DATA_SIZE_2)

#define SC_IPC_CMD_SET_BOOT_MEMORY_SELECT \
	SC_IPC_CMD(SC_IPC_CMD_CATEGORY_MAIN_PROCESSOR_CONTROL, 0x01U, 0U, \
		       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_WRITE, \
		       SC_IPC_CMD_DATA_SIZE_2)
#define SC_IPC_CMD_GET_BOOT_MEMORY_SELECT \
	SC_IPC_CMD(SC_IPC_CMD_CATEGORY_MAIN_PROCESSOR_CONTROL, 0x01U, 0U, \
		       SC_IPC_CMD_TYPE_PROTECTED_IMMEDIATE_READ, \
		       SC_IPC_CMD_DATA_SIZE_2)

#endif /* SC_IPC_H_ */
