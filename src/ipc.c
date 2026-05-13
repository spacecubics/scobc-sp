/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ipc.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "mppc.h"
#include "sensor.h"
#include "system.h"

LOG_MODULE_REGISTER(ipc, CONFIG_SC_IPC_LOG_LEVEL);

#define SC_IPC_NODE DT_NODELABEL(spi_target)
BUILD_ASSERT(DT_NODE_HAS_STATUS(SC_IPC_NODE, okay),
	     "spi_target node must be enabled in devicetree");
BUILD_ASSERT(DT_IRQ_HAS_CELL(SC_IPC_NODE, irq),
	     "spi_target interrupt must have irq cell");
BUILD_ASSERT(DT_IRQ_HAS_CELL(SC_IPC_NODE, priority),
	     "spi_target interrupt must have priority cell");

#define SC_IPC_API_VERSION	1U

#define SC_SPI_REG_BASE		DT_REG_ADDR(SC_IPC_NODE)
#define SC_SPI_IRQ		DT_IRQN(SC_IPC_NODE)
#define SC_SPI_IRQ_PRIORITY	DT_IRQ(SC_IPC_NODE, priority)
#define SC_SPI_IRQ_FLAGS	0

/* SC-SPI-TARGET Register List offsets. */
#define SC_SPI_REG_IP_VERSION		0x000UL
#define SC_SPI_REG_IP_CONFIGURATION	0x004UL
#define SC_SPI_REG_IP_RESET		0x010UL
#define SC_SPI_REG_INTERRUPT_STATUS	0x020UL
#define SC_SPI_REG_INTERRUPT_ENABLE	0x024UL
#define SC_SPI_REG_CONTROL		0x030UL
#define SC_SPI_REG_BUFFER_THRESHOLD	0x034UL
#define SC_SPI_REG_BUFFER_CONTROL	0x038UL
#define SC_SPI_REG_BUFFER_STATUS	0x03cUL
#define SC_SPI_REG_TX_BUFFER(n)		(0x100UL + (0x4UL * (uint32_t)(n)))
#define SC_SPI_REG_RX_BUFFER(n)		(0x200UL + (0x4UL * (uint32_t)(n)))

#define SC_SPI_INTERRUPT_FRAME_COMPLETE		BIT(0)
#define SC_SPI_INTERRUPT_FRAME_THRESHOLD	BIT(1)
#define SC_SPI_INTERRUPT_BUFFER_OVERRUN		BIT(16)
#define SC_SPI_INTERRUPT_INVALID_FRAME		BIT(17)
#define SC_SPI_INTERRUPT_USED			(SC_SPI_INTERRUPT_FRAME_COMPLETE | \
						 SC_SPI_INTERRUPT_FRAME_THRESHOLD | \
						 SC_SPI_INTERRUPT_BUFFER_OVERRUN | \
						 SC_SPI_INTERRUPT_INVALID_FRAME)

#define SC_SPI_CONTROL_IP_ENABLE		BIT(0)
#define SC_SPI_CONTROL_CPHA			BIT(8)
#define SC_SPI_CONTROL_CPOL			BIT(9)
#define SC_SPI_BUFFER_CONTROL_CPU_BUFFER_CHANGE	BIT(0)
#define SC_SPI_IP_CONFIG_MAX_FRAME_SIZE_MASK	0x3fUL
#define SC_SPI_BUFFER_STATUS_FRAME_SIZE_MASK	0x3fUL
#define SC_SPI_BUFFER_STATUS_CS_MASK		GENMASK(11, 8)
#define SC_SPI_BUFFER_STATUS_CS_SHIFT		8U

#define SC_IPC_READ_LATENCY_BYTES	2U
#define SC_IPC_INIT_PRIORITY		80


struct sc_ipc_context {
	uint8_t max_frame_size;
	uint8_t last_read_seq;
	uint16_t last_read_cmd;
	uint8_t last_read_response_len;
	uint8_t last_read_response[SC_IPC_RESPONSE_SEQ_SIZE +
				       SC_IPC_MAX_DATA_SIZE +
				       SC_IPC_DATA_CRC_SIZE];
	bool last_read_response_valid;
	bool initialized;
};

struct sc_ipc_cached_response {
	uint16_t cmd;
	uint8_t len;
	int ret;
	uint8_t payload[SC_IPC_MAX_DATA_SIZE + SC_IPC_DATA_CRC_SIZE];
};

static struct sc_ipc_context sc_ipc_ctx;

static const uint16_t sc_ipc_supported_read_commands[] = {
	SC_IPC_CMD_READ_API_VERSION,
	SC_IPC_CMD_GET_TEMP_AVERAGE,
	SC_IPC_CMD_GET_CURRENT_TOTAL,
	SC_IPC_CMD_GET_CURRENT_SYS,
	SC_IPC_CMD_GET_CURRENT_PS,
	SC_IPC_CMD_GET_CURRENT_PL,
	SC_IPC_CMD_GET_BOOT_MEMORY_SELECT,
};

static struct sc_ipc_cached_response
	sc_ipc_response_cache[2][ARRAY_SIZE(sc_ipc_supported_read_commands)];
static atomic_t sc_ipc_response_cache_active;

static void sc_spi_write(uint32_t reg, uint32_t val)
{
	sys_write32(val, SC_SPI_REG_BASE + reg);
}

static uint32_t sc_spi_read(uint32_t reg)
{
	return sys_read32(SC_SPI_REG_BASE + reg);
}

static uint8_t sc_spi_read_rx_byte(uint8_t index)
{
	return (uint8_t)(sc_spi_read(SC_SPI_REG_RX_BUFFER(index)) & 0xffU);
}

static void sc_spi_write_tx_byte(uint8_t index, uint8_t val)
{
	sc_spi_write(SC_SPI_REG_TX_BUFFER(index), val);
}

static uint8_t sc_spi_buffer_status_frame_size(uint32_t status)
{
	return (uint8_t)(status & SC_SPI_BUFFER_STATUS_FRAME_SIZE_MASK);
}

static uint8_t sc_spi_buffer_status_cs(uint32_t status)
{
	return (uint8_t)((status & SC_SPI_BUFFER_STATUS_CS_MASK) >>
			       SC_SPI_BUFFER_STATUS_CS_SHIFT);
}

static uint8_t crc8_sae_j1850(const uint8_t *buf, size_t len)
{
	uint8_t crc = 0xffU;
	size_t i;
	int bit;

	for (i = 0; i < len; i++) {
		crc ^= buf[i];
		for (bit = 0; bit < 8; bit++) {
			if ((crc & 0x80U) != 0U) {
				crc = (uint8_t)((crc << 1) ^ 0x1dU);
			} else {
				crc = (uint8_t)(crc << 1);
			}
		}
	}

	/* The current command spec defines poly/init, but not xorout.
	 * Use xorout=0x00 until a test vector says otherwise.
	 */
	return crc;
}

static uint16_t crc16_ccitt_false(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xffffU;
	size_t i;
	int bit;

	for (i = 0; i < len; i++) {
		crc ^= (uint16_t)buf[i] << 8;
		for (bit = 0; bit < 8; bit++) {
			if ((crc & 0x8000U) != 0U) {
				crc = (uint16_t)((crc << 1) ^ 0x1021U);
			} else {
				crc = (uint16_t)(crc << 1);
			}
		}
	}

	return crc;
}

static void put_be16(uint8_t *buf, uint16_t val)
{
	buf[0] = (uint8_t)(val >> 8);
	buf[1] = (uint8_t)val;
}

static void put_be32(uint8_t *buf, uint32_t val)
{
	buf[0] = (uint8_t)(val >> 24);
	buf[1] = (uint8_t)(val >> 16);
	buf[2] = (uint8_t)(val >> 8);
	buf[3] = (uint8_t)val;
}

static uint16_t get_be16(const uint8_t *buf)
{
	return ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
}

static int64_t div_round_closest_i64(int64_t dividend, int64_t divisor)
{
	if (dividend >= 0) {
		return (dividend + divisor / 2) / divisor;
	}

	return (dividend - divisor / 2) / divisor;
}

static uint16_t encode_temperature_micro_q025c(int64_t micro_degc)
{
	int64_t quarter_degc;

	quarter_degc = div_round_closest_i64(micro_degc, 250000LL);
	if (quarter_degc > INT16_MAX) {
		quarter_degc = INT16_MAX;
	} else if (quarter_degc < INT16_MIN) {
		quarter_degc = INT16_MIN;
	}

	return (uint16_t)(int16_t)quarter_degc;
}

static uint16_t encode_current_ma(const struct sensor_value *current)
{
	int64_t milliamp;

	milliamp = div_round_closest_i64(sensor_value_to_micro(current), 1000LL);
	if (milliamp > UINT16_MAX) {
		milliamp = UINT16_MAX;
	} else if (milliamp < 0) {
		milliamp = 0;
	}

	return (uint16_t)milliamp;
}

static int read_temperature(size_t index, uint16_t *val)
{
	struct sc_lm75_sensor_data sensor;
	int ret;

	ret = sc_lm75_get_sensor(index, &sensor);
	if (ret < 0 || sensor.ret < 0) {
		return ret < 0 ? ret : sensor.ret;
	}

	*val = encode_temperature_micro_q025c(sensor_value_to_micro(&sensor.temp));

	return 0;
}

static int read_cvmon_current_ma(size_t channel, uint16_t *val)
{
	struct sc_ina3221_sensor_data sensor;
	int ret;

	ret = sc_ina3221_get_sensor(0, &sensor);
	if (ret < 0 || sensor.ret < 0) {
		return ret < 0 ? ret : sensor.ret;
	}

	if (channel >= ARRAY_SIZE(sensor.channel) || !sensor.channel[channel].enabled) {
		return -ENOENT;
	}

	if (sensor.channel[channel].ret < 0) {
		return sensor.channel[channel].ret;
	}

	*val = encode_current_ma(&sensor.channel[channel].current);

	return 0;
}

static int read_current_total_ma(uint16_t *val)
{
	uint32_t sum = 0;
	uint16_t current;
	size_t valid_count = 0;
	size_t ch;
	int ret;

	for (ch = 0; ch < 3; ch++) {
		ret = read_cvmon_current_ma(ch, &current);
		if (ret < 0) {
			continue;
		}
		sum += current;
		valid_count++;
	}

	if (valid_count == 0) {
		return -EAGAIN;
	}

	if (sum > UINT16_MAX) {
		sum = UINT16_MAX;
	}

	*val = (uint16_t)sum;

	return 0;
}

static int build_read_response_data(uint16_t cmd, uint8_t *data, size_t *len)
{
	uint16_t v16;
	int ret = 0;

	switch (cmd) {
	case SC_IPC_CMD_READ_API_VERSION:
		put_be32(data, SC_IPC_API_VERSION);
		break;

	case SC_IPC_CMD_GET_TEMP_AVERAGE:
		/*
		 * Use Temperature Sensor 3 as the average temperature
		 * in the current release.
		 * This may be changed to some calculated value in the future.
		 */
		ret = read_temperature(2, &v16);
		if (ret == 0) {
			put_be16(data, v16);
		}
		break;

	case SC_IPC_CMD_GET_CURRENT_TOTAL:
		ret = read_current_total_ma(&v16);
		if (ret == 0) {
			put_be16(data, v16);
		}
		break;

	case SC_IPC_CMD_GET_CURRENT_SYS:
		ret = read_cvmon_current_ma(0, &v16);
		if (ret == 0) {
			put_be16(data, v16);
		}
		break;

	case SC_IPC_CMD_GET_CURRENT_PS:
		ret = read_cvmon_current_ma(1, &v16);
		if (ret == 0) {
			put_be16(data, v16);
		}
		break;

	case SC_IPC_CMD_GET_CURRENT_PL:
		ret = read_cvmon_current_ma(2, &v16);
		if (ret == 0) {
			put_be16(data, v16);
		}
		break;

	case SC_IPC_CMD_GET_BOOT_MEMORY_SELECT:
		v16 = (uint16_t)sc_system_get_boot_memory_port();
		put_be16(data, v16);
		break;

	default:
		return -ENOTSUP;
	}

	*len = sc_ipc_cmd_data_size(cmd);

	return ret;
}

static void build_error_read_response_payload(uint16_t cmd, uint8_t *payload,
					      uint8_t *payload_len)
{
	uint8_t data[SC_IPC_MAX_DATA_SIZE];
	size_t len;
	int cmd_len;
	uint16_t crc;

	cmd_len = sc_ipc_cmd_data_size(cmd);
	if (cmd_len < 0 || cmd_len > SC_IPC_MAX_DATA_SIZE) {
		len = 2;
	} else {
		len = (size_t)cmd_len;
	}

	memset(data, 0xff, len);
	memcpy(payload, data, len);

	crc = crc16_ccitt_false(data, len);
	payload[len] = (uint8_t)(crc >> 8);
	payload[len + 1U] = (uint8_t)crc;
	*payload_len = (uint8_t)(len + SC_IPC_DATA_CRC_SIZE);
}

static int build_read_response_payload(uint16_t cmd, uint8_t *payload,
				       uint8_t *payload_len)
{
	uint8_t data[SC_IPC_MAX_DATA_SIZE] = {0};
	size_t len = 0;
	uint16_t crc;
	int ret;

	ret = build_read_response_data(cmd, data, &len);
	if (ret < 0 || len > SC_IPC_MAX_DATA_SIZE) {
		build_error_read_response_payload(cmd, payload, payload_len);
		return ret < 0 ? ret : -EIO;
	}

	memcpy(payload, data, len);
	crc = crc16_ccitt_false(data, len);
	payload[len] = (uint8_t)(crc >> 8);
	payload[len + 1U] = (uint8_t)crc;
	*payload_len = (uint8_t)(len + SC_IPC_DATA_CRC_SIZE);

	return 0;
}

static int write_read_response_payload(const uint8_t *payload, uint8_t payload_len)
{
	uint8_t off = (uint8_t)(SC_IPC_COMMAND_FIELD_SIZE +
				 SC_IPC_READ_LATENCY_BYTES);
	uint16_t end = (uint16_t)off + payload_len;
	int i;

	if (end > sc_ipc_ctx.max_frame_size) {
		LOG_DBG("read response does not fit: off=%u payload_len=%u max_frame=%u",
			off, payload_len, sc_ipc_ctx.max_frame_size);
		return -EMSGSIZE;
	}

	for (i = 0; i < payload_len; i++) {
		sc_spi_write_tx_byte((uint8_t)(off + i), payload[i]);
	}

	return 0;
}

static int compose_and_write_read_response(uint8_t seq, uint16_t cmd,
					   const uint8_t *payload,
					   uint8_t payload_len)
{
	uint8_t response[SC_IPC_RESPONSE_SEQ_SIZE + SC_IPC_MAX_DATA_SIZE +
			 SC_IPC_DATA_CRC_SIZE];
	uint8_t response_len;
	int ret;

	if (payload_len > (sizeof(response) - SC_IPC_RESPONSE_SEQ_SIZE)) {
		return -EMSGSIZE;
	}

	response[0] = seq;
	memcpy(&response[SC_IPC_RESPONSE_SEQ_SIZE], payload, payload_len);
	response_len = (uint8_t)(SC_IPC_RESPONSE_SEQ_SIZE + payload_len);

	ret = write_read_response_payload(response, response_len);
	if (ret < 0) {
		return ret;
	}

	sc_ipc_ctx.last_read_seq = seq;
	sc_ipc_ctx.last_read_cmd = cmd;
	sc_ipc_ctx.last_read_response_len = response_len;
	memcpy(sc_ipc_ctx.last_read_response, response, response_len);
	sc_ipc_ctx.last_read_response_valid = true;

	return 0;
}

static const struct sc_ipc_cached_response *find_cached_response(uint16_t cmd)
{
	const struct sc_ipc_cached_response *cache;
	size_t i;

	cache = sc_ipc_response_cache[atomic_get(&sc_ipc_response_cache_active)];
	for (i = 0; i < ARRAY_SIZE(sc_ipc_supported_read_commands); i++) {
		if (cache[i].cmd == cmd) {
			return &cache[i];
		}
	}

	return NULL;
}

static void update_response_cache_once(void)
{
	struct sc_ipc_cached_response *cache;
	uint8_t inactive;
	size_t i;

	inactive = (uint8_t)(atomic_get(&sc_ipc_response_cache_active) ^ 1);
	cache = sc_ipc_response_cache[inactive];

	for (i = 0; i < ARRAY_SIZE(sc_ipc_supported_read_commands); i++) {
		cache[i].cmd = sc_ipc_supported_read_commands[i];
		cache[i].ret = build_read_response_payload(cache[i].cmd,
							cache[i].payload,
							&cache[i].len);
	}

	atomic_set(&sc_ipc_response_cache_active, inactive);
}

void sc_ipc_update_response_cache(void)
{
	update_response_cache_once();
}

static void update_boot_memory_response_cache(void)
{
	struct sc_ipc_cached_response *cache;
	size_t bank;
	size_t i;

	for (bank = 0; bank < ARRAY_SIZE(sc_ipc_response_cache); bank++) {
		cache = sc_ipc_response_cache[bank];

		for (i = 0; i < ARRAY_SIZE(sc_ipc_supported_read_commands); i++) {
			if (cache[i].cmd != SC_IPC_CMD_GET_BOOT_MEMORY_SELECT) {
				continue;
			}

			cache[i].ret = build_read_response_payload(cache[i].cmd,
								cache[i].payload,
								&cache[i].len);
		}
	}
}

static bool parse_command_header(uint8_t header[SC_IPC_COMMAND_FIELD_SIZE],
				 uint8_t *seq, uint16_t *cmd)
{
	uint8_t expected_crc;
	uint8_t i;

	for (i = 0; i < SC_IPC_COMMAND_FIELD_SIZE; i++) {
		header[i] = sc_spi_read_rx_byte(i);
	}

	expected_crc = crc8_sae_j1850(header, SC_IPC_COMMAND_CRC_INPUT_SIZE);
	if (expected_crc != sc_ipc_header_crc(header)) {
		LOG_DBG("bad command CRC: seq=0x%02x cmd=0x%04x got=0x%02x exp=0x%02x",
			sc_ipc_header_seq(header), sc_ipc_header_cmd(header),
			sc_ipc_header_crc(header), expected_crc);
		return false;
	}

	*seq = sc_ipc_header_seq(header);
	*cmd = sc_ipc_header_cmd(header);

	return true;
}

static void prepare_read_response(uint8_t seq, uint16_t cmd)
{
	const struct sc_ipc_cached_response *cached;
	uint8_t payload[SC_IPC_MAX_DATA_SIZE + SC_IPC_DATA_CRC_SIZE];
	uint8_t payload_len;
	int ret;

	/*
	 * Treat the same seq/cmd pair as a retransmission.  Re-send the exact
	 * previous response and do not refresh its contents.  Checking cmd as well
	 * avoids replaying a stale response after the requester has rebooted and
	 * restarted its sequence number from 1.
	 */
	if (sc_ipc_ctx.last_read_response_valid &&
	    seq == sc_ipc_ctx.last_read_seq &&
	    cmd == sc_ipc_ctx.last_read_cmd) {
		ret = write_read_response_payload(sc_ipc_ctx.last_read_response,
						  sc_ipc_ctx.last_read_response_len);
		if (ret < 0) {
			LOG_DBG("failed to re-send read response: cmd=0x%04x ret=%d",
				cmd, ret);
		}
		return;
	}

	cached = find_cached_response(cmd);
	if (cached == NULL) {
		build_error_read_response_payload(cmd, payload, &payload_len);
		ret = compose_and_write_read_response(seq, cmd, payload, payload_len);
		if (ret < 0) {
			LOG_DBG("failed to send unsupported command response: cmd=0x%04x ret=%d",
				cmd, ret);
		}
		return;
	}

	ret = compose_and_write_read_response(seq, cmd, cached->payload, cached->len);
	if (ret < 0) {
		LOG_DBG("failed to send read response: cmd=0x%04x ret=%d", cmd, ret);
		return;
	}

	if (cached->ret < 0) {
		LOG_DBG("read response uses error payload: cmd=0x%04x ret=%d",
			cmd, cached->ret);
	}
}

static void handle_frame_threshold(void)
{
	uint32_t status;
	uint8_t header[SC_IPC_COMMAND_FIELD_SIZE];
	uint8_t seq;
	uint16_t cmd;

	status = sc_spi_read(SC_SPI_REG_BUFFER_STATUS);

	LOG_DBG("frame threshold: cs=%u frame_size=%u buf_status=0x%08x",
		sc_spi_buffer_status_cs(status),
		sc_spi_buffer_status_frame_size(status), status);

	if (!parse_command_header(header, &seq, &cmd)) {
		return;
	}

	if (sc_ipc_cmd_is_write(cmd)) {
		/* Write commands need the whole frame, including data CRC. */
		return;
	}

	if (!sc_ipc_cmd_is_read(cmd)) {
		return;
	}

	prepare_read_response(seq, cmd);
}

static void handle_set_boot_memory_select(const uint8_t *data)
{
	uint16_t port = get_be16(data);
	int ret;

	ret = sc_system_set_boot_memory_port(port);
	if (ret < 0) {
		return;
	}

	update_boot_memory_response_cache();
}

static void handle_write_frame(uint16_t cmd, uint8_t frame_size)
{
	uint8_t data[SC_IPC_MAX_DATA_SIZE] = {0};
	uint16_t got_crc;
	uint16_t expected_crc;
	int data_size;
	uint8_t i;

	data_size = sc_ipc_cmd_data_size(cmd);
	if (data_size < 0 || data_size > SC_IPC_MAX_DATA_SIZE) {
		return;
	}

	if (frame_size < (SC_IPC_COMMAND_FIELD_SIZE + data_size +
			 SC_IPC_DATA_CRC_SIZE)) {
		return;
	}

	for (i = 0; i < data_size; i++) {
		data[i] = sc_spi_read_rx_byte((uint8_t)(SC_IPC_COMMAND_FIELD_SIZE + i));
	}

	got_crc = ((uint16_t)sc_spi_read_rx_byte((uint8_t)(SC_IPC_COMMAND_FIELD_SIZE + data_size)) << 8) |
		  sc_spi_read_rx_byte((uint8_t)(SC_IPC_COMMAND_FIELD_SIZE + data_size + 1));
	expected_crc = crc16_ccitt_false(data, (size_t)data_size);
	if (got_crc != expected_crc) {
		LOG_DBG("bad data CRC: cmd=0x%04x got=0x%04x exp=0x%04x",
			cmd, got_crc, expected_crc);
		return;
	}

	switch (cmd) {
	case SC_IPC_CMD_SET_BOOT_MEMORY_SELECT:
		handle_set_boot_memory_select(data);
		break;
	default:
		break;
	}
}

static void handle_frame_complete(void)
{
	uint32_t status;
	uint8_t frame_size;
	uint8_t header[SC_IPC_COMMAND_FIELD_SIZE];
	uint8_t seq;
	uint16_t cmd;

	status = sc_spi_read(SC_SPI_REG_BUFFER_STATUS);
	frame_size = sc_spi_buffer_status_frame_size(status);

	LOG_DBG("frame complete: cs=%u frame_size=%u buf_status=0x%08x",
		sc_spi_buffer_status_cs(status), frame_size, status);

	if (frame_size >= SC_IPC_COMMAND_FIELD_SIZE &&
	    parse_command_header(header, &seq, &cmd) && sc_ipc_cmd_is_write(cmd)) {
		handle_write_frame(cmd, frame_size);
	}

	/*
	 * CSn negation makes the SPI side switch to the next buffer automatically.
	 * Keep the CPU side on the completed buffer while handling the frame, then
	 * toggle it once so the next threshold interrupt sees the active SPI buffer.
	 * TX buffer clearing is left to the SC-SPI-TARGET IP.
	 */
	sc_spi_write(SC_SPI_REG_BUFFER_CONTROL,
		     SC_SPI_BUFFER_CONTROL_CPU_BUFFER_CHANGE);
}

static void sc_ipc_irq_handler(const void *arg)
{
	uint32_t status;
	uint32_t handled;

	ARG_UNUSED(arg);

	status = sc_spi_read(SC_SPI_REG_INTERRUPT_STATUS);
	handled = status & SC_SPI_INTERRUPT_USED;

	if (handled == 0U) {
		return;
	}

	LOG_DBG("irq status=0x%08x handled=0x%08x", status, handled);

	/*
	 * W/C register: clear the latched status bits from this snapshot before
	 * processing. This avoids clearing a new event that arrives while this
	 * handler is running.
	 */
	sc_spi_write(SC_SPI_REG_INTERRUPT_STATUS, handled);

	if ((status & SC_SPI_INTERRUPT_INVALID_FRAME) != 0U) {
		LOG_DBG("invalid frame interrupt");
	}

	if ((status & SC_SPI_INTERRUPT_BUFFER_OVERRUN) != 0U) {
		LOG_DBG("buffer overrun interrupt");
	}

	if ((status & SC_SPI_INTERRUPT_FRAME_THRESHOLD) != 0U) {
		handle_frame_threshold();
	}

	if ((status & SC_SPI_INTERRUPT_FRAME_COMPLETE) != 0U) {
		handle_frame_complete();
	}
}

static int sc_ipc_init(void)
{
	uint32_t ip_config;
	uint32_t max_frame_size;
	uint32_t min_read_frame_size;
	uint32_t control = 0;

	if (sc_ipc_ctx.initialized) {
		return 0;
	}

	/* Reset value is asserted, so always release software reset explicitly. */
	sc_spi_write(SC_SPI_REG_IP_RESET, 0);
	k_busy_wait(1);

	ip_config = sc_spi_read(SC_SPI_REG_IP_CONFIGURATION);
	max_frame_size = ip_config & SC_SPI_IP_CONFIG_MAX_FRAME_SIZE_MASK;
	if (max_frame_size == 0U) {
		LOG_ERR("SC-SPI-TARGET reports an invalid max frame size: config=0x%08x",
			ip_config);
		return -EINVAL;
	}
	sc_ipc_ctx.max_frame_size = (uint8_t)max_frame_size;

	min_read_frame_size = SC_IPC_COMMAND_FIELD_SIZE +
		SC_IPC_READ_LATENCY_BYTES +
		SC_IPC_RESPONSE_SEQ_SIZE + 2U +
		SC_IPC_DATA_CRC_SIZE;
	if (min_read_frame_size > sc_ipc_ctx.max_frame_size) {
		LOG_ERR("IPC minimum read frame is %u bytes, but target max frame size is %u",
			min_read_frame_size, sc_ipc_ctx.max_frame_size);
		return -EMSGSIZE;
	}

	/* Populate the response cache before the first SPI transaction. */
	update_response_cache_once();

	/* Protected Immediate Read needs an interrupt once Seq/CMD/CRC is present. */
	sc_spi_write(SC_SPI_REG_BUFFER_THRESHOLD, SC_IPC_COMMAND_FIELD_SIZE);
	sc_spi_write(SC_SPI_REG_INTERRUPT_STATUS, SC_SPI_INTERRUPT_USED);
	sc_spi_write(SC_SPI_REG_INTERRUPT_ENABLE, SC_SPI_INTERRUPT_USED);

#if defined(CONFIG_SC_IPC_SPI_MODE_1)
	control |= SC_SPI_CONTROL_CPHA;
#elif defined(CONFIG_SC_IPC_SPI_MODE_3)
	control |= SC_SPI_CONTROL_CPOL | SC_SPI_CONTROL_CPHA;
#else
#error "Unsupported SC IPC SPI mode"
#endif

	control |= SC_SPI_CONTROL_IP_ENABLE;
	sc_spi_write(SC_SPI_REG_CONTROL, control);

	IRQ_CONNECT(SC_SPI_IRQ, SC_SPI_IRQ_PRIORITY, sc_ipc_irq_handler,
		    NULL, SC_SPI_IRQ_FLAGS);
	irq_enable(SC_SPI_IRQ);

	sc_ipc_ctx.initialized = true;

	LOG_DBG("IPC initialized: base=0x%08lx irq=%u mode=%u latency=%u",
		(unsigned long)SC_SPI_REG_BASE, (unsigned int)SC_SPI_IRQ,
		IS_ENABLED(CONFIG_SC_IPC_SPI_MODE_3) ? 3U : 1U,
		SC_IPC_READ_LATENCY_BYTES);
	LOG_DBG("SC-SPI-TARGET config=0x%08x max_frame_size=%u",
		ip_config, sc_ipc_ctx.max_frame_size);

	return 0;
}

static int sc_ipc_sys_init(void)
{
	return sc_ipc_init();
}

SYS_INIT(sc_ipc_sys_init, APPLICATION, SC_IPC_INIT_PRIORITY);
