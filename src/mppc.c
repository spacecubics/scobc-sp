/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mppc.h"

#include <errno.h>
#include <stddef.h>

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(sc_mppc, CONFIG_SC_MPPC_LOG_LEVEL);

#define SC_MPPC_NODE DT_NODELABEL(mppc)
BUILD_ASSERT(DT_NODE_HAS_STATUS(SC_MPPC_NODE, okay),
	     "mppc node must be enabled in devicetree");
BUILD_ASSERT(DT_IRQ_HAS_CELL(SC_MPPC_NODE, irq),
	     "mppc interrupt must have irq cell");
BUILD_ASSERT(DT_IRQ_HAS_CELL(SC_MPPC_NODE, priority),
	     "mppc interrupt must have priority cell");

#define SC_MPPC_REG_BASE		DT_REG_ADDR(SC_MPPC_NODE)
#define SC_MPPC_IRQ			DT_IRQN(SC_MPPC_NODE)
#define SC_MPPC_IRQ_PRIORITY		DT_IRQ(SC_MPPC_NODE, priority)
#define SC_MPPC_IRQ_FLAGS		0

#define SC_MPPC_WORKQ_STACK_SIZE	1024
#define SC_MPPC_WORKQ_PRIORITY		K_PRIO_PREEMPT(1)
#define SC_MPPC_RESET_ASSERT_DELAY	K_MSEC(100)

#define SC_MPPC_REG_IP_VERSION				0x00UL
#define SC_MPPC_REG_INTERRUPT_STATUS			0x10UL
#define SC_MPPC_REG_INTERRUPT_ENABLE			0x14UL
#define SC_MPPC_REG_POWER_STATUS			0x20UL
#define SC_MPPC_REG_MAIN_PROCESSOR_RESET		0x24UL
#define SC_MPPC_REG_POWER_CYCLE_REQUEST			0x28UL
#define SC_MPPC_REG_PMC_LPD_POWER_CONTROL_STATUS	0x40UL
#define SC_MPPC_REG_FPD_POWER_CONTROL_STATUS		0x44UL
#define SC_MPPC_REG_SPD_POWER_CONTROL_STATUS		0x48UL
#define SC_MPPC_REG_PL_POWER_CONTROL_STATUS		0x4cUL
#define SC_MPPC_REG_GTYP_POWER_CONTROL_STATUS		0x50UL

#define SC_MPPC_INTERRUPT_STATUS_VDD_OUT_ERROR		BIT(0)
#define SC_MPPC_INTERRUPT_STATUS_PMC_LPD_ERROR		BIT(1)
#define SC_MPPC_INTERRUPT_STATUS_PMC_MIO_ERROR		BIT(2)
#define SC_MPPC_INTERRUPT_STATUS_FPD_ERROR		BIT(3)
#define SC_MPPC_INTERRUPT_STATUS_SPD_ERROR		BIT(4)
#define SC_MPPC_INTERRUPT_STATUS_PL_ERROR		BIT(5)
#define SC_MPPC_INTERRUPT_STATUS_GTYP_ERROR		BIT(6)
#define SC_MPPC_INTERRUPT_STATUS_POWER_CYCLE_REQUEST	BIT(16)
#define SC_MPPC_INTERRUPT_ENABLE_VDD_OUT		BIT(0)
#define SC_MPPC_INTERRUPT_ENABLE_PMC_LPD		BIT(1)
#define SC_MPPC_INTERRUPT_ENABLE_PMC_MIO		BIT(2)
#define SC_MPPC_INTERRUPT_ENABLE_FPD			BIT(3)
#define SC_MPPC_INTERRUPT_ENABLE_SPD			BIT(4)
#define SC_MPPC_INTERRUPT_ENABLE_PL			BIT(5)
#define SC_MPPC_INTERRUPT_ENABLE_GTYP			BIT(6)
#define SC_MPPC_INTERRUPT_ENABLE_POWER_CYCLE_REQUEST	BIT(16)
#define SC_MPPC_INTERRUPT_ENABLE_USED			SC_MPPC_INTERRUPT_ENABLE_POWER_CYCLE_REQUEST

#define SC_MPPC_WRITE_KEY		0x5a5a0000UL
#define SC_MPPC_RESET_RELEASE_VALUE	(SC_MPPC_WRITE_KEY | 0x0UL)
#define SC_MPPC_RESET_ASSERT_VALUE	(SC_MPPC_WRITE_KEY | 0x1UL)

#define SC_MPPC_POWER_CYCLE_REQUEST_MASK	BIT(0)
#define SC_MPPC_POWER_CYCLE_REQUEST_ASSERT	0x1UL

#define SC_MPPC_PMC_LPD_POWER_CONTROL_ENABLE	BIT(0)
#define SC_MPPC_PMC_MIO_POWER_CONTROL_ENABLE	BIT(1)
#define SC_MPPC_FPD_POWER_CONTROL_ENABLE	BIT(0)
#define SC_MPPC_SPD_POWER_CONTROL_ENABLE	BIT(0)
#define SC_MPPC_PL_POWER_CONTROL_ENABLE		BIT(0)
#define SC_MPPC_GTYP_POWER_CONTROL_ENABLE	BIT(0)

struct sc_mppc_power_control {
	uint32_t reg;
	uint32_t enable_mask;
};

static const struct sc_mppc_power_control power_on_sequence[] = {
	{
		.reg = SC_MPPC_REG_PMC_LPD_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_PMC_LPD_POWER_CONTROL_ENABLE |
			       SC_MPPC_PMC_MIO_POWER_CONTROL_ENABLE,
	},
	{
		.reg = SC_MPPC_REG_FPD_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_FPD_POWER_CONTROL_ENABLE,
	},
	{
		.reg = SC_MPPC_REG_SPD_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_SPD_POWER_CONTROL_ENABLE,
	},
	{
		.reg = SC_MPPC_REG_PL_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_PL_POWER_CONTROL_ENABLE,
	},
	{
		.reg = SC_MPPC_REG_GTYP_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_GTYP_POWER_CONTROL_ENABLE,
	},
};

static const struct sc_mppc_power_control power_off_sequence[] = {
	{
		.reg = SC_MPPC_REG_GTYP_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_GTYP_POWER_CONTROL_ENABLE,
	},
	{
		.reg = SC_MPPC_REG_PL_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_PL_POWER_CONTROL_ENABLE,
	},
	{
		.reg = SC_MPPC_REG_SPD_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_SPD_POWER_CONTROL_ENABLE,
	},
	{
		.reg = SC_MPPC_REG_FPD_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_FPD_POWER_CONTROL_ENABLE,
	},
	{
		.reg = SC_MPPC_REG_PMC_LPD_POWER_CONTROL_STATUS,
		.enable_mask = SC_MPPC_PMC_LPD_POWER_CONTROL_ENABLE |
			       SC_MPPC_PMC_MIO_POWER_CONTROL_ENABLE,
	},
};

static void sc_mppc_write(uint32_t reg, uint32_t val)
{
	sys_write32(val, SC_MPPC_REG_BASE + reg);
}

static uint32_t sc_mppc_read(uint32_t reg)
{
	return sys_read32(SC_MPPC_REG_BASE + reg);
}

static void sc_mppc_interrupt_enable(uint32_t enable_mask)
{
	sc_mppc_write(SC_MPPC_REG_INTERRUPT_ENABLE, enable_mask);
}

static void sc_mppc_interrupt_clear(uint32_t clear_mask)
{
	sc_mppc_write(SC_MPPC_REG_INTERRUPT_STATUS, clear_mask);
}

static int sc_mppc_control_enable(uint32_t reg, uint32_t enable_mask)
{
	uint32_t status;
	int i;

	sc_mppc_write(reg, SC_MPPC_WRITE_KEY | enable_mask);
	k_sleep(K_MSEC(100));

	for (i = 0; i < 100; i++) {
		status = sc_mppc_read(reg);
		if ((status & enable_mask) == enable_mask) {
			return 0;
		}

		k_sleep(K_MSEC(10));
	}

	return -ETIMEDOUT;
}

static int sc_mppc_control_disable(uint32_t reg, uint32_t enable_mask)
{
	uint32_t status;
	int i;

	sc_mppc_write(reg, SC_MPPC_WRITE_KEY);
	k_sleep(K_MSEC(100));

	for (i = 0; i < 100; i++) {
		status = sc_mppc_read(reg);
		if ((status & enable_mask) == 0) {
			return 0;
		}

		k_sleep(K_MSEC(10));
	}

	return -ETIMEDOUT;
}

uint32_t sc_mppc_get_power_status(void)
{
	return sc_mppc_read(SC_MPPC_REG_POWER_STATUS);
}

static bool sc_mppc_power_cycle_requested(void)
{
	uint32_t request;

	request = sc_mppc_read(SC_MPPC_REG_POWER_CYCLE_REQUEST);

	return (request & SC_MPPC_POWER_CYCLE_REQUEST_MASK) ==
	       SC_MPPC_POWER_CYCLE_REQUEST_ASSERT;
}

static void sc_mppc_reset_assert(void)
{
	sc_mppc_write(SC_MPPC_REG_MAIN_PROCESSOR_RESET,
		      SC_MPPC_RESET_ASSERT_VALUE);
}

static void sc_mppc_reset_release(void)
{
	sc_mppc_write(SC_MPPC_REG_MAIN_PROCESSOR_RESET,
		      SC_MPPC_RESET_RELEASE_VALUE);
}

int sc_mppc_main_power_on(void)
{
	int ret;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(power_on_sequence); i++) {
		LOG_DBG("Enabling power control reg 0x%02x", power_on_sequence[i].reg);
		ret = sc_mppc_control_enable(power_on_sequence[i].reg,
					       power_on_sequence[i].enable_mask);
		if (ret < 0) {
			LOG_ERR("Failed to enable power control reg 0x%02x: %d",
				power_on_sequence[i].reg, ret);
			return ret;
		}
	}

	k_sleep(K_USEC(10));
	sc_mppc_reset_release();

	return 0;
}

int sc_mppc_main_power_off(void)
{
	int ret;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(power_off_sequence); i++) {
		LOG_DBG("Disabling power control reg 0x%02x", power_off_sequence[i].reg);
		ret = sc_mppc_control_disable(power_off_sequence[i].reg,
						 power_off_sequence[i].enable_mask);
		if (ret < 0) {
			LOG_ERR("Failed to disable power control reg 0x%02x: %d",
				power_off_sequence[i].reg, ret);
			return ret;
		}
	}

	sc_mppc_reset_assert();

	return 0;
}

void sc_mppc_main_power_cycle(void)
{
	LOG_DBG("Performing power cycle");
	sc_mppc_reset_assert();
	k_sleep(SC_MPPC_RESET_ASSERT_DELAY);
	sc_mppc_reset_release();
}

K_THREAD_STACK_DEFINE(sc_mppc_workq_stack, SC_MPPC_WORKQ_STACK_SIZE);

static struct k_work_q sc_mppc_workq;
static struct k_work sc_mppc_power_cycle_work;
static atomic_t sc_mppc_power_cycle_work_pending;

static void sc_mppc_power_cycle_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (sc_mppc_power_cycle_requested()) {
		LOG_DBG("Power cycle request asserted");
		sc_mppc_main_power_cycle();
	}

	atomic_clear(&sc_mppc_power_cycle_work_pending);
	irq_enable(SC_MPPC_IRQ);
}

static void sc_mppc_submit_power_cycle_work(void)
{
	int ret;

	if (!atomic_cas(&sc_mppc_power_cycle_work_pending, false, true)) {
		return;
	}

	irq_disable(SC_MPPC_IRQ);

	ret = k_work_submit_to_queue(&sc_mppc_workq,
				     &sc_mppc_power_cycle_work);
	if (ret < 0) {
		atomic_clear(&sc_mppc_power_cycle_work_pending);
		irq_enable(SC_MPPC_IRQ);
	}
}

static void sc_mppc_irq_handler(const void *arg)
{
	uint32_t status;

	ARG_UNUSED(arg);

	status = sc_mppc_read(SC_MPPC_REG_INTERRUPT_STATUS);

	if (!(status & SC_MPPC_INTERRUPT_STATUS_POWER_CYCLE_REQUEST)) {
		return;
	}

	sc_mppc_interrupt_clear(SC_MPPC_INTERRUPT_STATUS_POWER_CYCLE_REQUEST);

	sc_mppc_submit_power_cycle_work();
}

static int sc_mppc_init(void)
{
	k_work_queue_start(&sc_mppc_workq, sc_mppc_workq_stack,
			   K_THREAD_STACK_SIZEOF(sc_mppc_workq_stack),
			   SC_MPPC_WORKQ_PRIORITY, NULL);
	k_work_init(&sc_mppc_power_cycle_work, sc_mppc_power_cycle_work_handler);

	atomic_clear(&sc_mppc_power_cycle_work_pending);

	IRQ_CONNECT(SC_MPPC_IRQ, SC_MPPC_IRQ_PRIORITY, sc_mppc_irq_handler,
		    NULL, SC_MPPC_IRQ_FLAGS);

	sc_mppc_interrupt_enable(SC_MPPC_INTERRUPT_ENABLE_USED);
	irq_enable(SC_MPPC_IRQ);

	return 0;
}

SYS_INIT(sc_mppc_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
