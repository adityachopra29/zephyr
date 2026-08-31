/*
 * SPDX-FileCopyrightText: 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * AM13E Input/Output Crossbar (XBAR) MUX driver.
 *
 * Two independent, unrelated XBAR blocks, each its own compatible:
 *
 *   ti,am13e-xbar-input  - INPUTXBAR.  INPUTSELECT[28], one register per
 *                          input channel holding the raw GPIO pin number
 *                          routed onto that channel (or the DRIVE_HIGH/
 *                          DRIVE_LOW pseudo-pin sentinels 0xFE/0xFF).
 *                          cells[0] = channel (addressing); state = pin.
 *
 *                          Per TRM Figure 20-1, each GPIO also feeds the
 *                          crossbar through a RAW_INPUTSELECT_P<A..D> mux
 *                          (one bit per pin) that picks between the raw pad
 *                          signal and that GPIO module's own ".trigger"
 *                          output; reset default is ".trigger", which stays
 *                          low unless something separately drives it. Every
 *                          consumer of this binding wants the raw pad level,
 *                          so .set() always forces that pin's RAW_INPUTSELECT
 *                          bit to 1 in addition to programming INPUTSELECT.
 *
 *   ti,am13e-xbar-output - OUTPUTXBAR. OUTPUTXBAR_GXSEL[12], one pair of
 *                          32-bit one-hot source-select registers
 *                          (G0SEL/G1SEL) per output. source = group << 8
 *                          | bit, group selects G0SEL (0) or G1SEL (1).
 *                          cells[0] = output (addressing); state = source.
 *
 * AM13E has no clock_control driver; XBAR clock/power is already enabled
 * by soc.c's PWREN_XBAR_BIT, so unlike drivers/mux/mux_nxp_xbar.c this
 * driver does not call clock_control_on().
 *
 * Register layout/semantics per am13e230x_sdk's hw_xbar.h / dl_xbar.h.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/mux.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(mux_am13e_xbar, CONFIG_MUX_LOG_LEVEL);

struct mux_am13e_xbar_data {
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
};

static int mux_am13e_xbar_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

#if DT_HAS_COMPAT_STATUS_OKAY(ti_am13e_xbar_input)

#define AM13E_XBAR_INPUT_NUM_CHANNELS 28U
#define AM13E_XBAR_INPUT_SELECT(ch)   ((ch) * 4U)

#define AM13E_XBAR_RAW_INPUTSELECT_PA        0x410U
#define AM13E_XBAR_RAW_INPUTSELECT_STRIDE    4U
#define AM13E_XBAR_RAW_INPUTSELECT_NUM_PORTS 4U
#define AM13E_XBAR_RAW_INPUTSELECT(port)                                                           \
	(AM13E_XBAR_RAW_INPUTSELECT_PA + (port) * AM13E_XBAR_RAW_INPUTSELECT_STRIDE)

static int mux_am13e_xbar_input_set(const struct device *dev, const struct mux_control *control,
				    uint32_t state)
{
	struct mux_am13e_xbar_data *data = dev->data;
	uint32_t base = DEVICE_MMIO_GET(dev);
	uint32_t channel = control->cells[0];
	k_spinlock_key_t key;

	if (channel >= AM13E_XBAR_INPUT_NUM_CHANNELS) {
		LOG_ERR("input xbar channel %u out of range", channel);
		return -EINVAL;
	}

	if (state > 0xFFU) {
		LOG_ERR("input xbar pin 0x%x exceeds 8-bit field", state);
		return -EINVAL;
	}

	sys_write32(state, base + AM13E_XBAR_INPUT_SELECT(channel));

	if (state / 32U < AM13E_XBAR_RAW_INPUTSELECT_NUM_PORTS) {
		uint32_t port = state / 32U;
		uint32_t bit = state % 32U;
		uint32_t reg = base + AM13E_XBAR_RAW_INPUTSELECT(port);

		key = k_spin_lock(&data->lock);
		sys_write32(sys_read32(reg) | BIT(bit), reg);
		k_spin_unlock(&data->lock, key);
	}

	LOG_DBG("input xbar: channel=%u <- pin=%u", channel, state);

	return 0;
}

static int mux_am13e_xbar_input_get_state(const struct device *dev,
					  const struct mux_control *control, uint32_t *state)
{
	uint32_t base = DEVICE_MMIO_GET(dev);
	uint32_t channel = control->cells[0];

	if (channel >= AM13E_XBAR_INPUT_NUM_CHANNELS) {
		LOG_ERR("input xbar channel %u out of range", channel);
		return -EINVAL;
	}

	*state = sys_read32(base + AM13E_XBAR_INPUT_SELECT(channel));

	return 0;
}

static DEVICE_API(mux_control, mux_am13e_xbar_input_driver_api) = {
	.set = mux_am13e_xbar_input_set,
	.get_state = mux_am13e_xbar_input_get_state,
};

#define AM13E_XBAR_INPUT_INIT(inst)                                                                \
	static struct mux_am13e_xbar_data mux_am13e_xbar_input_data_##inst;                        \
	static const struct {                                                                      \
		DEVICE_MMIO_ROM;                                                                   \
	} mux_am13e_xbar_input_cfg_##inst = {                                                      \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mux_am13e_xbar_init, NULL, &mux_am13e_xbar_input_data_##inst,  \
			      &mux_am13e_xbar_input_cfg_##inst, POST_KERNEL,                       \
			      CONFIG_MUX_INIT_PRIORITY, &mux_am13e_xbar_input_driver_api);

#define DT_DRV_COMPAT ti_am13e_xbar_input
DT_INST_FOREACH_STATUS_OKAY(AM13E_XBAR_INPUT_INIT)
#undef DT_DRV_COMPAT

#endif /* DT_HAS_COMPAT_STATUS_OKAY(ti_am13e_xbar_input) */

#if DT_HAS_COMPAT_STATUS_OKAY(ti_am13e_xbar_output)

#define AM13E_XBAR_OUTPUT_NUM_OUTPUTS  12U
#define AM13E_XBAR_OUTPUT_GXSEL_BASE   0x100U
#define AM13E_XBAR_OUTPUT_GXSEL_STRIDE 0x40U
#define AM13E_XBAR_OUTPUT_G0SEL(out)                                                               \
	(AM13E_XBAR_OUTPUT_GXSEL_BASE + (out) * AM13E_XBAR_OUTPUT_GXSEL_STRIDE)
#define AM13E_XBAR_OUTPUT_G1SEL(out) (AM13E_XBAR_OUTPUT_G0SEL(out) + 4U)

static int mux_am13e_xbar_output_set(const struct device *dev, const struct mux_control *control,
				     uint32_t state)
{
	struct mux_am13e_xbar_data *data = dev->data;
	uint32_t base = DEVICE_MMIO_GET(dev);
	uint32_t output = control->cells[0];
	uint32_t group = state >> 8;
	uint32_t bit = state & 0xFFU;
	k_spinlock_key_t key;

	if (output >= AM13E_XBAR_OUTPUT_NUM_OUTPUTS) {
		LOG_ERR("output xbar output %u out of range", output);
		return -EINVAL;
	}

	if (group > 1U || bit > 31U) {
		LOG_ERR("output xbar source 0x%x out of range", state);
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	/* One-hot OR-tree: only one source may drive an output at a time, so
	 * clear both group registers before setting the requested source bit.
	 */
	sys_write32(0U, base + AM13E_XBAR_OUTPUT_G0SEL(output));
	sys_write32(0U, base + AM13E_XBAR_OUTPUT_G1SEL(output));
	sys_write32(BIT(bit), base + AM13E_XBAR_OUTPUT_G0SEL(output) + 4U * group);

	k_spin_unlock(&data->lock, key);

	LOG_DBG("output xbar routed: out=%u <- source=0x%x", output, state);

	return 0;
}

static DEVICE_API(mux_control, mux_am13e_xbar_output_driver_api) = {
	.set = mux_am13e_xbar_output_set,
};

#define AM13E_XBAR_OUTPUT_INIT(inst)                                                               \
	static struct mux_am13e_xbar_data mux_am13e_xbar_output_data_##inst;                       \
	static const struct {                                                                      \
		DEVICE_MMIO_ROM;                                                                   \
	} mux_am13e_xbar_output_cfg_##inst = {                                                     \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mux_am13e_xbar_init, NULL, &mux_am13e_xbar_output_data_##inst, \
			      &mux_am13e_xbar_output_cfg_##inst, POST_KERNEL,                      \
			      CONFIG_MUX_INIT_PRIORITY, &mux_am13e_xbar_output_driver_api);

#define DT_DRV_COMPAT ti_am13e_xbar_output
DT_INST_FOREACH_STATUS_OKAY(AM13E_XBAR_OUTPUT_INIT)
#undef DT_DRV_COMPAT

#endif /* DT_HAS_COMPAT_STATUS_OKAY(ti_am13e_xbar_output) */
