/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Texas Instruments
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mspm0_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <soc.h>

LOG_MODULE_REGISTER(ti_mcpwm);

#define DT_DRV_COMPAT ti_am13e2_mcpwm

#define TI_MCPWM_PERIOD_CYCLES_MAX (0xFFFF)
#define TI_MCPWM_NUM_CHANNELS      (6)
#define TI_MCPWM_NUM_PAIRS         (3)

#define TI_MCPWM_SYSCTL_PWREN_KEY (0x26U << 24)

/* Pair register block bases relative to MCPWM instance */
#define TI_MCPWM_PAIR_OFFSET(pair) (0x100U + ((pair) * 0x200U))

struct ti_mcpwm_pair_regs {
	volatile uint32_t CMPA;     /**< Counter-Compare A, offset: 0x00 */
	volatile uint32_t CMPAS;    /**< Counter-Compare A Shadow, offset: 0x04 */
	volatile uint32_t CMPB;     /**< Counter-Compare B, offset: 0x08 */
	volatile uint32_t CMPBS;    /**< Counter-Compare B Shadow, offset: 0x0C */
	uint8_t RESERVED_0[0x10];   /**< Reserved, offset: 0x10 - 0x1C */
	volatile uint32_t AQCTLA;   /**< AQ Control A, offset: 0x20 */
	volatile uint32_t AQCTLAS;  /**< AQ Control A Shadow, offset: 0x24 */
	volatile uint32_t AQCTLB;   /**< AQ Control B, offset: 0x28 */
	volatile uint32_t AQCTLBS;  /**< AQ Control B Shadow, offset: 0x2C */
	volatile uint32_t AQSFRC;   /**< AQ Software Force, offset: 0x30 */
	volatile uint32_t AQOTSFRC; /**< AQ One-Time Software Force, offset: 0x34 */
};

struct ti_mcpwm_regs {
	uint8_t RESERVED_0[0x10];  /**< Reserved, offset: 0x00 - 0x0C */
	volatile uint32_t TBCTL;   /**< Time-Base Control, offset: 0x10 */
	volatile uint32_t TBPRD;   /**< Time-Base Period, offset: 0x14 */
	volatile uint32_t TBPRDS;  /**< Time-Base Period Shadow, offset: 0x18 */
	uint8_t RESERVED_1[0x14];  /**< Reserved, offset: 0x1C - 0x2C */
	volatile uint32_t CMPCTL;  /**< Counter-Compare Control, offset: 0x30 */
	uint8_t RESERVED_2[0x1C];  /**< Reserved, offset: 0x34 - 0x4C */
	volatile uint32_t AQCTL;   /**< Action Qualifier Control, offset: 0x50 */
};

/* TBCTL */
#define TI_MCPWM_TBCTL_CTRMODE         GENMASK(1, 0)
#define TI_MCPWM_TBCTL_CTRMODE_UP      (0)
#define TI_MCPWM_TBCTL_CTRMODE_UP_DOWN (1)
#define TI_MCPWM_TBCTL_CTRMODE_STOP    (2)
#define TI_MCPWM_TBCTL_CLKDIV          GENMASK(5, 2)
#define TI_MCPWM_TBCTL_CLKDIV_MAX      (15)
#define TI_MCPWM_TBCTL_PRDLD           BIT(8)

/* CMPCTL — shadow-to-active load mode per compare (0 = load on CTR=0) */
#define TI_MCPWM_CMPCTL_LOAD_ZERO      (0)
#define TI_MCPWM_CMPCTL_PWM1_LOADAMODE GENMASK(1, 0)
#define TI_MCPWM_CMPCTL_PWM1_LOADBMODE GENMASK(3, 2)
#define TI_MCPWM_CMPCTL_PWM2_LOADAMODE GENMASK(9, 8)
#define TI_MCPWM_CMPCTL_PWM2_LOADBMODE GENMASK(11, 10)
#define TI_MCPWM_CMPCTL_PWM3_LOADAMODE GENMASK(17, 16)
#define TI_MCPWM_CMPCTL_PWM3_LOADBMODE GENMASK(19, 18)

/* AQCTL — shadow-to-active load mode per AQ (0 = load on CTR=0) */
#define TI_MCPWM_AQCTL_LOAD_ZERO      (0)
#define TI_MCPWM_AQCTL_PWM1_LDAQAMODE GENMASK(1, 0)
#define TI_MCPWM_AQCTL_PWM1_LDAQBMODE GENMASK(3, 2)
#define TI_MCPWM_AQCTL_PWM2_LDAQAMODE GENMASK(9, 8)
#define TI_MCPWM_AQCTL_PWM2_LDAQBMODE GENMASK(11, 10)
#define TI_MCPWM_AQCTL_PWM3_LDAQAMODE GENMASK(17, 16)
#define TI_MCPWM_AQCTL_PWM3_LDAQBMODE GENMASK(19, 18)

/* AQCTLA / AQCTLB action fields (same layout for all pairs) */
#define TI_MCPWM_AQCTL_ZRO           GENMASK(1, 0)
#define TI_MCPWM_AQCTL_PRD           GENMASK(3, 2)
#define TI_MCPWM_AQCTL_CAU           GENMASK(5, 4)
#define TI_MCPWM_AQCTL_CAD           GENMASK(7, 6)
#define TI_MCPWM_AQCTL_CBU           GENMASK(9, 8)
#define TI_MCPWM_AQCTL_CBD           GENMASK(11, 10)
#define TI_MCPWM_AQCTL_FLD_NO_CHANGE (0)
#define TI_MCPWM_AQCTL_FLD_CLR       (1)
#define TI_MCPWM_AQCTL_FLD_SET       (2)
#define TI_MCPWM_AQCTL_FLD_TOGGLE    (3)

/* AQSFRC — continuous / one-shot software force */
#define TI_MCPWM_AQSFRC_PWMA            GENMASK(2, 0)
#define TI_MCPWM_AQSFRC_PWMB            GENMASK(6, 4)
#define TI_MCPWM_AQSFRC_DISABLED        (0)
#define TI_MCPWM_AQSFRC_CONTINUOUS_LOW  (1)
#define TI_MCPWM_AQSFRC_CONTINUOUS_HIGH (2)

#define DEV_CFG(dev)  ((const struct ti_mcpwm_cfg *)(dev)->config)
#define DEV_DATA(dev) ((struct ti_mcpwm_data *)(dev)->data)
#define DEV_REGS(dev) ((struct ti_mcpwm_regs *)DEVICE_MMIO_GET(dev))

struct ti_mcpwm_cfg {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	struct mspm0_sys_clock clock_subsys;
	const struct device *pwren_syscon;
	uint32_t pwren_offset;
	uint8_t pwren_bit;
	const struct device *tbclk_syscon;
	uint32_t tbclk_offset;
	uint8_t tbclk_bit;
	const struct pinctrl_dev_config *pcfg;
};

struct ti_mcpwm_data {
	DEVICE_MMIO_RAM;
	uint32_t period_cycles[TI_MCPWM_NUM_CHANNELS];
	uint32_t prescale_div;
	uint8_t active_mask;
	bool tbclk_held;
	bool symmetric;
};

/* Global MCPWM TBCLKSYNC gate — shared by every instance */
static atomic_t ti_mcpwm_tbclk_refcount;

static struct ti_mcpwm_pair_regs *ti_mcpwm_pair(struct ti_mcpwm_regs *regs, uint32_t pair)
{
	return (struct ti_mcpwm_pair_regs *)((uint8_t *)regs + TI_MCPWM_PAIR_OFFSET(pair));
}

static int ti_mcpwm_pwren_enable(const struct ti_mcpwm_cfg *cfg)
{
	uint32_t mask;
	int ret;

	if (!device_is_ready(cfg->pwren_syscon)) {
		LOG_ERR("SYSCTL (pwren) device not ready");
		return -ENODEV;
	}

	mask = BIT(cfg->pwren_bit) | TI_MCPWM_SYSCTL_PWREN_KEY;
	ret = syscon_update_bits(cfg->pwren_syscon, cfg->pwren_offset, mask, mask);
	if (ret < 0) {
		LOG_ERR("failed to set SYSCTL PWREN bit %u: %d", cfg->pwren_bit, ret);
		return ret;
	}

	msp_delay_peripheral_startup();

	return 0;
}

static int ti_mcpwm_tbclk_acquire(const struct ti_mcpwm_cfg *cfg)
{
	uint32_t tbclk_reg;
	int err;

	if (atomic_inc(&ti_mcpwm_tbclk_refcount) != 0) {
		return 0;
	}

	if (!device_is_ready(cfg->tbclk_syscon)) {
		LOG_ERR("SYSCTL (tbclk) device not ready");
		atomic_dec(&ti_mcpwm_tbclk_refcount);
		return -ENODEV;
	}

	err = syscon_read_reg(cfg->tbclk_syscon, cfg->tbclk_offset, &tbclk_reg);
	if (err != 0) {
		LOG_ERR("failed to read TBCLKSYNC register");
		atomic_dec(&ti_mcpwm_tbclk_refcount);
		return err;
	}

	tbclk_reg |= BIT(cfg->tbclk_bit);
	err = syscon_write_reg(cfg->tbclk_syscon, cfg->tbclk_offset, tbclk_reg);
	if (err != 0) {
		LOG_ERR("failed to set TBCLKSYNC");
		atomic_dec(&ti_mcpwm_tbclk_refcount);
		return err;
	}

	return 0;
}

static int ti_mcpwm_tbclk_release(const struct ti_mcpwm_cfg *cfg)
{
	uint32_t tbclk_reg;
	int err;

	if (atomic_dec(&ti_mcpwm_tbclk_refcount) != 1) {
		return 0;
	}

	err = syscon_read_reg(cfg->tbclk_syscon, cfg->tbclk_offset, &tbclk_reg);
	if (err != 0) {
		LOG_ERR("failed to read TBCLKSYNC register");
		return err;
	}

	tbclk_reg &= ~BIT(cfg->tbclk_bit);
	err = syscon_write_reg(cfg->tbclk_syscon, cfg->tbclk_offset, tbclk_reg);
	if (err != 0) {
		LOG_ERR("failed to clear TBCLKSYNC");
		return err;
	}

	return 0;
}

static void ti_mcpwm_aq_force(struct ti_mcpwm_pair_regs *pair, bool is_b, uint32_t force)
{
	uint32_t aqsfrc = pair->AQSFRC;

	if (is_b) {
		aqsfrc &= ~TI_MCPWM_AQSFRC_PWMB;
		aqsfrc |= FIELD_PREP(TI_MCPWM_AQSFRC_PWMB, force);
	} else {
		aqsfrc &= ~TI_MCPWM_AQSFRC_PWMA;
		aqsfrc |= FIELD_PREP(TI_MCPWM_AQSFRC_PWMA, force);
	}

	pair->AQSFRC = aqsfrc;
}

static int ti_mcpwm_configure_tbctl(const struct device *dev, uint32_t period_cycles)
{
	struct ti_mcpwm_regs *regs = DEV_REGS(dev);
	struct ti_mcpwm_data *data = DEV_DATA(dev);
	uint32_t tbctl;
	uint32_t prescale_div;

	tbctl = regs->TBCTL;

	/* shadow load TBPRDS → TBPRD on CTR = 0 */
	tbctl &= ~TI_MCPWM_TBCTL_PRDLD;

	tbctl &= ~TI_MCPWM_TBCTL_CTRMODE;
	if (data->symmetric) {
		tbctl |= FIELD_PREP(TI_MCPWM_TBCTL_CTRMODE, TI_MCPWM_TBCTL_CTRMODE_UP_DOWN);
	} else {
		tbctl |= FIELD_PREP(TI_MCPWM_TBCTL_CTRMODE, TI_MCPWM_TBCTL_CTRMODE_UP);
	}

	for (uint32_t clkdiv = 0; clkdiv <= TI_MCPWM_TBCTL_CLKDIV_MAX; clkdiv++) {
		prescale_div = BIT(clkdiv);

		if (period_cycles <= (uint32_t)TI_MCPWM_PERIOD_CYCLES_MAX * prescale_div) {
			tbctl &= ~TI_MCPWM_TBCTL_CLKDIV;
			tbctl |= FIELD_PREP(TI_MCPWM_TBCTL_CLKDIV, clkdiv);
			regs->TBCTL = tbctl;
			data->prescale_div = prescale_div;
			return 0;
		}
	}

	return -EINVAL;
}

static void ti_mcpwm_configure_aq(struct ti_mcpwm_pair_regs *pair, bool is_b, bool symmetric,
				  bool polarity)
{
	uint32_t upmask = is_b ? TI_MCPWM_AQCTL_CBU : TI_MCPWM_AQCTL_CAU;
	uint32_t downmask = is_b ? TI_MCPWM_AQCTL_CBD : TI_MCPWM_AQCTL_CAD;
	uint32_t aqctl = is_b ? pair->AQCTLB : pair->AQCTLA;

	aqctl &= ~(TI_MCPWM_AQCTL_ZRO | TI_MCPWM_AQCTL_PRD | upmask | downmask);

	if (polarity == PWM_POLARITY_NORMAL) {
		aqctl |= FIELD_PREP(upmask, TI_MCPWM_AQCTL_FLD_CLR);
		aqctl &= ~TI_MCPWM_AQCTL_PRD;

		if (symmetric) {
			aqctl &= ~TI_MCPWM_AQCTL_ZRO;
			aqctl |= FIELD_PREP(downmask, TI_MCPWM_AQCTL_FLD_SET);
		} else {
			aqctl |= FIELD_PREP(TI_MCPWM_AQCTL_ZRO, TI_MCPWM_AQCTL_FLD_SET);
			aqctl &= ~downmask;
		}
	} else {
		aqctl |= FIELD_PREP(upmask, TI_MCPWM_AQCTL_FLD_SET);
		aqctl &= ~TI_MCPWM_AQCTL_ZRO;

		if (symmetric) {
			aqctl &= ~TI_MCPWM_AQCTL_PRD;
			aqctl |= FIELD_PREP(downmask, TI_MCPWM_AQCTL_FLD_CLR);
		} else {
			aqctl |= FIELD_PREP(TI_MCPWM_AQCTL_PRD, TI_MCPWM_AQCTL_FLD_CLR);
			aqctl &= ~downmask;
		}
	}

	if (is_b) {
		pair->AQCTLB = aqctl;
		pair->AQCTLBS = aqctl;
	} else {
		pair->AQCTLA = aqctl;
		pair->AQCTLAS = aqctl;
	}
}

static int ti_mcpwm_enable_channel(const struct device *dev, uint32_t channel)
{
	struct ti_mcpwm_data *data = DEV_DATA(dev);
	struct ti_mcpwm_regs *regs = DEV_REGS(dev);
	struct ti_mcpwm_pair_regs *pair = ti_mcpwm_pair(regs, channel / 2U);
	bool is_b = (channel % 2U) != 0U;
	int err;

	ti_mcpwm_aq_force(pair, is_b, TI_MCPWM_AQSFRC_DISABLED);

	data->active_mask |= BIT(channel);

	if (data->tbclk_held) {
		return 0;
	}

	err = ti_mcpwm_tbclk_acquire(DEV_CFG(dev));
	if (err != 0) {
		data->active_mask &= ~BIT(channel);
		return err;
	}

	data->tbclk_held = true;
	return 0;
}

static int ti_mcpwm_disable_channel(const struct device *dev, uint32_t channel)
{
	struct ti_mcpwm_data *data = DEV_DATA(dev);
	struct ti_mcpwm_regs *regs = DEV_REGS(dev);
	struct ti_mcpwm_pair_regs *pair = ti_mcpwm_pair(regs, channel / 2U);
	bool is_b = (channel % 2U) != 0U;
	int err = 0;

	ti_mcpwm_aq_force(pair, is_b, TI_MCPWM_AQSFRC_CONTINUOUS_LOW);

	data->period_cycles[channel] = 0;
	data->active_mask &= ~BIT(channel);

	if (data->active_mask != 0U || !data->tbclk_held) {
		return 0;
	}

	err = ti_mcpwm_tbclk_release(DEV_CFG(dev));
	if (err == 0) {
		data->tbclk_held = false;
	}

	return err;
}

static int ti_mcpwm_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			       uint32_t pulse_cycles, pwm_flags_t flags)
{
	struct ti_mcpwm_regs *regs = DEV_REGS(dev);
	struct ti_mcpwm_data *data = DEV_DATA(dev);
	struct ti_mcpwm_pair_regs *pair;
	uint32_t tb_period;
	uint32_t tb_pulse;
	bool is_b;
	int err;

	if (channel >= TI_MCPWM_NUM_CHANNELS) {
		LOG_ERR("invalid channel number %u", channel);
		return -EINVAL;
	}

	for (uint32_t i = 0; i < TI_MCPWM_NUM_CHANNELS; i++) {
		if (i != channel && data->period_cycles[i] != 0U &&
		    data->period_cycles[i] != period_cycles) {
			LOG_ERR("period value must be same as other channels");
			return -EINVAL;
		}
	}

	if (period_cycles == 0U) {
		err = ti_mcpwm_disable_channel(dev, channel);
		if (err != 0) {
			LOG_ERR("failed to disable MCPWM channel %u", channel);
			return err;
		}
		return 0;
	}

	if (pulse_cycles > period_cycles) {
		pulse_cycles = period_cycles;
	}

	err = ti_mcpwm_enable_channel(dev, channel);
	if (err != 0) {
		return err;
	}

	pair = ti_mcpwm_pair(regs, channel / 2U);
	is_b = (channel % 2U) != 0U;

	ti_mcpwm_configure_aq(pair, is_b, data->symmetric, flags & PWM_POLARITY_MASK);

	err = ti_mcpwm_configure_tbctl(dev, period_cycles);
	if (err != 0) {
		LOG_ERR("failed to configure clock prescaler");
		return err;
	}

	tb_period = period_cycles / data->prescale_div;
	tb_pulse = pulse_cycles / data->prescale_div;

	if (data->symmetric) {
		tb_period /= 2U;
		tb_pulse /= 2U;
	}

	/* active + shadow so the first cycle is correct and later loads stay in sync */
	regs->TBPRD = tb_period;
	regs->TBPRDS = tb_period;

	if (is_b) {
		pair->CMPB = tb_pulse;
		pair->CMPBS = tb_pulse;
	} else {
		pair->CMPA = tb_pulse;
		pair->CMPAS = tb_pulse;
	}

	data->period_cycles[channel] = period_cycles;
	return 0;
}

static int ti_mcpwm_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
	const struct ti_mcpwm_cfg *cfg = DEV_CFG(dev);
	struct mspm0_sys_clock clock_subsys = cfg->clock_subsys;
	uint32_t rate;
	int ret;

	ARG_UNUSED(channel);

	if (cycles == NULL) {
		return -EINVAL;
	}

	ret = clock_control_get_rate(cfg->clock_dev, (clock_control_subsys_t)&clock_subsys, &rate);
	if (ret < 0) {
		return ret;
	}

	*cycles = rate;
	return 0;
}

static int ti_mcpwm_init(const struct device *dev)
{
	const struct ti_mcpwm_cfg *cfg = DEV_CFG(dev);
	struct ti_mcpwm_regs *regs;
	int ret;

	ret = ti_mcpwm_pwren_enable(cfg);
	if (ret < 0) {
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	regs = DEV_REGS(dev);

	/* Shadow → active on CTR = 0 for all compares and AQ actions */
	regs->CMPCTL = FIELD_PREP(TI_MCPWM_CMPCTL_PWM1_LOADAMODE, TI_MCPWM_CMPCTL_LOAD_ZERO) |
		       FIELD_PREP(TI_MCPWM_CMPCTL_PWM1_LOADBMODE, TI_MCPWM_CMPCTL_LOAD_ZERO) |
		       FIELD_PREP(TI_MCPWM_CMPCTL_PWM2_LOADAMODE, TI_MCPWM_CMPCTL_LOAD_ZERO) |
		       FIELD_PREP(TI_MCPWM_CMPCTL_PWM2_LOADBMODE, TI_MCPWM_CMPCTL_LOAD_ZERO) |
		       FIELD_PREP(TI_MCPWM_CMPCTL_PWM3_LOADAMODE, TI_MCPWM_CMPCTL_LOAD_ZERO) |
		       FIELD_PREP(TI_MCPWM_CMPCTL_PWM3_LOADBMODE, TI_MCPWM_CMPCTL_LOAD_ZERO);

	regs->AQCTL = FIELD_PREP(TI_MCPWM_AQCTL_PWM1_LDAQAMODE, TI_MCPWM_AQCTL_LOAD_ZERO) |
		      FIELD_PREP(TI_MCPWM_AQCTL_PWM1_LDAQBMODE, TI_MCPWM_AQCTL_LOAD_ZERO) |
		      FIELD_PREP(TI_MCPWM_AQCTL_PWM2_LDAQAMODE, TI_MCPWM_AQCTL_LOAD_ZERO) |
		      FIELD_PREP(TI_MCPWM_AQCTL_PWM2_LDAQBMODE, TI_MCPWM_AQCTL_LOAD_ZERO) |
		      FIELD_PREP(TI_MCPWM_AQCTL_PWM3_LDAQAMODE, TI_MCPWM_AQCTL_LOAD_ZERO) |
		      FIELD_PREP(TI_MCPWM_AQCTL_PWM3_LDAQBMODE, TI_MCPWM_AQCTL_LOAD_ZERO);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("failed to configure pinctrl: %d", ret);
		return ret;
	}

	return 0;
}

static DEVICE_API(pwm, ti_mcpwm_api) = {
	.set_cycles = ti_mcpwm_set_cycles,
	.get_cycles_per_sec = ti_mcpwm_get_cycles_per_sec,
};

#define TI_MCPWM_INIT(n)                                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct ti_mcpwm_cfg ti_mcpwm_config_##n = {                                   \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = MSPM0_CLOCK_SUBSYS_FN(n),                                          \
		.pwren_syscon = DEVICE_DT_GET(DT_INST_PHANDLE(n, ti_pwren)),                       \
		.pwren_offset = DT_INST_PHA(n, ti_pwren, offset),                                  \
		.pwren_bit = DT_INST_PHA(n, ti_pwren, bit),                                        \
		.tbclk_syscon = DEVICE_DT_GET(DT_INST_PHANDLE(n, ti_tbclk)),                       \
		.tbclk_offset = DT_INST_PHA(n, ti_tbclk, offset),                                  \
		.tbclk_bit = DT_INST_PHA(n, ti_tbclk, bit),                                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
                                                                                                   \
	static struct ti_mcpwm_data ti_mcpwm_data_##n = {                                          \
		.symmetric = DT_INST_PROP(n, symmetric),                                           \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ti_mcpwm_init, NULL, &ti_mcpwm_data_##n, &ti_mcpwm_config_##n,    \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &ti_mcpwm_api);

DT_INST_FOREACH_STATUS_OKAY(TI_MCPWM_INIT)
