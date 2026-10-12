/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Texas Instruments
 * SPDX-License-Identifier: Apache-2.0
 *
 * EQEP position/speed via index period (lp_am13e230).
 *
 * INDEX alarm reports latched position each simulated revolution. A local
 * GPIO pulses index once every LINES_PER_REV electrical periods (k_timer at
 * the A/B electrical rate). A/B come from an external MCPWM quadrature
 * source; jumper PA2 (index GPIO out) -> PB18 (EQEP0I).
 *
 * Wiring:
 *   MCPWM PB3 (A) -> PB11 (EQEP0A)
 *   MCPWM PB1 (B) -> PB12 (EQEP0B)
 *   PA2 (index)   -> PB18 (EQEP0I)   on-board jumper
 *   GND           -> GND
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/counter/ti_am3352_eqep.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define EQEP_NODE DT_ALIAS(eqep0)

/* Decoder is configured for 4x counting (all edges of both A and B). */
#define EQEP_COUNTS_PER_REV (CONFIG_APP_EQEP_LINES_PER_REV * 4U)

static const struct gpio_dt_spec index_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), index_gpios);

static struct counter_alarm_cfg index_alarm_cfg;

static uint32_t period_count;
static int64_t old_pos;
static bool have_prev_index;

/*
 * INDEX alarm: position is QPOSILAT at the index edge. RPM from Δposition
 * over one rev using electrical freq and lines/rev.
 */
static void eqep_index_callback(const struct device *dev, uint8_t chan_id, uint32_t position,
				void *user_data)
{
	ARG_UNUSED(user_data);

	if (have_prev_index) {
		int64_t rpm = ((old_pos - (int64_t)position) *
			       (int64_t)CONFIG_APP_EQEP_ELECTRICAL_FREQ_HZ * 60) /
			      ((int64_t)CONFIG_APP_EQEP_LINES_PER_REV *
			       (int64_t)EQEP_COUNTS_PER_REV);

		printk("Current position: %d, RPM : %lld\n", (int)position, (long long)rpm);
	} else {
		printk("Current position: %d, RPM : (first index)\n", (int)position);
		have_prev_index = true;
	}

	old_pos = (int64_t)position;

	/* Alarms are one-shot: re-arm INDEX for the next revolution. */
	(void)counter_set_channel_alarm(dev, chan_id, &index_alarm_cfg);
}

/*
 * One tick per simulated electrical cycle. Once per simulated revolution,
 * pulse the index GPIO for exactly one electrical period.
 */
static void index_pulse_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	period_count++;
	if (period_count >= CONFIG_APP_EQEP_LINES_PER_REV) {
		period_count = 0;
		gpio_pin_set_dt(&index_gpio, 1);
	} else if (period_count == 1U) {
		gpio_pin_set_dt(&index_gpio, 0);
	}
}

K_TIMER_DEFINE(electrical_period_timer, index_pulse_handler, NULL);

int main(void)
{
	const struct device *const eqep_dev = DEVICE_DT_GET(EQEP_NODE);
	const struct ti_eqep_dec_cfg dec_cfg = {
		.source = TI_EQEP_SRC_QUADRATURE,
		.rising_edge_only = false,
		.swap_inputs = false,
	};
	const struct ti_eqep_qep_cfg qep_cfg = {
		.reset_mode = TI_EQEP_RESET_MODE_MAX,
		.index_latch = TI_EQEP_INDEX_LATCH_RISING,
		.strobe_latch = TI_EQEP_STROBE_LATCH_RISING,
		.capture_latch = TI_EQEP_CAPTURE_LATCH_TIMEOUT,
	};
	struct counter_top_cfg top_cfg = {
		.ticks = 0xFFFFFFFFU,
		.flags = 0U,
	};
	uint32_t period_us;
	int err;

	printk("EQEP position/speed (index window) on %s\n", CONFIG_BOARD_TARGET);
	printk("Expected speed: %u RPM\n",
	       CONFIG_APP_EQEP_ELECTRICAL_FREQ_HZ * 60U / CONFIG_APP_EQEP_LINES_PER_REV);
	printk("Lines/rev %u → %u counts/rev (4x); electrical %u Hz\n",
	       CONFIG_APP_EQEP_LINES_PER_REV, EQEP_COUNTS_PER_REV,
	       CONFIG_APP_EQEP_ELECTRICAL_FREQ_HZ);
	printk("Wire MCPWM A->PB11, B->PB12; jumper PA2(index)->PB18; share GND\n");

	if (!device_is_ready(eqep_dev) || !gpio_is_ready_dt(&index_gpio)) {
		printk("A required device is not ready\n");
		return 0;
	}

	printk("eqep0: frequency=%u Hz\n", counter_get_frequency(eqep_dev));

	/* Hold index low before arming rising-edge latch to avoid a spurious edge. */
	err = gpio_pin_configure_dt(&index_gpio, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printk("Failed to configure index GPIO (err %d)\n", err);
		return 0;
	}

	ti_eqep_configure_decoder(eqep_dev, &dec_cfg);
	ti_eqep_configure_qep(eqep_dev, &qep_cfg);

	err = counter_set_top_value(eqep_dev, &top_cfg);
	if (err != 0) {
		printk("Failed to set top value (err %d)\n", err);
		return 0;
	}

	err = counter_start(eqep_dev);
	if (err != 0) {
		printk("Failed to start counter (err %d)\n", err);
		return 0;
	}

	index_alarm_cfg.callback = eqep_index_callback;
	index_alarm_cfg.user_data = NULL;
	index_alarm_cfg.flags = 0;

	err = counter_set_channel_alarm(eqep_dev, TI_EQEP_ALARM_CHAN_INDEX, &index_alarm_cfg);
	if (err != 0) {
		printk("Failed to enable EQEP index alarm (err %d)\n", err);
		return 0;
	}

	/* One tick per electrical cycle; index pulse every LINES_PER_REV ticks. */
	period_us = USEC_PER_SEC / CONFIG_APP_EQEP_ELECTRICAL_FREQ_HZ;
	k_timer_start(&electrical_period_timer, K_USEC(period_us), K_USEC(period_us));

	printk("Reporting position/speed on each simulated index edge...\n");

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
