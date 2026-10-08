/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Texas Instruments
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/counter/ti_am3352_eqep.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define SIGNAL_FREQ_HZ 100U
/* One high step followed by three low steps gives a 25% duty cycle. */
#define SIGNAL_STEPS   4U
#define SIGNAL_STEP_US (USEC_PER_SEC / (SIGNAL_FREQ_HZ * SIGNAL_STEPS))

/* Kept small so the counter wraps every few windows, exercising the
 * wraparound path. It must exceed the counts seen in a single window.
 */
#define POSITION_TOP 999U
BUILD_ASSERT(POSITION_TOP > 2U * SIGNAL_FREQ_HZ, "position counter wraps within a window");

/* The software-timed signal may gain or lose an edge per window. */
#define FREQ_TOLERANCE_HZ 1U

struct fr_sample {
	uint32_t position;
	uint32_t delta;
	bool wrapped;
};

static const struct device *const eqep = DEVICE_DT_GET(DT_ALIAS(eqep0));
static const struct gpio_dt_spec signal = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), signal_gpios);

K_MSGQ_DEFINE(fr_msgq, sizeof(struct fr_sample), 8, 4);

static struct counter_alarm_cfg window_cfg;
static uint32_t prev_position;

static void signal_step(struct k_timer *timer)
{
	static uint32_t step;

	ARG_UNUSED(timer);

	gpio_pin_set_dt(&signal, step == 0U);
	step = (step + 1U) % SIGNAL_STEPS;
}

K_TIMER_DEFINE(signal_timer, signal_step, NULL);

/* The position counter runs 0..top and then restarts from 0. */
static uint32_t position_delta(uint32_t prev, uint32_t cur, uint32_t top, bool *wrapped)
{
	*wrapped = cur < prev;

	if (!*wrapped) {
		return cur - prev;
	}

	return (top - prev) + cur + 1U;
}

static void window_expired(const struct device *dev, uint8_t chan_id, uint32_t ticks,
			   void *user_data)
{
	struct fr_sample sample = {.position = ticks};
	int ret;

	ARG_UNUSED(user_data);

	sample.delta =
		position_delta(prev_position, ticks, counter_get_top_value(dev), &sample.wrapped);
	prev_position = ticks;

	(void)k_msgq_put(&fr_msgq, &sample, K_NO_WAIT);

	/* The driver disarms the alarm on every expiry while the unit timer
	 * keeps running, so re-arming here keeps the windows back to back.
	 */
	ret = counter_set_channel_alarm(dev, chan_id, &window_cfg);
	if (ret != 0) {
		printk("Failed to re-arm unit timer (%d)\n", ret);
	}
}

static int eqep_setup(void)
{
	const struct ti_eqep_dec_cfg dec_cfg = {
		.source = TI_EQEP_SRC_UP,
		.rising_edge_only = true,
	};
	const struct ti_eqep_qep_cfg qep_cfg = {
		.reset_mode = TI_EQEP_RESET_MODE_MAX,
		.capture_latch = TI_EQEP_CAPTURE_LATCH_TIMEOUT,
	};
	const struct counter_top_cfg top_cfg = {
		.ticks = POSITION_TOP,
	};
	int ret;

	ti_eqep_configure_decoder(eqep, &dec_cfg);
	ti_eqep_configure_qep(eqep, &qep_cfg);

	ret = counter_set_top_value(eqep, &top_cfg);
	if (ret != 0) {
		printk("Failed to set top value (%d)\n", ret);
		return ret;
	}

	ret = counter_start(eqep);
	if (ret != 0) {
		printk("Failed to start counter (%d)\n", ret);
		return ret;
	}

	/* One unit timer period of eQEP clocks is a 1 s window, so the
	 * position delta per window is the signal frequency in Hz.
	 */
	window_cfg.callback = window_expired;
	window_cfg.ticks = counter_get_frequency(eqep);

	ret = counter_set_channel_alarm(eqep, TI_EQEP_ALARM_CHAN_TIMEOUT, &window_cfg);
	if (ret != 0) {
		printk("Failed to arm unit timer (%d)\n", ret);
		return ret;
	}

	return 0;
}

int main(void)
{
	struct fr_sample sample;
	bool first = true;
	int ret;

	printk("TI EQEP frequency measurement sample\n");

	if (!device_is_ready(eqep)) {
		printk("EQEP device %s not ready\n", eqep->name);
		return 0;
	}

	if (!gpio_is_ready_dt(&signal)) {
		printk("Signal GPIO not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&signal, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		printk("Failed to configure signal GPIO (%d)\n", ret);
		return 0;
	}

	printk("Signal: %u Hz, 25%% duty; eQEP clock %u Hz; position top %u\n", SIGNAL_FREQ_HZ,
	       counter_get_frequency(eqep), POSITION_TOP);

	k_timer_start(&signal_timer, K_USEC(SIGNAL_STEP_US), K_USEC(SIGNAL_STEP_US));

	ret = eqep_setup();
	if (ret != 0) {
		return 0;
	}

	while (true) {
		k_msgq_get(&fr_msgq, &sample, K_FOREVER);

		/* The first window opens partway through a signal period. */
		if (first) {
			first = false;
			continue;
		}

		printk("position %4u, measured %u Hz%s%s\n", sample.position, sample.delta,
		       sample.wrapped ? " (counter wrapped)" : "",
		       (sample.delta + FREQ_TOLERANCE_HZ < SIGNAL_FREQ_HZ ||
			sample.delta > SIGNAL_FREQ_HZ + FREQ_TOLERANCE_HZ)
			       ? " [out of tolerance]"
			       : "");
	}

	return 0;
}
