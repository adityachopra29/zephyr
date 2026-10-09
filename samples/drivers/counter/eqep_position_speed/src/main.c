/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Texas Instruments
 * SPDX-License-Identifier: Apache-2.0
 *
 * Position + speed (FR and PR) using EQEP0 on lp_am13e230.
 *
 * Expects an external MCPWM quadrature source (e.g. SDK
 * mcpwm_quadrature_gen), ~800 us electrical period (~1250 Hz):
 *   generator PB3 (A) -> PB11 (EQEP0A / INPUTXBAR17)
 *   generator PB1 (B) -> PB12 (EQEP0B / INPUTXBAR18)
 *   GND               -> GND
 *
 * FR uses a 1 s software window on QPOSCNT (same math as the unit-timer
 * FR path). Capture latch is CPU-read so PR works without UTOI.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/counter/ti_am3352_eqep.h>
#include <zephyr/sys/printk.h>

/* MCPWM up-down period 19999 @ 50 MHz TBCLK → ~800 us / ~1250 Hz. */
#define SIGNAL_CYCLES_HZ   1250U
#define COUNTS_PER_CYCLE   4U /* 4x quadrature decode */
#define EXPECTED_COUNTS_HZ (SIGNAL_CYCLES_HZ * COUNTS_PER_CYCLE)

/* Same counts/rev as the AM13 SDK eqep_position_speed example. */
#define ENCODER_RESOLUTION 6000U
#define EXPECTED_RPM       50U
#define RPM_TOLERANCE      5U
#define COUNT_TOLERANCE_HZ 50U

/* Must exceed counts seen in one 1 s FR window so delta math stays valid. */
#define POSITION_TOP 19999U
BUILD_ASSERT(POSITION_TOP > 2U * EXPECTED_COUNTS_HZ,
	     "position counter wraps within a window");

#define UPEVNT_DIV 2U
#define WINDOW_MS  1000U

struct speed_sample {
	uint32_t position;
	uint32_t delta;
	int32_t rpm_fr;
	bool wrapped;
	bool counting_up;
};

static const struct device *const eqep = DEVICE_DT_GET(DT_ALIAS(eqep0));

K_MSGQ_DEFINE(speed_msgq, sizeof(struct speed_sample), 8, 4);

static uint32_t prev_position;
static bool have_prev;

static uint32_t position_delta(uint32_t prev, uint32_t cur, uint32_t top, bool *wrapped)
{
	*wrapped = cur < prev;

	if (!*wrapped) {
		return cur - prev;
	}

	return (top - prev) + cur + 1U;
}

static int32_t rpm_from_counts_per_sec(uint32_t counts_per_sec, bool counting_up)
{
	int32_t rpm = (int32_t)((counts_per_sec * 60U) / ENCODER_RESOLUTION);

	return counting_up ? rpm : -rpm;
}

static void window_timer_handler(struct k_timer *timer)
{
	struct speed_sample sample = {
		.counting_up = counter_is_counting_up(eqep),
	};
	uint32_t pos = 0;

	ARG_UNUSED(timer);

	/* QCLM=CPU: reading QPOSCNT also latches capture for PR. */
	if (counter_get_value(eqep, &pos) != 0) {
		return;
	}

	sample.position = pos;

	if (!have_prev) {
		prev_position = pos;
		have_prev = true;
		return;
	}

	sample.delta =
		position_delta(prev_position, pos, counter_get_top_value(eqep), &sample.wrapped);
	prev_position = pos;
	sample.rpm_fr = rpm_from_counts_per_sec(sample.delta, sample.counting_up);

	(void)k_msgq_put(&speed_msgq, &sample, K_NO_WAIT);
}

K_TIMER_DEFINE(window_timer, window_timer_handler, NULL);

static bool rpm_pr_from_capture(int32_t *rpm_pr)
{
	uint32_t cap_timer;
	uint32_t cap_period;
	uint32_t freq;
	int ret;

	ret = ti_eqep_get_latched_capture_values(eqep, &cap_timer, &cap_period, true);
	freq = counter_get_frequency(eqep);
	if (ret != 0 || cap_period == 0U || freq == 0U) {
		return false;
	}

	uint64_t counts_per_sec =
		((uint64_t)UPEVNT_DIV * (uint64_t)freq) / (uint64_t)cap_period;

	*rpm_pr = rpm_from_counts_per_sec((uint32_t)counts_per_sec, counter_is_counting_up(eqep));
	return true;
}

static int eqep_setup(void)
{
	const struct ti_eqep_dec_cfg dec_cfg = {
		.source = TI_EQEP_SRC_QUADRATURE,
	};
	const struct ti_eqep_qep_cfg qep_cfg = {
		.reset_mode = TI_EQEP_RESET_MODE_MAX,
		.capture_latch = TI_EQEP_CAPTURE_LATCH_CPU,
	};
	const struct ti_eqep_cap_cfg cap_cfg = {
		.enable = true,
		.clock_prescaler = TI_EQEP_CAP_CLK_DIV_128,
		.unit_position_prescaler = TI_EQEP_UNIT_POS_DIV_2,
	};
	const struct counter_top_cfg top_cfg = {
		.ticks = POSITION_TOP,
	};
	uint32_t freq;
	int ret;

	ti_eqep_configure_decoder(eqep, &dec_cfg);
	ti_eqep_configure_qep(eqep, &qep_cfg);
	ti_eqep_configure_capture(eqep, &cap_cfg);

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

	freq = counter_get_frequency(eqep);
	printk("eQEP clock %u Hz; 1 s software FR windows (CPU capture latch)\n", freq);

	have_prev = false;
	k_timer_start(&window_timer, K_MSEC(WINDOW_MS), K_MSEC(WINDOW_MS));
	return 0;
}

static bool rpm_out_of_tol(int32_t rpm)
{
	int32_t diff = rpm - (int32_t)EXPECTED_RPM;

	if (diff < 0) {
		diff = -diff;
	}

	return diff > (int32_t)RPM_TOLERANCE;
}

static bool counts_out_of_tol(uint32_t delta)
{
	uint32_t lo = EXPECTED_COUNTS_HZ - COUNT_TOLERANCE_HZ;
	uint32_t hi = EXPECTED_COUNTS_HZ + COUNT_TOLERANCE_HZ;

	return (delta < lo) || (delta > hi);
}

int main(void)
{
	struct speed_sample sample;
	int ret;

	printk("TI EQEP position/speed sample (FR + PR, MCPWM stimulus)\n");

	if (!device_is_ready(eqep)) {
		printk("EQEP device %s not ready\n", eqep->name);
		return 0;
	}

	printk("Expect ~%u Hz electrical (800 us), %u counts/s, resolution %u, ~%u RPM\n",
	       SIGNAL_CYCLES_HZ, EXPECTED_COUNTS_HZ, ENCODER_RESOLUTION, EXPECTED_RPM);
	printk("Wire MCPWM gen PB3->PB11 (A), PB1->PB12 (B), share GND\n");

	ret = eqep_setup();
	if (ret != 0) {
		return 0;
	}

	while (true) {
		ret = k_msgq_get(&speed_msgq, &sample, K_SECONDS(2));
		if (ret != 0) {
			uint32_t pos = 0;
			uint32_t pending = counter_get_pending_int(eqep);

			(void)counter_get_value(eqep, &pos);
			printk("waiting for window... live pos=%u pending=0x%x\n", pos, pending);
			continue;
		}

		int32_t rpm_pr = 0;
		bool pr_valid = rpm_pr_from_capture(&rpm_pr);
		bool bad = counts_out_of_tol(sample.delta) || rpm_out_of_tol(sample.rpm_fr) ||
			   (pr_valid && rpm_out_of_tol(rpm_pr));

		printk("pos %5u  delta %u/s  FR %d RPM", sample.position, sample.delta,
		       sample.rpm_fr);
		if (pr_valid) {
			printk("  PR %d RPM", rpm_pr);
		} else {
			printk("  PR n/a");
		}
		printk("%s%s%s\n", sample.counting_up ? "" : "  [CCW]",
		       sample.wrapped ? "  (wrap)" : "", bad ? "  [out of tolerance]" : "");
	}

	return 0;
}
