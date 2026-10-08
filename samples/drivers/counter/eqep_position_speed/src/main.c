/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Texas Instruments
 * SPDX-License-Identifier: Apache-2.0
 *
 * Position + speed (FR and PR) using EQEP0 on lp_am13e230.
 *
 * Wire (SDK-compatible A/B jumpers):
 *   PB3  (signal A) -> PB11 (EQEP0A / INPUTXBAR17)
 *   PB1  (signal B) -> PB12 (EQEP0B / INPUTXBAR18)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/counter/ti_am3352_eqep.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

/* Electrical cycles/s of the fake quadrature; 4x decode → 4 counts/cycle. */
#define SIGNAL_CYCLES_HZ 100U
#define QUAD_STEPS       4U
#define SIGNAL_STEP_US   (USEC_PER_SEC / (SIGNAL_CYCLES_HZ * QUAD_STEPS))
#define COUNTS_PER_CYCLE 4U

/* Treat one "revolution" as this many counts so expected speed is neat. */
#define ENCODER_RESOLUTION (SIGNAL_CYCLES_HZ * COUNTS_PER_CYCLE)
#define EXPECTED_RPM       60U
#define RPM_TOLERANCE      5U

#define POSITION_TOP 9999U
BUILD_ASSERT(POSITION_TOP > 2U * ENCODER_RESOLUTION,
	     "position counter wraps within a window");

#define CAP_CLK_DIV  128U
#define UPEVNT_DIV   2U

struct speed_sample {
	uint32_t position;
	uint32_t delta;
	int32_t rpm_fr;
	int32_t rpm_pr;
	bool wrapped;
	bool counting_up;
	bool pr_valid;
};

static const struct device *const eqep = DEVICE_DT_GET(DT_ALIAS(eqep0));
static const struct gpio_dt_spec signal_a =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), signal_a_gpios);
static const struct gpio_dt_spec signal_b =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), signal_b_gpios);

K_MSGQ_DEFINE(speed_msgq, sizeof(struct speed_sample), 8, 4);

static struct counter_alarm_cfg window_cfg;
static uint32_t prev_position;

/*
 * Gray-code quadrature (forward). One full cycle of 4 steps advances the
 * position counter by 4 with 4x decoding.
 */
static void signal_step(struct k_timer *timer)
{
	static const uint8_t ab[QUAD_STEPS] = {0x0, 0x1, 0x3, 0x2};
	static uint32_t step;

	ARG_UNUSED(timer);

	gpio_pin_set_dt(&signal_a, ab[step] & 0x1);
	gpio_pin_set_dt(&signal_b, (ab[step] >> 1) & 0x1);
	step = (step + 1U) % QUAD_STEPS;
}

K_TIMER_DEFINE(signal_timer, signal_step, NULL);

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

static void window_expired(const struct device *dev, uint8_t chan_id, uint32_t ticks,
			   void *user_data)
{
	struct speed_sample sample = {
		.position = ticks,
		.counting_up = counter_is_counting_up(dev),
	};
	uint32_t cap_timer;
	uint32_t cap_period;
	uint32_t freq;
	int ret;

	ARG_UNUSED(user_data);

	sample.delta =
		position_delta(prev_position, ticks, counter_get_top_value(dev), &sample.wrapped);
	prev_position = ticks;

	/* FR: counts in the 1 s unit-timer window → RPM. */
	sample.rpm_fr = rpm_from_counts_per_sec(sample.delta, sample.counting_up);

	/*
	 * PR: unit-timeout also latches QCPRDLAT (QCLM=timeout). Period is
	 * between UPEVNT_DIV position clocks, scaled to eQEP clock ticks.
	 */
	ret = ti_eqep_get_latched_capture_values(dev, &cap_timer, &cap_period, true);
	freq = counter_get_frequency(dev);
	if (ret == 0 && cap_period != 0U && freq != 0U) {
		uint64_t counts_per_sec =
			((uint64_t)UPEVNT_DIV * (uint64_t)freq) / (uint64_t)cap_period;

		sample.rpm_pr = rpm_from_counts_per_sec((uint32_t)counts_per_sec, sample.counting_up);
		sample.pr_valid = true;
	}

	(void)k_msgq_put(&speed_msgq, &sample, K_NO_WAIT);

	ret = counter_set_channel_alarm(dev, chan_id, &window_cfg);
	if (ret != 0) {
		printk("Failed to re-arm unit timer (%d)\n", ret);
	}
}

static int eqep_setup(void)
{
	const struct ti_eqep_dec_cfg dec_cfg = {
		.source = TI_EQEP_SRC_QUADRATURE,
	};
	const struct ti_eqep_qep_cfg qep_cfg = {
		.reset_mode = TI_EQEP_RESET_MODE_MAX,
		.capture_latch = TI_EQEP_CAPTURE_LATCH_TIMEOUT,
	};
	const struct ti_eqep_cap_cfg cap_cfg = {
		.enable = true,
		.clock_prescaler = TI_EQEP_CAP_CLK_DIV_128,
		.unit_position_prescaler = TI_EQEP_UNIT_POS_DIV_2,
	};
	const struct counter_top_cfg top_cfg = {
		.ticks = POSITION_TOP,
	};
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

	window_cfg.callback = window_expired;
	window_cfg.ticks = counter_get_frequency(eqep);

	ret = counter_set_channel_alarm(eqep, TI_EQEP_ALARM_CHAN_TIMEOUT, &window_cfg);
	if (ret != 0) {
		printk("Failed to arm unit timer (%d)\n", ret);
		return ret;
	}

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

int main(void)
{
	struct speed_sample sample;
	bool first = true;
	int ret;

	printk("TI EQEP position/speed sample (FR + PR)\n");

	if (!device_is_ready(eqep)) {
		printk("EQEP device %s not ready\n", eqep->name);
		return 0;
	}

	if (!gpio_is_ready_dt(&signal_a) || !gpio_is_ready_dt(&signal_b)) {
		printk("Signal GPIOs not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&signal_a, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		printk("Failed to configure signal A (%d)\n", ret);
		return 0;
	}

	ret = gpio_pin_configure_dt(&signal_b, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		printk("Failed to configure signal B (%d)\n", ret);
		return 0;
	}

	printk("Quad: %u Hz electrical (%u counts/s); resolution %u; expect ~%u RPM\n",
	       SIGNAL_CYCLES_HZ, SIGNAL_CYCLES_HZ * COUNTS_PER_CYCLE, ENCODER_RESOLUTION,
	       EXPECTED_RPM);
	printk("Wire PB3->PB11 (A) and PB1->PB12 (B)\n");

	k_timer_start(&signal_timer, K_USEC(SIGNAL_STEP_US), K_USEC(SIGNAL_STEP_US));

	ret = eqep_setup();
	if (ret != 0) {
		return 0;
	}

	while (true) {
		k_msgq_get(&speed_msgq, &sample, K_FOREVER);

		if (first) {
			first = false;
			continue;
		}

		printk("pos %4u  FR %d RPM", sample.position, sample.rpm_fr);
		if (sample.pr_valid) {
			printk("  PR %d RPM", sample.rpm_pr);
		} else {
			printk("  PR n/a");
		}
		printk("%s%s%s\n", sample.counting_up ? "" : "  [CCW]",
		       sample.wrapped ? "  (wrap)" : "",
		       (rpm_out_of_tol(sample.rpm_fr) ||
			(sample.pr_valid && rpm_out_of_tol(sample.rpm_pr)))
			       ? "  [out of tolerance]"
			       : "");
	}

	return 0;
}
