#ifndef DRAIN_AM_H_
#define DRAIN_AM_H_

#include <stdint.h>

// Drain-modulation PWM plan shared by the firmware and the host tests.
//
// LEDC runs at 320 kHz with 7-bit resolution: 80 MHz / (128 * 500/256) uses
// the timer's fractional divider exactly, so every PWM period is 3.125 us and
// one 32 ksample/s period is exactly ten PWM periods (twenty at 16 ksample/s).
// The channel's overflow counter marks sample boundaries in hardware, and one
// linear hardware fade per sample interpolates the envelope across them.
static constexpr uint32_t DRAIN_PWM_HZ = 320000;
static constexpr uint32_t DRAIN_PWM_BITS = 7;
// LEDC duty registers carry four fractional bits. Ramps are planned in these
// 1/16-LSB units so slow envelope slopes are not lost to integer steps.
static constexpr uint32_t DRAIN_DUTY_FRAC_BITS = 4;
// Full scale stops one LSB short of 2^bits; 100% duty is not a valid LEDC
// compare value at every resolution.
static constexpr int32_t DRAIN_DUTY_MAX_Q4 =
    ((1 << DRAIN_PWM_BITS) - 1) << DRAIN_DUTY_FRAC_BITS;
static constexpr uint32_t DRAIN_FADE_SCALE_MAX = 1023;
// Whole-sample alignment history. Delays are limited so neither path needs
// more than seven samples of history.
static constexpr uint32_t DRAIN_HISTORY = 8;
static constexpr int32_t DRAIN_DELAY_MIN_STEPS = -60;
static constexpr int32_t DRAIN_DELAY_MAX_STEPS = 80;

struct DrainRamp {
  uint32_t increase;
  uint32_t scale;
  uint32_t steps;
};

// Plan one linear fade of `steps` PWM periods from *duty_q4 toward target_q4.
// An integer scale cannot always land exactly on the target, so *duty_q4 is
// advanced to the value the hardware will actually reach. The remainder is
// carried into the next ramp instead of being forced with a step.
static inline DrainRamp drain_plan_ramp(int32_t *duty_q4, int32_t target_q4,
                                        uint32_t steps) {
  if (target_q4 < 0) target_q4 = 0;
  if (target_q4 > DRAIN_DUTY_MAX_Q4) target_q4 = DRAIN_DUTY_MAX_Q4;
  int32_t delta = target_q4 - *duty_q4;
  uint32_t magnitude = static_cast<uint32_t>(delta < 0 ? -delta : delta);
  uint32_t scale = magnitude / steps;
  if (scale > DRAIN_FADE_SCALE_MAX) scale = DRAIN_FADE_SCALE_MAX;
  int32_t travel = static_cast<int32_t>(scale * steps);
  *duty_q4 += delta < 0 ? -travel : travel;
  return {delta < 0 ? 0U : 1U, scale, steps};
}

// Amplitude delay relative to the phase update, split into the parts the TX
// loop applies. A positive delay holds amplitude back; a negative one holds
// phase back. `boundary` (1..steps_per_sample) is the PWM boundary after the
// sample tick on which the amplitude fade latches.
struct DrainAlignment {
  uint32_t amplitude_samples;
  uint32_t phase_samples;
  uint32_t boundary;
};

static inline DrainAlignment drain_split_delay(int32_t delay_steps,
                                               uint32_t steps_per_sample) {
  int32_t period = static_cast<int32_t>(steps_per_sample);
  int32_t shifted = delay_steps - 1;
  int32_t samples = shifted >= 0 ? shifted / period
                                 : -((-shifted + period - 1) / period);
  int32_t boundary = delay_steps - samples * period;
  DrainAlignment alignment = {};
  alignment.amplitude_samples = samples > 0 ? static_cast<uint32_t>(samples) : 0;
  alignment.phase_samples = samples < 0 ? static_cast<uint32_t>(-samples) : 0;
  alignment.boundary = static_cast<uint32_t>(boundary);
  return alignment;
}

#endif
