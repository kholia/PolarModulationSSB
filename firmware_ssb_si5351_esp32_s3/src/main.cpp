// Polar SSB generator for ESP32-S3 using a low-level FIFO-updated Si5351.

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/ledc_ll.h"
#include "sdkconfig.h"

#include "audio_data.h"
#include "ddx_common.h"
#include "drain_am.h"
#include "phase_optimized_data.h"
#include "polar_mod.h"
#include "si5351.h"
#include "speech_dsp.h"
#include "tx_dsp.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "polar-ssb";

struct OperatingProfile {
  uint32_t i2c_hz;
  uint32_t sample_rate_hz;
  // esp_timer fallback schedule, used only if the LEDC tick fails its check.
  uint8_t period_short_us;
  uint8_t period_long_us;
  uint8_t period_mask;
  uint8_t wav_divisor;
  const char *name;
};

static constexpr OperatingProfile PROFILES[] = {
    {1000000, 16000, 62, 63, 1, 2, "1 MHz / 16 ksample/s"},
    {2000000, 16000, 62, 63, 1, 2, "2 MHz / 16 ksample/s"},
    {2000000, 32000, 31, 32, 3, 1, "2 MHz / 32 ksample/s (experimental)"},
    {3000000, 32000, 31, 32, 3, 1, "3 MHz / 32 ksample/s (experimental)"},
};

#ifdef CONFIG_POLAR_PROFILE_3MHZ_32K
static volatile uint32_t active_profile_index = 3;
#elif defined(CONFIG_POLAR_PROFILE_2MHZ_32K)
static volatile uint32_t active_profile_index = 2;
#elif defined(CONFIG_POLAR_PROFILE_2MHZ_16K)
static volatile uint32_t active_profile_index = 1;
#else
static volatile uint32_t active_profile_index = 0;
#endif

static volatile bool start_tx = true;
static volatile bool tx_running;
static volatile bool is_ssb = true;
static volatile bool use_two_tone_test;
static volatile bool use_melody_test;
static volatile bool use_wav_test = true;
static volatile bool use_phase_tone_test;
// The precomputed trajectory deliberately moves phase error into a wideband
// dump region. Keep it disabled at boot because measured RF spectrum quality,
// not reconstructed-audio quality alone, determines whether it is usable.
static volatile bool use_optimized_phase;
static volatile bool use_speech_dsp = true;
#ifdef CONFIG_POLAR_DRAIN_AM
static volatile bool use_drain_am = true;
#else
static volatile bool use_drain_am = false;
#endif
// Amplitude delay relative to the phase update in 3.125 us PWM periods.
// Positive values hold the envelope back. Calibrate per band and sideband.
static volatile int32_t amp_delay_steps = CONFIG_POLAR_DRAIN_AM_DELAY_STEPS;
static volatile uint32_t tx_deadline_misses;
static volatile uint32_t tx_amp_late;
static volatile bool tx_hw_ticks;
static volatile int32_t tx_worst_lateness_us;
static volatile uint64_t tx_phase_cycles_total;
static volatile uint64_t tx_work_cycles_total;
static volatile uint32_t tx_profiled_samples;
static volatile uint32_t tx_phase_cycles_max;
static volatile uint32_t tx_work_cycles_max;

static volatile float mod_depth = 0.75f;
// The hardware fade now interpolates the envelope linearly between samples,
// so the old one-pole smoother is off by default; it only adds envelope delay.
static volatile float ampl_smooth = 0.0f;
static volatile float gate_off_thresh = 0.015f;
static volatile float gate_on_thresh = 0.040f;
static volatile uint32_t gate_holdoff_ms = 50;
static constexpr bool ENABLE_RF_GATE = false;

uint64_t freq = 14200000ULL;
static uint16_t offset = 1200;

static constexpr ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
static constexpr ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;
static constexpr uint32_t PWM_MAX = (1U << DRAIN_PWM_BITS) - 1;
static constexpr uint32_t PWM_TICK_EVENT =
    LEDC_LL_EVENT_CHANNEL_OVF_CNT(PWM_MODE, PWM_CHANNEL);
static ledc_dev_t *const ledc_hw = LEDC_LL_GET_HW(0);

static void init_am_pwm() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = PWM_MODE;
  timer.duty_resolution = LEDC_TIMER_7_BIT;
  timer.timer_num = PWM_TIMER;
  timer.freq_hz = DRAIN_PWM_HZ;
  // APB is required: 80 MHz / (128 * 500/256) is exact, which locks ten PWM
  // periods to one 32 ksample/s period.
  timer.clk_cfg = LEDC_USE_APB_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));

  ledc_channel_config_t channel = {};
  channel.gpio_num = CONFIG_POLAR_AM_PWM_GPIO;
  channel.speed_mode = PWM_MODE;
  channel.channel = PWM_CHANNEL;
  channel.timer_sel = PWM_TIMER;
  channel.duty = 0;
  channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&channel));

  uint32_t divider = 0;
  ledc_ll_get_clock_divider(ledc_hw, PWM_MODE, PWM_TIMER, &divider);
  if (divider != 500)
    ESP_LOGE(TAG, "drain PWM divider is %lu/256, not 500/256; PWM is not "
                  "locked to the sample rate",
             static_cast<unsigned long>(divider));
}

// Load a start duty and a linear fade; both latch on the next PWM boundary.
// Only the TX task touches this channel and the fade service is not
// installed, so the registers are written directly to keep this short.
static inline void IRAM_ATTR program_am_pwm(int32_t start_q4,
                                            const DrainRamp &ramp) {
  ledc_hw->channel_group[PWM_MODE].channel[PWM_CHANNEL].duty.duty =
      static_cast<uint32_t>(start_q4);
  ledc_ll_set_fade_param(ledc_hw, PWM_MODE, PWM_CHANNEL, ramp.increase, 1,
                         ramp.scale, ramp.steps);
  ledc_ll_set_sig_out_en(ledc_hw, PWM_MODE, PWM_CHANNEL, true);
  ledc_ll_set_duty_start(ledc_hw, PWM_MODE, PWM_CHANNEL);
  ledc_ll_ls_channel_update(ledc_hw, PWM_MODE, PWM_CHANNEL);
}

static inline void write_am_pwm(uint32_t value) {
  if (value > PWM_MAX) value = PWM_MAX;
  program_am_pwm(static_cast<int32_t>(value << DRAIN_DUTY_FRAC_BITS),
                 {1, 0, 1});
}

// The PWM channel's overflow counter raises a raw event every
// `steps_per_sample` PWM periods. Polling that bit gives sample ticks that
// sit exactly on PWM boundaries, which is what makes the amplitude delay
// resolvable to one 3.125 us PWM period.
static void start_pwm_ticks(uint32_t steps_per_sample) {
  ledc_ll_channel_enable_timer_ovt_cnt(ledc_hw, PWM_MODE, PWM_CHANNEL, false);
  ledc_ll_channel_set_maximum_timer_ovf_cnt(ledc_hw, PWM_MODE, PWM_CHANNEL,
                                            steps_per_sample);
  ledc_ll_channel_reset_timer_ovf_cnt(ledc_hw, PWM_MODE, PWM_CHANNEL);
  ledc_ll_channel_enable_timer_ovt_cnt(ledc_hw, PWM_MODE, PWM_CHANNEL, true);
  ledc_ll_ls_channel_update(ledc_hw, PWM_MODE, PWM_CHANNEL);
  ledc_ll_clear_intr_status(ledc_hw, PWM_TICK_EVENT);
}

static void stop_pwm_ticks() {
  ledc_ll_channel_enable_timer_ovt_cnt(ledc_hw, PWM_MODE, PWM_CHANNEL, false);
  ledc_ll_ls_channel_update(ledc_hw, PWM_MODE, PWM_CHANNEL);
  ledc_ll_clear_intr_status(ledc_hw, PWM_TICK_EVENT);
}

static inline bool IRAM_ATTR pwm_tick_pending() {
  return (ledc_hw->int_raw.val & PWM_TICK_EVENT) != 0;
}

// The overflow-counter event is not used by ESP-IDF itself, so check it
// before trusting it: three consecutive events must be one sample apart.
static bool verify_pwm_ticks(uint32_t sample_rate_hz) {
  const uint32_t cycles_per_us = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
  const uint32_t expected =
      cycles_per_us * 1000000U / sample_rate_hz;  // 7500 or 15000 cycles
  uint32_t stamps[3];
  ledc_ll_clear_intr_status(ledc_hw, PWM_TICK_EVENT);
  for (uint32_t &stamp : stamps) {
    uint32_t start = esp_cpu_get_cycle_count();
    while (!pwm_tick_pending()) {
      if (esp_cpu_get_cycle_count() - start > 1000U * cycles_per_us)
        return false;
    }
    stamp = esp_cpu_get_cycle_count();
    ledc_ll_clear_intr_status(ledc_hw, PWM_TICK_EVENT);
  }
  for (int i = 1; i < 3; ++i) {
    uint32_t interval = stamps[i] - stamps[i - 1];
    uint32_t error = interval > expected ? interval - expected : expected - interval;
    if (error > 2U * cycles_per_us) return false;
  }
  return true;
}

static inline void set_fixed_tx_amplitude() {
  write_am_pwm(static_cast<uint16_t>(PWM_MAX * mod_depth));
}

static const OperatingProfile &active_profile() {
  return PROFILES[active_profile_index];
}

static bool select_profile(uint32_t index) {
  if (index >= sizeof(PROFILES) / sizeof(PROFILES[0])) return false;
  if (tx_running || start_tx) {
    printf("Profile change rejected: stop TX first with *t*\n");
    return false;
  }

  const OperatingProfile &profile = PROFILES[index];
  if (!si5351_set_i2c_speed(profile.i2c_hz)) {
    printf("Profile change failed: I2C rejected %lu Hz\n",
           static_cast<unsigned long>(profile.i2c_hz));
    return false;
  }
  active_profile_index = index;
  tx_dsp_set_sample_rate(profile.sample_rate_hz);
  printf("Profile: %s\n", profile.name);
  return true;
}

static int16_t get_two_tone_sample(uint32_t sample_index,
                                   uint32_t sample_rate_hz) {
  // 700 and 1900 Hz share a 100 Hz period. Wrapping the index keeps the float
  // time base exact; an unwrapped float t degrades the tones to about -42 dBc
  // after 10 s and -27 dBc after 60 s of transmission.
  float t = static_cast<float>(sample_index % (sample_rate_hz / 100)) /
            sample_rate_hz;
  float sample = 0.5f * sinf(2.0f * M_PI * 700.0f * t) +
                 0.5f * sinf(2.0f * M_PI * 1900.0f * t);
  return static_cast<int16_t>(sample * 16384.0f);
}

static int16_t get_melody_sample(uint32_t sample_index,
                                 uint32_t sample_rate_hz) {
  static constexpr float frequencies[] = {523.25f, 659.25f, 783.99f, 1046.50f};
  uint32_t note_index = (sample_index / (sample_rate_hz / 4)) % 4;
  float t = static_cast<float>(sample_index) / sample_rate_hz;
  return static_cast<int16_t>(0.8f * sinf(2.0f * M_PI * frequencies[note_index] * t) *
                              16384.0f);
}

static int16_t get_wav_sample(uint32_t sample_index, uint8_t divisor) {
  if (wav_data_len == 0) return 0;
  if (divisor == 2) {
    uint32_t index = (sample_index * 2) % wav_data_len;
    uint32_t next = (index + 1) % wav_data_len;
    return static_cast<int16_t>((static_cast<int32_t>(wav_data[index]) +
                                 static_cast<int32_t>(wav_data[next])) /
                                2);
  }
  return wav_data[sample_index % wav_data_len];
}

static void tx_ssb() {
  const OperatingProfile profile = active_profile();
  const bool optimized_phase_active =
      use_optimized_phase &&
      profile.sample_rate_hz == optimized_phase_sample_rate_hz;
  const bool drain_am = use_drain_am;
  const uint32_t steps_per_sample = DRAIN_PWM_HZ / profile.sample_rate_hz;
  const int32_t delay_steps = amp_delay_steps;
  const DrainAlignment alignment =
      drain_split_delay(delay_steps, steps_per_sample);
  const uint32_t cycles_per_us = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
  const uint32_t step_cycles = cycles_per_us * 1000000U / DRAIN_PWM_HZ;
  const uint32_t sample_cycles = steps_per_sample * step_cycles;
  // The fade must be written after PWM boundary (boundary - 1) and before
  // `boundary`, where it latches. Aim a quarter period into that window.
  const uint32_t amp_window_cycles =
      alignment.boundary == 1 ? 0
                              : (alignment.boundary - 1) * step_cycles +
                                    step_cycles / 4;
  const uint32_t amp_deadline_cycles =
      alignment.boundary * step_cycles - step_cycles / 8;
  // Run the DSP in the larger gap: after the amplitude write when it comes
  // early in the period, before it when it comes late.
  const bool dsp_before_amp = 2 * alignment.boundary > steps_per_sample;

  // Whole-sample alignment: both paths read their value from these rings,
  // indexed by DSP output count, so either path can be held back.
  int32_t phase_history[DRAIN_HISTORY] = {};
  int32_t amp_history[DRAIN_HISTORY] = {};
  uint32_t history_count = 0;
  int32_t am_duty_q4 = 0;

  int64_t last_sample_time;
  uint32_t test_sample_index = 0;
  uint32_t sample_counter = 0;
  float smooth_amplitude = 0.0f;
  double b_accumulator = 0.0;

  bool gate_enabled = true;
  bool gate_pending_disable = false;
  bool gate_pending_enable = false;
  uint32_t gate_holdoff_samples =
      (gate_holdoff_ms * profile.sample_rate_hz) / 1000;
  uint32_t holdoff_counter = 0;

  tx_dsp_reset();
  pm_phase_limiter = drain_am ? 0 : 1;
  gpio_set_level(static_cast<gpio_num_t>(PTT), 1);
  if (drain_am)
    write_am_pwm(0);
  else
    set_fixed_tx_amplitude();
  ESP_LOGI(TAG, "configuring CLK0 for %llu Hz SSB", freq);
  si5351_setup_ssb(freq, profile.sample_rate_hz);
  printf("[TX-SSB] Start %llu Hz (%s)\n", freq, profile.name);
  if (drain_am)
    printf("[DRAIN-AM] amplitude delay %ld steps (%.2f us): amplitude +%lu, "
           "phase +%lu samples, fade latches on PWM boundary %lu/%lu; "
           "phase limiter off\n",
           static_cast<long>(delay_steps),
           delay_steps * 1000000.0 / DRAIN_PWM_HZ,
           static_cast<unsigned long>(alignment.amplitude_samples),
           static_cast<unsigned long>(alignment.phase_samples),
           static_cast<unsigned long>(alignment.boundary),
           static_cast<unsigned long>(steps_per_sample));
  else
    printf("[DRAIN-AM] off: fixed drain, phase-only (enable with *a1*)\n");
  if (use_phase_tone_test)
    printf("[PHASE-TEST] Alternating 700/1900 Hz every second; use *v* to disable\n");
  else if (optimized_phase_active)
    printf("[PHASE-OPT] Experimental 10 s trajectory; wideband dump energy; "
           "dummy-load testing only\n");
  else if (use_optimized_phase)
    printf("[PHASE-OPT] Requires the 16 ksample/s profile; using live DSP path\n");

  start_pwm_ticks(steps_per_sample);
  const bool hw_ticks = verify_pwm_ticks(profile.sample_rate_hz);
  tx_hw_ticks = hw_ticks;
  if (!hw_ticks) {
    stop_pwm_ticks();
    ESP_LOGE(TAG, "LEDC sample tick failed its self-check; falling back to "
                  "the esp_timer schedule (amplitude alignment is coarse)");
  }

  // Start the sample clock only after the blocking PLL configuration and UART
  // messages. Starting it at function entry made every TX begin about 2 ms
  // behind and created an unnecessary burst of catch-up phase writes.
  tx_deadline_misses = 0;
  tx_amp_late = 0;
  tx_worst_lateness_us = 0;
  tx_phase_cycles_total = 0;
  tx_work_cycles_total = 0;
  tx_profiled_samples = 0;
  tx_phase_cycles_max = 0;
  tx_work_cycles_max = 0;
  last_sample_time = esp_timer_get_time();
  uint32_t previous_tick_cycles = esp_cpu_get_cycle_count();
  if (hw_ticks) ledc_ll_clear_intr_status(ledc_hw, PWM_TICK_EVENT);

  while (start_tx) {
    uint32_t tick_cycles;
    if (hw_ticks) {
      if (pwm_tick_pending()) {
        // The previous sample overran: this boundary has already passed.
        tx_deadline_misses = tx_deadline_misses + 1;
      } else {
        while (!pwm_tick_pending()) {
          if (!start_tx) break;
        }
        if (!start_tx) break;
      }
      ledc_ll_clear_intr_status(ledc_hw, PWM_TICK_EVENT);
      tick_cycles = esp_cpu_get_cycle_count();
      int32_t lateness = static_cast<int32_t>(tick_cycles - previous_tick_cycles -
                                              sample_cycles) /
                         static_cast<int32_t>(cycles_per_us);
      if (sample_counter > 0 && lateness > tx_worst_lateness_us)
        tx_worst_lateness_us = lateness;
      previous_tick_cycles = tick_cycles;
    } else {
      int64_t now = esp_timer_get_time();
      int64_t target_time =
          last_sample_time +
          ((sample_counter & profile.period_mask) == profile.period_mask
               ? profile.period_long_us
               : profile.period_short_us);
      if (now < target_time) continue;

      int32_t lateness = static_cast<int32_t>(now - target_time);
      if (lateness >= profile.period_short_us) {
        tx_deadline_misses = tx_deadline_misses + 1;
        // A late sample cannot be recovered in the past. Rebase instead of
        // issuing several phase updates back-to-back, which turns scheduler
        // jitter into a concentrated RF/audio phase burst.
        last_sample_time = now;
      } else {
        last_sample_time = target_time;
      }
      if (lateness > tx_worst_lateness_us) tx_worst_lateness_us = lateness;
      tick_cycles = esp_cpu_get_cycle_count();
    }
    sample_counter++;

    auto history_at = [&](const int32_t *ring, uint32_t delay) -> int32_t {
      if (history_count <= delay) return 0;
      return ring[(history_count - 1 - delay) % DRAIN_HISTORY];
    };
    // Snapshot both values at the tick: the DSP below appends to the history,
    // and a fade written after it must still use this tick's sample.
    const int32_t amp_target = history_at(amp_history, alignment.amplitude_samples);
    const int32_t phase_value = history_at(phase_history, alignment.phase_samples);
    bool amp_pending = drain_am;
    auto update_amplitude = [&]() {
      uint32_t elapsed = esp_cpu_get_cycle_count() - tick_cycles;
      if (elapsed > amp_deadline_cycles) {
        // Too late for the intended boundary; the fade latches one or more
        // PWM periods later for this sample only.
        tx_amp_late = tx_amp_late + 1;
      } else {
        while (esp_cpu_get_cycle_count() - tick_cycles < amp_window_cycles) {
        }
      }
      int32_t start_q4 = am_duty_q4;
      DrainRamp ramp = drain_plan_ramp(&am_duty_q4, amp_target, steps_per_sample);
      program_am_pwm(start_q4, ramp);
      amp_pending = false;
    };

    // The first boundary's window opens at the tick itself, so the fade goes
    // out before the comparatively slow FIFO refill.
    if (amp_pending && alignment.boundary == 1) update_amplitude();

    // This starts a low-level FIFO phase update. The wire transfer runs in
    // parallel with the DSP below and is checked before the next submission.
    uint32_t phase_cycle_start = esp_cpu_get_cycle_count();
    si5351_write_phase_fast(phase_value);
    uint32_t phase_cycles = esp_cpu_get_cycle_count() - phase_cycle_start;
    tx_phase_cycles_total += phase_cycles;
    if (phase_cycles > tx_phase_cycles_max) tx_phase_cycles_max = phase_cycles;

    if (amp_pending && !dsp_before_amp) update_amplitude();

    uint32_t work_cycle_start = esp_cpu_get_cycle_count();
    float amplitude;
    float phase_difference;
    if (use_phase_tone_test) {
      // Deterministic phase-only transport test. A constant phase increment
      // shifts CLK0 by exactly the requested audio tone frequency.
      uint32_t tone_hz = ((test_sample_index / profile.sample_rate_hz) & 1U)
                             ? 1900U
                             : 700U;
      test_sample_index++;
      amplitude = 1.0f;
      phase_difference =
          2.0f * M_PI * static_cast<float>(tone_hz) / profile.sample_rate_hz;
      if (modulation_mode == MOD_LSB) phase_difference = -phase_difference;
    } else if (optimized_phase_active) {
      // The offline optimizer already performed speech conditioning, Hilbert
      // analysis and constant-envelope synthesis. Its error-feedback int8
      // quantizer keeps cumulative phase error below half an LSB.
      int8_t phase_code =
          optimized_phase_steps[test_sample_index % optimized_phase_steps_len];
      test_sample_index++;
      amplitude = 1.0f;
      phase_difference = phase_code * optimized_phase_step_scale;
      if (modulation_mode == MOD_LSB) phase_difference = -phase_difference;
    } else {
      int16_t sample = 0;
      if (use_wav_test) {
        if (use_speech_dsp) {
          uint32_t loop_length = wav_data_len / profile.wav_divisor;
          if (loop_length > 0 && test_sample_index > 0 &&
              test_sample_index % loop_length == 0)
            speech_dsp_retrigger_fadein();
        }
        sample = get_wav_sample(test_sample_index++, profile.wav_divisor);
      } else if (use_melody_test) {
        sample = get_melody_sample(test_sample_index++, profile.sample_rate_hz);
      } else if (use_two_tone_test) {
        sample = get_two_tone_sample(test_sample_index++, profile.sample_rate_hz);
      }

      float input = static_cast<float>(sample) / 32768.0f;
      tx_dsp_process(input, use_speech_dsp, &amplitude, &phase_difference);
    }

    b_accumulator += static_cast<double>(phase_difference * ssb_phase_to_b_factor);
    int32_t b_diff = static_cast<int32_t>(b_accumulator);
    b_accumulator -= b_diff;

    float target_amplitude = amplitude * mod_depth;
    if (target_amplitude > 1.0f) target_amplitude = 1.0f;
    if (target_amplitude < 0.0f) target_amplitude = 0.0f;
    smooth_amplitude = smooth_amplitude * ampl_smooth +
                       target_amplitude * (1.0f - ampl_smooth);

    phase_history[history_count % DRAIN_HISTORY] = b_diff;
    amp_history[history_count % DRAIN_HISTORY] =
        static_cast<int32_t>(smooth_amplitude * DRAIN_DUTY_MAX_Q4 + 0.5f);
    history_count++;
    uint32_t work_cycles = esp_cpu_get_cycle_count() - work_cycle_start;
    tx_work_cycles_total += work_cycles;
    if (work_cycles > tx_work_cycles_max) tx_work_cycles_max = work_cycles;
    tx_profiled_samples = tx_profiled_samples + 1;

    if (amp_pending) update_amplitude();

    if (ENABLE_RF_GATE) {
      if (gate_enabled) {
        if (amplitude < gate_off_thresh) {
          if (++holdoff_counter >= gate_holdoff_samples) {
            gate_pending_disable = true;
            gate_enabled = false;
            holdoff_counter = 0;
          }
        } else {
          holdoff_counter = 0;
        }
      } else if (amplitude >= gate_on_thresh) {
        gate_pending_enable = true;
        gate_enabled = true;
        holdoff_counter = 0;
      }

      if (gate_pending_disable) {
        si5351_dma_wait();
        si5351_output_enable(SI5351_CLK0, 0);
        gate_pending_disable = false;
      } else if (gate_pending_enable) {
        si5351_dma_wait();
        si5351_output_enable(SI5351_CLK0, 1);
        gate_pending_enable = false;
      }
    }
  }

  if (hw_ticks) stop_pwm_ticks();
  si5351_dma_wait();
  si5351_output_enable(SI5351_CLK0, 0);
  write_am_pwm(0);
  gpio_set_level(static_cast<gpio_num_t>(PTT), 0);
  printf("[TX-SSB] End; deadline misses=%lu, worst lateness=%ld us, "
         "late amplitude updates=%lu, %s ticks\n",
         static_cast<unsigned long>(tx_deadline_misses),
         static_cast<long>(tx_worst_lateness_us),
         static_cast<unsigned long>(tx_amp_late),
         hw_ticks ? "LEDC" : "esp_timer");
}

static void tx_ft8() {
  gpio_set_level(static_cast<gpio_num_t>(PTT), 1);
  set_fixed_tx_amplitude();
  printf("[TX-FT8] Start %llu\n", freq);
  si5351_set_freq((freq + offset) * 100ULL, SI5351_CLK0);
  si5351_output_enable(SI5351_CLK0, 1);
  vTaskDelay(pdMS_TO_TICKS(1000));
  si5351_output_enable(SI5351_CLK0, 0);
  write_am_pwm(0);
  gpio_set_level(static_cast<gpio_num_t>(PTT), 0);
  printf("[TX-FT8] End\n");
}

static void tx_task(void *) {
  while (true) {
    if (start_tx) {
      tx_running = true;
      if (is_ssb)
        tx_ssb();
      else
        tx_ft8();
      start_tx = false;
      tx_running = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static char cat_buffer[1024];
static size_t cat_index;

static void process_cat_command(char *command) {
  if (command[0] != '*' || command[2] == '\0') return;
  switch (command[1]) {
    case 'u':
      is_ssb = true;
      modulation_mode = MOD_USB;
      printf("USB\n");
      break;
    case 'l':
      is_ssb = true;
      modulation_mode = MOD_LSB;
      printf("LSB\n");
      break;
    case 'k':
      use_two_tone_test = !use_two_tone_test;
      printf("TwoTone: %d\n", use_two_tone_test);
      break;
    case 'm':
      use_melody_test = !use_melody_test;
      printf("Melody: %d\n", use_melody_test);
      break;
    case 'w':
      use_wav_test = !use_wav_test;
      printf("WAV: %d\n", use_wav_test);
      break;
    case 'v':
      use_phase_tone_test = !use_phase_tone_test;
      printf("Phase tone test: %d\n", use_phase_tone_test);
      break;
    case 'o':
      if (command[2] == '1')
        use_optimized_phase = true;
      else if (command[2] == '0')
        use_optimized_phase = false;
      else
        use_optimized_phase = !use_optimized_phase;
      printf("Optimized phase trajectory: %s (takes effect on next TX)\n",
             use_optimized_phase ? "ON" : "OFF");
      break;
    case 't':
      if (command[2] == '1')
        start_tx = true;
      else if (command[2] == '0')
        start_tx = false;
      else
        start_tx = !start_tx;
      printf("TX: %d\n", start_tx);
      break;
    case 'f':
      freq = strtoull(command + 2, nullptr, 10);
      printf("Freq: %llu\n", freq);
      break;
    case 'a':
      if (command[2] == '1')
        use_drain_am = true;
      else if (command[2] == '0')
        use_drain_am = false;
      else
        use_drain_am = !use_drain_am;
      printf("Drain AM: %s (takes effect on next TX)\n",
             use_drain_am ? "ON" : "OFF");
      break;
    case 'd': {
      // Amplitude delay relative to phase, in 3.125 us PWM periods.
      long value = strtol(command + 2, nullptr, 10);
      if (command[2] != '*' && value >= DRAIN_DELAY_MIN_STEPS &&
          value <= DRAIN_DELAY_MAX_STEPS)
        amp_delay_steps = static_cast<int32_t>(value);
      printf("Amplitude delay: %ld steps (%.2f us; range %ld..%ld; takes "
             "effect on next TX)\n",
             static_cast<long>(amp_delay_steps),
             amp_delay_steps * 1000000.0 / DRAIN_PWM_HZ,
             static_cast<long>(DRAIN_DELAY_MIN_STEPS),
             static_cast<long>(DRAIN_DELAY_MAX_STEPS));
      break;
    }
    case 'i': {
      uint32_t speed_khz = strtoul(command + 2, nullptr, 10);
      if (speed_khz == 1000)
        select_profile(0);
      else if (speed_khz == 2000)
        select_profile(1);
      else if (speed_khz == 2032)
        select_profile(2);
      else if (speed_khz == 3032)
        select_profile(3);
      else
        printf("Profiles: *i1000* (1M/16k), *i2000* (2M/16k), "
               "*i2032* (2M/32k), or *i3032* (3M/32k)\n");
      break;
    }
    case 'c':
      use_speech_dsp = !use_speech_dsp;
      printf("Speech DSP: %s\n", use_speech_dsp ? "ON" : "OFF");
      break;
    case 'n': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 1 && value <= 100) mod_depth = value / 100.0f;
      printf("Mod depth: %.2f\n", mod_depth);
      break;
    }
    case 's': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value <= 999) ampl_smooth = value / 1000.0f;
      printf("Smoothing: %.3f\n", ampl_smooth);
      break;
    }
    case 'g': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 1 && value <= 500) gate_off_thresh = value / 1000.0f;
      printf("Gate OFF: %.3f\n", gate_off_thresh);
      break;
    }
    case 'h': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 1 && value <= 500) gate_on_thresh = value / 1000.0f;
      printf("Gate ON: %.3f\n", gate_on_thresh);
      break;
    }
    case 'p': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 1 && value <= 1000) gate_holdoff_ms = value;
      printf("Gate holdoff: %lu ms\n", static_cast<unsigned long>(gate_holdoff_ms));
      break;
    }
    case 'A': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 10 && value <= 900) sdsp_agc_target = value / 1000.0f;
      printf("AGC target: %.3f\n", sdsp_agc_target);
      break;
    }
    case 'G': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 5 && value <= 200) sdsp_agc_max_gain = value / 10.0f;
      printf("AGC max gain: %.1f\n", sdsp_agc_max_gain);
      break;
    }
    case 'C': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 50 && value <= 900) sdsp_clip_level = value / 1000.0f;
      printf("Clip level: %.3f\n", sdsp_clip_level);
      break;
    }
    case 'E': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value <= 200) sdsp_preemph = value / 100.0f;
      printf("Pre-emphasis: %.2f\n", sdsp_preemph);
      break;
    }
    case 'B': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (tx_running || start_tx) {
        printf("Bandwidth change rejected: stop TX first with *t*\n");
      } else if (!speech_dsp_set_bandwidth(value)) {
        printf("Bandwidth must be 2000, 2200, 2400, 2700, or 3000 Hz\n");
      }
      printf("Speech bandwidth: %lu Hz\n",
             static_cast<unsigned long>(sdsp_bandwidth_hz));
      break;
    }
    case 'L': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 50 && value <= 1500) pm_env_threshold = value / 1000.0f;
      printf("Envelope threshold: %.3f\n", pm_env_threshold);
      break;
    }
    case 'R': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 10 && value <= 200) pm_env_ratio = value / 10.0f;
      printf("Envelope ratio: %.1f:1\n", pm_env_ratio);
      break;
    }
    case 'e': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 1 && value <= 100) pm_max_phase_step = value / 10.0f;
      printf("Phase limit: %.1f\n", pm_max_phase_step);
      break;
    }
    case 'j': {
      uint32_t value = strtoul(command + 2, nullptr, 10);
      if (value >= 1 && value <= 1000) pm_soft_knee = value / 10000.0f;
      printf("Soft knee: %.4f\n", pm_soft_knee);
      break;
    }
    default:
      printf("Unknown CAT command\n");
      break;
  }
}

static void console_task(void *) {
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  if (flags >= 0) (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  constexpr int64_t phase_report_interval_us = 5000000;
  int64_t previous_phase_report_us = esp_timer_get_time();
  int64_t next_phase_report_us =
      previous_phase_report_us + phase_report_interval_us;
  uint32_t previous_phase_writes = 0;

  while (true) {
    char character;
    while (read(STDIN_FILENO, &character, 1) == 1) {
      if (cat_index + 1 >= sizeof(cat_buffer)) cat_index = 0;
      cat_buffer[cat_index++] = character;
      cat_buffer[cat_index] = '\0';
      if (character == '*' && cat_index > 1) {
        char *start = strchr(cat_buffer, '*');
        if (start && start != &cat_buffer[cat_index - 1]) {
          process_cat_command(start);
          cat_index = 0;
          cat_buffer[0] = '\0';
        }
      }
    }
    if (errno != EAGAIN && errno != 0) errno = 0;
    int64_t now = esp_timer_get_time();
    if (tx_running && now >= next_phase_report_us) {
      struct Si5351PhaseStats stats;
      si5351_get_phase_stats(&stats);
      const OperatingProfile &profile = active_profile();
      const char *tx_mode =
          use_phase_tone_test
              ? "PHASE-TEST"
              : ((use_optimized_phase &&
                  profile.sample_rate_hz == optimized_phase_sample_rate_hz)
                     ? "PHASE-OPT"
                     : "LIVE");
      float measured_rate = 0.0f;
      if (previous_phase_report_us != 0 && stats.writes >= previous_phase_writes) {
        measured_rate =
            static_cast<float>(stats.writes - previous_phase_writes) * 1000000.0f /
            static_cast<float>(now - previous_phase_report_us);
      }
      ESP_LOGI(TAG,
               "[%s] phase writes=%lu (%.0f/s), b_diff=[%ld,%ld] last=%ld, P1=%lu P2=%lu, "
               "i2c=%.2f B/write, submit_err=%lu wait_err=%lu upper_carry=%lu, "
               "deadline_miss=%lu worst_late=%ld us amp_late=%lu %s, "
               "phase=%.1f/%0.1f us work=%.1f/%0.1f us avg/max",
               tx_mode,
               static_cast<unsigned long>(stats.writes),
               measured_rate,
               static_cast<long>(stats.min_b_diff),
               static_cast<long>(stats.max_b_diff),
               static_cast<long>(stats.last_b_diff),
               static_cast<unsigned long>(stats.last_p1),
               static_cast<unsigned long>(stats.last_p2),
               stats.writes == 0
                   ? 0.0
                   : static_cast<double>(stats.command_bytes) / stats.writes,
               static_cast<unsigned long>(stats.submit_errors),
               static_cast<unsigned long>(stats.wait_errors),
               static_cast<unsigned long>(stats.p1_upper_changes),
               static_cast<unsigned long>(tx_deadline_misses),
               static_cast<long>(tx_worst_lateness_us),
               static_cast<unsigned long>(tx_amp_late),
               tx_hw_ticks ? "LEDC" : "esp_timer",
               tx_profiled_samples == 0
                   ? 0.0
                   : static_cast<double>(tx_phase_cycles_total) /
                         tx_profiled_samples / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
               static_cast<double>(tx_phase_cycles_max) /
                   CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
               tx_profiled_samples == 0
                   ? 0.0
                   : static_cast<double>(tx_work_cycles_total) /
                         tx_profiled_samples / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
               static_cast<double>(tx_work_cycles_max) /
                   CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
      previous_phase_writes = stats.writes;
      previous_phase_report_us = now;
      next_phase_report_us = now + phase_report_interval_us;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

extern "C" void app_main(void) {
  gpio_config_t ptt = {};
  ptt.pin_bit_mask = 1ULL << PTT;
  ptt.mode = GPIO_MODE_OUTPUT;
  ptt.pull_up_en = GPIO_PULLUP_DISABLE;
  ptt.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ptt.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&ptt));
  gpio_set_level(static_cast<gpio_num_t>(PTT), 0);
  init_am_pwm();

  const OperatingProfile &profile = active_profile();
  tx_dsp_set_sample_rate(profile.sample_rate_hz);

  // Allow the external TCXO and Si5351 oscillator path to settle after power-up.
  vTaskDelay(pdMS_TO_TICKS(3000));
  // The 25 MHz TCXO directly drives XA, so disable the internal crystal load
  // capacitors rather than loading the driven reference as a passive crystal.
  while (!si5351_init(0x60, SI5351_CRYSTAL_LOAD_0PF,
                      CONFIG_POLAR_SI5351_XTAL_HZ, 0)) {
    ESP_LOGW(TAG, "Si5351 not ready; retrying");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  if (!si5351_set_i2c_speed(profile.i2c_hz)) {
    ESP_LOGE(TAG, "Unable to select the %lu Hz modulation bus profile; TX disabled",
             static_cast<unsigned long>(profile.i2c_hz));
    return;
  }
  si5351_dma_init();
  ESP_LOGI(TAG,
           "Si5351 ready; XA reference=%d Hz, load=0 pF, SDA=GPIO%d, "
           "SCL=GPIO%d; default profile: %s",
           CONFIG_POLAR_SI5351_XTAL_HZ, CONFIG_POLAR_I2C_SDA_GPIO,
           CONFIG_POLAR_I2C_SCL_GPIO, profile.name);
  ESP_LOGI(TAG,
           "voice DSP: AGC=%.3f/max %.1f, clip=%.3f, preemphasis=%.2f, "
           "phase limit=%.1f, knee=%.4f",
           sdsp_agc_target, sdsp_agc_max_gain, sdsp_clip_level, sdsp_preemph,
           pm_max_phase_step, pm_soft_knee);
  ESP_LOGI(TAG, "boot mode: live phase-only SSB (optimized trajectory OFF)");

  xTaskCreatePinnedToCore(tx_task, "polar-tx", 8192, nullptr,
                          configMAX_PRIORITIES - 1, nullptr, 1);
  xTaskCreatePinnedToCore(console_task, "cat-console", 4096, nullptr, 5, nullptr,
                          0);
}
