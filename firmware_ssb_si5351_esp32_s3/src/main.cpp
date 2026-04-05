// Polar SSB generator for ESP32-S3 using an asynchronously updated Si5351.

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
#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "audio_data.h"
#include "ddx_common.h"
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
  uint8_t period_short_us;
  uint8_t period_long_us;
  uint8_t period_mask;
  uint8_t interpolation_interval_us;
  uint8_t wav_divisor;
  const char *name;
};

static constexpr OperatingProfile PROFILES[] = {
    {1000000, 16000, 62, 63, 1, 5, 2, "1 MHz / 16 ksample/s"},
    {2000000, 16000, 62, 63, 1, 5, 2, "2 MHz / 16 ksample/s"},
    {2000000, 32000, 31, 32, 3, 3, 1, "2 MHz / 32 ksample/s (experimental)"},
    {3000000, 32000, 31, 32, 3, 3, 1, "3 MHz / 32 ksample/s (experimental)"},
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
static volatile int32_t phase_delay_us;
static volatile bool use_speech_dsp = true;
static volatile uint32_t tx_deadline_misses;
static volatile int32_t tx_worst_lateness_us;
static volatile uint64_t tx_phase_cycles_total;
static volatile uint64_t tx_work_cycles_total;
static volatile uint32_t tx_profiled_samples;
static volatile uint32_t tx_phase_cycles_max;
static volatile uint32_t tx_work_cycles_max;

static volatile float mod_depth = 0.75f;
static volatile float ampl_smooth = 0.15f;
static volatile float gate_off_thresh = 0.015f;
static volatile float gate_on_thresh = 0.040f;
static volatile uint32_t gate_holdoff_ms = 50;
static constexpr bool ENABLE_AM_MODULATION = false;
static constexpr bool ENABLE_RF_GATE = false;

uint64_t freq = 14200000ULL;
static uint16_t offset = 1200;

static constexpr ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
static constexpr ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;
static constexpr uint32_t PWM_BITS = 8;
static constexpr uint32_t PWM_MAX = (1U << PWM_BITS) - 1;

static void init_am_pwm() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = PWM_MODE;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = PWM_TIMER;
  timer.freq_hz = 312500;  // 80 MHz APB / 256
  timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));

  ledc_channel_config_t channel = {};
  channel.gpio_num = CONFIG_POLAR_AM_PWM_GPIO;
  channel.speed_mode = PWM_MODE;
  channel.channel = PWM_CHANNEL;
  channel.timer_sel = PWM_TIMER;
  channel.duty = 0;
  channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

static inline void write_am_pwm(uint16_t value) {
  if (value > PWM_MAX) value = PWM_MAX;
  // Only the TX task touches this channel, so the non-thread-safe direct API
  // is sufficient and does not require ESP-IDF's LEDC fade service.
  (void)ledc_set_duty(PWM_MODE, PWM_CHANNEL, value);
  (void)ledc_update_duty(PWM_MODE, PWM_CHANNEL);
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
  float t = static_cast<float>(sample_index) / sample_rate_hz;
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
  int64_t last_sample_time;
  uint32_t test_sample_index = 0;
  int32_t next_b_diff = 0;
  int next_amplitude = 16384;
  int previous_amplitude = 16384;
  int interpolation_base = 16384;
  int interpolation_slope = 0;
  uint32_t interpolation_step = 12;
  uint32_t sample_counter = 0;
  float smooth_amplitude = 16384.0f;
  float sigma_delta_error = 0.0f;
  double b_accumulator = 0.0;

  bool gate_enabled = true;
  bool gate_pending_disable = false;
  bool gate_pending_enable = false;
  uint32_t gate_holdoff_samples =
      (gate_holdoff_ms * profile.sample_rate_hz) / 1000;
  uint32_t holdoff_counter = 0;

  tx_dsp_reset();
  gpio_set_level(static_cast<gpio_num_t>(PTT), 1);
  if (!ENABLE_AM_MODULATION) set_fixed_tx_amplitude();
  ESP_LOGI(TAG, "configuring CLK0 for %llu Hz SSB", freq);
  si5351_setup_ssb(freq, profile.sample_rate_hz);
  printf("[TX-SSB] Start %llu Hz (%s)\n", freq, profile.name);
  if (use_phase_tone_test)
    printf("[PHASE-TEST] Alternating 700/1900 Hz every second; use *v* to disable\n");
  else if (optimized_phase_active)
    printf("[PHASE-OPT] Experimental 10 s trajectory; wideband dump energy; "
           "dummy-load testing only\n");
  else if (use_optimized_phase)
    printf("[PHASE-OPT] Requires the 16 ksample/s profile; using live DSP path\n");

  // Start the sample clock only after the blocking PLL configuration and UART
  // messages. Starting it at function entry made every TX begin about 2 ms
  // behind and created an unnecessary burst of catch-up phase writes.
  tx_deadline_misses = 0;
  tx_worst_lateness_us = 0;
  tx_phase_cycles_total = 0;
  tx_work_cycles_total = 0;
  tx_profiled_samples = 0;
  tx_phase_cycles_max = 0;
  tx_work_cycles_max = 0;
  last_sample_time = esp_timer_get_time();

  while (start_tx) {
    int64_t now = esp_timer_get_time();

    if (ENABLE_AM_MODULATION && interpolation_step < 12 &&
        now >= last_sample_time +
                   static_cast<int64_t>(interpolation_step + 1) *
                       profile.interpolation_interval_us) {
      int amplitude = interpolation_base +
                      interpolation_slope * static_cast<int>(interpolation_step + 1) /
                          12;
      if (amplitude < 0) amplitude = 0;
      if (amplitude > 65535) amplitude = 65535;
      float shaped = static_cast<float>(amplitude) / 256.0f + sigma_delta_error;
      int pwm = static_cast<int>(shaped + 0.5f);
      if (pwm < 0) pwm = 0;
      if (pwm > static_cast<int>(PWM_MAX)) pwm = PWM_MAX;
      sigma_delta_error = shaped - static_cast<float>(pwm);
      write_am_pwm(static_cast<uint16_t>(pwm));
      interpolation_step++;
    }

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
    sample_counter++;

    // This submits the phase update asynchronously. I2C runs in parallel with
    // the DSP below and must finish before the next sample reuses its buffer.
    uint32_t phase_cycle_start = esp_cpu_get_cycle_count();
    si5351_write_phase_fast(next_b_diff);
    uint32_t phase_cycles = esp_cpu_get_cycle_count() - phase_cycle_start;
    tx_phase_cycles_total += phase_cycles;
    if (phase_cycles > tx_phase_cycles_max) tx_phase_cycles_max = phase_cycles;

    if (phase_delay_us > 0) esp_rom_delay_us(phase_delay_us);
    if (ENABLE_AM_MODULATION) {
      float shaped =
          static_cast<float>(next_amplitude) / 256.0f + sigma_delta_error;
      int pwm = static_cast<int>(shaped + 0.5f);
      if (pwm < 0) pwm = 0;
      if (pwm > static_cast<int>(PWM_MAX)) pwm = PWM_MAX;
      sigma_delta_error = shaped - static_cast<float>(pwm);
      write_am_pwm(static_cast<uint16_t>(pwm));
    }

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
    uint32_t work_cycles = esp_cpu_get_cycle_count() - work_cycle_start;
    tx_work_cycles_total += work_cycles;
    if (work_cycles > tx_work_cycles_max) tx_work_cycles_max = work_cycles;
    tx_profiled_samples = tx_profiled_samples + 1;

    b_accumulator += static_cast<double>(phase_difference * ssb_phase_to_b_factor);
    next_b_diff = static_cast<int32_t>(b_accumulator);
    b_accumulator -= next_b_diff;
    if (b_accumulator > 1.0) b_accumulator -= 1.0;
    if (b_accumulator < -1.0) b_accumulator += 1.0;

    float target_amplitude = amplitude * 65535.0f * mod_depth;
    if (target_amplitude > 65535.0f) target_amplitude = 65535.0f;
    if (target_amplitude < 0.0f) target_amplitude = 0.0f;
    smooth_amplitude = smooth_amplitude * ampl_smooth +
                       target_amplitude * (1.0f - ampl_smooth);
    next_amplitude = static_cast<int>(smooth_amplitude);

    interpolation_base = previous_amplitude;
    interpolation_slope = next_amplitude - previous_amplitude;
    previous_amplitude = next_amplitude;
    interpolation_step = 0;

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

  si5351_dma_wait();
  si5351_output_enable(SI5351_CLK0, 0);
  write_am_pwm(0);
  gpio_set_level(static_cast<gpio_num_t>(PTT), 0);
  printf("[TX-SSB] End; deadline misses=%lu, worst lateness=%ld us\n",
         static_cast<unsigned long>(tx_deadline_misses),
         static_cast<long>(tx_worst_lateness_us));
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
    case 'd':
      phase_delay_us = atoi(command + 2);
      printf("Delay: %ld us\n", static_cast<long>(phase_delay_us));
      break;
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
               "deadline_miss=%lu worst_late=%ld us, "
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
