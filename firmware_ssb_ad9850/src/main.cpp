// SSB using Polar Modulation for Pico 2 (RP2350)
// USB stdio used for CAT and Debug

#include <string>
#include <ostream>
#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"

#include "ddx_common.h"
#include "polar_mod.h"
#include "ad9850.h"
#include "audio_data.h"
// #include "audio_data2.h"  // Disabled to save flash
#include "audio_data3.h"     // Winston Churchill speech (15 seconds)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define SAMPLE_BUFFER_SIZE 32

// Sample rate set via CMakeLists.txt (SAMPLE_RATE=32000 or 16000)
// 32kHz requires I2C >= 2.0MHz (2.5MHz recommended with 2k pull-ups)
#ifndef SAMPLE_RATE
#define SAMPLE_RATE 32000
#endif

// Derived timing constants
#if SAMPLE_RATE == 32000
#define SAMPLE_PERIOD_US_A 31  // Alternate 31/31/31/32 for 31.25us average
#define SAMPLE_PERIOD_US_B 32
#define SAMPLE_PERIOD_MASK 3  // Every 4th sample gets +1us
#define WAV_DIVISOR 1         // wav_data is 32kHz, play at native rate
#define DEFAULT_I2C_KHZ 2500  // 2.5MHz I2C needed for 32kHz
#else
#define SAMPLE_PERIOD_US_A 62  // Alternate 62/63 for 62.5us average
#define SAMPLE_PERIOD_US_B 63
#define SAMPLE_PERIOD_MASK 1  // Every 2nd sample gets +1us
#define WAV_DIVISOR 2         // wav_data is 32kHz, play every 2nd sample
#define DEFAULT_I2C_KHZ 1500  // 1.5MHz I2C sufficient for 16kHz
#endif

// Configuration Macros
#define USE_DRAIN_MODULATION 1
#define USE_PT8211 0
#define USE_AD9850 1
#define USE_ADC 0
#define USE_CORE1 1
#define USE_WATCHDOG 0
#define INVERT_PWM 0  // Set to 1 for buck regulators that need inverted PWM

// Pins
#define ONBOARD 16  // NeoPixel

// PT8211 I2S Pins
#define I2S_DATA_PIN 22
#define I2S_BCK_PIN 18
#define I2S_WS_PIN 19

// Drain Modulation (Single-pin 12-bit PWM DAC)
// 12-bit: 250MHz / 4096 = 61kHz PWM freq — still 20x above 3kHz audio BW
// +12 dB amplitude SNR vs 10-bit (~72 dB vs ~60 dB)
#define AM_PWM_PIN 0
#define AM_PWM_MAX 4095
#define AM_PWM_SHIFT 4  // 65535 >> 4 = 4095

void init_am_pwm() {
  gpio_set_function(AM_PWM_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(AM_PWM_PIN);
  pwm_config c = pwm_get_default_config();
  pwm_config_set_wrap(&c, AM_PWM_MAX);  // 12-bit resolution
  pwm_config_set_clkdiv(&c, 1.0f);      // No division = fastest PWM
  pwm_init(slice_num, &c, true);
}

void write_am_pwm(uint16_t value) {
  if (value > AM_PWM_MAX) value = AM_PWM_MAX;
#if INVERT_PWM
  value = AM_PWM_MAX - value;
#endif
  pwm_set_gpio_level(AM_PWM_PIN, value);
}

// Global State
volatile bool start_tx = true;  // Auto-start TX on boot
volatile bool is_ssb = true;
volatile bool use_two_tone_test = false;
volatile bool use_melody_test = false;
volatile bool use_wav_test = true;                 // WAV audio ON by default
volatile int wav_select = 2;                       // 0 = original, 1 = ARRL QEX SSB wideband test, 2 = Winston Churchill (default)
volatile int32_t phase_delay_us = 0;               // Delay for AD9850 to align with LM386
volatile uint32_t i2c_baud_khz = DEFAULT_I2C_KHZ;  // I2C speed in kHz (tunable with *iNNN*)

// Runtime-tunable parameters (CAT commands)
volatile float mod_depth = 0.75f;    // *nNN*  Modulation depth (0.01-1.0)

uint64_t freq = 14200000ULL;  // Default 20m
uint16_t offset = 1200;
absolute_time_t tx_timestamp;

// Audio Buffers
int16_t sample_buffer[SAMPLE_BUFFER_SIZE];
volatile int samples_read = 0;

#define CAT_MAX 1024

// Functions
void usb_stdio_task();
void tx_ssb();
void tx_ft8();

// Phase accumulators for test tones — avoids float precision loss after ~8 minutes
// (float sample_idx / SAMPLE_RATE loses sub-sample precision past 2^24 samples)
static float tt_phase0 = 0, tt_phase1 = 0;
static const float tt_inc0 = 2.0f * M_PI * 700.0f / (float)SAMPLE_RATE;
static const float tt_inc1 = 2.0f * M_PI * 1900.0f / (float)SAMPLE_RATE;

int16_t get_two_tone_sample(uint32_t sample_idx) {
  (void)sample_idx;
  float s = 0.5f * sinf(tt_phase0) + 0.5f * sinf(tt_phase1);
  tt_phase0 += tt_inc0;
  tt_phase1 += tt_inc1;
  if (tt_phase0 > 2.0f * M_PI) tt_phase0 -= 2.0f * M_PI;
  if (tt_phase1 > 2.0f * M_PI) tt_phase1 -= 2.0f * M_PI;
  return (int16_t)(s * 16384.0f);
}

static float mel_phase = 0;

int16_t get_melody_sample(uint32_t sample_idx) {
  uint32_t note_idx = (sample_idx / (SAMPLE_RATE / 4)) % 4;  // 0.25s per note
  static const float freqs[] = { 523.25f, 659.25f, 783.99f, 1046.50f };
  float inc = 2.0f * M_PI * freqs[note_idx] / (float)SAMPLE_RATE;
  float s = 0.8f * sinf(mel_phase);
  mel_phase += inc;
  if (mel_phase > 2.0f * M_PI) mel_phase -= 2.0f * M_PI;
  return (int16_t)(s * 16384.0f);
}

int16_t get_wav_sample(uint32_t sample_idx) {
  // wav arrays are at 32kHz; WAV_DIVISOR adapts to current sample rate
  // At 32kHz: read every sample (divisor=1). At 16kHz: read every 2nd (divisor=2).

  // Select audio source based on wav_select
  const int16_t* data = NULL;
  uint32_t data_len = 0;

  if (wav_select == 0) {
    data = wav_data;
    data_len = wav_data_len;
  } else if (wav_select == 2) {
    data = wav_data3;
    data_len = wav_data3_len;
  } else {
    // wav_select == 1 or other: default to empty (audio_data2 disabled)
    return 0;
  }

  if (data_len == 0) return 0;
#if WAV_DIVISOR == 2
  // Anti-alias: average adjacent samples before decimation.
  // Without this, content between 8-16kHz aliases into the audio band.
  uint32_t idx = (sample_idx * 2) % data_len;
  uint32_t idx1 = (idx + 1) % data_len;
  return (int16_t)(((int32_t)data[idx] + (int32_t)data[idx1]) / 2);
#else
  return data[sample_idx % data_len];
#endif
}

void tx_ssb() {
  uint64_t last_sample_time = time_us_64();
  uint32_t test_sample_idx = 0;
  uint32_t sample_counter = 0;
  int pending_ampl = 0;  // Amplitude buffered for DMA-aligned output

  // Reset all DSP state from any previous TX session
  polar_mod_reset();
  tt_phase0 = tt_phase1 = mel_phase = 0;  // Reset test tone phase accumulators

  // Ramp up amplitude over first 50 samples to avoid startup pop
  const int RAMP_SAMPLES = 50;

  gpio_put(PTT, 1);
  printf("[TX-SSB] Start %llu (%dkHz / 250MHz / SPI %luMHz / 12-bit PWM / DMA)\n",
         freq, SAMPLE_RATE / 1000, AD9850_SPI_BAUD / 1000000);
  printf("[TX-SSB] TwoTone=%d WAV=%d Melody=%d HIFI=1\n",
         use_two_tone_test, use_wav_test, use_melody_test);
  ad9850_setup_ssb(freq);
#if USE_DRAIN_MODULATION
  init_am_pwm();
#endif
#if USE_PT8211
  init_pt8211();
#endif

  while (start_tx) {
    uint64_t now = time_us_64();

    // Accurate sample timing using fractional microsecond compensation
    // 16kHz: alternate 62/63us (avg 62.5). 32kHz: 31/31/31/32us (avg 31.25)
    uint64_t target_time = last_sample_time + ((sample_counter & SAMPLE_PERIOD_MASK) == SAMPLE_PERIOD_MASK ? SAMPLE_PERIOD_US_B : SAMPLE_PERIOD_US_A);
    if (now >= target_time) {
      last_sample_time = target_time;
      sample_counter++;

      // ── Step 1: Latch PREVIOUS frequency (DMA completed during idle time) ──
      // Then write PREVIOUS amplitude — both correspond to the same sample,
      // so phase and amplitude are perfectly time-aligned (<0.1µs gap).
      ad9850_dma_latch();
#if USE_DRAIN_MODULATION
      write_am_pwm((uint16_t)(pending_ampl >> AM_PWM_SHIFT));
#endif

      // ── Step 2: Get current audio sample ───────────────────────────────────
      int16_t sample = 0;
      if (use_wav_test) {
        sample = get_wav_sample(test_sample_idx++);
      } else if (use_melody_test) sample = get_melody_sample(test_sample_idx++);
      else if (use_two_tone_test) sample = get_two_tone_sample(test_sample_idx++);
      else if (samples_read > 0) {
        sample = sample_buffer[0];
        samples_read = 0;
      }

      // ── Step 3: DSP — compute amplitude and phase ──────────────────────────
      float f_sample = (float)sample / 32768.0f;
      float f_ampl, f_phase_diff;
      modulation_am_pm_f(f_sample, &f_ampl, &f_phase_diff);

      // Phase → frequency deviation (radians/sample → Hz)
      float freq_dev = f_phase_diff * (float)SAMPLE_RATE / (2.0f * M_PI);

      // Amplitude scaling (12-bit PWM: 65535 → 4095 via >>4)
      float target_ampl = f_ampl * 65535.0f * mod_depth;
      if (target_ampl > 65535.0f) target_ampl = 65535.0f;
      if (target_ampl < 0.0f) target_ampl = 0.0f;

      // Startup ramp to avoid pop
      if (sample_counter <= RAMP_SAMPLES) {
        target_ampl *= (float)sample_counter / (float)RAMP_SAMPLES;
      }

      // ── Step 4: Start DMA for current frequency (non-blocking) ─────────────
      // SPI transfer runs in background (~4µs) during the idle wait for next sample.
      // This frees ~13% of CPU budget vs blocking SPI.
      if (phase_delay_us > 0) sleep_us(phase_delay_us);
      ad9850_dma_start(freq_dev);

      // ── Step 5: Save amplitude — will be written at Step 1 next iteration ──
      pending_ampl = (int)target_ampl;
    }

    tight_loop_contents();
  }

  // Flush last pending DMA transfer
  ad9850_dma_wait();
  gpio_put(PTT, 0);
  printf("[TX-SSB] End\n");
}

void tx_ft8() {
  gpio_put(PTT, 1);
  printf("[TX-FT8] Start %llu\n", freq);
  ad9850_set_frequency((uint32_t)(freq + offset));
  ad9850_output_enable(true);
  sleep_ms(1000);
  ad9850_output_enable(false);
  gpio_put(PTT, 0);
  printf("[TX-FT8] End\n");
}

void core1_entry() {
  while (1) {
    if (start_tx) {
      if (is_ssb) tx_ssb();
      else tx_ft8();
      start_tx = false;
    }
    sleep_ms(10);
  }
}

static char cat_buf[CAT_MAX];
static int cat_idx = 0;

void usb_stdio_task() {
  int c = getchar_timeout_us(0);
  while (c != PICO_ERROR_TIMEOUT) {
    cat_buf[cat_idx++] = (char)c;
    if (cat_idx >= CAT_MAX) cat_idx = 0;
    cat_buf[cat_idx] = 0;
    char *s = strchr(cat_buf, '*');
    char *e = strrchr(cat_buf, '*');
    if (s && e && s != e) {
      if (s[1] == 'u') {
        is_ssb = true;
        modulation_mode = MOD_USB;
        printf("USB\n");
      } else if (s[1] == 'l') {
        is_ssb = true;
        modulation_mode = MOD_LSB;
        printf("LSB\n");
      } else if (s[1] == 'k') {
        use_two_tone_test = !use_two_tone_test;
        printf("TwoTone: %d\n", use_two_tone_test);
      } else if (s[1] == 'm') {
        use_melody_test = !use_melody_test;
        printf("Melody: %d\n", use_melody_test);
      } else if (s[1] == 'w') {
        use_wav_test = !use_wav_test;
        printf("WAV: %d\n", use_wav_test);
      } else if (s[1] == 'a') {
        wav_select = (wav_select + 1) % 3;
        printf("WAV select: %d (%s)\n", wav_select,
               wav_select == 0 ? "original" :
               wav_select == 1 ? "ARRL QEX (disabled)" : "Winston Churchill");
      } else if (s[1] == 't') {
        start_tx = !start_tx;
        printf("TX: %d\n", start_tx);
      } else if (s[1] == 'f') {
        freq = strtoul(s + 2, NULL, 10);
        printf("Freq: %llu\n", freq);
      } else if (s[1] == 'd') {
        phase_delay_us = atoi(s + 2);
        printf("Delay: %d us\n", phase_delay_us);
      } else if (s[1] == 'i') {
        // I2C speed command kept for compatibility (not used with AD9850)
        printf("I2C speed command not used with AD9850 (SPI interface)\n");
        // ── Runtime tuning commands ────────────────────────────────────────
      } else if (s[1] == 'n') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 1 && v <= 100) {
          mod_depth = (float)v / 100.0f;
          printf("Mod depth: %.2f\n", mod_depth);
        }
      } else if (s[1] == 'e') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 1 && v <= 100) {
          pm_max_phase_step = (float)v / 10.0f;
          printf("Phase limit: %.1f\n", pm_max_phase_step);
        }
      } else if (s[1] == 'j') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 1 && v <= 1000) {
          pm_soft_knee = (float)v / 10000.0f;
          printf("Soft knee: %.4f\n", pm_soft_knee);
        }
      }
      cat_idx = 0;
      memset(cat_buf, 0, CAT_MAX);
    }
    c = getchar_timeout_us(0);
  }
}

int main() {
  set_sys_clock_khz(250000, true);
  stdio_init_all();
  gpio_init(PTT);
  gpio_set_dir(PTT, GPIO_OUT);
  gpio_put(PTT, 0);

#if USE_AD9850
  // Initialize AD9850 DDS chip
  // Initialize SPI pins
  gpio_init(AD9850_CS);
  gpio_set_dir(AD9850_CS, GPIO_OUT);
  gpio_put(AD9850_CS, true);  // Deselect

  gpio_init(AD9850_RESET);
  gpio_set_dir(AD9850_RESET, GPIO_OUT);

  gpio_init(AD9850_FQ_UD);
  gpio_set_dir(AD9850_FQ_UD, GPIO_OUT);

  // Initialize SPI
  spi_init(AD9850_SPI_PORT, AD9850_SPI_BAUD);
  gpio_set_function(AD9850_SPI_SCK, GPIO_FUNC_SPI);
  gpio_set_function(AD9850_SPI_MOSI, GPIO_FUNC_SPI);

  // Initialize AD9850
  ad9850_init(AD9850_SPI_PORT, AD9850_CS, AD9850_RESET, AD9850_FQ_UD, AD9850_SPI_BAUD);
  ad9850_reset();
  ad9850_power_up();
  ad9850_dma_init();
  printf("AD9850 OK\n");
#endif

#if USE_CORE1
  multicore_launch_core1(core1_entry);
#endif

  while (1) {
    usb_stdio_task();
    // board_id_task();
    tight_loop_contents();
  }
  return 0;
}
