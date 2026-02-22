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
#include "si5351.h"
#include "audio_data.h"
// #include "audio_data2.h"  // Disabled to save flash

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
#define INTERP_INTERVAL_US 3  // 31.25/12 ≈ 2.6us, round to 3
#define WAV_DIVISOR 1         // wav_data is 32kHz, play at native rate
#define DEFAULT_I2C_KHZ 2500  // 2.5MHz I2C needed for 32kHz
#else
#define SAMPLE_PERIOD_US_A 62  // Alternate 62/63 for 62.5us average
#define SAMPLE_PERIOD_US_B 63
#define SAMPLE_PERIOD_MASK 1  // Every 2nd sample gets +1us
#define INTERP_INTERVAL_US 5  // 62.5/12 ≈ 5.2us, round to 5
#define WAV_DIVISOR 2         // wav_data is 32kHz, play every 2nd sample
#define DEFAULT_I2C_KHZ 1500  // 1.5MHz I2C sufficient for 16kHz
#endif

// Configuration Macros
#define USE_DRAIN_MODULATION 1
#define USE_PT8211 0
#define USE_SI5351 1
#define USE_ADC 0
#define USE_CORE1 1
#define USE_WATCHDOG 0
#define USE_GATE_BLANKING 1
#ifndef USE_SPEECH_DSP
#define USE_SPEECH_DSP 1  // SSB speech processor: AGC + clipper + filter for ~6dB more talk power
#endif
#define INVERT_PWM 0  // Set to 1 for buck regulators that need inverted PWM

#if USE_SPEECH_DSP
#include "speech_dsp.h"
#endif

// Pins
#define ONBOARD 16  // NeoPixel

// PT8211 I2S Pins
#define I2S_DATA_PIN 22
#define I2S_BCK_PIN 18
#define I2S_WS_PIN 19

// Drain Modulation (Single-pin 10-bit PWM DAC)
#define AM_PWM_PIN 0

// Gate Blanking Configuration
#define GATE_BLANK_THRESHOLD_OFF 0.015f  // Disable gate when amplitude < 1.5%
#define GATE_BLANK_THRESHOLD_ON 0.040f   // Enable gate when amplitude > 4.0% (hysteresis)
#define GATE_BLANK_HOLDOFF_MS 50         // Hold gate on for 50ms after last signal

void init_am_pwm() {
  gpio_set_function(AM_PWM_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(AM_PWM_PIN);
  pwm_config c = pwm_get_default_config();
  // Higher PWM frequency for smoother amplitude control
  // At 250MHz sys clock: 250MHz / (1023+1) = 244kHz PWM freq
  pwm_config_set_wrap(&c, 1023);    // 10-bit resolution
  pwm_config_set_clkdiv(&c, 1.0f);  // No division = fastest PWM
  pwm_init(slice_num, &c, true);
}

void write_am_pwm(uint16_t value) {
  if (value > 1023) value = 1023;
#if INVERT_PWM
  // Invert PWM for buck regulators (0 becomes 1023, 1023 becomes 0)
  value = 1023 - value;
#endif
  pwm_set_gpio_level(AM_PWM_PIN, value);
}

// Global State
volatile bool start_tx = true;
volatile bool is_ssb = true;
volatile bool use_two_tone_test = false;
volatile bool use_melody_test = false;
volatile bool use_wav_test = true;
volatile int wav_select = 1;                       // 0 = original, 1 = ARRL QEX SSB wideband test
volatile int32_t phase_delay_us = 0;               // Delay for Si5351 to align with LM386
volatile uint32_t i2c_baud_khz = DEFAULT_I2C_KHZ;  // I2C speed in kHz (tunable with *iNNN*)
#if USE_SPEECH_DSP
volatile bool use_speech_dsp = true;  // Runtime toggle for speech compressor (*c*)
#endif
// Runtime-tunable parameters (CAT commands)
volatile float mod_depth = 0.75f;                           // *nNN*  Modulation depth (0.01-1.0)
volatile float ampl_smooth = 0.15f;                         // *sNNN* Amplitude smoothing (0.0-1.0)
volatile float gate_off_thresh = GATE_BLANK_THRESHOLD_OFF;  // *gNNN* Gate OFF threshold
volatile float gate_on_thresh = GATE_BLANK_THRESHOLD_ON;    // *hNNN* Gate ON threshold
volatile uint32_t gate_holdoff_ms = GATE_BLANK_HOLDOFF_MS;  // *pNNN* Gate holdoff (ms)

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

int16_t get_two_tone_sample(uint32_t sample_idx) {
  float t = (float)sample_idx / (float)SAMPLE_RATE;
  float s = 0.5f * sinf(2.0f * M_PI * 700.0f * t) + 0.5f * sinf(2.0f * M_PI * 1900.0f * t);
  return (int16_t)(s * 16384.0f);
}

int16_t get_melody_sample(uint32_t sample_idx) {
  uint32_t note_idx = (sample_idx / (SAMPLE_RATE / 4)) % 4;  // 0.25s per note
  float freqs[] = { 523.25f, 659.25f, 783.99f, 1046.50f };
  float f = freqs[note_idx];
  float t = (float)sample_idx / (float)SAMPLE_RATE;
  float s = 0.8f * sinf(2.0f * M_PI * f * t);
  return (int16_t)(s * 16384.0f);
}

int16_t get_wav_sample(uint32_t sample_idx) {
  // wav arrays are at 32kHz; WAV_DIVISOR adapts to current sample rate
  // At 32kHz: read every sample (divisor=1). At 16kHz: read every 2nd (divisor=2).
  if (wav_data_len == 0) return 0;
#if WAV_DIVISOR == 2
  // Anti-alias: average adjacent samples before decimation.
  // Without this, content between 8-16kHz aliases into the audio band.
  uint32_t idx = (sample_idx * 2) % wav_data_len;
  uint32_t idx1 = (idx + 1) % wav_data_len;
  return (int16_t)(((int32_t)wav_data[idx] + (int32_t)wav_data[idx1]) / 2);
#else
  return wav_data[sample_idx % wav_data_len];
#endif
}

void tx_ssb() {
  // Note: b_accu is now static double inside the loop for better precision
  uint64_t last_sample_time = time_us_64();
  uint32_t test_sample_idx = 0;
  int32_t next_b_diff = 0;
  int next_ampl = 16384;      // Start at moderate level, not zero
  int prev_ampl = 16384;      // Previous output amplitude for interpolation
  int interp_base = 16384;    // Base amplitude for 12x interpolation
  int interp_slope = 0;       // Amplitude delta over one sample period
  uint32_t interp_step = 12;  // Current interpolation step (12 = done)
  uint32_t sample_counter = 0;
  float smooth_ampl = 16384.0f;  // Initialize smoothing filter
  float sd_error = 0.0f;         // Sigma-delta error accumulator

#if USE_GATE_BLANKING
  bool gate_enabled = true;  // si5351_setup_ssb enables output, so start in sync
  bool gate_pending_disable = false;
  bool gate_pending_enable = false;
  uint32_t gate_holdoff_samples = (gate_holdoff_ms * SAMPLE_RATE) / 1000;
  uint32_t holdoff_counter = 0;
#endif

  // Reset all DSP state from any previous TX session
  polar_mod_reset();
#if USE_SPEECH_DSP
  speech_dsp_reset();
#endif

  gpio_put(PTT, 1);
  printf("[TX-SSB] Start %llu (%dkHz / 250MHz / I2C %lukHz)\n",
         freq, SAMPLE_RATE / 1000, (unsigned long)i2c_baud_khz);
  si5351_setup_ssb(freq);
#if USE_DRAIN_MODULATION
  init_am_pwm();
#endif
#if USE_PT8211
  init_pt8211();
#endif

  while (start_tx) {
    uint64_t now = time_us_64();

    // 12x amplitude interpolation (~192kHz at 16kHz, ~384kHz at 32kHz)
    // Linearly interpolates between previous and next amplitude values
#if USE_DRAIN_MODULATION
    if (interp_step < 12) {
      if (now >= last_sample_time + (uint64_t)(interp_step + 1) * INTERP_INTERVAL_US) {
        int ampl = interp_base + interp_slope * (int)(interp_step + 1) / 12;
        if (ampl < 0) ampl = 0;
        if (ampl > 65535) ampl = 65535;
        float sd_t = (float)ampl / 64.0f + sd_error;
        int sd_v = (int)(sd_t + 0.5f);
        if (sd_v < 0) sd_v = 0;
        if (sd_v > 1023) sd_v = 1023;
        sd_error = sd_t - (float)sd_v;
        write_am_pwm((uint16_t)sd_v);
        interp_step++;
      }
    }
#endif

    // Accurate sample timing using fractional microsecond compensation
    // 16kHz: alternate 62/63us (avg 62.5). 32kHz: 31/31/31/32us (avg 31.25)
    uint64_t target_time = last_sample_time + ((sample_counter & SAMPLE_PERIOD_MASK) == SAMPLE_PERIOD_MASK ? SAMPLE_PERIOD_US_B : SAMPLE_PERIOD_US_A);
    if (now >= target_time) {
      last_sample_time = target_time;
      sample_counter++;

      // 1. UPDATE PHASE FIRST for better sync
      si5351_write_phase_fast(next_b_diff);

      // 2. UPDATE AMPLITUDE (With optional calibration delay to align with phase)
      if (phase_delay_us > 0) sleep_us(phase_delay_us);
#if USE_DRAIN_MODULATION
      // Sigma-delta noise shaping: pushes quantization noise to inaudible frequencies
      {
        float sd_target = (float)next_ampl / 64.0f + sd_error;
        int sd_pwm = (int)(sd_target + 0.5f);
        if (sd_pwm < 0) sd_pwm = 0;
        if (sd_pwm > 1023) sd_pwm = 1023;
        sd_error = sd_target - (float)sd_pwm;
        write_am_pwm((uint16_t)sd_pwm);
      }
#endif
#if USE_PT8211
      write_pt8211((uint16_t)next_ampl, (uint16_t)next_ampl);
#endif

      // 3. CALCULATE NEXT VALUES
      int16_t sample = 0;
      if (use_wav_test) {
#if USE_SPEECH_DSP
        // Re-trigger fade-in at WAV loop boundary to suppress pop from discontinuity
        if (use_speech_dsp) {
          uint32_t wav_loop_len = wav_data_len / WAV_DIVISOR;
          if (wav_loop_len > 0 && test_sample_idx > 0 && (test_sample_idx % wav_loop_len) == 0)
            speech_dsp_retrigger_fadein();
        }
#endif
        sample = get_wav_sample(test_sample_idx++);
      } else if (use_melody_test) sample = get_melody_sample(test_sample_idx++);
      else if (use_two_tone_test) sample = get_two_tone_sample(test_sample_idx++);
      else if (samples_read > 0) {
        sample = sample_buffer[0];
        samples_read = 0;
      }

      float f_sample = (float)sample / 32768.0f;
#if USE_SPEECH_DSP
      if (use_speech_dsp)
        f_sample = speech_process(f_sample);
#endif
      float f_ampl, f_phase_diff;
      modulation_am_pm_f(f_sample, &f_ampl, &f_phase_diff);

      // Use double precision for phase accumulator to prevent long-term drift
      // Note: static persists across loop iterations; reset via polar_mod_reset path
      static double b_accu_d;
      if (sample_counter == 1) b_accu_d = 0.0;  // Reset on first sample of new TX
      b_accu_d += (double)(f_phase_diff * ssb_phase_to_b_factor);
      next_b_diff = (int32_t)b_accu_d;
      b_accu_d -= (double)next_b_diff;

      // Prevent accumulator drift by clamping fractional part
      if (b_accu_d > 1.0) b_accu_d -= 1.0;
      if (b_accu_d < -1.0) b_accu_d += 1.0;

      // Scale amplitude with proper limiting and smoothing
      // Modulation depth tunable via *nNN* CAT command (default 0.75)
      float target_ampl = f_ampl * 65535.0f * mod_depth;
      if (target_ampl > 65535.0f) target_ampl = 65535.0f;
      if (target_ampl < 0.0f) target_ampl = 0.0f;

      // Light smoothing: tunable via *sNNN* CAT command (default 0.15)
      // With 12x interpolation handling the fine steps, minimal smoothing is needed
      smooth_ampl = smooth_ampl * ampl_smooth + target_ampl * (1.0f - ampl_smooth);
      next_ampl = (int)smooth_ampl;

      // 12x amplitude interpolation: linear interpolation from prev to next
      // Between ticks, 11 intermediate values are output at ~5us intervals (~192kHz)
      interp_base = prev_ampl;
      interp_slope = next_ampl - prev_ampl;
      prev_ampl = next_ampl;
      interp_step = 0;  // Start interpolation (steps 0-11 between ticks)

#if USE_GATE_BLANKING
      // Gate blanking: set flags for deferred I2C (outside timing-critical path)
      float norm_ampl = f_ampl;

      if (gate_enabled) {
        if (norm_ampl < gate_off_thresh) {
          holdoff_counter++;
          if (holdoff_counter >= gate_holdoff_samples) {
            gate_pending_disable = true;
            gate_enabled = false;
            holdoff_counter = 0;
          }
        } else {
          holdoff_counter = 0;
        }
      } else {
        if (norm_ampl >= gate_on_thresh) {
          gate_pending_enable = true;
          gate_enabled = true;
          holdoff_counter = 0;
        }
      }
#endif
    }

#if USE_GATE_BLANKING
    // Deferred gate I2C: execute outside sample-timing window to avoid jitter
    if (gate_pending_disable) {
      si5351_dma_wait();  // Ensure DMA I2C is idle before blocking I2C call
      si5351_output_enable(SI5351_CLK0, 0);
      gate_pending_disable = false;
    } else if (gate_pending_enable) {
      si5351_dma_wait();  // Ensure DMA I2C is idle before blocking I2C call
      si5351_output_enable(SI5351_CLK0, 1);
      gate_pending_enable = false;
    }
#endif

    tight_loop_contents();
  }
  si5351_dma_wait();  // Ensure DMA I2C completes before final disable
  si5351_output_enable(SI5351_CLK0, 0);
  gpio_put(PTT, 0);
  printf("[TX-SSB] End\n");
}

void tx_ft8() {
  gpio_put(PTT, 1);
  printf("[TX-FT8] Start %llu\n", freq);
  uint64_t base = (freq + offset) * 100ULL;
  si5351_set_freq(base, SI5351_CLK0);
  si5351_output_enable(SI5351_CLK0, 1);
  sleep_ms(1000);
  si5351_output_enable(SI5351_CLK0, 0);
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
        wav_select = (wav_select + 1) % 2;
        printf("WAV select: %d (%s)\n", wav_select,
               wav_select == 0 ? "original" : "ARRL QEX SSB");
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
        uint32_t new_baud = strtoul(s + 2, NULL, 10);
        if (new_baud >= 100 && new_baud <= 3400) {
          i2c_baud_khz = new_baud;
#ifdef USE_PIO_I2C
          float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)i2c_baud_khz * 1000.0f);
          if (div < 1.0f) div = 1.0f;
          pio_sm_set_clkdiv(i2c_pio_inst, i2c_pio_sm, div);
#else
          i2c_set_baudrate(i2c1, i2c_baud_khz * 1000);
#endif
          printf("I2C: %lu kHz\n", (unsigned long)i2c_baud_khz);
        } else {
          printf("I2C: range 100-3400 kHz (current: %lu)\n", (unsigned long)i2c_baud_khz);
        }
#if USE_SPEECH_DSP
      } else if (s[1] == 'c') {
        use_speech_dsp = !use_speech_dsp;
        printf("Speech compressor: %s\n", use_speech_dsp ? "ON" : "OFF");
#endif
        // ── Runtime DSP tuning commands ────────────────────────────────────
      } else if (s[1] == 'n') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 1 && v <= 100) {
          mod_depth = (float)v / 100.0f;
          printf("Mod depth: %.2f\n", mod_depth);
        }
      } else if (s[1] == 's') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 0 && v <= 999) {
          ampl_smooth = (float)v / 1000.0f;
          printf("Smoothing: %.3f\n", ampl_smooth);
        }
#if USE_GATE_BLANKING
      } else if (s[1] == 'g') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 1 && v <= 500) {
          gate_off_thresh = (float)v / 1000.0f;
          printf("Gate OFF: %.3f\n", gate_off_thresh);
        }
      } else if (s[1] == 'h') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 1 && v <= 500) {
          gate_on_thresh = (float)v / 1000.0f;
          printf("Gate ON: %.3f\n", gate_on_thresh);
        }
      } else if (s[1] == 'p') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 1 && v <= 1000) {
          gate_holdoff_ms = v;
          printf("Gate holdoff: %lums\n", (unsigned long)gate_holdoff_ms);
        }
#endif
#if USE_SPEECH_DSP
      } else if (s[1] == 'A') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 10 && v <= 900) {
          sdsp_agc_target = (float)v / 1000.0f;
          printf("AGC target: %.3f\n", sdsp_agc_target);
        }
      } else if (s[1] == 'G') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 5 && v <= 200) {
          sdsp_agc_max_gain = (float)v / 10.0f;
          printf("AGC max gain: %.1f\n", sdsp_agc_max_gain);
        }
      } else if (s[1] == 'C') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 50 && v <= 900) {
          sdsp_clip_level = (float)v / 1000.0f;
          printf("Clip level: %.3f\n", sdsp_clip_level);
        }
      } else if (s[1] == 'E') {
        uint32_t v = strtoul(s + 2, NULL, 10);
        if (v >= 0 && v <= 200) {
          sdsp_preemph = (float)v / 100.0f;
          printf("Pre-emph: %.2f\n", sdsp_preemph);
        }
#endif
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

#ifdef USE_PIO_I2C
#include "hardware/pio.h"
#include "i2c.pio.h"

PIO i2c_pio_inst;
uint i2c_pio_sm;

void init_i2c1() {
  // Guard: called from both main() and si5351_init(), PIO programs can only be loaded once
  static bool initialized = false;
  if (initialized) return;
  initialized = true;

  // Use PIO1 for I2C-HS (PIO0 is used by NeoPixel ws2812)
  i2c_pio_inst = pio1;
  uint offset = pio_add_program(i2c_pio_inst, &i2c_program);
  pio_add_program(i2c_pio_inst, &set_scl_sda_program);
  i2c_pio_sm = pio_claim_unused_sm(i2c_pio_inst, true);
  i2c_program_init(i2c_pio_inst, i2c_pio_sm, offset,
                   I2C1_SDA, I2C1_SCL, PIO_I2C_PIN_HS);
  printf("[PIO-I2C] Init on PIO1 SM%u (SDA=%d SCL=%d)\n",
         i2c_pio_sm, I2C1_SDA, I2C1_SCL);
}
#else
void init_i2c1() {
  i2c_init(i2c1, i2c_baud_khz * 1000);
  gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C1_SDA);
  gpio_pull_up(I2C1_SCL);
}
#endif

void board_id_task() {
  static uint32_t last_print_ms = 0;
  uint32_t current_ms = to_ms_since_boot(get_absolute_time());
  if (current_ms - last_print_ms >= 5000) {
    last_print_ms = current_ms;
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    printf("Board ID: ");
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) printf("%02x", id.id[i]);
    printf("\n");
  }
}

int main() {
  set_sys_clock_khz(250000, true);
  stdio_init_all();
  gpio_init(PTT);
  gpio_set_dir(PTT, GPIO_OUT);
  gpio_put(PTT, 0);

#if USE_SI5351
  // Si5351 needs time after power-on to initialize its crystal oscillator.
  // On cold boot the chip may not respond immediately, so retry until it works.
  sleep_ms(3000);
  init_i2c1();
  while (true) {
    if (si5351_init(0x60, SI5351_CRYSTAL_LOAD_8PF, 26000000, 0)) {
      printf("Si5351 OK\n");
      break;
    }
    printf("Si5351 FAIL - retrying...\n");
    sleep_ms(500);
  }
  si5351_dma_init();
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
