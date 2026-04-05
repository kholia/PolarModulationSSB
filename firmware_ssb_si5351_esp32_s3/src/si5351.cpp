/*
 * si5351.c - Si5351 library
 *
 * Copyright (C) 2015 - 2019 Jason Milldrum <milldrum@gmail.com>
 *                           Dana H. Myers <k6jq@comcast.net>
 *
 * Raspberry Pi Pico port by Dhiru Kholia (2023).
 * ESP32-S3 ESP-IDF port (2026).
 *
 * Some tuning algorithms derived from clk-si5351.c in the Linux kernel.
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "si5351.h"
#include <limits.h>
#include <stdint.h>

#include <stdio.h>
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_i2c.h"
#include "esp_log.h"
#include "ddx_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define __not_in_flash_func(x) x

static const char *TAG = "si5351";

struct Si5351Status dev_status = { 0, 0, 0, 0, 0 };
struct Si5351IntStatus dev_int_status = { 0, 0, 0, 0 };

enum si5351_pll pll_assignment[8];
uint64_t clk_freq[8];
uint64_t plla_freq;
uint64_t pllb_freq;
enum si5351_pll_input plla_ref_osc;
enum si5351_pll_input pllb_ref_osc;
uint32_t xtal_freq[2];

// private:
uint64_t pll_calc(enum si5351_pll, uint64_t, struct Si5351RegSet *, int32_t, uint8_t);
uint64_t multisynth_calc(uint64_t, uint64_t, struct Si5351RegSet *);
uint64_t multisynth67_calc(uint64_t, uint64_t, struct Si5351RegSet *);
void update_sys_status(struct Si5351Status *);
void update_int_status(struct Si5351IntStatus *);
void ms_div(enum si5351_clock, uint8_t, uint8_t);
uint8_t select_r_div(uint64_t *);
uint8_t select_r_div_ms67(uint64_t *);
int32_t ref_correction[2];
uint8_t clkin_div;
uint8_t i2c_bus_addr;
bool clk_first_set[8];

uint32_t si5351_ssb_base_p1 = 0;
uint8_t si5351_ssb_pll_addr = SI5351_PLLA_PARAMETERS;
static uint8_t output_enable_state = 0xFF;
float ssb_phase_to_b_factor = 0;
static uint32_t last_phase_p1;
static uint32_t last_phase_p2;
static uint32_t si5351_ssb_base_p2;
static volatile struct Si5351PhaseStats phase_stats;

// polar_i2c_submit() copies this calculation buffer into its own queue.
static uint8_t si5351_dma_cmds[7];
static bool si5351_async_ready;

static void reset_phase_stats(uint32_t initial_p1, uint32_t initial_p2) {
  phase_stats.writes = 0;
  phase_stats.submit_errors = 0;
  phase_stats.wait_errors = 0;
  phase_stats.p1_upper_changes = 0;
  phase_stats.command_bytes = 0;
  phase_stats.min_b_diff = INT32_MAX;
  phase_stats.max_b_diff = INT32_MIN;
  phase_stats.last_b_diff = 0;
  phase_stats.last_p1 = initial_p1;
  phase_stats.last_p2 = initial_p2;
}

void si5351_get_phase_stats(struct Si5351PhaseStats *stats) {
  if (!stats) return;
  stats->writes = phase_stats.writes;
  stats->submit_errors = phase_stats.submit_errors;
  stats->wait_errors = phase_stats.wait_errors;
  stats->p1_upper_changes = phase_stats.p1_upper_changes;
  stats->command_bytes = phase_stats.command_bytes;
  stats->min_b_diff = phase_stats.min_b_diff;
  stats->max_b_diff = phase_stats.max_b_diff;
  stats->last_b_diff = phase_stats.last_b_diff;
  stats->last_p1 = phase_stats.last_p1;
  stats->last_p2 = phase_stats.last_p2;
}

void si5351_dma_init(void) {
  si5351_async_ready = polar_i2c_enable_async() == ESP_OK;
  if (!si5351_async_ready) ESP_LOGE(TAG, "low-level phase writes unavailable");
}

void si5351_dma_wait(void) {
  if (polar_i2c_wait() != ESP_OK) {
    phase_stats.wait_errors = phase_stats.wait_errors + 1;
    ESP_LOGE(TAG, "phase write timed out");
  }
}

void si5351_write_phase_fast(int32_t b_diff) {
  constexpr int64_t c = 1048575;

  phase_stats.writes = phase_stats.writes + 1;
  phase_stats.last_b_diff = b_diff;
  if (b_diff < phase_stats.min_b_diff) phase_stats.min_b_diff = b_diff;
  if (b_diff > phase_stats.max_b_diff) phase_stats.max_b_diff = b_diff;

  // The base P1/P2 pair is already known. A phase-frequency perturbation adds
  // exactly 128*b_diff to P2; normalize it with one carry/borrow instead of
  // repeating two 64-bit divisions for every sample.
  int64_t p2_signed = static_cast<int64_t>(si5351_ssb_base_p2) +
                      static_cast<int64_t>(b_diff) * 128LL;
  int32_t p1_adjust = 0;
  while (p2_signed >= c) {
    p2_signed -= c;
    p1_adjust++;
  }
  while (p2_signed < 0) {
    p2_signed += c;
    p1_adjust--;
  }
  uint32_t p1 = static_cast<uint32_t>(
      static_cast<int64_t>(si5351_ssb_base_p1) + p1_adjust);
  uint32_t p2 = static_cast<uint32_t>(p2_signed);

  // Write the shortest contiguous suffix which changed. polar_i2c_submit()
  // copies it into a two-entry ring, allowing the next command to be prepared
  // while the preceding transaction is still leaving the I2C peripheral.
  size_t len;
  if ((p1 & ~0xFFU) != (last_phase_p1 & ~0xFFU)) {
    // Rare carry into upper P1 bits: update P1[17:0] and P2 atomically.
    si5351_dma_cmds[0] = si5351_ssb_pll_addr + 2;
    si5351_dma_cmds[1] = (p1 >> 16) & 0x03;
    si5351_dma_cmds[2] = (p1 >> 8) & 0xFF;
    si5351_dma_cmds[3] = p1 & 0xFF;
    si5351_dma_cmds[4] = 0xF0 | ((p2 >> 16) & 0x0F);
    si5351_dma_cmds[5] = (p2 >> 8) & 0xFF;
    si5351_dma_cmds[6] = p2 & 0xFF;
    len = 7;
    phase_stats.p1_upper_changes = phase_stats.p1_upper_changes + 1;
  } else if ((p1 & 0xFFU) != (last_phase_p1 & 0xFFU)) {
    si5351_dma_cmds[0] = si5351_ssb_pll_addr + 4;
    si5351_dma_cmds[1] = p1 & 0xFF;
    si5351_dma_cmds[2] = 0xF0 | ((p2 >> 16) & 0x0F);
    si5351_dma_cmds[3] = (p2 >> 8) & 0xFF;
    si5351_dma_cmds[4] = p2 & 0xFF;
    len = 5;
  } else if ((p2 >> 8) == (last_phase_p2 >> 8)) {
    si5351_dma_cmds[0] = si5351_ssb_pll_addr + 7;
    si5351_dma_cmds[1] = p2 & 0xFF;
    len = 2;
  } else if ((p2 >> 16) == (last_phase_p2 >> 16)) {
    si5351_dma_cmds[0] = si5351_ssb_pll_addr + 6;
    si5351_dma_cmds[1] = (p2 >> 8) & 0xFF;
    si5351_dma_cmds[2] = p2 & 0xFF;
    len = 3;
  } else {
    si5351_dma_cmds[0] = si5351_ssb_pll_addr + 5;
    si5351_dma_cmds[1] = 0xF0 | ((p2 >> 16) & 0x0F);
    si5351_dma_cmds[2] = (p2 >> 8) & 0xFF;
    si5351_dma_cmds[3] = p2 & 0xFF;
    len = 4;
  }
  phase_stats.command_bytes = phase_stats.command_bytes + len;

  esp_err_t err = si5351_async_ready
                      ? polar_i2c_submit(si5351_dma_cmds, len)
                      : polar_i2c_write(si5351_dma_cmds, len);
  if (err != ESP_OK) {
    phase_stats.submit_errors = phase_stats.submit_errors + 1;
  } else {
    last_phase_p1 = p1;
    last_phase_p2 = p2;
    phase_stats.last_p1 = p1;
    phase_stats.last_p2 = p2;
  }
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static uint32_t select_ssb_ms_div(uint64_t freq_hz, uint32_t f_xtal) {
  uint32_t best_div = 0;
  uint64_t best_fraction_error = UINT64_MAX;
  uint64_t best_vco_error = UINT64_MAX;

  for (uint32_t div = 6; div <= 254; div += 2) {
    uint64_t vco = freq_hz * div;
    if (vco < SI5351_PLL_VCO_MIN || vco > SI5351_PLL_VCO_MAX) continue;

    // Prefer a PLL fraction close to 1/2. This maximizes headroom for positive
    // and negative phase excursions and avoids P1 carry-boundary churn.
    uint64_t remainder = vco % f_xtal;
    uint64_t twice_remainder = remainder * 2;
    uint64_t fraction_error = twice_remainder > f_xtal
                                  ? twice_remainder - f_xtal
                                  : f_xtal - twice_remainder;
    uint64_t vco_error = vco > 800000000ULL ? vco - 800000000ULL
                                             : 800000000ULL - vco;
    if (fraction_error < best_fraction_error ||
        (fraction_error == best_fraction_error && vco_error < best_vco_error)) {
      best_div = div;
      best_fraction_error = fraction_error;
      best_vco_error = vco_error;
    }
  }

  return best_div ? best_div : 6;
}

void si5351_setup_ssb(uint64_t freq_hz, uint32_t sample_rate_hz) {
  uint32_t f_xtal = xtal_freq[0];
  // Fixed c = 1048575 for maximum resolution
  uint32_t ms_div = select_ssb_ms_div(freq_hz, f_xtal);

  uint64_t vco_freq = freq_hz * ms_div;
  uint32_t a = (uint32_t)(vco_freq / f_xtal);
  // Use 64-bit arithmetic for better precision
  uint64_t temp = (vco_freq % f_xtal) * 1048575ULL;
  uint32_t b = (uint32_t)(temp / f_xtal);
  uint32_t c = 1048575;

  // Manually build and write PLL registers to ensure c=1048575
  uint32_t p1 = 128 * a + ((128 * b) / c) - 512;
  uint32_t p2 = 128 * b % c;
  uint32_t p3 = c;

  uint8_t pll_params[8];
  pll_params[0] = (uint8_t)((p3 >> 8) & 0xFF);
  pll_params[1] = (uint8_t)(p3 & 0xFF);
  pll_params[2] = (uint8_t)((p1 >> 16) & 0x03);
  pll_params[3] = (uint8_t)((p1 >> 8) & 0xFF);
  pll_params[4] = (uint8_t)(p1 & 0xFF);
  pll_params[5] = (uint8_t)(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F));
  pll_params[6] = (uint8_t)((p2 >> 8) & 0xFF);
  pll_params[7] = (uint8_t)(p2 & 0xFF);
  si5351_write_bulk(SI5351_PLLA_PARAMETERS, 8, pll_params);

  // Configure CLK0's integer Multisynth without register reads. Once the I2C
  // callback is installed, ESP-IDF's experimental async mode is safe for the
  // transmit-only modulation path but not for transmit/receive transactions.
  uint32_t ms_p1 = 128 * ms_div - 512;
  uint8_t ms_params[8] = {
      0,
      1,
      (uint8_t)((ms_p1 >> 16) & 0x03),
      (uint8_t)((ms_p1 >> 8) & 0xFF),
      (uint8_t)(ms_p1 & 0xFF),
      0,
      0,
      0,
  };
  si5351_write_bulk(SI5351_CLK0_PARAMETERS, 8, ms_params);
  si5351_write(SI5351_CLK0_CTRL,
               SI5351_CLK_INTEGER_MODE | SI5351_CLK_INPUT_MULTISYNTH_N |
                   SI5351_CLK_DRIVE_STRENGTH_2MA);

  si5351_ssb_base_p1 = p1;
  si5351_ssb_base_p2 = p2;
  si5351_ssb_pll_addr = SI5351_PLLA_PARAMETERS;
  last_phase_p1 = p1;
  last_phase_p2 = p2;
  reset_phase_stats(p1, p2);

  // Scale phase increments for the active 16/32 ksample/s profile.
  // Compute in double to avoid precision loss (numerator exceeds float's 7-digit precision)
  ssb_phase_to_b_factor = (float)(((double)1048575 * (double)ms_div *
                                   (double)sample_rate_hz) /
                                  (2.0 * M_PI * (double)f_xtal));

  ESP_LOGI(TAG,
           "SSB PLL: CLK0=%llu Hz, VCO=%llu Hz, MS=%lu, PLL=%lu+%lu/%lu, "
           "phase factor=%.3f",
           freq_hz, vco_freq, static_cast<unsigned long>(ms_div),
           static_cast<unsigned long>(a), static_cast<unsigned long>(b),
           static_cast<unsigned long>(c), ssb_phase_to_b_factor);

  pll_reset(SI5351_PLLA);
  si5351_output_enable(SI5351_CLK0, 1);
} /********************/
/* Public functions */
/********************/

/*
 * si5351_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t xtal_load_c, uint32_t ref_osc_freq, int32_t corr)
 *
 * Setup communications to the Si5351 and set the crystal
 * load capacitance.
 *
 * xtal_load_c - Crystal load capacitance. Use the SI5351_CRYSTAL_LOAD_*PF
 * defines in the header file
 * xo_freq - Crystal/reference oscillator frequency in 1 Hz increments.
 * Defaults to 25000000 if a 0 is used here.
 * corr - Frequency correction constant in parts-per-billion
 *
 * Returns a boolean that indicates whether a device was found on the desired
 * I2C address.
 *
 */
bool si5351_init(uint8_t i2c_addr, uint8_t xtal_load_c, uint32_t xo_freq, int32_t corr) {
  i2c_bus_addr = i2c_addr;
  xtal_freq[0] = SI5351_XTAL_FREQ;

  int tries = 0;

  // Start by using XO ref osc as default for each PLL
  plla_ref_osc = SI5351_PLL_INPUT_XO;
  pllb_ref_osc = SI5351_PLL_INPUT_XO;
  clkin_div = SI5351_CLKIN_DIV_1;

  // Keep probing and register initialization within the Si5351's published
  // I2C limit. The high-speed profile is selected only after init completes.
  constexpr uint32_t init_i2c_hz = 400000;
  ESP_LOGI(TAG, "probing address 0x%02x at %lu Hz", i2c_addr,
           static_cast<unsigned long>(init_i2c_hz));
  if (polar_i2c_init(i2c_addr, init_i2c_hz) != ESP_OK)
    return false;

  // Check for a device on the bus, bail out if it is not there
  uint8_t reg_val = 0;
  uint8_t status_address = SI5351_DEVICE_STATUS;
  if (polar_i2c_write_read(&status_address, 1, &reg_val, 1) != ESP_OK)
    return false;
  ESP_LOGI(TAG, "device status register: 0x%02x", reg_val);

  // A successful register read proves that the device is present. Other
  // status bits (including revision and loss-of-signal) need not be zero.
  uint8_t status_reg = reg_val;
  while (status_reg & SI5351_STATUS_SYS_INIT) {
    if (++tries > 256) {
      ESP_LOGW(TAG, "SYS_INIT did not clear (status 0x%02x)", status_reg);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    status_reg = si5351_read(SI5351_DEVICE_STATUS);
  }

  // Set crystal load capacitance
  si5351_write(SI5351_CRYSTAL_LOAD,
               (xtal_load_c & SI5351_CRYSTAL_LOAD_MASK) | 0b00010010);

  // Set up the XO reference frequency
  if (xo_freq != 0) {
    set_ref_freq(xo_freq, SI5351_PLL_INPUT_XO);
  } else {
    set_ref_freq(SI5351_XTAL_FREQ, SI5351_PLL_INPUT_XO);
  }

  // Set the frequency calibration for the XO
  si5351_set_correction(corr, SI5351_PLL_INPUT_XO);

  si5351_reset();

  ESP_LOGI(TAG, "register initialization complete");
  return true;
}

/*
 * si5351_reset(void)
 *
 * Call to reset the Si5351 to the state initialized by the library.
 *
 */
void si5351_reset(void) {
  // Initialize the CLK outputs according to flowchart in datasheet
  // First, turn them off
  si5351_write(16, 0x80);
  si5351_write(17, 0x80);
  si5351_write(18, 0x80);
  si5351_write(19, 0x80);
  si5351_write(20, 0x80);
  si5351_write(21, 0x80);
  si5351_write(22, 0x80);
  si5351_write(23, 0x80);
  output_enable_state = 0xFF;
  si5351_write(SI5351_OUTPUT_ENABLE_CTRL, output_enable_state);

  // Turn the clocks back on...
  si5351_write(16, 0x0c);
  si5351_write(17, 0x0c);
  si5351_write(18, 0x0c);
  si5351_write(19, 0x0c);
  si5351_write(20, 0x0c);
  si5351_write(21, 0x0c);
  si5351_write(22, 0x0c);
  si5351_write(23, 0x0c);

  // Set PLLA and PLLB to 800 MHz for automatic tuning
  set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  set_pll(SI5351_PLL_FIXED, SI5351_PLLB);

  // Make PLL to CLK assignments for automatic tuning
  pll_assignment[0] = SI5351_PLLA;
  pll_assignment[1] = SI5351_PLLA;
  pll_assignment[2] = SI5351_PLLA;
  pll_assignment[3] = SI5351_PLLA;
  pll_assignment[4] = SI5351_PLLA;
  pll_assignment[5] = SI5351_PLLA;
  pll_assignment[6] = SI5351_PLLB;
  pll_assignment[7] = SI5351_PLLB;

  set_ms_source(SI5351_CLK0, SI5351_PLLA);
  set_ms_source(SI5351_CLK1, SI5351_PLLA);
  set_ms_source(SI5351_CLK2, SI5351_PLLA);
  set_ms_source(SI5351_CLK3, SI5351_PLLA);
  set_ms_source(SI5351_CLK4, SI5351_PLLA);
  set_ms_source(SI5351_CLK5, SI5351_PLLA);
  set_ms_source(SI5351_CLK6, SI5351_PLLB);
  set_ms_source(SI5351_CLK7, SI5351_PLLB);

  // Reset the VCXO param
  si5351_write(SI5351_VXCO_PARAMETERS_LOW, 0);
  si5351_write(SI5351_VXCO_PARAMETERS_MID, 0);
  si5351_write(SI5351_VXCO_PARAMETERS_HIGH, 0);

  // Then reset the PLLs
  pll_reset(SI5351_PLLA);
  pll_reset(SI5351_PLLB);

  // Set initial frequencies
  uint8_t i;
  for (i = 0; i < 8; i++) {
    clk_freq[i] = 0;
    si5351_output_enable((enum si5351_clock)i, 0);
    clk_first_set[i] = false;
  }
}

/*
 * si5351_set_freq(uint64_t freq, enum si5351_clock clk)
 *
 * Sets the clock frequency of the specified CLK output.
 * Frequency range of 8 kHz to 150 MHz
 *
 * freq - Output frequency in Hz
 * clk - Clock output
 *   (use the si5351_clock enum)
 */
uint8_t si5351_set_freq(uint64_t freq, enum si5351_clock clk) {
  struct Si5351RegSet ms_reg;
  uint64_t pll_freq;
  uint8_t int_mode = 0;
  uint8_t div_by_4 = 0;
  uint8_t r_div = 0;

  // Check which Multisynth is being set
  if ((uint8_t)clk <= (uint8_t)SI5351_CLK5) {
    // MS0 through MS5 logic
    // ---------------------

    // Lower bounds check
    if (freq > 0 && freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) {
      freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;
    }

    // Upper bounds check
    if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT) {
      freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
    }

    // If requested freq >100 MHz and no other outputs are already >100 MHz,
    // we need to recalculate PLLA and then recalculate all other CLK outputs
    // on same PLL
    if (freq > (SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT)) {
      // Check other clocks on same PLL
      uint8_t i;
      for (i = 0; i < 6; i++) {
        if (clk_freq[i] > (SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT)) {
          if (i != (uint8_t)clk && pll_assignment[i] == pll_assignment[clk]) {
            return 1;  // won't set if any other clks already >100 MHz
          }
        }
      }

      // Enable the output on first set_freq only
      if (clk_first_set[(uint8_t)clk] == false) {
        si5351_output_enable(clk, 1);
        clk_first_set[(uint8_t)clk] = true;
      }

      // Set the freq in memory
      clk_freq[(uint8_t)clk] = freq;

      // Calculate the proper PLL frequency
      pll_freq = multisynth_calc(freq, 0, &ms_reg);

      // Set PLL
      set_pll(pll_freq, pll_assignment[clk]);

      // Recalculate params for other synths on same PLL
      for (i = 0; i < 6; i++) {
        if (clk_freq[i] != 0) {
          if (pll_assignment[i] == pll_assignment[clk]) {
            struct Si5351RegSet temp_reg;
            uint64_t temp_freq;

            // Select the proper R div value
            temp_freq = clk_freq[i];
            r_div = select_r_div(&temp_freq);

            multisynth_calc(temp_freq, pll_freq, &temp_reg);

            // If freq > 150 MHz, we need to use DIVBY4 and integer mode
            if (temp_freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT) {
              div_by_4 = 1;
              int_mode = 1;
            } else {
              div_by_4 = 0;
              int_mode = 0;
            }

            // Set multisynth registers
            set_ms((enum si5351_clock)i, temp_reg, int_mode, r_div, div_by_4);
          }
        }
      }

      // Reset the PLL
      pll_reset(pll_assignment[clk]);
    } else {
      clk_freq[(uint8_t)clk] = freq;

      // Enable the output on first set_freq only
      if (clk_first_set[(uint8_t)clk] == false) {
        si5351_output_enable(clk, 1);
        clk_first_set[(uint8_t)clk] = true;
      }

      // Select the proper R div value
      r_div = select_r_div(&freq);

      // Calculate the synth parameters
      if (pll_assignment[clk] == SI5351_PLLA) {
        multisynth_calc(freq, plla_freq, &ms_reg);
      } else {
        multisynth_calc(freq, pllb_freq, &ms_reg);
      }

      // Set multisynth registers
      set_ms(clk, ms_reg, int_mode, r_div, div_by_4);

      // Reset the PLL
      //pll_reset(pll_assignment[clk]);
    }

    return 0;
  } else {
    // MS6 and MS7 logic
    // -----------------

    // Lower bounds check
    if (freq > 0 && freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT) {
      freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;
    }

    // Upper bounds check
    if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT) {
      freq = SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT - 1;
    }

    // If one of CLK6 or CLK7 is already set when trying to set the other,
    // we have to ensure that it will also have an integer division ratio
    // with the same PLL, otherwise do not set it.
    if (clk == SI5351_CLK6) {
      if (clk_freq[7] != 0) {
        if (pllb_freq % freq == 0) {
          if ((pllb_freq / freq) % 2 != 0) {
            // Not an even divide ratio, no bueno
            return 1;
          } else {
            // Set the freq in memory
            clk_freq[(uint8_t)clk] = freq;

            // Select the proper R div value
            r_div = select_r_div_ms67(&freq);

            multisynth67_calc(freq, pllb_freq, &ms_reg);
          }
        } else {
          // Not an integer divide ratio, no good
          return 1;
        }
      } else {
        // No previous assignment, so set PLLB based on CLK6

        // Set the freq in memory
        clk_freq[(uint8_t)clk] = freq;

        // Select the proper R div value
        r_div = select_r_div_ms67(&freq);

        pll_freq = multisynth67_calc(freq, 0, &ms_reg);
        //pllb_freq = pll_freq;
        set_pll(pll_freq, SI5351_PLLB);
      }
    } else {
      if (clk_freq[6] != 0) {
        if (pllb_freq % freq == 0) {
          if ((pllb_freq / freq) % 2 != 0) {
            // Not an even divide ratio, no bueno
            return 1;
          } else {
            // Set the freq in memory
            clk_freq[(uint8_t)clk] = freq;

            // Select the proper R div value
            r_div = select_r_div_ms67(&freq);

            multisynth67_calc(freq, pllb_freq, &ms_reg);
          }
        } else {
          // Not an integer divide ratio, no good
          return 1;
        }
      } else {
        // No previous assignment, so set PLLB based on CLK7

        // Set the freq in memory
        clk_freq[(uint8_t)clk] = freq;

        // Select the proper R div value
        r_div = select_r_div_ms67(&freq);

        pll_freq = multisynth67_calc(freq, 0, &ms_reg);
        //pllb_freq = pll_freq;
        set_pll(pll_freq, pll_assignment[clk]);
      }
    }

    div_by_4 = 0;
    int_mode = 0;

    // Set multisynth registers (MS must be set before PLL)
    set_ms(clk, ms_reg, int_mode, r_div, div_by_4);

    return 0;
  }
}

/*
 * set_freq_manual(uint64_t freq, uint64_t pll_freq, enum si5351_clock clk)
 *
 * Sets the clock frequency of the specified CLK output using the given PLL
 * frequency. You must ensure that the MS is assigned to the correct PLL and
 * that the PLL is set to the correct frequency before using this method.
 *
 * It is important to note that if you use this method, you will have to
 * track that all settings are sane yourself.
 *
 * freq - Output frequency in Hz
 * pll_freq - Frequency of the PLL driving the Multisynth in Hz * 100
 * clk - Clock output
 *   (use the si5351_clock enum)
 */
uint8_t set_freq_manual(uint64_t freq, uint64_t pll_freq, enum si5351_clock clk) {
  struct Si5351RegSet ms_reg;
  uint8_t int_mode = 0;
  uint8_t div_by_4 = 0;

  // Lower bounds check
  if (freq > 0 && freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) {
    freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;
  }

  // Upper bounds check
  if (freq > SI5351_CLKOUT_MAX_FREQ * SI5351_FREQ_MULT) {
    freq = SI5351_CLKOUT_MAX_FREQ * SI5351_FREQ_MULT;
  }

  uint8_t r_div;

  clk_freq[(uint8_t)clk] = freq;

  set_pll(pll_freq, pll_assignment[clk]);

  // Enable the output
  si5351_output_enable(clk, 1);

  // Select the proper R div value
  r_div = select_r_div(&freq);

  // Calculate the synth parameters
  multisynth_calc(freq, pll_freq, &ms_reg);

  // If freq > 150 MHz, we need to use DIVBY4 and integer mode
  if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT) {
    div_by_4 = 1;
    int_mode = 1;
  }

  // Set multisynth registers (MS must be set before PLL)
  set_ms(clk, ms_reg, int_mode, r_div, div_by_4);

  return 0;
}

/*
 * set_pll(uint64_t pll_freq, enum si5351_pll target_pll)
 *
 * Set the specified PLL to a specific oscillation frequency
 *
 * pll_freq - Desired PLL frequency in Hz * 100
 * target_pll - Which PLL to set
 *     (use the si5351_pll enum)
 */
void set_pll(uint64_t pll_freq, enum si5351_pll target_pll) {
  struct Si5351RegSet pll_reg;

  if (target_pll == SI5351_PLLA) {
    pll_calc(SI5351_PLLA, pll_freq, &pll_reg, ref_correction[plla_ref_osc], 0);
  } else {
    pll_calc(SI5351_PLLB, pll_freq, &pll_reg, ref_correction[pllb_ref_osc], 0);
  }

  // Derive the register values to write

  // Prepare an array for parameters to be written to
  uint8_t __p[20];
  uint8_t *params = __p;
  uint8_t i = 0;
  uint8_t temp;

  // Registers 26-27
  temp = ((pll_reg.p3 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p3 & 0xFF);
  params[i++] = temp;

  // Register 28
  temp = (uint8_t)((pll_reg.p1 >> 16) & 0x03);
  params[i++] = temp;

  // Registers 29-30
  temp = (uint8_t)((pll_reg.p1 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p1 & 0xFF);
  params[i++] = temp;

  // Register 31
  temp = (uint8_t)((pll_reg.p3 >> 12) & 0xF0);
  temp += (uint8_t)((pll_reg.p2 >> 16) & 0x0F);
  params[i++] = temp;

  // Registers 32-33
  temp = (uint8_t)((pll_reg.p2 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p2 & 0xFF);
  params[i++] = temp;

  // Write the parameters
  if (target_pll == SI5351_PLLA) {
    si5351_write_bulk(SI5351_PLLA_PARAMETERS, i, params);
    plla_freq = pll_freq;
  } else if (target_pll == SI5351_PLLB) {
    si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, params);
    pllb_freq = pll_freq;
  }
}

/*
 * set_ms(enum si5351_clock clk, struct Si5351RegSet ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4)
 *
 * Set the specified multisynth parameters. Not normally needed, but public for advanced users.
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * int_mode - Set integer mode
 *  Set to 1 to enable, 0 to disable
 * r_div - Desired r_div ratio
 * div_by_4 - Set Divide By 4 mode
 *   Set to 1 to enable, 0 to disable
 */
void set_ms(enum si5351_clock clk, struct Si5351RegSet ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4) {
  uint8_t __p[20];
  uint8_t *params = __p;

  uint8_t i = 0;
  uint8_t temp;
  uint8_t reg_val;


  if ((uint8_t)clk <= (uint8_t)SI5351_CLK5) {
    // Registers 42-43 for CLK0
    temp = (uint8_t)((ms_reg.p3 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(ms_reg.p3 & 0xFF);
    params[i++] = temp;

    // Register 44 for CLK0
    reg_val = si5351_read((SI5351_CLK0_PARAMETERS + 2) + (clk * 8));
    reg_val &= ~(0x03);
    temp = reg_val | ((uint8_t)((ms_reg.p1 >> 16) & 0x03));
    params[i++] = temp;

    // Registers 45-46 for CLK0
    temp = (uint8_t)((ms_reg.p1 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(ms_reg.p1 & 0xFF);
    params[i++] = temp;

    // Register 47 for CLK0
    temp = (uint8_t)((ms_reg.p3 >> 12) & 0xF0);
    temp += (uint8_t)((ms_reg.p2 >> 16) & 0x0F);
    params[i++] = temp;

    // Registers 48-49 for CLK0
    temp = (uint8_t)((ms_reg.p2 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(ms_reg.p2 & 0xFF);
    params[i++] = temp;
  } else {
    // MS6 and MS7 only use one register
    temp = ms_reg.p1;
  }

  // Write the parameters
  switch (clk) {
    case SI5351_CLK0:
      si5351_write_bulk(SI5351_CLK0_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK1:
      si5351_write_bulk(SI5351_CLK1_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK2:
      si5351_write_bulk(SI5351_CLK2_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK3:
      si5351_write_bulk(SI5351_CLK3_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK4:
      si5351_write_bulk(SI5351_CLK4_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK5:
      si5351_write_bulk(SI5351_CLK5_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK6:
      si5351_write(SI5351_CLK6_PARAMETERS, temp);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK7:
      si5351_write(SI5351_CLK7_PARAMETERS, temp);
      ms_div(clk, r_div, div_by_4);
      break;
  }
}

/*
 * si5351_output_enable(enum si5351_clock clk, uint8_t enable)
 *
 * Enable or disable a chosen output
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 */
void si5351_output_enable(enum si5351_clock clk, uint8_t enable) {
  if (enable == 1) {
    output_enable_state &= ~(1 << (uint8_t)clk);
  } else {
    output_enable_state |= (1 << (uint8_t)clk);
  }

  si5351_write(SI5351_OUTPUT_ENABLE_CTRL, output_enable_state);
}

/*
 * si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
 *
 * Sets the drive strength of the specified clock output
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * drive - Desired drive level
 *   (use the si5351_drive enum)
 */
void si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive) {
  uint8_t reg_val;
  const uint8_t mask = 0x03;

  reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);
  reg_val &= ~(mask);

  switch (drive) {
    case SI5351_DRIVE_2MA:
      reg_val |= 0x00;
      break;
    case SI5351_DRIVE_4MA:
      reg_val |= 0x01;
      break;
    case SI5351_DRIVE_6MA:
      reg_val |= 0x02;
      break;
    case SI5351_DRIVE_8MA:
      reg_val |= 0x03;
      break;
    default:
      break;
  }

  si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * update_status(void)
 *
 * Call this to update the status structs, then access them
 * via the dev_status and dev_int_status global members.
 *
 * See the header file for the struct definitions. These
 * correspond to the flag names for registers 0 and 1 in
 * the Si5351 datasheet.
 */
void update_status(void) {
  update_sys_status(&dev_status);
  update_int_status(&dev_int_status);
}

/*
 * set_correction(int32_t corr, enum si5351_pll_input ref_osc)
 *
 * corr - Correction factor in ppb
 * ref_osc - Desired reference oscillator
 *     (use the si5351_pll_input enum)
 *
 * Use this to set the oscillator correction factor.
 * This value is a signed 32-bit integer of the
 * parts-per-billion value that the actual oscillation
 * frequency deviates from the specified frequency.
 *
 * The frequency calibration is done as a one-time procedure.
 * Any desired test frequency within the normal range of the
 * Si5351 should be set, then the actual output frequency
 * should be measured as accurately as possible. The
 * difference between the measured and specified frequencies
 * should be calculated in Hertz, then multiplied by 10 in
 * order to get the parts-per-billion value.
 *
 * Since the Si5351 itself has an intrinsic 0 PPM error, this
 * correction factor is good across the entire tuning range of
 * the Si5351. Once this calibration is done accurately, it
 * should not have to be done again for the same Si5351 and
 * crystal.
 */
void si5351_set_correction(int32_t corr, enum si5351_pll_input ref_osc) {
  ref_correction[(uint8_t)ref_osc] = corr;

  // Recalculate and set PLL freqs based on correction value
  set_pll(plla_freq, SI5351_PLLA);
  set_pll(pllb_freq, SI5351_PLLB);
}

/*
 * set_phase(enum si5351_clock clk, uint8_t phase)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * phase - 7-bit phase word
 *   (in units of VCO/4 period)
 *
 * Write the 7-bit phase register. This must be used
 * with a user-set PLL frequency so that the user can
 * calculate the proper tuning word based on the PLL period.
 */
void set_phase(enum si5351_clock clk, uint8_t phase) {
  // Mask off the upper bit since it is reserved
  phase = phase & 0b01111111;

  si5351_write(SI5351_CLK0_PHASE_OFFSET + (uint8_t)clk, phase);
}

/*
 * get_correction(enum si5351_pll_input ref_osc)
 *
 * ref_osc - Desired reference oscillator
 *     0: crystal oscillator (XO)
 *     1: external clock input (CLKIN)
 *
 * Returns the oscillator correction factor stored
 * in RAM.
 */
int32_t get_correction(enum si5351_pll_input ref_osc) {
  return ref_correction[(uint8_t)ref_osc];
}

/*
 * pll_reset(enum si5351_pll target_pll)
 *
 * target_pll - Which PLL to reset
 *     (use the si5351_pll enum)
 *
 * Apply a reset to the indicated PLL.
 */
void pll_reset(enum si5351_pll target_pll) {
  if (target_pll == SI5351_PLLA) {
    si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_A);
  } else if (target_pll == SI5351_PLLB) {
    si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_B);
  }
}

/*
 * set_ms_source(enum si5351_clock clk, enum si5351_pll pll)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * pll - Which PLL to use as the source
 *     (use the si5351_pll enum)
 *
 * Set the desired PLL source for a multisynth.
 */
void set_ms_source(enum si5351_clock clk, enum si5351_pll pll) {
  uint8_t reg_val;

  reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

  if (pll == SI5351_PLLA) {
    reg_val &= ~(SI5351_CLK_PLL_SELECT);
  } else if (pll == SI5351_PLLB) {
    reg_val |= SI5351_CLK_PLL_SELECT;
  }

  si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);

  pll_assignment[(uint8_t)clk] = pll;
}

/*
 * set_int(enum si5351_clock clk, uint8_t int_mode)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 *
 * Set the indicated multisynth into integer mode.
 */
void set_int(enum si5351_clock clk, uint8_t enable) {
  uint8_t reg_val;
  reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

  if (enable == 1) {
    reg_val |= (SI5351_CLK_INTEGER_MODE);
  } else {
    reg_val &= ~(SI5351_CLK_INTEGER_MODE);
  }

  si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);

  // Integer mode indication
  /*
	switch(clk)
	{
	case SI5351_CLK0:
		clk0_int_mode = enable;
		break;
	case SI5351_CLK1:
		clk1_int_mode = enable;
		break;
	case SI5351_CLK2:
		clk2_int_mode = enable;
		break;
	default:
		break;
	}
	*/
}

/*
 * set_clock_pwr(enum si5351_clock clk, uint8_t pwr)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * pwr - Set to 1 to enable, 0 to disable
 *
 * Enable or disable power to a clock output (a power
 * saving feature).
 */
void si5351_set_clock_pwr(enum si5351_clock clk, uint8_t pwr) {
  uint8_t reg_val;  //, reg;
  reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

  if (pwr == 1) {
    reg_val &= 0b01111111;
  } else {
    reg_val |= 0b10000000;
  }

  si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * set_clock_invert(enum si5351_clock clk, uint8_t inv)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * inv - Set to 1 to enable, 0 to disable
 *
 * Enable to invert the clock output waveform.
 */
void set_clock_invert(enum si5351_clock clk, uint8_t inv) {
  uint8_t reg_val;
  reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

  if (inv == 1) {
    reg_val |= (SI5351_CLK_INVERT);
  } else {
    reg_val &= ~(SI5351_CLK_INVERT);
  }

  si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * set_clock_source(enum si5351_clock clk, enum si5351_clock_source src)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * src - Which clock source to use for the multisynth
 *   (use the si5351_clock_source enum)
 *
 * Set the clock source for a multisynth (based on the options
 * presented for Registers 16-23 in the Silicon Labs AN619 document).
 * Choices are XTAL, CLKIN, MS0, or the multisynth associated with
 * the clock output.
 */
void set_clock_source(enum si5351_clock clk, enum si5351_clock_source src) {
  uint8_t reg_val;
  reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

  // Clear the bits first
  reg_val &= ~(SI5351_CLK_INPUT_MASK);

  switch (src) {
    case SI5351_CLK_SRC_XTAL:
      reg_val |= (SI5351_CLK_INPUT_XTAL);
      break;
    case SI5351_CLK_SRC_CLKIN:
      reg_val |= (SI5351_CLK_INPUT_CLKIN);
      break;
    case SI5351_CLK_SRC_MS0:
      if (clk == SI5351_CLK0) {
        return;
      }

      reg_val |= (SI5351_CLK_INPUT_MULTISYNTH_0_4);
      break;
    case SI5351_CLK_SRC_MS:
      reg_val |= (SI5351_CLK_INPUT_MULTISYNTH_N);
      break;
    default:
      return;
  }

  si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * set_clock_disable(enum si5351_clock clk, enum si5351_clock_disable dis_state)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * dis_state - Desired state of the output upon disable
 *   (use the si5351_clock_disable enum)
 *
 * Set the state of the clock output when it is disabled. Per page 27
 * of AN619 (Registers 24 and 25), there are four possible values: low,
 * high, high impedance, and never disabled.
 */
void set_clock_disable(enum si5351_clock clk, enum si5351_clock_disable dis_state) {
  uint8_t reg_val, reg;

  if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) {
    reg = SI5351_CLK3_0_DISABLE_STATE;
  } else if (clk >= SI5351_CLK4 && clk <= SI5351_CLK7) {
    reg = SI5351_CLK7_4_DISABLE_STATE;
  } else return;

  reg_val = si5351_read(reg);

  if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) {
    reg_val &= ~(0b11 << (clk * 2));
    reg_val |= dis_state << (clk * 2);
  } else if (clk >= SI5351_CLK4 && clk <= SI5351_CLK7) {
    reg_val &= ~(0b11 << ((clk - 4) * 2));
    reg_val |= dis_state << ((clk - 4) * 2);
  }

  si5351_write(reg, reg_val);
}

/*
 * set_clock_fanout(enum si5351_clock_fanout fanout, uint8_t enable)
 *
 * fanout - Desired clock fanout
 *   (use the si5351_clock_fanout enum)
 * enable - Set to 1 to enable, 0 to disable
 *
 * Use this function to enable or disable the clock fanout options
 * for individual clock outputs. If you intend to output the XO or
 * CLKIN on the clock outputs, enable this first.
 *
 * By default, only the Multisynth fanout is enabled at startup.
 */
void set_clock_fanout(enum si5351_clock_fanout fanout, uint8_t enable) {
  uint8_t reg_val;
  reg_val = si5351_read(SI5351_FANOUT_ENABLE);

  switch (fanout) {
    case SI5351_FANOUT_CLKIN:
      if (enable) {
        reg_val |= SI5351_CLKIN_ENABLE;
      } else {
        reg_val &= ~(SI5351_CLKIN_ENABLE);
      }
      break;
    case SI5351_FANOUT_XO:
      if (enable) {
        reg_val |= SI5351_XTAL_ENABLE;
      } else {
        reg_val &= ~(SI5351_XTAL_ENABLE);
      }
      break;
    case SI5351_FANOUT_MS:
      if (enable) {
        reg_val |= SI5351_MULTISYNTH_ENABLE;
      } else {
        reg_val &= ~(SI5351_MULTISYNTH_ENABLE);
      }
      break;
  }

  si5351_write(SI5351_FANOUT_ENABLE, reg_val);
}

/*
 * set_pll_input(enum si5351_pll pll, enum si5351_pll_input input)
 *
 * pll - Which PLL to use as the source
 *     (use the si5351_pll enum)
 * input - Which reference oscillator to use as PLL input
 *     (use the si5351_pll_input enum)
 *
 * Set the desired reference oscillator source for the given PLL.
 */
void set_pll_input(enum si5351_pll pll, enum si5351_pll_input input) {
  uint8_t reg_val;
  reg_val = si5351_read(SI5351_PLL_INPUT_SOURCE);

  // Clear the bits first
  //reg_val &= ~(SI5351_CLKIN_DIV_MASK);

  switch (pll) {
    case SI5351_PLLA:
      if (input == SI5351_PLL_INPUT_CLKIN) {
        reg_val |= SI5351_PLLA_SOURCE;
        reg_val |= clkin_div;
        plla_ref_osc = SI5351_PLL_INPUT_CLKIN;
      } else {
        reg_val &= ~(SI5351_PLLA_SOURCE);
        plla_ref_osc = SI5351_PLL_INPUT_XO;
      }
      break;
    case SI5351_PLLB:
      if (input == SI5351_PLL_INPUT_CLKIN) {
        reg_val |= SI5351_PLLB_SOURCE;
        reg_val |= clkin_div;
        pllb_ref_osc = SI5351_PLL_INPUT_CLKIN;
      } else {
        reg_val &= ~(SI5351_PLLB_SOURCE);
        pllb_ref_osc = SI5351_PLL_INPUT_XO;
      }
      break;
    default:
      return;
  }

  si5351_write(SI5351_PLL_INPUT_SOURCE, reg_val);

  set_pll(plla_freq, SI5351_PLLA);
  set_pll(pllb_freq, SI5351_PLLB);
}

/*
 * set_vcxo(uint64_t pll_freq, uint8_t ppm)
 *
 * pll_freq - Desired PLL base frequency in Hz * 100
 * ppm - VCXO pull limit in ppm
 *
 * Set the parameters for the VCXO on the Si5351B.
 */
void set_vcxo(uint64_t pll_freq, uint8_t ppm) {
  struct Si5351RegSet pll_reg;
  uint64_t vcxo_param;

  // Bounds check
  if (ppm < SI5351_VCXO_PULL_MIN) {
    ppm = SI5351_VCXO_PULL_MIN;
  }

  if (ppm > SI5351_VCXO_PULL_MAX) {
    ppm = SI5351_VCXO_PULL_MAX;
  }

  // Set PLLB params
  vcxo_param = pll_calc(SI5351_PLLB, pll_freq, &pll_reg, ref_correction[pllb_ref_osc], 1);

  // Derive the register values to write

  // Prepare an array for parameters to be written to
  uint8_t __p[20];
  uint8_t *params = __p;

  uint8_t i = 0;
  uint8_t temp;

  // Registers 26-27
  temp = ((pll_reg.p3 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p3 & 0xFF);
  params[i++] = temp;

  // Register 28
  temp = (uint8_t)((pll_reg.p1 >> 16) & 0x03);
  params[i++] = temp;

  // Registers 29-30
  temp = (uint8_t)((pll_reg.p1 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p1 & 0xFF);
  params[i++] = temp;

  // Register 31
  temp = (uint8_t)((pll_reg.p3 >> 12) & 0xF0);
  temp += (uint8_t)((pll_reg.p2 >> 16) & 0x0F);
  params[i++] = temp;

  // Registers 32-33
  temp = (uint8_t)((pll_reg.p2 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p2 & 0xFF);
  params[i++] = temp;

  // Write the parameters
  si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, params);

  // Write the VCXO parameters
  vcxo_param = ((vcxo_param * ppm * SI5351_VCXO_MARGIN) / 100ULL) / 1000000ULL;

  temp = (uint8_t)(vcxo_param & 0xFF);
  si5351_write(SI5351_VXCO_PARAMETERS_LOW, temp);

  temp = (uint8_t)((vcxo_param >> 8) & 0xFF);
  si5351_write(SI5351_VXCO_PARAMETERS_MID, temp);

  temp = (uint8_t)((vcxo_param >> 16) & 0x3F);
  si5351_write(SI5351_VXCO_PARAMETERS_HIGH, temp);
}

/*
 * set_ref_freq(uint32_t ref_freq, enum si5351_pll_input ref_osc)
 *
 * ref_freq - Reference oscillator frequency in Hz
 * ref_osc - Which reference oscillator frequency to set
 *    (use the si5351_pll_input enum)
 *
 * Set the reference frequency value for the desired reference oscillator
 */
void set_ref_freq(uint32_t ref_freq, enum si5351_pll_input ref_osc) {
  // uint8_t reg_val;
  //reg_val = si5351_read(SI5351_PLL_INPUT_SOURCE);

  // Clear the bits first
  //reg_val &= ~(SI5351_CLKIN_DIV_MASK);

  if (ref_freq <= 30000000UL) {
    xtal_freq[(uint8_t)ref_osc] = ref_freq;
    //reg_val |= SI5351_CLKIN_DIV_1;
    if (ref_osc == SI5351_PLL_INPUT_CLKIN) {
      clkin_div = SI5351_CLKIN_DIV_1;
    }
  } else if (ref_freq > 30000000UL && ref_freq <= 60000000UL) {
    xtal_freq[(uint8_t)ref_osc] = ref_freq / 2;
    //reg_val |= SI5351_CLKIN_DIV_2;
    if (ref_osc == SI5351_PLL_INPUT_CLKIN) {
      clkin_div = SI5351_CLKIN_DIV_2;
    }
  } else if (ref_freq > 60000000UL && ref_freq <= 100000000UL) {
    xtal_freq[(uint8_t)ref_osc] = ref_freq / 4;
    //reg_val |= SI5351_CLKIN_DIV_4;
    if (ref_osc == SI5351_PLL_INPUT_CLKIN) {
      clkin_div = SI5351_CLKIN_DIV_4;
    }
  } else {
    //reg_val |= SI5351_CLKIN_DIV_1;
  }

  //si5351_write(SI5351_PLL_INPUT_SOURCE, reg_val);
}

/*********************/
/* Private functions */
/*********************/

uint64_t pll_calc(enum si5351_pll pll, uint64_t freq, struct Si5351RegSet *reg, int32_t correction, uint8_t vcxo) {
  uint64_t ref_freq;
  if (pll == SI5351_PLLA) {
    ref_freq = xtal_freq[(uint8_t)plla_ref_osc] * SI5351_FREQ_MULT;
  } else {
    ref_freq = xtal_freq[(uint8_t)pllb_ref_osc] * SI5351_FREQ_MULT;
  }
  //ref_freq = 15974400ULL * SI5351_FREQ_MULT;
  uint32_t a, b, c, p1, p2, p3;
  uint64_t lltmp;  //, denom;

  // Factor calibration value into nominal crystal frequency
  // Measured in parts-per-billion

  ref_freq = ref_freq + (int32_t)((((((int64_t)correction) << 31) / 1000000000LL) * ref_freq) >> 31);

  // PLL bounds checking
  if (freq < SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT) {
    freq = SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT;
  }
  if (freq > SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT) {
    freq = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;
  }

  // Determine integer part of feedback equation
  a = freq / ref_freq;

  if (a < SI5351_PLL_A_MIN) {
    freq = ref_freq * SI5351_PLL_A_MIN;
  }
  if (a > SI5351_PLL_A_MAX) {
    freq = ref_freq * SI5351_PLL_A_MAX;
  }

  // Find best approximation for b/c = fVCO mod fIN
  // denom = 1000ULL * 1000ULL;
  // lltmp = freq % ref_freq;
  // lltmp *= denom;
  // do_div(lltmp, ref_freq);

  //b = (((uint64_t)(freq % ref_freq)) * RFRAC_DENOM) / ref_freq;
  if (vcxo) {
    b = (((uint64_t)(freq % ref_freq)) * 1000000ULL) / ref_freq;
    c = 1000000ULL;
  } else {
    b = (((uint64_t)(freq % ref_freq)) * RFRAC_DENOM) / ref_freq;
    c = b ? RFRAC_DENOM : 1;
  }

  // Calculate parameters
  p1 = 128 * a + ((128 * b) / c) - 512;
  p2 = 128 * b - c * ((128 * b) / c);
  p3 = c;

  // Recalculate frequency as fIN * (a + b/c)
  lltmp = ref_freq;
  lltmp *= b;
  do_div(lltmp, c);
  freq = lltmp;
  freq += ref_freq * a;

  reg->p1 = p1;
  reg->p2 = p2;
  reg->p3 = p3;

  if (vcxo) {
    return (uint64_t)(128 * a * 1000000ULL + b);
  } else {
    return freq;
  }
}

uint64_t multisynth_calc(uint64_t freq, uint64_t pll_freq, struct Si5351RegSet *reg) {
  uint64_t lltmp;
  uint32_t a, b, c, p1, p2, p3;
  uint8_t divby4 = 0;
  uint8_t ret_val = 0;

  // Multisynth bounds checking
  if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT) {
    freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
  }
  if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT) {
    freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;
  }

  if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT) {
    divby4 = 1;
  }

  if (pll_freq == 0) {
    // Find largest integer divider for max
    // VCO frequency and given target frequency
    if (divby4 == 0) {
      lltmp = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;  // margin needed?
      do_div(lltmp, freq);
      if (lltmp == 5) {
        lltmp = 4;
      } else if (lltmp == 7) {
        lltmp = 6;
      }
      a = (uint32_t)lltmp;
    } else {
      a = 4;
    }

    b = 0;
    c = 1;
    pll_freq = a * freq;
  } else {
    // Preset PLL, so return the actual freq for these params instead of PLL freq
    ret_val = 1;

    // Determine integer part of feedback equation
    a = pll_freq / freq;

    if (a < SI5351_MULTISYNTH_A_MIN) {
      freq = pll_freq / SI5351_MULTISYNTH_A_MIN;
    }
    if (a > SI5351_MULTISYNTH_A_MAX) {
      freq = pll_freq / SI5351_MULTISYNTH_A_MAX;
    }

    b = (pll_freq % freq * RFRAC_DENOM) / freq;
    c = b ? RFRAC_DENOM : 1;
  }

  // Calculate parameters
  if (divby4 == 1) {
    p3 = 1;
    p2 = 0;
    p1 = 0;
  } else {
    p1 = 128 * a + ((128 * b) / c) - 512;
    p2 = 128 * b - c * ((128 * b) / c);
    p3 = c;
  }

  reg->p1 = p1;
  reg->p2 = p2;
  reg->p3 = p3;

  if (ret_val == 0) {
    return pll_freq;
  } else {
    return freq;
  }
}

uint64_t multisynth67_calc(uint64_t freq, uint64_t pll_freq, struct Si5351RegSet *reg) {
  //uint8_t p1;
  // uint8_t ret_val = 0;
  uint32_t a;
  uint64_t lltmp;

  // Multisynth bounds checking
  if (freq > SI5351_MULTISYNTH67_MAX_FREQ * SI5351_FREQ_MULT) {
    freq = SI5351_MULTISYNTH67_MAX_FREQ * SI5351_FREQ_MULT;
  }
  if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT) {
    freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;
  }

  if (pll_freq == 0) {
    // Find largest integer divider for max
    // VCO frequency and given target frequency
    lltmp = (SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT) - 100000000UL;  // margin needed?
    do_div(lltmp, freq);
    a = (uint32_t)lltmp;

    // Divisor has to be even
    if (a % 2 != 0) {
      a++;
    }

    // Divisor bounds check
    if (a < SI5351_MULTISYNTH_A_MIN) {
      a = SI5351_MULTISYNTH_A_MIN;
    }
    if (a > SI5351_MULTISYNTH67_A_MAX) {
      a = SI5351_MULTISYNTH67_A_MAX;
    }

    pll_freq = a * freq;

    // PLL bounds checking
    if (pll_freq > (SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT)) {
      a -= 2;
      pll_freq = a * freq;
    } else if (pll_freq < (SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT)) {
      a += 2;
      pll_freq = a * freq;
    }

    reg->p1 = (uint8_t)a;
    reg->p2 = 0;
    reg->p3 = 0;
    return pll_freq;
  } else {
    // Multisynth frequency must be integer division of PLL
    if (pll_freq % freq) {
      // No good
      return 0;
    } else {
      a = pll_freq / freq;

      // Division ratio bounds check
      if (a < SI5351_MULTISYNTH_A_MIN || a > SI5351_MULTISYNTH67_A_MAX) {
        // No bueno
        return 0;
      } else {
        reg->p1 = (uint8_t)a;
        reg->p2 = 0;
        reg->p3 = 0;
        return 1;
      }
    }
  }
}

void update_sys_status(struct Si5351Status *status) {
  uint8_t reg_val = 0;

  reg_val = si5351_read(SI5351_DEVICE_STATUS);

  // Parse the register
  status->SYS_INIT = (reg_val >> 7) & 0x01;
  status->LOL_B = (reg_val >> 6) & 0x01;
  status->LOL_A = (reg_val >> 5) & 0x01;
  status->LOS = (reg_val >> 4) & 0x01;
  status->REVID = reg_val & 0x03;
}

void update_int_status(struct Si5351IntStatus *int_status) {
  uint8_t reg_val = 0;

  reg_val = si5351_read(SI5351_INTERRUPT_STATUS);

  // Parse the register
  int_status->SYS_INIT_STKY = (reg_val >> 7) & 0x01;
  int_status->LOL_B_STKY = (reg_val >> 6) & 0x01;
  int_status->LOL_A_STKY = (reg_val >> 5) & 0x01;
  int_status->LOS_STKY = (reg_val >> 4) & 0x01;
}

void ms_div(enum si5351_clock clk, uint8_t r_div, uint8_t div_by_4) {
  uint8_t reg_val = 0;
  uint8_t reg_addr = 0;

  switch (clk) {
    case SI5351_CLK0:
      reg_addr = SI5351_CLK0_PARAMETERS + 2;
      break;
    case SI5351_CLK1:
      reg_addr = SI5351_CLK1_PARAMETERS + 2;
      break;
    case SI5351_CLK2:
      reg_addr = SI5351_CLK2_PARAMETERS + 2;
      break;
    case SI5351_CLK3:
      reg_addr = SI5351_CLK3_PARAMETERS + 2;
      break;
    case SI5351_CLK4:
      reg_addr = SI5351_CLK4_PARAMETERS + 2;
      break;
    case SI5351_CLK5:
      reg_addr = SI5351_CLK5_PARAMETERS + 2;
      break;
    case SI5351_CLK6:
      reg_addr = SI5351_CLK6_7_OUTPUT_DIVIDER;
      break;
    case SI5351_CLK7:
      reg_addr = SI5351_CLK6_7_OUTPUT_DIVIDER;
      break;
  }

  reg_val = si5351_read(reg_addr);

  if (clk <= (uint8_t)SI5351_CLK5) {
    // Clear the relevant bits
    reg_val &= ~(0x7c);

    if (div_by_4 == 0) {
      reg_val &= ~(SI5351_OUTPUT_CLK_DIVBY4);
    } else {
      reg_val |= (SI5351_OUTPUT_CLK_DIVBY4);
    }

    reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);
  } else if (clk == SI5351_CLK6) {
    // Clear the relevant bits
    reg_val &= ~(0x07);

    reg_val |= r_div;
  } else if (clk == SI5351_CLK7) {
    // Clear the relevant bits
    reg_val &= ~(0x70);

    reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);
  }

  si5351_write(reg_addr, reg_val);
}

uint8_t select_r_div(uint64_t *freq) {
  uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;

  // Choose the correct R divider
  if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2)) {
    r_div = SI5351_OUTPUT_CLK_DIV_128;
    *freq *= 128ULL;
  } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4)) {
    r_div = SI5351_OUTPUT_CLK_DIV_64;
    *freq *= 64ULL;
  } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8)) {
    r_div = SI5351_OUTPUT_CLK_DIV_32;
    *freq *= 32ULL;
  } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16)) {
    r_div = SI5351_OUTPUT_CLK_DIV_16;
    *freq *= 16ULL;
  } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32)) {
    r_div = SI5351_OUTPUT_CLK_DIV_8;
    *freq *= 8ULL;
  } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64)) {
    r_div = SI5351_OUTPUT_CLK_DIV_4;
    *freq *= 4ULL;
  } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128)) {
    r_div = SI5351_OUTPUT_CLK_DIV_2;
    *freq *= 2ULL;
  }

  return r_div;
}

uint8_t select_r_div_ms67(uint64_t *freq) {
  uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;

  // Choose the correct R divider
  if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 2)) {
    r_div = SI5351_OUTPUT_CLK_DIV_128;
    *freq *= 128ULL;
  } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 2) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 4)) {
    r_div = SI5351_OUTPUT_CLK_DIV_64;
    *freq *= 64ULL;
  } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 4) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 8)) {
    r_div = SI5351_OUTPUT_CLK_DIV_32;
    *freq *= 32ULL;
  } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 8) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 16)) {
    r_div = SI5351_OUTPUT_CLK_DIV_16;
    *freq *= 16ULL;
  } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 16) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 32)) {
    r_div = SI5351_OUTPUT_CLK_DIV_8;
    *freq *= 8ULL;
  } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 32) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 64)) {
    r_div = SI5351_OUTPUT_CLK_DIV_4;
    *freq *= 4ULL;
  } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 64) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 128)) {
    r_div = SI5351_OUTPUT_CLK_DIV_2;
    *freq *= 2ULL;
  }

  return r_div;
}

// I2C register access for ESP-IDF.

uint8_t si5351_write_bulk(uint8_t regAddr, uint8_t length, uint8_t *data) {
  uint8_t msg[16];
  if (length + 1 > sizeof(msg)) return 1;

  // Append register address to front of data packet
  msg[0] = regAddr;
  for (int i = 0; i < length; i++) {
    msg[i + 1] = data[i];
  }

  return polar_i2c_write(msg, length + 1) == ESP_OK ? 0 : 1;
}

uint8_t si5351_write(uint8_t regAddr, uint8_t data) {
  si5351_write_bulk(regAddr, 1, &data);

  return 0;
}

uint8_t si5351_read(uint8_t regAddr) {
  uint8_t buf = 0;
  (void)polar_i2c_write_read(&regAddr, 1, &buf, 1);
  return buf;
}

bool si5351_set_i2c_speed(uint32_t speed_hz) {
  return polar_i2c_set_speed(speed_hz) == ESP_OK;
}
