/*
 * ad9850.cpp - AD9850 DDS driver for Pico using HARDWARE SPI + DMA
 *
 * AD9850 requires LSB-first SPI data. Pico hardware SPI only supports MSB-first,
 * so we bit-reverse each byte using a 256-byte lookup table.
 *
 * DMA SPI: The SSB hot path (ad9850_dma_start) uses DMA for non-blocking SPI
 * transfers. This frees ~4µs per sample (13% of the 31.25µs budget at 32kHz)
 * for DSP computation. The caller uses ad9850_dma_latch() to pulse FQ_UD
 * once the transfer completes.
 *
 * Tuning word formula: FTW = (147573952589ULL * freq) >> 32
 * This is: freq * 2^32 / 125000000 (matching the working Arduino demo)
 */

#include "ad9850.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "ddx_common.h"

/*
 * Bit-reverse lookup table (256 entries)
 * Replaces the per-byte shift-and-OR computation with a single array lookup.
 * Called 5 times per sample at 32kHz = 160,000 lookups/sec.
 */
static const uint8_t rev8_lut[256] = {
  0x00,0x80,0x40,0xC0,0x20,0xA0,0x60,0xE0,0x10,0x90,0x50,0xD0,0x30,0xB0,0x70,0xF0,
  0x08,0x88,0x48,0xC8,0x28,0xA8,0x68,0xE8,0x18,0x98,0x58,0xD8,0x38,0xB8,0x78,0xF8,
  0x04,0x84,0x44,0xC4,0x24,0xA4,0x64,0xE4,0x14,0x94,0x54,0xD4,0x34,0xB4,0x74,0xF4,
  0x0C,0x8C,0x4C,0xCC,0x2C,0xAC,0x6C,0xEC,0x1C,0x9C,0x5C,0xDC,0x3C,0xBC,0x7C,0xFC,
  0x02,0x82,0x42,0xC2,0x22,0xA2,0x62,0xE2,0x12,0x92,0x52,0xD2,0x32,0xB2,0x72,0xF2,
  0x0A,0x8A,0x4A,0xCA,0x2A,0xAA,0x6A,0xEA,0x1A,0x9A,0x5A,0xDA,0x3A,0xBA,0x7A,0xFA,
  0x06,0x86,0x46,0xC6,0x26,0xA6,0x66,0xE6,0x16,0x96,0x56,0xD6,0x36,0xB6,0x76,0xF6,
  0x0E,0x8E,0x4E,0xCE,0x2E,0xAE,0x6E,0xEE,0x1E,0x9E,0x5E,0xDE,0x3E,0xBE,0x7E,0xFE,
  0x01,0x81,0x41,0xC1,0x21,0xA1,0x61,0xE1,0x11,0x91,0x51,0xD1,0x31,0xB1,0x71,0xF1,
  0x09,0x89,0x49,0xC9,0x29,0xA9,0x69,0xE9,0x19,0x99,0x59,0xD9,0x39,0xB9,0x79,0xF9,
  0x05,0x85,0x45,0xC5,0x25,0xA5,0x65,0xE5,0x15,0x95,0x55,0xD5,0x35,0xB5,0x75,0xF5,
  0x0D,0x8D,0x4D,0xCD,0x2D,0xAD,0x6D,0xED,0x1D,0x9D,0x5D,0xDD,0x3D,0xBD,0x7D,0xFD,
  0x03,0x83,0x43,0xC3,0x23,0xA3,0x63,0xE3,0x13,0x93,0x53,0xD3,0x33,0xB3,0x73,0xF3,
  0x0B,0x8B,0x4B,0xCB,0x2B,0xAB,0x6B,0xEB,0x1B,0x9B,0x5B,0xDB,0x3B,0xBB,0x7B,0xFB,
  0x07,0x87,0x47,0xC7,0x27,0xA7,0x67,0xE7,0x17,0x97,0x57,0xD7,0x37,0xB7,0x77,0xF7,
  0x0F,0x8F,0x4F,0xCF,0x2F,0xAF,0x6F,0xEF,0x1F,0x9F,0x5F,0xDF,0x3F,0xBF,0x7F,0xFF
};

/*
 * AD9850 state
 */
static uint32_t ad9850_current_freq = 0;
static uint32_t ad9850_tuning_word = 0;
static int32_t ad9850_offset = 0;
static bool ad9850_auto_update = true;

// SSB phase modulation parameters
uint64_t ad9850_base_freq = 0;
float ad9850_phase_to_freq_factor = 0;
float ssb_phase_to_b_factor = 0;
uint32_t ad9850_freq_resolution = 1;

// Cached base tuning word for fast SSB frequency deviation updates
static uint32_t base_tuning_word = 0;

// DMA state
static int ad9850_dma_chan = -1;
static volatile bool dma_active = false;
static uint8_t dma_spi_buf[5];  // DMA source buffer (must persist during transfer)

/*
 * Wait for any in-flight DMA + SPI to complete, then latch FQ_UD.
 * Called before blocking SPI operations and at sample boundaries.
 * After DSP computation (~15-20µs), the 4µs DMA is already done,
 * so this typically returns in <0.1µs.
 */
void __not_in_flash_func(ad9850_dma_latch)(void) {
  if (!dma_active) return;
  dma_channel_wait_for_finish_blocking(ad9850_dma_chan);
  while (spi_is_busy(AD9850_SPI_PORT)) tight_loop_contents();
  // CS LOW (end SPI select)
  gpio_put(AD9850_CS, 0);
  // Pulse FQ_UD to latch the new frequency word
  gpio_put(AD9850_FQ_UD, 1);
  gpio_put(AD9850_FQ_UD, 0);
  dma_active = false;
}

/*
 * Write tuning word + control byte to AD9850 via BLOCKING hardware SPI.
 * Used for setup/teardown operations (init, reset, set_frequency, power control).
 * The SSB hot path uses DMA instead (ad9850_dma_start).
 */
static void ad9850_write_data(uint32_t tuning_word, uint8_t config) {
  // Flush any in-flight DMA before using blocking SPI
  ad9850_dma_latch();

  // Build 5 bytes, bit-reversed via LUT for MSB-first hardware SPI
  uint8_t data[5];
  data[0] = rev8_lut[(uint8_t)(tuning_word & 0xFF)];
  data[1] = rev8_lut[(uint8_t)((tuning_word >> 8) & 0xFF)];
  data[2] = rev8_lut[(uint8_t)((tuning_word >> 16) & 0xFF)];
  data[3] = rev8_lut[(uint8_t)((tuning_word >> 24) & 0xFF)];
  data[4] = rev8_lut[config & 0xFC];

  // CS HIGH to select
  gpio_put(AD9850_CS, 1);

  // Send all 5 bytes via blocking hardware SPI
  spi_write_blocking(AD9850_SPI_PORT, data, 5);

  // CS LOW
  gpio_put(AD9850_CS, 0);

  // Pulse FQ_UD to latch new frequency
  if (ad9850_auto_update) {
    gpio_put(AD9850_FQ_UD, 1);
    gpio_put(AD9850_FQ_UD, 0);
  }
}

/*
 * Initialize AD9850 with hardware SPI
 */
bool ad9850_init(spi_inst_t *spi, uint8_t cs_pin, uint8_t reset_pin, uint8_t fq_ud_pin, uint32_t spi_baud) {
  (void)spi;
  (void)spi_baud;

  printf("[AD9850] Init: CS=%d RESET=%d FQ_UD=%d SCK=%d MOSI=%d\n",
         cs_pin, reset_pin, fq_ud_pin, AD9850_SPI_SCK, AD9850_SPI_MOSI);

  gpio_init(cs_pin);
  gpio_set_dir(cs_pin, GPIO_OUT);
  gpio_put(cs_pin, 0);

  gpio_init(reset_pin);
  gpio_set_dir(reset_pin, GPIO_OUT);
  gpio_put(reset_pin, 0);

  gpio_init(fq_ud_pin);
  gpio_set_dir(fq_ud_pin, GPIO_OUT);
  gpio_put(fq_ud_pin, 0);

  spi_init(AD9850_SPI_PORT, AD9850_SPI_BAUD);
  gpio_set_function(AD9850_SPI_SCK, GPIO_FUNC_SPI);
  gpio_set_function(AD9850_SPI_MOSI, GPIO_FUNC_SPI);

  ad9850_current_freq = 0;
  ad9850_tuning_word = 0;
  ad9850_auto_update = true;

  printf("[AD9850] HW SPI init: %lu MHz on spi0\n", AD9850_SPI_BAUD / 1000000);
  return true;
}

/*
 * Reset AD9850
 */
void ad9850_reset(void) {
  printf("[AD9850] Reset...\n");

  gpio_put(AD9850_CS, 1);

  gpio_put(AD9850_RESET, 1);
  uint8_t zero = 0x00;
  spi_write_blocking(AD9850_SPI_PORT, &zero, 1);
  gpio_put(AD9850_RESET, 0);

  gpio_put(AD9850_CS, 0);

  ad9850_current_freq = 0;
  ad9850_tuning_word = 0;
  ad9850_offset = 0;
  ad9850_auto_update = true;

  ad9850_write_data(0, 0);

  printf("[AD9850] Reset done\n");
}

void ad9850_power_down(void) {
  ad9850_write_data(ad9850_tuning_word, 0x04);
}

void ad9850_power_up(void) {
  ad9850_write_data(ad9850_tuning_word, 0);
}

/*
 * Set frequency (blocking) using the exact formula from the working Arduino demo
 */
bool ad9850_set_frequency(uint32_t freq_hz) {
  if (freq_hz > AD9850_MAX_FREQ) {
    freq_hz = AD9850_MAX_FREQ;
  }

  ad9850_tuning_word = (uint32_t)((147573952589ULL * freq_hz) >> 32);
  ad9850_current_freq = freq_hz;
  ad9850_tuning_word += ad9850_offset;

  printf("[AD9850] Set %lu Hz -> 0x%08lX\n",
         (unsigned long)ad9850_current_freq, (unsigned long)ad9850_tuning_word);

  ad9850_write_data(ad9850_tuning_word, 0);

  return true;
}

uint32_t ad9850_get_frequency(void) {
  return ad9850_current_freq;
}

/*
 * Setup for SSB polar modulation
 */
void ad9850_setup_ssb(uint64_t freq_hz) {
  if (freq_hz > AD9850_MAX_FREQ) freq_hz = AD9850_MAX_FREQ;
  if (freq_hz < AD9850_MIN_FREQ) freq_hz = AD9850_MIN_FREQ;

  ad9850_base_freq = freq_hz;
  base_tuning_word = (uint32_t)((147573952589ULL * (uint32_t)freq_hz) >> 32);
  ad9850_set_frequency((uint32_t)freq_hz);

  printf("[AD9850] SSB: %llu Hz (base FTW=0x%08lX) ready\n",
         freq_hz, (unsigned long)base_tuning_word);
}

/*
 * Initialize DMA channel for non-blocking SPI transfers.
 * Configures a DMA channel to write 5 bytes to the SPI TX data register,
 * paced by the SPI TX DREQ so bytes are sent at the SPI clock rate.
 */
void ad9850_dma_init(void) {
  ad9850_dma_chan = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(ad9850_dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_dreq(&c, spi_get_dreq(AD9850_SPI_PORT, true));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);

  dma_channel_configure(
    ad9850_dma_chan, &c,
    &spi_get_hw(AD9850_SPI_PORT)->dr,  // dest: SPI TX data register
    dma_spi_buf,                         // src: our buffer (updated per transfer)
    5,                                   // 5 bytes per AD9850 update
    false                                // don't start yet
  );

  dma_active = false;
  printf("[AD9850] DMA chan %d configured for SPI\n", ad9850_dma_chan);
}

/*
 * Start a non-blocking DMA SPI transfer to the AD9850.
 * Prepares the 5-byte SPI buffer with the bit-reversed tuning word,
 * sets CS HIGH, and kicks off the DMA. Returns immediately.
 *
 * The caller MUST call ad9850_dma_latch() before the next ad9850_dma_start()
 * to wait for completion and pulse FQ_UD to latch the frequency.
 *
 * freq_dev_hz: frequency deviation from carrier in Hz (float precision,
 * preserving the AD9850's full 0.029 Hz resolution).
 */
void __not_in_flash_func(ad9850_dma_start)(float freq_dev_hz) {
  // Compute tuning word with full fractional precision
  // 34.359738368f = 2^32 / 125000000 (Hz-to-FTW conversion factor)
  int32_t dev_ftw = (int32_t)lroundf(freq_dev_hz * 34.359738368f);
  uint32_t ftw = base_tuning_word + (uint32_t)dev_ftw;

  // Prepare SPI buffer (bit-reversed via LUT)
  dma_spi_buf[0] = rev8_lut[ftw & 0xFF];
  dma_spi_buf[1] = rev8_lut[(ftw >> 8) & 0xFF];
  dma_spi_buf[2] = rev8_lut[(ftw >> 16) & 0xFF];
  dma_spi_buf[3] = rev8_lut[(ftw >> 24) & 0xFF];
  dma_spi_buf[4] = 0;  // config byte: power on, no phase offset

  // CS HIGH to select AD9850
  gpio_put(AD9850_CS, 1);

  // Start DMA transfer (non-blocking)
  dma_channel_transfer_from_buffer_now(ad9850_dma_chan, dma_spi_buf, 5);
  dma_active = true;
}

/*
 * Convenience function: latch previous + start new (blocking-equivalent API).
 * Used by non-DMA-aware callers and the ramp-up path.
 */
void __not_in_flash_func(ad9850_write_phase_fast)(float freq_dev_hz) {
  ad9850_dma_latch();
  ad9850_dma_start(freq_dev_hz);
}

/*
 * Wait for current DMA transfer to complete and latch the frequency.
 * Call at end of TX to flush the last pending transfer.
 */
void ad9850_dma_wait(void) {
  ad9850_dma_latch();
}

void ad9850_output_enable(bool enable) {
  if (enable) {
    ad9850_power_up();
  } else {
    ad9850_power_down();
  }
}
