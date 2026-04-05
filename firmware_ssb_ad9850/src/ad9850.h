/*
 * ad9850.h - AD9850 DDS library for SSB generation using polar modulation
 *
 * Adapted from AD985X library by Rob Tillaart for Pico SDK
 * Modified for SSB polar modulation application
 */

#ifndef __AD9850_H
#define __AD9850_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

// AD9850 configuration
#define AD9850_CLOCK_FREQ 125000000UL  // 125 MHz reference clock
#define AD9850_MAX_FREQ 40000000UL     // 40 MHz max output frequency
#define AD9850_MIN_FREQ 1000UL         // 1 kHz min output frequency

  // SSB phase modulation parameters
  extern uint64_t ad9850_base_freq;
  extern float ad9850_phase_to_freq_factor;
  extern float ssb_phase_to_b_factor;  // Legacy compatibility for polar modulation
  extern uint32_t ad9850_freq_resolution;

  // AD9850 initialization
  bool ad9850_init(spi_inst_t *spi, uint8_t cs_pin, uint8_t reset_pin, uint8_t fq_ud_pin, uint32_t spi_baud);
  void ad9850_reset(void);
  void ad9850_power_down(void);
  void ad9850_power_up(void);

  // Frequency control
  bool ad9850_set_frequency(uint32_t freq_hz);
  uint32_t ad9850_get_frequency(void);

  // SSB-specific functions for polar modulation
  void ad9850_setup_ssb(uint64_t freq_hz);
  void ad9850_write_phase_fast(float freq_dev_hz);  // convenience: latch prev + start new
  void ad9850_dma_init(void);
  void ad9850_dma_start(float freq_dev_hz);  // non-blocking: prepare buffer + kick DMA
  void ad9850_dma_latch(void);               // wait for DMA + SPI done, pulse FQ_UD
  void ad9850_dma_wait(void);                // alias for dma_latch (flush at TX end)

  // Output control
  void ad9850_output_enable(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* __AD9850_H */
