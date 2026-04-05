// Simple AD9850 carrier test at 14.200 MHz
// Hardware SPI, continuous transmission

#include <stdio.h>
#include "pico/stdlib.h"
#include "ad9850.h"
#include "ddx_common.h"

#define TEST_FREQ 14200000UL

int main() {
  stdio_init_all();
  sleep_ms(3000);

  printf("\n=== AD9850 Carrier Test ===\n");
  printf("Freq: %lu Hz (%.3f MHz)\n", TEST_FREQ, (float)TEST_FREQ / 1000000.0f);
  printf("SPI: spi0 SCK=GP2 MOSI=GP3 CS=GP1 RESET=GP8 FQUD=GP1\n\n");

  ad9850_init(NULL, AD9850_CS, AD9850_RESET, AD9850_FQ_UD, 0);
  ad9850_reset();
  ad9850_power_up();
  ad9850_set_frequency(TEST_FREQ);

  printf("\n=== Transmitting %lu Hz ===\n\n",
         (unsigned long)ad9850_get_frequency());

  while (true) {
    sleep_ms(5000);
    printf("[STATUS] Freq: %lu Hz\n", (unsigned long)ad9850_get_frequency());
    tight_loop_contents();
  }

  return 0;
}
