#include <stdio.h>
#include "esp_cpu.h"
#include "audio_data.h"
#include "tx_dsp.h"
static void run(uint32_t rate) {
  tx_dsp_set_sample_rate(rate);
  const int N = 4000;
  uint32_t maxc = 0; uint64_t tot = 0; float a, p, acc = 0;
  for (int n = 0; n < N; ++n) {
    float x = wav_data[(n * 7 + 20000) % wav_data_len] / 32768.0f;
    uint32_t t0 = esp_cpu_get_cycle_count();
    tx_dsp_process(x, 1, &a, &p);
    uint32_t c = esp_cpu_get_cycle_count() - t0;
    acc += p; tot += c; if (n > 200 && c > maxc) maxc = c;
  }
  printf("RATE %lu: avg %llu max %lu cycles/sample (acc %f)\n", (unsigned long)rate, tot / N, (unsigned long)maxc, acc);
}
extern "C" void app_main(void) {
  // Calibrate the QEMU cycle counter against a known instruction count.
  uint32_t t0 = esp_cpu_get_cycle_count();
  for (int i = 0; i < 10000; ++i)
    asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop");
  printf("CAL: %lu cycles for ~120000 insns\n",
         (unsigned long)(esp_cpu_get_cycle_count() - t0)); run(16000); run(32000); run(16000); printf("BENCH DONE\n"); }
