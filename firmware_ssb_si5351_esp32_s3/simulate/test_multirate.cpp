#include <complex>
#include <math.h>
#include <stdio.h>

#include "polar_mod.h"
#include "tx_dsp.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float sideband_suppression(uint32_t sample_rate, uint32_t tone_hz) {
  tx_dsp_set_sample_rate(sample_rate);
  tx_dsp_reset();
  modulation_mode = MOD_USB;

  constexpr uint32_t warmup = 4096;
  uint32_t measure_samples = sample_rate;
  double phase = 0.0;
  std::complex<double> wanted(0.0, 0.0);
  std::complex<double> unwanted(0.0, 0.0);

  for (uint32_t n = 0; n < warmup + measure_samples; ++n) {
    float input = 0.5f * sinf(2.0f * static_cast<float>(M_PI) * tone_hz * n /
                              sample_rate);
    float amplitude;
    float phase_step;
    tx_dsp_process(input, 0, &amplitude, &phase_step);
    phase += phase_step;
    if (n < warmup) continue;

    uint32_t m = n - warmup;
    double test_phase = 2.0 * M_PI * tone_hz * m / sample_rate;
    std::complex<double> signal = std::polar(static_cast<double>(amplitude), phase);
    wanted += signal * std::polar(1.0, -test_phase);
    unwanted += signal * std::polar(1.0, test_phase);
  }

  return 20.0f * log10f(static_cast<float>(
    std::abs(wanted) / (std::abs(unwanted) + 1e-12)));
}

static float envelope_peak(uint32_t sample_rate) {
  tx_dsp_set_sample_rate(sample_rate);
  tx_dsp_reset();
  float peak = 0.0f;
  for (uint32_t n = 0; n < sample_rate; ++n) {
    float input = 2.0f * sinf(2.0f * static_cast<float>(M_PI) * 1000.0f * n /
                              sample_rate);
    float amplitude;
    float phase_step;
    tx_dsp_process(input, 0, &amplitude, &phase_step);
    if (n > 4096 && amplitude > peak) peak = amplitude;
  }
  return peak;
}

int main() {
  bool passed = true;
  for (uint32_t rate : { 16000U, 32000U }) {
    for (uint32_t tone : { 300U, 900U, 1500U, 2400U, 2700U }) {
      float suppression = sideband_suppression(rate, tone);
      printf("%5u Hz, %4u Hz tone: %6.2f dB opposite-sideband suppression\n",
             rate, tone, suppression);
      if (suppression < 45.0f) passed = false;
    }
    float peak = envelope_peak(rate);
    printf("%5u Hz, 2.0-peak input: %.5f envelope peak\n", rate, peak);
    if (peak > pm_env_ceiling + 0.002f) passed = false;
  }
  return passed ? 0 : 1;
}
