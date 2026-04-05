#include <complex>
#include <math.h>
#include <stdio.h>

#include "../src/polar_mod.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct SidebandResult {
  float wanted_db;
  float unwanted_db;
  float suppression_db;
};

static SidebandResult measure(const std::complex<double> &wanted,
                              const std::complex<double> &unwanted) {
  double wanted_mag = std::abs(wanted);
  double unwanted_mag = std::abs(unwanted);
  float wanted_db = 20.0f * log10f(static_cast<float>(wanted_mag) + 1e-12f);
  float unwanted_db =
      20.0f * log10f(static_cast<float>(unwanted_mag) + 1e-12f);
  return {wanted_db, unwanted_db, wanted_db - unwanted_db};
}

int main() {
  constexpr int fs = 16000;
  constexpr int warmup_samples = 2048;
  constexpr int measure_samples = fs;  // One second: every test tone is coherent.

  printf("Tone,Polar wanted,Polar unwanted,Polar suppression,"
         "Phase-only wanted,Phase-only unwanted,Phase-only suppression\n");

  for (int tone_hz = 300; tone_hz <= 3000; tone_hz += 300) {
    polar_mod_reset();
    modulation_mode = MOD_USB;
    double phase = 0.0;
    std::complex<double> polar_wanted(0.0, 0.0);
    std::complex<double> polar_unwanted(0.0, 0.0);
    std::complex<double> phase_wanted(0.0, 0.0);
    std::complex<double> phase_unwanted(0.0, 0.0);

    for (int i = 0; i < warmup_samples + measure_samples; ++i) {
      float input =
          0.5f * sinf(2.0f * static_cast<float>(M_PI) * tone_hz * i / fs);
      float amplitude;
      float phase_difference;
      modulation_am_pm_f(input, &amplitude, &phase_difference);
      phase += phase_difference;

      if (i < warmup_samples) continue;
      int n = i - warmup_samples;
      double analysis_phase = 2.0 * M_PI * tone_hz * n / fs;
      std::complex<double> positive_kernel =
          std::polar(1.0, -analysis_phase);
      std::complex<double> negative_kernel =
          std::polar(1.0, analysis_phase);
      std::complex<double> unit_signal = std::polar(1.0, phase);
      std::complex<double> polar_signal =
          static_cast<double>(amplitude) * unit_signal;

      polar_wanted += polar_signal * positive_kernel;
      polar_unwanted += polar_signal * negative_kernel;
      phase_wanted += unit_signal * positive_kernel;
      phase_unwanted += unit_signal * negative_kernel;
    }

    SidebandResult polar = measure(polar_wanted, polar_unwanted);
    SidebandResult phase_only = measure(phase_wanted, phase_unwanted);
    printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", tone_hz,
           polar.wanted_db, polar.unwanted_db, polar.suppression_db,
           phase_only.wanted_db, phase_only.unwanted_db,
           phase_only.suppression_db);
  }
}
