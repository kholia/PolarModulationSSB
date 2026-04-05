#include <stdio.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include "../src/polar_mod.h"
#include "../ft8_lib/fft/kiss_fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
  int nfft = 1024;
  int fs = 16000;

  std::vector<int16_t> input(nfft);
  for (int i = 0; i < nfft; i++) {
    float t = (float)i / fs;
    // Two tones: 700Hz and 1900Hz
    float s = 0.5f * sinf(2.0f * M_PI * 700.0f * t) + 0.5f * sinf(2.0f * M_PI * 1900.0f * t);
    input[i] = (int16_t)(s * 16384.0f);
  }

  kiss_fft_cfg cfg = kiss_fft_alloc(nfft, 0, NULL, NULL);
  std::vector<kiss_fft_cpx> cin(nfft);
  std::vector<kiss_fft_cpx> cout(nfft);

  double phase_accu_rad = 0;

  modulation_mode = MOD_USB;

  for (int i = 0; i < nfft; i++) {
    int ampl, phase_diff;
    // Use the float version for simulation
    float f_ampl, f_phase_diff;
    float f_data = (float)input[i] / 32768.0f;
    modulation_am_pm_f(f_data, &f_ampl, &f_phase_diff);

    // f_phase_diff is in radians
    phase_accu_rad += f_phase_diff;

    cin[i].r = (float)(f_ampl * cos(phase_accu_rad));
    cin[i].i = (float)(f_ampl * sin(phase_accu_rad));
  }

  // Apply Hann window
  for (int i = 0; i < nfft; i++) {
    float win = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (nfft - 1)));
    cin[i].r *= win;
    cin[i].i *= win;
  }

  kiss_fft(cfg, cin.data(), cout.data());

  printf("Freq(Hz), Magnitude(dB)\n");
  for (int i = 0; i < nfft; i++) {
    int idx = (i + nfft / 2) % nfft;
    float freq = (float)(idx)*fs / nfft;
    if (freq >= fs / 2) freq -= fs;

    float mag = sqrtf(cout[idx].r * cout[idx].r + cout[idx].i * cout[idx].i);
    float db = 20.0f * log10f(mag + 1e-6f);
    printf("%.1f, %.2f\n", freq, db);
  }

  free(cfg);
  return 0;
}
