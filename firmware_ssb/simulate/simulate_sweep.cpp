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

  kiss_fft_cfg cfg = kiss_fft_alloc(nfft, 0, NULL, NULL);

  printf("SweepFreq(Hz), Wanted(dB), Unwanted(dB), Suppression(dB)\n");

  for (int f_test = 300; f_test <= 3000; f_test += 300) {
    std::vector<kiss_fft_cpx> cin(nfft);
    std::vector<kiss_fft_cpx> cout(nfft);
    double phase_accu_rad = 0;
    modulation_mode = MOD_USB;

    for (int i = 0; i < nfft; i++) {
      float t = (float)i / fs;
      float f_data = 0.5f * sinf(2.0f * M_PI * f_test * t);

      float f_ampl, f_phase_diff;
      modulation_am_pm_f(f_data, &f_ampl, &f_phase_diff);

      phase_accu_rad += f_phase_diff;
      cin[i].r = (float)(f_ampl * cos(phase_accu_rad));
      cin[i].i = (float)(f_ampl * sin(phase_accu_rad));
    }

    // Window
    for (int i = 0; i < nfft; i++) {
      float win = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (nfft - 1)));
      cin[i].r *= win;
      cin[i].i *= win;
    }

    kiss_fft(cfg, cin.data(), cout.data());

    // Find peak at +f_test and -f_test
    float wanted_mag = 0;
    float unwanted_mag = 0;

    for (int i = 0; i < nfft; i++) {
      float freq = (float)i * fs / nfft;
      if (freq > fs / 2) freq -= fs;
      float mag = sqrtf(cout[i].r * cout[i].r + cout[i].i * cout[i].i);

      if (fabs(freq - f_test) < (fs / nfft)) {
        if (mag > wanted_mag) wanted_mag = mag;
      }
      if (fabs(freq + f_test) < (fs / nfft)) {
        if (mag > unwanted_mag) unwanted_mag = mag;
      }
    }

    float wanted_db = 20.0f * log10f(wanted_mag + 1e-6f);
    float unwanted_db = 20.0f * log10f(unwanted_mag + 1e-6f);
    printf("%d, %.2f, %.2f, %.2f\n", f_test, wanted_db, unwanted_db, wanted_db - unwanted_db);
  }

  free(cfg);
  return 0;
}
