// Build a dump of per-sample (amplitude, phase step) from the firmware DSP.
// Usage: dump_dsp_stream RATE wav|two SECONDS [TONE_GAIN]; env PM_LIMITER/PM_STEP/PM_KNEE/PM_RATIO override.
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "audio_data.h"
#include "tx_dsp.h"
#include "speech_dsp.h"
#include "polar_mod.h"
// argv: rate mode(wav|two) seconds
int main(int argc, char **argv) {
  unsigned rate = atoi(argv[1]); int two = argv[2][0] == 't'; double secs = atof(argv[3]);
  tx_dsp_set_sample_rate(rate);
  // Optional overrides: PM_STEP (rad, phase limiter), PM_KNEE, PM_RATIO (1 = no envelope compression)
  if (getenv("PM_LIMITER")) pm_phase_limiter = atoi(getenv("PM_LIMITER"));
  if (getenv("PM_STEP")) pm_max_phase_step = atof(getenv("PM_STEP"));
  if (getenv("PM_KNEE")) pm_soft_knee = atof(getenv("PM_KNEE"));
  if (getenv("PM_RATIO")) pm_env_ratio = atof(getenv("PM_RATIO"));
  unsigned n = (unsigned)(secs * rate);
  for (unsigned i = 0; i < n; ++i) {
    float x;
    if (two) { unsigned k = i % (rate / 100); double t = (double)k / rate;
      double g = argc > 4 ? atof(argv[4]) : 0.5; x = g * sin(2*M_PI*700*t) + g * sin(2*M_PI*1900*t); }
    else x = wav_data[(rate == 16000 ? i * 2 : i) % wav_data_len] / 32768.0f;
    float a, p; tx_dsp_process(x, !two, &a, &p);
    printf("%.9g %.9g\n", a, p);
  }
}
