#include <stdio.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include "../src/polar_mod.h"
#include "../src/audio_data.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RESTORE_AMPLITUDE 0

int main() {
    int fs = 32000;
    uint32_t n_samples = wav_data_len;

    std::vector<float> output(n_samples);
    double phase_accu = 0;

    // Use USB mode for simulation
    modulation_mode = MOD_USB;

    printf("Processing %u samples (AM Restore: %d)...\n", n_samples, RESTORE_AMPLITUDE);

    for (uint32_t i = 0; i < n_samples; i++) {
        float f_data = (float)wav_data[i] / 32768.0f;
        float f_ampl, f_phase_diff;

        // Run through the EXACT same pipeline as the Pico 2
        modulation_am_pm_f(f_data, &f_ampl, &f_phase_diff);

        // In SSB, the signal is A(t) * cos(wc*t + phi(t))
        // To hear it at baseband, we just use the accumulated phase
        phase_accu += f_phase_diff;

        // Reconstruct the real part of the analytic signal
#if RESTORE_AMPLITUDE
        output[i] = 2.0f * f_ampl * cos(phase_accu); // Master gain for SSB
#else
        output[i] = 0.3f * cos(phase_accu); // Very low volume for phase-only
#endif
    }
    // Write to raw file
    FILE* f = fopen("processed.raw", "wb");
    for (uint32_t i = 0; i < n_samples; i++) {
        int16_t sample = (int16_t)(output[i] * 32767.0f);
        fwrite(&sample, sizeof(int16_t), 1, f);
    }
    fclose(f);

    printf("Done. Saved to processed.raw\n");
    return 0;
}
