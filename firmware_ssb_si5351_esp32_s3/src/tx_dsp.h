#ifndef TX_DSP_H
#define TX_DSP_H

#include <stdint.h>

// Coordinates speech conditioning and polar conversion. The 32 ksample/s
// profile uses matched halfband decimation/interpolation around a 16 ksample/s
// analytic-signal core.
void tx_dsp_set_sample_rate(uint32_t output_rate_hz);
void tx_dsp_reset(void);
uint32_t tx_dsp_analysis_rate(void);

// Process one sample at the selected output rate. enable_speech_dsp is an int
// to keep this header usable from both C and C++ callers.
void tx_dsp_process(float sample, int enable_speech_dsp, float *ampl_out,
                    float *phase_diff_out);

#endif
