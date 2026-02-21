#ifndef SPEECH_DSP_H
#define SPEECH_DSP_H

#include <stdint.h>

// SSB speech processor: AGC → clip → filter pipeline
// Increases average-to-peak ratio by ~6dB for dramatically better talk power
// and intelligibility, matching commercial speech processors (Heil, W2IHY, etc.)

void speech_dsp_reset(void);

// Re-trigger the startup fade-in ramp without resetting filters/AGC.
// Call at WAV loop boundaries to suppress the pop from audio discontinuity.
void speech_dsp_retrigger_fadein(void);

// Process a single audio sample (float, ±1.0 range)
// Call at 16kHz sample rate, before modulation_am_pm_f()
float speech_process(float sample);

#endif
