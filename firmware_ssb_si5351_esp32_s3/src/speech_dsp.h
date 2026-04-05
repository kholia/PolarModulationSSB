#ifndef SPEECH_DSP_H
#define SPEECH_DSP_H

#include <stdint.h>

// SSB speech processor: AGC → clip → filter pipeline
// Increases average-to-peak ratio by ~6dB for dramatically better talk power
// and intelligibility, matching commercial speech processors (Heil, W2IHY, etc.)

// Runtime-tunable parameters (CAT commands: *ANNN*, *GNNN*, *CNNN*, *ENNN*)
extern float sdsp_agc_target;       // AGC target level (default 0.15)
extern float sdsp_agc_max_gain;     // AGC maximum gain (default 1.5)
extern float sdsp_clip_level;       // Hard clipper threshold (default 0.4)
extern float sdsp_preemph;          // Pre-emphasis coefficient (default 0.0)
extern uint32_t sdsp_bandwidth_hz;  // Voice low-pass selection (default 2700)

void speech_dsp_reset(void);

// Select the coefficient bank and time constants for 16 or 32 ksample/s.
void speech_dsp_set_sample_rate(uint32_t sample_rate_hz);

// Select one of the matched 2000/2200/2400/2700/3000 Hz low-pass banks.
// Returns non-zero when the requested bank exists.
int speech_dsp_set_bandwidth(uint32_t bandwidth_hz);

// Re-trigger the startup fade-in ramp without resetting filters/AGC.
// Call at WAV loop boundaries to suppress the pop from audio discontinuity.
void speech_dsp_retrigger_fadein(void);

// Process a single audio sample (float, ±1.0 range)
// Call at the selected 16/32 ksample/s rate, before modulation_am_pm_f().
float speech_process(float sample);

#endif
