#include "polar_mod.h"
#include <stdlib.h>
#include <math.h>

#ifdef PICO_BOARD
#include "pico.h"
#else
#define __not_in_flash_func(x) x
#endif

int modulation_mode = MOD_USB;

// Runtime-tunable parameters (set via CAT commands *eNNN*, *jNNN*)
#if SAMPLE_RATE == 32000
float pm_max_phase_step = 1.0f;
#else
float pm_max_phase_step = 2.0f;
#endif
float pm_soft_knee = 0.002f;

#ifndef SAMPLE_RATE
#define SAMPLE_RATE 32000
#endif

#define USE_AGC 0
#define USE_COMPRESSOR 0  // Disable compressor to reduce metallic sound

// Filter delay lines (file-scope for reset access)
static float delay_bp[8] = { 0 };
static float agc_gain_f = 1.0f;
static float pre_emph_state = 0;
static float hilbert_history[127] = { 0 };
static int hilbert_idx = 0;
static float last_angle = 0;
static float dc_state = 0.0f;
static float comp_env = 0;  // Compressor envelope (file-scope for reset access)

void polar_mod_reset(void) {
  for (int i = 0; i < 8; i++) delay_bp[i] = 0;
  agc_gain_f = 1.0f;
  pre_emph_state = 0;
  for (int i = 0; i < 127; i++) hilbert_history[i] = 0;
  hilbert_idx = 0;
  last_angle = 0;
  dc_state = 0.0f;
  comp_env = 0;  // Fix #10: reset compressor envelope between TX sessions
}

// Biquad filter implementation (float)
static inline float biquad_f(float x, float b0, float b1, float b2, float a1, float a2, float *delay) {
  float y = b0 * x + delay[0];
  delay[0] = b1 * x - a1 * y + delay[1];
  delay[1] = b2 * x - a2 * y;
  return y;
}

// 300Hz-3000Hz Bandpass (2nd order HP + 2nd order LP cascade)
float __not_in_flash_func(filter_bandpass_16k)(float x) {
#if SAMPLE_RATE == 32000
  // Highpass 300Hz at 32kHz (2nd order Butterworth)
  float s1 = biquad_f(x, 0.9592f, -1.9183f, 0.9592f, -1.9166f, 0.9201f, &delay_bp[0]);
  // Lowpass 3000Hz at 32kHz (2nd order Butterworth)
  float s2 = biquad_f(s1, 0.0609f, 0.1217f, 0.0609f, -1.1930f, 0.4364f, &delay_bp[2]);
#else
  // Highpass 300Hz at 16kHz (2nd order Butterworth)
  float s1 = biquad_f(x, 0.9186f, -1.8372f, 0.9186f, -1.8347f, 0.8408f, &delay_bp[0]);
  // Lowpass 3000Hz at 16kHz (2nd order Butterworth)
  float s2 = biquad_f(s1, 0.2452f, 0.4905f, 0.2452f, -0.1369f, 0.1177f, &delay_bp[2]);
#endif
  return s2;
}

// Audio Compressor / Limiter
float __not_in_flash_func(compressor)(float x) {
  float abs_x = fabsf(x);
  // Fast attack, slow release envelope follower - Smoothed for better audio quality
  if (abs_x > comp_env) comp_env = 0.95f * comp_env + 0.05f * abs_x;
  else comp_env = 0.9995f * comp_env + 0.0005f * abs_x;

  if (comp_env < 0.01f) return x;
  float gain = 1.0f / (1.0f + comp_env * 2.5f);
  return x * gain * 1.2f;
}

static const float h_coeffs[127] = {
  0.00000000f, -0.00000000f, -0.00000938f, -0.00000000f, -0.00003919f, -0.00000000f, -0.00009284f, -0.00000000f, -0.00017502f, -0.00000000f, -0.00029180f, -0.00000000f, -0.00045073f, -0.00000000f, -0.00066091f, -0.00000000f, -0.00093308f, -0.00000000f, -0.00127965f, -0.00000000f, -0.00171479f, -0.00000000f, -0.00225449f, -0.00000000f, -0.00291679f, -0.00000000f, -0.00372193f, -0.00000000f, -0.00469281f, -0.00000000f, -0.00585552f, -0.00000000f, -0.00724031f, -0.00000000f, -0.00888295f, -0.00000000f, -0.01082685f, -0.00000000f, -0.01312641f, -0.00000000f, -0.01585207f, -0.00000000f, -0.01909859f, -0.00000000f, -0.02299849f, -0.00000000f, -0.02774522f, -0.00000000f, -0.03363493f, -0.00000000f, -0.04114687f, -0.00000000f, -0.05111144f, -0.00000000f, -0.06510241f, -0.00000000f, -0.08650115f, -0.00000000f, -0.12411493f, -0.00000000f, -0.21026729f, -0.00000000f, -0.63597101f, 0.00000000f, 0.63597101f, 0.00000000f, 0.21026729f, 0.00000000f, 0.12411493f, 0.00000000f, 0.08650115f, 0.00000000f, 0.06510241f, 0.00000000f, 0.05111144f, 0.00000000f, 0.04114687f, 0.00000000f, 0.03363493f, 0.00000000f, 0.02774522f, 0.00000000f, 0.02299849f, 0.00000000f, 0.01909859f, 0.00000000f, 0.01585207f, 0.00000000f, 0.01312641f, 0.00000000f, 0.01082685f, 0.00000000f, 0.00888295f, 0.00000000f, 0.00724031f, 0.00000000f, 0.00585552f, 0.00000000f, 0.00469281f, 0.00000000f, 0.00372193f, 0.00000000f, 0.00291679f, 0.00000000f, 0.00225449f, 0.00000000f, 0.00171479f, 0.00000000f, 0.00127965f, 0.00000000f, 0.00093308f, 0.00000000f, 0.00066091f, 0.00000000f, 0.00045073f, 0.00000000f, 0.00029180f, 0.00000000f, 0.00017502f, 0.00000000f, 0.00009284f, 0.00000000f, 0.00003919f, 0.00000000f, 0.00000938f, 0.00000000f, -0.00000000f
};

void __not_in_flash_func(hilbert_f)(float sample_in, float *i_out, float *q_out) {
  hilbert_history[hilbert_idx] = sample_in;
  float q = 0;
  // Stride-2: only even indices 2..124 have non-zero coefficients.
  // Odd indices and 0/63/126 are exactly zero. This halves the loop count
  // (62 iterations instead of 127) and eliminates the per-tap branch.
  for (int k = 2; k <= 124; k += 2) {
    int h_idx = hilbert_idx - k;
    if (h_idx < 0) h_idx += 127;
    q += hilbert_history[h_idx] * h_coeffs[k];
  }
  int i_idx = hilbert_idx - 63;
  if (i_idx < 0) i_idx += 127;
  *i_out = hilbert_history[i_idx];
  *q_out = q;
  hilbert_idx++;
  if (hilbert_idx >= 127) hilbert_idx = 0;
}

int __not_in_flash_func(modulation_am_pm_f)(float data, float *ampl_out, float *phase_diff_out) {

  // ── Pre-processing: skip when speech DSP already handles these ─────────
#if USE_SPEECH_DSP
  // Fix #1-3: Speech DSP already provides DC blocking, bandpass, and
  // pre-emphasis. Repeating them here adds group delay that misaligns
  // the Hilbert I/Q paths and colors the audio.
  float processed_data = data;
#else
  // Pre-emphasis
  float emph_data = data - 0.05f * pre_emph_state;
  pre_emph_state = data;

  // Bandpass filtering
  float bp_data = filter_bandpass_16k(emph_data);

#if USE_COMPRESSOR
  float comp_data = compressor(bp_data);
#else
  float comp_data = bp_data;
#endif

#if USE_AGC
  float abs_data = fabsf(comp_data);
  if (abs_data > 0.001f) {
    if (abs_data * agc_gain_f > 0.1f) agc_gain_f *= 0.9999f;
    else if (abs_data * agc_gain_f < 0.02f) agc_gain_f *= 1.00001f;
  }
  float processed_data = comp_data * agc_gain_f;
#else
  float processed_data = comp_data;
#endif

  // DC blocker
#if SAMPLE_RATE == 32000
  float dc_out = processed_data - dc_state;
  dc_state = dc_state * 0.9995f + processed_data * 0.0005f;
  processed_data = dc_out;
#else
  float dc_out = processed_data - dc_state;
  dc_state = dc_state * 0.999f + processed_data * 0.001f;
  processed_data = dc_out;
#endif
#endif  // USE_SPEECH_DSP

  // ── Hilbert transform: extract I/Q components ─────────────────────────
  float x, y;
  hilbert_f(processed_data, &x, &y);
  float ampl = hypotf(x, y);
  float angle = atan2f(y, x);

  // ── Fix #5: Soft knee for PHASE ONLY at low amplitudes ────────────────
  // When amplitude is near zero, atan2 output is noisy/undefined.
  // Blend the phase toward last_angle to prevent noise bursts.
  // Do NOT modify amplitude — it must reflect the true envelope.
  const float soft_knee = pm_soft_knee;
  float knee_blend = ampl * ampl / (ampl * ampl + soft_knee * soft_knee);
  angle = angle * knee_blend + last_angle * (1.0f - knee_blend);

  float angle_diff = angle - last_angle;
  if (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
  if (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;

  // ── Fix #6: Soft phase limiter (tanh) instead of hard clip ────────────
  // Hard clipping phase creates unfiltered spurs in the demodulated audio.
  // tanh() smoothly saturates — no harmonics generated at the transition.
  const float max_phase_step = pm_max_phase_step;
  angle_diff = max_phase_step * tanhf(angle_diff / max_phase_step);

  // ── Fix #7: Track ACTUAL transmitted phase, not raw atan2 output ──────
  // The phase limiter may reduce angle_diff below what atan2 requested.
  // last_angle must reflect where we actually went, not where we wanted
  // to go. Otherwise, the next sample's diff is computed from a phase
  // we never actually transmitted, causing accumulating error.
  last_angle += angle_diff;
  if (last_angle > M_PI) last_angle -= 2.0f * M_PI;
  if (last_angle < -M_PI) last_angle += 2.0f * M_PI;

  if (modulation_mode == MOD_LSB) angle_diff = -angle_diff;

  *ampl_out = ampl;
  *phase_diff_out = angle_diff;
  return 0;
}

int modulation_am_pm(int16_t data, int *ampl_out, int *phase_diff_out) {
  float f_data = (float)data / 32768.0f;
  float f_ampl, f_phase_diff;
  modulation_am_pm_f(f_data, &f_ampl, &f_phase_diff);
  *ampl_out = (int)(f_ampl * 32768.0f);
  *phase_diff_out = (int)(f_phase_diff * 16777216.0f / (2.0f * M_PI));
  return 0;
}
