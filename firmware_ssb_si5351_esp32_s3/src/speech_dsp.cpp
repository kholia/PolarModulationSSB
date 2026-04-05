#include "speech_dsp.h"
#include <math.h>

#ifdef PICO_BOARD
#include "pico.h"
#else
#define __not_in_flash_func(x) x
#endif

// Runtime-tunable parameters (set via CAT commands *ANNN*, *GNNN*, *CNNN*, *ENNN*)
float sdsp_agc_target = 0.15f;
float sdsp_agc_max_gain = 1.5f;
float sdsp_clip_level = 0.4f;
float sdsp_preemph = 0.0f;
uint32_t sdsp_bandwidth_hz = 2700;

static uint32_t speech_sample_rate_hz = 16000;

// ── Biquad filter (same structure as polar_mod.cpp) ──────────────────────────
static inline float biquad(float x, float b0, float b1, float b2,
                           float a1, float a2, float *d) {
  float y = b0 * x + d[0];
  d[0] = b1 * x - a1 * y + d[1];
  d[1] = b2 * x - a2 * y;
  return y;
}

struct FilterProfile {
  float hp_b0, hp_b1, hp_b2, hp_a1, hp_a2;
  float lp_b0, lp_b1, lp_b2, lp_a1, lp_a2;
  float dc_alpha, dc_beta;
  float agc_attack, agc_release, agc_atk_c, agc_rel_c;
};

static constexpr FilterProfile PROFILE_16K = {
  0.9186f, -1.8372f, 0.9186f, -1.8347f, 0.8408f,
  0.1568f, 0.3136f, 0.1568f, -0.6102f, 0.2374f,
  0.999f, 0.001f, 0.94f, 0.9998f, 0.06f, 0.0002f
};
static constexpr FilterProfile PROFILE_32K = {
  0.9592f, -1.9183f, 0.9592f, -1.9166f, 0.9201f,
  0.0506f, 0.1012f, 0.0506f, -1.2711f, 0.4733f,
  0.9995f, 0.0005f, 0.97f, 0.9999f, 0.03f, 0.0001f
};

static const FilterProfile *profile = &PROFILE_16K;

struct LowpassCoefficients {
  uint32_t cutoff_hz;
  float b0, b1, b2, a1, a2;
};

// Ron G4GXO's switchable transmit-bandwidth idea, expressed as inexpensive
// Butterworth biquad banks for this firmware's 16/32k processing rates.
static constexpr LowpassCoefficients LOWPASS_16K[] = {
  { 2000, 0.0976311f, 0.1952621f, 0.0976311f, -0.9428090f, 0.3333333f },
  { 2200, 0.1139867f, 0.2279734f, 0.1139867f, -0.8447071f, 0.3006538f },
  { 2400, 0.1311064f, 0.2622129f, 0.1311064f, -0.7477892f, 0.2722149f },
  { 2700, 0.1568f, 0.3136f, 0.1568f, -0.6102f, 0.2374f },
  { 3000, 0.1866943f, 0.3733887f, 0.1866943f, -0.4629380f, 0.2097154f },
};
static constexpr LowpassCoefficients LOWPASS_32K[] = {
  { 2000, 0.0299546f, 0.0599092f, 0.0299546f, -1.4542436f, 0.5740619f },
  { 2200, 0.0354376f, 0.0708751f, 0.0354376f, -1.4014154f, 0.5431657f },
  { 2400, 0.0412535f, 0.0825071f, 0.0412535f, -1.3489677f, 0.5139819f },
  { 2700, 0.0506f, 0.1012f, 0.0506f, -1.2711f, 0.4733f },
  { 3000, 0.0604985f, 0.1209970f, 0.0604985f, -1.1939134f, 0.4359074f },
};
static const LowpassCoefficients *lowpass = &LOWPASS_16K[3];

// ── Filter delay lines ───────────────────────────────────────────────────────
// Pre-clip bandpass 300–2700 Hz (2nd order)
static float pre_hp_d[2] = { 0 };
static float pre_lp_d[2] = { 0 };

// Post-clip bandpass 300–2700 Hz (2nd order)
// NOTE: Kept at 2nd order (not 4th) to minimize group delay and ringing.
// Combined with the pre-clip filter, total is 4th order — enough to reject
// clipping harmonics without smearing speech transients.
static float post_hp_d[2] = { 0 };
static float post_lp_d[2] = { 0 };

// ── Processing state ─────────────────────────────────────────────────────────
static float dc_z = 0;          // DC blocker state
static float emph_prev = 0;     // Pre-emphasis previous sample
static float agc_env = 0;       // AGC envelope follower
static float agc_gain = 1.0f;   // AGC current gain (smoothed)
static int fadein_counter = 0;  // Startup fade-in counter

void speech_dsp_set_sample_rate(uint32_t sample_rate_hz) {
  speech_sample_rate_hz = sample_rate_hz == 32000 ? 32000 : 16000;
  profile = speech_sample_rate_hz == 32000 ? &PROFILE_32K : &PROFILE_16K;
  const LowpassCoefficients *bank =
    speech_sample_rate_hz == 32000 ? LOWPASS_32K : LOWPASS_16K;
  for (int i = 0; i < 5; ++i) {
    if (bank[i].cutoff_hz == sdsp_bandwidth_hz) {
      lowpass = &bank[i];
      break;
    }
  }
  speech_dsp_reset();
}

int speech_dsp_set_bandwidth(uint32_t bandwidth_hz) {
  const LowpassCoefficients *bank =
    speech_sample_rate_hz == 32000 ? LOWPASS_32K : LOWPASS_16K;
  for (int i = 0; i < 5; ++i) {
    if (bank[i].cutoff_hz == bandwidth_hz) {
      sdsp_bandwidth_hz = bandwidth_hz;
      lowpass = &bank[i];
      speech_dsp_reset();
      return 1;
    }
  }
  return 0;
}

void speech_dsp_reset(void) {
  pre_hp_d[0] = pre_hp_d[1] = 0;
  pre_lp_d[0] = pre_lp_d[1] = 0;
  post_hp_d[0] = post_hp_d[1] = 0;
  post_lp_d[0] = post_lp_d[1] = 0;
  dc_z = 0;
  emph_prev = 0;
  agc_env = 0;
  agc_gain = 1.0f;
  fadein_counter = 0;
}

void speech_dsp_retrigger_fadein(void) {
  fadein_counter = 0;
}

// ── Main speech processing pipeline ──────────────────────────────────────────
//
// Pipeline (clip → filter → re-clip):
//   1. DC blocker              – remove DC offset from ADC or WAV data
//   2. Pre-emphasis (0.0)      – disabled to retain the source voice balance
//   3. Pre-clip bandpass       – 300–2700 Hz (2nd order)
//   4. AGC (target 0.15, 1.5x max) – retain dynamics and avoid clip dominance
//   5. Hard clipper             – protection at ±0.4
//   6. Post-clip bandpass      – 300–2700 Hz (2nd order, -12dB/oct)
//   7. Re-clip                  – catch peaks regenerated by filter ringing
//   8. Output scaling          – normalize to ±1.0 range
//
// Total filter order: 4th (2nd pre + 2nd post). Low group delay, minimal
// ringing. Previous 8th-order pipeline caused slurriness from excessive
// phase dispersion (~8ms group delay at 300Hz).
//
float __not_in_flash_func(speech_process)(float x) {

  // ── 1. DC blocker (HPF at ~2.5 Hz) ──────────────────────────────────────
  float dc_out = x - dc_z;
  dc_z = dc_z * profile->dc_alpha + x * profile->dc_beta;

  // ── 2. Pre-emphasis: ~+6dB/octave above ~1 kHz ──────────────────────────
  // Runtime-tunable; zero by default to preserve the original spectral flavor.
  float emph = dc_out + sdsp_preemph * (dc_out - emph_prev);
  emph_prev = dc_out;

  // ── 3. Pre-clip bandpass 300–2700 Hz ─────────────────────────────────────
  float s1 = biquad(emph, profile->hp_b0, profile->hp_b1, profile->hp_b2,
                    profile->hp_a1, profile->hp_a2, pre_hp_d);
  float s2 = biquad(s1, lowpass->b0, lowpass->b1, lowpass->b2,
                    lowpass->a1, lowpass->a2, pre_lp_d);

  // ── 4. AGC: normalize level before clipper (smoothed gain) ───────────────
  float abs_s = fabsf(s2);
  if (abs_s > agc_env)
    agc_env = agc_env * profile->agc_attack + abs_s * profile->agc_atk_c;
  else
    agc_env = agc_env * profile->agc_release + abs_s * profile->agc_rel_c;

  // The conservative default target and gain ceiling retain voice dynamics;
  // the clipper below is primarily peak protection. Parameters remain tunable
  // via CAT commands *ANNN* and *GNNN*.
  float target_gain = agc_gain;  // Hold previous if env too low
  if (agc_env > 0.005f)
    target_gain = sdsp_agc_target / agc_env;
  if (target_gain > sdsp_agc_max_gain) target_gain = sdsp_agc_max_gain;
  if (target_gain < 0.1f) target_gain = 0.1f;         // Min -20dB
  agc_gain = agc_gain * 0.95f + target_gain * 0.05f;  // One-pole smoother

  float agc_out = s2 * agc_gain;

  // ── 5. Hard clipper ──────────────────────────────────────────────────────
  const float clip_level = sdsp_clip_level;
  float clipped = agc_out;
  if (clipped > clip_level) clipped = clip_level;
  if (clipped < -clip_level) clipped = -clip_level;

  // ── 6. Post-clip bandpass 300–2700 Hz (2nd order) ────────────────────────
  // Removes clipping harmonics. 2nd order gives -12dB/oct — enough when
  // combined with pre-clip filter (total 4th order), and avoids the ringing
  // and group delay problems of higher-order cascades.
  float p1 = biquad(clipped, profile->hp_b0, profile->hp_b1, profile->hp_b2,
                    profile->hp_a1, profile->hp_a2, post_hp_d);
  float p2 = biquad(p1, lowpass->b0, lowpass->b1, lowpass->b2,
                    lowpass->a1, lowpass->a2, post_lp_d);

  // ── 7. Re-clip (catches peaks from filter ringing) ───────────────────────
  // Post-clip filter ringing can regenerate small peaks above clip level.
  // Re-clipping is cheap and recovers that headroom. No filter needed after
  // this — the re-clip barely produces harmonics since overshoot is tiny.
  if (p2 > clip_level) p2 = clip_level;
  if (p2 < -clip_level) p2 = -clip_level;

  // ── 8. Output scaling ───────────────────────────────────────────────────
  float out = p2 * (1.0f / clip_level);

  // ── Startup fade-in: suppress filter transient pop ─────────────────────
  const int fadein_samples = static_cast<int>(speech_sample_rate_hz / 100);
  if (fadein_counter < fadein_samples) {
    out *= (float)fadein_counter / (float)fadein_samples;
    fadein_counter++;
  }

  return out;
}
