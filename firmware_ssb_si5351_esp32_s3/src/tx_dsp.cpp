#include "tx_dsp.h"

#include "polar_mod.h"
#include "speech_dsp.h"

#ifdef PICO_BOARD
#include "pico.h"
#else
#define __not_in_flash_func(x) x
#endif

static uint32_t output_rate_hz = 16000;
static uint32_t analysis_rate_hz = 16000;

// 31-tap, 2:1 halfband FIR. Only eight symmetric tap pairs plus the centre
// tap are non-zero. The outer coefficients sum to 0.5 and the centre is
// exactly 0.5, giving unity DC gain in the decimator and unity gain in both
// phases of the x2 interpolator.
static constexpr float hb_pair_coeffs[8] = {
  -0.00170314997f,
  0.00294208732f,
  -0.00674098786f,
  0.01411670691f,
  -0.02682840270f,
  0.04917845530f,
  -0.09709528282f,
  0.31613057382f,
};

static float decim_history[31];
static int decim_index;
static bool decim_phase;

static float interp_i_history[16];
static float interp_q_history[16];
static int interp_index;
// The 32k path is a two-tick pipeline. A ready I/Q pair is transmitted while
// the next low-rate sample is prepared in two balanced stages:
//   tick 0: halfband decimation + speech DSP
//   tick 1: Hilbert/envelope analysis + complex interpolation
// This avoids putting the entire 16k analysis chain on every other 31 us tick.
static float ready_i[2];
static float ready_q[2];
static int ready_phase;
static float pending_processed;
static bool pending_analysis;

static inline int wrap_subtract(int index, int amount, int length) {
  int result = index - amount;
  if (result < 0) result += length;
  return result;
}

static bool halfband_decimate(float sample, float *output) {
  decim_history[decim_index] = sample;
  decim_phase = !decim_phase;
  if (!decim_phase) {
    decim_index++;
    if (decim_index == 31) decim_index = 0;
    return false;
  }

  float sum = 0.5f * decim_history[wrap_subtract(decim_index, 15, 31)];
  for (int pair = 0; pair < 8; ++pair) {
    int early_tap = pair * 2;
    int late_tap = 30 - early_tap;
    float samples =
      decim_history[wrap_subtract(decim_index, early_tap, 31)] +
      decim_history[wrap_subtract(decim_index, late_tap, 31)];
    sum += hb_pair_coeffs[pair] * samples;
  }
  *output = sum;

  decim_index++;
  if (decim_index == 31) decim_index = 0;
  return true;
}

static void halfband_interpolate(float i, float q, float *even_i,
                                 float *even_q, float *odd_i, float *odd_q) {
  interp_i_history[interp_index] = i;
  interp_q_history[interp_index] = q;

  float i_sum = 0.0f;
  float q_sum = 0.0f;
  for (int pair = 0; pair < 8; ++pair) {
    int early = wrap_subtract(interp_index, pair, 16);
    int late = wrap_subtract(interp_index, 15 - pair, 16);
    i_sum += hb_pair_coeffs[pair] *
             (interp_i_history[early] + interp_i_history[late]);
    q_sum += hb_pair_coeffs[pair] *
             (interp_q_history[early] + interp_q_history[late]);
  }

  // The interpolation filter has gain two to compensate for zero stuffing.
  *even_i = 2.0f * i_sum;
  *even_q = 2.0f * q_sum;
  int centre = wrap_subtract(interp_index, 7, 16);
  *odd_i = interp_i_history[centre];
  *odd_q = interp_q_history[centre];

  interp_index++;
  if (interp_index == 16) interp_index = 0;
}

void tx_dsp_set_sample_rate(uint32_t requested_rate_hz) {
  output_rate_hz = requested_rate_hz == 32000 ? 32000 : 16000;
  analysis_rate_hz = 16000;
  polar_mod_set_rates(output_rate_hz, analysis_rate_hz);
  speech_dsp_set_sample_rate(analysis_rate_hz);
  tx_dsp_reset();
}

uint32_t tx_dsp_analysis_rate(void) {
  return analysis_rate_hz;
}

void tx_dsp_reset(void) {
  for (float &sample : decim_history) sample = 0.0f;
  for (float &sample : interp_i_history) sample = 0.0f;
  for (float &sample : interp_q_history) sample = 0.0f;
  decim_index = 0;
  decim_phase = false;
  interp_index = 0;
  ready_i[0] = ready_i[1] = 0.0f;
  ready_q[0] = ready_q[1] = 0.0f;
  ready_phase = 0;
  pending_processed = 0.0f;
  pending_analysis = false;
  polar_mod_reset();
  speech_dsp_reset();
}

void __not_in_flash_func(tx_dsp_process)(float sample, int enable_speech_dsp,
                                         float *ampl_out,
                                         float *phase_diff_out) {
  if (output_rate_hz == 16000) {
    float processed = enable_speech_dsp ? speech_process(sample) : sample;
    modulation_am_pm_f(processed, ampl_out, phase_diff_out);
    return;
  }

  // Fetch the previously prepared output before this tick can replace the
  // ready pair. The extra pair of latency is insignificant for voice, but
  // makes the per-tick execution time much more uniform.
  float i = ready_i[ready_phase];
  float q = ready_q[ready_phase];
  float decimated;
  if (halfband_decimate(sample, &decimated)) {
    pending_processed =
      enable_speech_dsp ? speech_process(decimated) : decimated;
    pending_analysis = true;
  } else if (pending_analysis) {
    float base_i;
    float base_q;
    polar_mod_analyze_f(pending_processed, &base_i, &base_q);
    halfband_interpolate(base_i, base_q, &ready_i[0], &ready_q[0],
                         &ready_i[1], &ready_q[1]);
    pending_analysis = false;
  }

  ready_phase ^= 1;
  polar_mod_encode_iq_f(i, q, ampl_out, phase_diff_out);
}
