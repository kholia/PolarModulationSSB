#ifndef POLAR_MOD_H_
#define POLAR_MOD_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Modulation modes (same as STM32 version)
#define MOD_FMN 0
#define MOD_LSB 1
#define MOD_USB 2
#define MOD_CW 3
#define MOD_FM 4
#define MOD_AM 5
#define MOD_FSK 6
#define MOD_FMW 8

  extern int modulation_mode;

  // Runtime-tunable polar modulation parameters.
  extern float pm_max_phase_step;  // Max step (default 0.4 @32k, 0.8 @16k)
  extern float pm_soft_knee;       // Soft knee threshold (default 0.0002)
  extern float pm_env_threshold;   // Analytic-envelope compressor threshold
  extern float pm_env_ratio;       // Compressor ratio; 1.0 disables compression
  extern float pm_env_ceiling;     // Absolute analytic-envelope ceiling

  // Change filter coefficients and phase limiter for a 16/32 ksample/s profile.
  void polar_mod_set_sample_rate(uint32_t sample_rate_hz);
  void polar_mod_set_rates(uint32_t output_rate_hz,
                           uint32_t analysis_rate_hz);

  // Main processing function
  int modulation_am_pm(int16_t data, int *ampl_out, int *phase_diff_out);
  int modulation_am_pm_f(float data, float *ampl_out, float *phase_diff_out);

  // Split form used by the 32 ksample/s multirate pipeline. Analysis runs at
  // 16 ksample/s; the resulting complex vectors are interpolated before the
  // phase encoder is called at the output sample rate.
  void polar_mod_analyze_f(float data, float *i_out, float *q_out);
  void polar_mod_encode_iq_f(float i, float q, float *ampl_out,
                             float *phase_diff_out);

  // Reset all DSP state (call before starting a new TX session)
  void polar_mod_reset(void);

  void set_dc_offsets(float x, float y);

  // Support functions (exported for potential tests or debugging)
  int mic_agc_fast(int ampl);
  int soft_limiter(int x);
  void hilbert(int sample_in, int *i_out, int *q_out);
  void cordic(int x, int y, int *out_abs, int *out_angle);

#ifdef __cplusplus
}
#endif

#endif /* POLAR_MOD_H_ */
