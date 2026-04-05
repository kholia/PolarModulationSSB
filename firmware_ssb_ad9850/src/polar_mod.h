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

  // Runtime-tunable polar modulation parameters (CAT commands: *eNNN*, *jNNN*)
  extern float pm_max_phase_step;  // Phase limiter max step (default 1.0 @32k, 2.0 @16k)
  extern float pm_soft_knee;       // Soft knee threshold (default 0.002)

  // Main processing function
  int modulation_am_pm(int16_t data, int *ampl_out, int *phase_diff_out);
  int modulation_am_pm_f(float data, float *ampl_out, float *phase_diff_out);

  // Reset all DSP state (call before starting a new TX session)
  void polar_mod_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* POLAR_MOD_H_ */
