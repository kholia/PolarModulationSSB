// Host checks for the drain-modulation ramp planner and delay split.
#include <initializer_list>
#include <stdio.h>
#include <stdlib.h>

#include "drain_am.h"

static bool check_ramps(uint32_t steps) {
  // The planned fade must never leave the duty range, must move toward the
  // target, and its carried error must stay below one fade quantum.
  int32_t duty = 0;
  uint32_t seed = 12345;
  for (int i = 0; i < 200000; ++i) {
    seed = seed * 1664525U + 1013904223U;
    int32_t target = static_cast<int32_t>(seed % (DRAIN_DUTY_MAX_Q4 + 1));
    if (i % 1000 < 50) target = (i & 1) ? 0 : DRAIN_DUTY_MAX_Q4;  // full swings
    int32_t start = duty;
    DrainRamp ramp = drain_plan_ramp(&duty, target, steps);
    int32_t travel = static_cast<int32_t>(ramp.scale * ramp.steps);
    int32_t expected = start + (ramp.increase ? travel : -travel);
    int32_t error = target - duty;
    if (duty != expected || duty < 0 || duty > DRAIN_DUTY_MAX_Q4 ||
        ramp.steps != steps || error >= static_cast<int32_t>(steps) ||
        -error >= static_cast<int32_t>(steps)) {
      printf("FAIL ramp steps=%u start=%d target=%d end=%d scale=%u\n", (unsigned)steps,
             (int)start, (int)target, (int)duty, (unsigned)ramp.scale);
      return false;
    }
  }
  return true;
}

static bool check_split(uint32_t steps) {
  for (int32_t d = DRAIN_DELAY_MIN_STEPS; d <= DRAIN_DELAY_MAX_STEPS; ++d) {
    DrainAlignment a = drain_split_delay(d, steps);
    int32_t rebuilt = static_cast<int32_t>(a.amplitude_samples * steps) -
                      static_cast<int32_t>(a.phase_samples * steps) +
                      static_cast<int32_t>(a.boundary);
    if (rebuilt != d || a.boundary < 1 || a.boundary > steps ||
        (a.amplitude_samples && a.phase_samples) ||
        a.amplitude_samples >= DRAIN_HISTORY || a.phase_samples >= DRAIN_HISTORY) {
      printf("FAIL split steps=%u d=%d -> amp %u phase %u boundary %u\n", (unsigned)steps,
             (int)d, (unsigned)a.amplitude_samples, (unsigned)a.phase_samples, (unsigned)a.boundary);
      return false;
    }
  }
  return true;
}

int main() {
  bool ok = true;
  for (uint32_t steps : {10U, 20U}) {
    ok &= check_ramps(steps);
    ok &= check_split(steps);
  }
  printf("drain_am: %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
