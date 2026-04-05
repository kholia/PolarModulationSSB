// Parameter sweeper for the constant-envelope phase-only DSP path.
// Correlation is a useful regression proxy, not a substitute for an RF capture
// or an intelligibility listening test.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "audio_data.h"
#include "polar_mod.h"
#include "speech_dsp.h"

struct Parameters {
  // Keep the legacy profile as the sweep baseline so future runs retain a
  // stable before/after comparison with the captured RF sample.
  float agc_target = 0.35f;
  float agc_max_gain = 3.0f;
  float clip_level = 0.4f;
  float preemphasis = 0.7f;
  float phase_limit = 2.0f;
  float soft_knee = 0.002f;
};

static std::vector<float> input;

struct Evaluation {
  double correlation;
  int lag;
  double low_level_phase_rms;
  double low_level_phase_peak;
  double low_level_fraction;
};

struct Candidate {
  Parameters parameters;
  Evaluation evaluation;
};

static double pearson_at_lag(const std::vector<float> &first,
                             const std::vector<float> &second, int lag) {
  int begin = 1000;
  int end = std::min<int>(first.size(), second.size() - lag) - 1000;
  double first_sum = 0;
  double second_sum = 0;
  double first_square_sum = 0;
  double second_square_sum = 0;
  double cross_sum = 0;
  int count = 0;
  for (int index = begin; index < end; ++index) {
    double x = first[index];
    double y = second[index + lag];
    first_sum += x;
    second_sum += y;
    first_square_sum += x * x;
    second_square_sum += y * y;
    cross_sum += x * y;
    ++count;
  }
  double covariance = cross_sum - first_sum * second_sum / count;
  double first_variance =
      first_square_sum - first_sum * first_sum / count;
  double second_variance =
      second_square_sum - second_sum * second_sum / count;
  return covariance /
         std::sqrt(first_variance * second_variance + 1e-30);
}

static Evaluation evaluate(const Parameters &parameters) {
  speech_dsp_set_sample_rate(16000);
  polar_mod_set_sample_rate(16000);
  sdsp_agc_target = parameters.agc_target;
  sdsp_agc_max_gain = parameters.agc_max_gain;
  sdsp_clip_level = parameters.clip_level;
  sdsp_preemph = parameters.preemphasis;
  pm_max_phase_step = parameters.phase_limit;
  pm_soft_knee = parameters.soft_knee;
  speech_dsp_reset();
  polar_mod_reset();

  std::vector<float> output(input.size());
  double phase = 0;
  double low_level_phase_power = 0;
  double low_level_phase_peak = 0;
  size_t low_level_samples = 0;
  for (size_t index = 0; index < input.size(); ++index) {
#ifdef TUNE_LINEAR_PATH
    float sample = input[index];
#else
    float sample = speech_process(input[index]);
#endif
    float amplitude;
    float phase_difference;
    modulation_am_pm_f(sample, &amplitude, &phase_difference);
    if (amplitude < 0.01f) {
      low_level_phase_power += phase_difference * phase_difference;
      low_level_phase_peak =
          std::max(low_level_phase_peak,
                   static_cast<double>(std::abs(phase_difference)));
      ++low_level_samples;
    }
    phase += phase_difference;
    output[index] = std::cos(phase);
  }

  double best_correlation = 0;
  int best_lag = 0;
  for (int lag = 40; lag <= 140; ++lag) {
    double correlation = std::abs(pearson_at_lag(input, output, lag));
    if (correlation > best_correlation) {
      best_correlation = correlation;
      best_lag = lag;
    }
  }
  double low_level_phase_rms =
      low_level_samples
          ? std::sqrt(low_level_phase_power / low_level_samples)
          : 0;
  return {best_correlation, best_lag, low_level_phase_rms,
          low_level_phase_peak,
          static_cast<double>(low_level_samples) / input.size()};
}

static void result(const char *stage, const char *parameter, float value,
                   const Parameters &parameters,
                   const Evaluation *known_evaluation = nullptr) {
  Evaluation evaluation =
      known_evaluation ? *known_evaluation : evaluate(parameters);
#ifdef TUNE_LINEAR_PATH
  const char *path = "linear";
#else
  const char *path = "speech_dsp";
#endif
  std::printf("%s,%s,%s,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
              "%.8f,%.8f,%.6f\n",
              path, stage, parameter, value, evaluation.correlation,
              evaluation.lag, parameters.preemphasis, parameters.clip_level,
              parameters.agc_target, parameters.agc_max_gain,
              parameters.phase_limit, parameters.soft_knee,
              evaluation.low_level_phase_rms,
              evaluation.low_level_phase_peak,
              evaluation.low_level_fraction);
}

static void run_cleanup_sweep() {
  Parameters natural;
  natural.preemphasis = 0.0f;
  natural.clip_level = 0.4f;
  natural.agc_target = 0.15f;
  natural.agc_max_gain = 1.5f;
  for (float phase_limit : {0.6f, 0.8f, 1.0f, 1.2f}) {
    for (float soft_knee : {0.0002f, 0.0005f, 0.001f, 0.002f, 0.005f,
                            0.01f, 0.02f, 0.05f}) {
      Parameters candidate = natural;
      candidate.phase_limit = phase_limit;
      candidate.soft_knee = soft_knee;
      result("cleanup", "phase_knee", soft_knee, candidate);
    }
  }
}

static void run_one_factor(const Parameters &baseline) {
  result("one_factor", "baseline", 0, baseline);

  for (float value : {0.0f, 0.2f, 0.4f, 0.55f, 0.7f, 0.85f, 1.0f}) {
    Parameters candidate = baseline;
    candidate.preemphasis = value;
    result("one_factor", "preemphasis", value, candidate);
  }
  for (float value : {0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.8f, 1.0f}) {
    Parameters candidate = baseline;
    candidate.clip_level = value;
    result("one_factor", "clip_level", value, candidate);
  }
  for (float value : {0.12f, 0.15f, 0.2f, 0.25f, 0.35f, 0.45f, 0.6f,
                      0.8f}) {
    Parameters candidate = baseline;
    candidate.agc_target = value;
    result("one_factor", "agc_target", value, candidate);
  }
  for (float value : {1.5f, 2.0f, 3.0f, 4.0f, 6.0f}) {
    Parameters candidate = baseline;
    candidate.agc_max_gain = value;
    result("one_factor", "agc_max_gain", value, candidate);
  }
  for (float value : {0.8f, 1.0f, 1.2f, 1.5f, 2.0f, 2.5f, 3.14f}) {
    Parameters candidate = baseline;
    candidate.phase_limit = value;
    result("one_factor", "phase_limit", value, candidate);
  }
  for (float value : {0.0002f, 0.0005f, 0.001f, 0.002f, 0.005f, 0.01f,
                      0.02f}) {
    Parameters candidate = baseline;
    candidate.soft_knee = value;
    result("one_factor", "soft_knee", value, candidate);
  }
}

static void run_joint_sweep(const Parameters &baseline) {
  // Stage 1 deliberately keeps the polar parameters fixed. Retain the best
  // speech-stage candidates, then sweep their phase limiter and low-level
  // knee. This captures interactions without making the search needlessly
  // enormous.
  std::vector<Candidate> speech_candidates;
  for (float preemphasis : {0.0f, 0.2f, 0.4f, 0.55f, 0.7f}) {
    for (float clip_level : {0.4f, 0.6f, 0.8f, 1.0f}) {
      for (float agc_target : {0.12f, 0.15f, 0.2f, 0.25f, 0.35f}) {
        for (float agc_max_gain : {1.5f, 2.0f, 3.0f}) {
          Parameters candidate = baseline;
          candidate.preemphasis = preemphasis;
          candidate.clip_level = clip_level;
          candidate.agc_target = agc_target;
          candidate.agc_max_gain = agc_max_gain;
          speech_candidates.push_back({candidate, evaluate(candidate)});
        }
      }
    }
  }
  std::sort(speech_candidates.begin(), speech_candidates.end(),
            [](const Candidate &a, const Candidate &b) {
              return a.evaluation.correlation > b.evaluation.correlation;
            });

  const size_t finalists = std::min<size_t>(12, speech_candidates.size());
  for (size_t index = 0; index < finalists; ++index) {
    result("speech_finalist", "rank", static_cast<float>(index + 1),
           speech_candidates[index].parameters,
           &speech_candidates[index].evaluation);
  }

  std::vector<Candidate> polar_candidates;
  for (size_t index = 0; index < finalists; ++index) {
    for (float phase_limit : {0.8f, 1.0f, 1.2f, 1.5f, 2.0f}) {
      for (float soft_knee : {0.0002f, 0.0005f, 0.001f, 0.002f, 0.005f,
                              0.01f}) {
        Parameters candidate = speech_candidates[index].parameters;
        candidate.phase_limit = phase_limit;
        candidate.soft_knee = soft_knee;
        polar_candidates.push_back({candidate, evaluate(candidate)});
      }
    }
  }
  std::sort(polar_candidates.begin(), polar_candidates.end(),
            [](const Candidate &a, const Candidate &b) {
              return a.evaluation.correlation > b.evaluation.correlation;
            });
  const size_t results = std::min<size_t>(30, polar_candidates.size());
  for (size_t index = 0; index < results; ++index) {
    result("joint_finalist", "rank", static_cast<float>(index + 1),
           polar_candidates[index].parameters,
           &polar_candidates[index].evaluation);
  }
}

int main(int argc, char **argv) {
  input.reserve(wav_data_len / 2);
  for (uint32_t index = 0; index + 1 < wav_data_len; index += 2) {
    input.push_back((wav_data[index] + wav_data[index + 1]) /
                    (2.0f * 32768.0f));
  }

  std::printf("path,stage,parameter,value,correlation,lag_samples,"
              "preemphasis,clip_level,agc_target,agc_max_gain,phase_limit,"
              "soft_knee,low_level_phase_rms,low_level_phase_peak,"
              "low_level_fraction\n");
  Parameters baseline;
  if (argc == 2 && std::strcmp(argv[1], "--cleanup") == 0)
    run_cleanup_sweep();
  else if (argc == 2 && std::strcmp(argv[1], "--joint") == 0)
    run_joint_sweep(baseline);
  else
    run_one_factor(baseline);
}
