// ── Full Offline SSB Loopback Simulator ──────────────────────────────────────
//
// Mirrors the exact firmware DSP chain:
//   1. Read input WAV (any sample rate, resampled to SAMPLE_RATE)
//   2. Speech DSP: AGC → clip → filter (same as speech_dsp.cpp)
//   3. Polar modulation: Hilbert → AM/PM extraction (same as polar_mod.cpp)
//   4. Hardware simulation: 10-bit PWM quantization, sigma-delta, smoothing
//   5. SSB demodulation: amplitude × cos(accumulated_phase)
//   6. Write output WAV + print quality metrics
//
// Usage:
//   ./simulate_loopback [input.wav]          # reads WAV file
//   ./simulate_loopback                      # uses embedded audio_data.h
//
// Build:
//   g++ -O2 -DSAMPLE_RATE=16000 -DUSE_SPEECH_DSP=1 -I../src \
//       simulate_loopback.cpp ../src/polar_mod.cpp ../src/speech_dsp.cpp \
//       -lm -o simulate_loopback

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <vector>

#include "polar_mod.h"

#if USE_SPEECH_DSP
#include "speech_dsp.h"
#endif

#include "../src/audio_data.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef SAMPLE_RATE
#define SAMPLE_RATE 16000
#endif

#ifndef SIMULATE_QUANTIZATION
#define SIMULATE_QUANTIZATION 1
#endif

// ── WAV Reader ───────────────────────────────────────────────────────────────

struct WavData {
    std::vector<float> samples;
    int sample_rate;
    int channels;
    int bits_per_sample;
};

static uint32_t read_u32(FILE *f) {
    uint8_t b[4];
    fread(b, 1, 4, f);
    return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);
}

static uint16_t read_u16(FILE *f) {
    uint8_t b[2];
    fread(b, 1, 2, f);
    return b[0] | (b[1] << 8);
}

bool read_wav(const char *path, WavData &wav) {
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return false; }

    // RIFF header
    uint8_t riff[4]; fread(riff, 1, 4, f);
    if (memcmp(riff, "RIFF", 4) != 0) { fclose(f); return false; }
    read_u32(f); // file size
    uint8_t wave[4]; fread(wave, 1, 4, f);
    if (memcmp(wave, "WAVE", 4) != 0) { fclose(f); return false; }

    uint16_t fmt_tag = 0;
    uint16_t nch = 0;
    uint32_t sr = 0;
    uint16_t bps = 0;
    bool got_fmt = false;

    // Find chunks
    while (!feof(f)) {
        uint8_t id[4];
        if (fread(id, 1, 4, f) != 4) break;
        uint32_t chunk_size = read_u32(f);

        if (memcmp(id, "fmt ", 4) == 0) {
            long chunk_start = ftell(f);
            fmt_tag = read_u16(f);
            nch = read_u16(f);
            sr = read_u32(f);
            read_u32(f); // byte rate
            read_u16(f); // block align
            bps = read_u16(f);
            got_fmt = true;
            fseek(f, chunk_start + chunk_size, SEEK_SET);
        } else if (memcmp(id, "data", 4) == 0 && got_fmt) {
            wav.sample_rate = sr;
            wav.channels = nch;
            wav.bits_per_sample = bps;

            if (fmt_tag == 1 && bps == 16) {
                // PCM 16-bit
                int n_samples = chunk_size / 2;
                int n_frames = n_samples / nch;
                wav.samples.resize(n_frames);
                for (int i = 0; i < n_frames; i++) {
                    int16_t s = (int16_t)read_u16(f);
                    wav.samples[i] = (float)s / 32768.0f;
                    // Skip extra channels
                    for (int c = 1; c < nch; c++) read_u16(f);
                }
            } else if (fmt_tag == 3 && bps == 32) {
                // Float 32-bit
                int n_frames = chunk_size / (4 * nch);
                wav.samples.resize(n_frames);
                for (int i = 0; i < n_frames; i++) {
                    float s;
                    fread(&s, 4, 1, f);
                    wav.samples[i] = s;
                    // Skip extra channels
                    for (int c = 1; c < nch; c++) { float tmp; fread(&tmp, 4, 1, f); }
                }
            } else if (fmt_tag == 1 && bps == 24) {
                // PCM 24-bit
                int n_frames = chunk_size / (3 * nch);
                wav.samples.resize(n_frames);
                for (int i = 0; i < n_frames; i++) {
                    uint8_t b[3]; fread(b, 1, 3, f);
                    int32_t s = (b[0] << 8) | (b[1] << 16) | (b[2] << 24);
                    wav.samples[i] = (float)s / 2147483648.0f;
                    for (int c = 1; c < nch; c++) fread(b, 1, 3, f);
                }
            } else {
                fprintf(stderr, "Unsupported WAV format: tag=%d bps=%d\n", fmt_tag, bps);
                fclose(f); return false;
            }
            fclose(f);
            return true;
        } else {
            // Skip unknown chunk
            fseek(f, chunk_size, SEEK_CUR);
        }
    }
    fclose(f);
    return false;
}

// ── WAV Writer ───────────────────────────────────────────────────────────────

bool write_wav(const char *path, const std::vector<float> &samples, int sample_rate) {
    FILE *f = fopen(path, "wb");
    if (!f) return false;

    uint32_t n = (uint32_t)samples.size();
    uint32_t data_size = n * 2;  // 16-bit mono
    uint32_t file_size = 36 + data_size;

    // RIFF header
    fwrite("RIFF", 1, 4, f);
    fwrite(&file_size, 4, 1, f);
    fwrite("WAVE", 1, 4, f);

    // fmt chunk
    fwrite("fmt ", 1, 4, f);
    uint32_t fmt_size = 16;
    fwrite(&fmt_size, 4, 1, f);
    uint16_t fmt_tag = 1;       // PCM
    uint16_t nch = 1;
    uint32_t sr = sample_rate;
    uint32_t byte_rate = sr * 2;
    uint16_t block_align = 2;
    uint16_t bps = 16;
    fwrite(&fmt_tag, 2, 1, f);
    fwrite(&nch, 2, 1, f);
    fwrite(&sr, 4, 1, f);
    fwrite(&byte_rate, 4, 1, f);
    fwrite(&block_align, 2, 1, f);
    fwrite(&bps, 2, 1, f);

    // data chunk
    fwrite("data", 1, 4, f);
    fwrite(&data_size, 4, 1, f);
    for (uint32_t i = 0; i < n; i++) {
        float s = samples[i];
        if (s > 1.0f) s = 1.0f;
        if (s < -1.0f) s = -1.0f;
        int16_t v = (int16_t)(s * 32767.0f);
        fwrite(&v, 2, 1, f);
    }
    fclose(f);
    return true;
}

// ── Resampler (linear interpolation) ─────────────────────────────────────────

std::vector<float> resample(const std::vector<float> &in, int in_rate, int out_rate) {
    if (in_rate == out_rate) return in;

    double ratio = (double)in_rate / (double)out_rate;
    int out_len = (int)((double)in.size() / ratio);
    std::vector<float> out(out_len);

    for (int i = 0; i < out_len; i++) {
        double pos = i * ratio;
        int idx = (int)pos;
        double frac = pos - idx;
        if (idx + 1 < (int)in.size())
            out[i] = (float)((1.0 - frac) * in[idx] + frac * in[idx + 1]);
        else
            out[i] = in[idx < (int)in.size() ? idx : (int)in.size() - 1];
    }
    return out;
}

// ── Quality Metrics ──────────────────────────────────────────────────────────

void print_metrics(const std::vector<float> &input, const std::vector<float> &output,
                   int sample_rate) {
    int n_in = (int)input.size();
    int n_out = (int)output.size();

    // RMS and crest factor
    auto calc_stats = [](const std::vector<float> &s, const char *name) {
        if (s.empty()) return;
        float peak = 0, sum_sq = 0;
        for (float v : s) {
            float a = fabsf(v);
            if (a > peak) peak = a;
            sum_sq += v * v;
        }
        float rms = sqrtf(sum_sq / s.size());
        float peak_db = peak > 0 ? 20.0f * log10f(peak) : -99.0f;
        float rms_db = rms > 0 ? 20.0f * log10f(rms) : -99.0f;
        float crest = (rms > 0 && peak > 0) ? 20.0f * log10f(peak / rms) : 0.0f;
        printf("  %-12s Peak: %+6.1f dBFS  RMS: %+6.1f dBFS  Crest: %.1f dB\n",
               name, peak_db, rms_db, crest);
    };

    printf("\n=== LEVEL ANALYSIS ===\n");
    calc_stats(input, "Input:");
    calc_stats(output, "Output:");

    // Spectral energy by band (simple DFT on windowed segments)
    auto band_energy = [](const std::vector<float> &s, int sr,
                          int f_lo, int f_hi) -> float {
        int N = 1024;
        if ((int)s.size() < N) return 0;
        float total = 0;
        int n_win = 0;
        for (int start = 0; start + N <= (int)s.size(); start += (int)s.size() / 8) {
            for (int k = f_lo * N / sr; k <= f_hi * N / sr && k < N / 2; k++) {
                if (k < 1) continue;
                float re = 0, im = 0;
                for (int n = 0; n < N; n++) {
                    float w = 0.5f * (1.0f - cosf(2.0f * M_PI * n / N));
                    float v = s[start + n] * w;
                    re += v * cosf(2.0f * M_PI * k * n / (float)N);
                    im += v * sinf(2.0f * M_PI * k * n / (float)N);
                }
                total += re * re + im * im;
            }
            n_win++;
        }
        return n_win > 0 ? total / n_win : 0;
    };

    printf("\n=== SPECTRAL ENERGY (relative to 1-2kHz) ===\n");
    printf("  %-12s %10s %10s %10s\n", "Band", "Input", "Output", "Delta");
    printf("  -------------------------------------------\n");

    struct Band { const char *name; int lo, hi; };
    Band bands[] = {
        {"0-300Hz", 0, 300}, {"300-1kHz", 300, 1000}, {"1-2kHz", 1000, 2000},
        {"2-3kHz", 2000, 3000}, {"3-4kHz", 3000, 4000}
    };

    float ref_in = band_energy(input, sample_rate, 1000, 2000);
    float ref_out = band_energy(output, sample_rate, 1000, 2000);
    if (ref_in == 0) ref_in = 1;
    if (ref_out == 0) ref_out = 1;

    for (auto &b : bands) {
        float e_in = band_energy(input, sample_rate, b.lo, b.hi);
        float e_out = band_energy(output, sample_rate, b.lo, b.hi);
        float db_in = e_in > 0 ? 10.0f * log10f(e_in / ref_in) : -99.0f;
        float db_out = e_out > 0 ? 10.0f * log10f(e_out / ref_out) : -99.0f;
        printf("  %-12s %+8.1f dB %+8.1f dB %+8.1f dB\n",
               b.name, db_in, db_out, db_out - db_in);
    }

    // Cross-correlation to find best alignment and similarity
    // Use the shorter signal length, search ±500 samples for peak
    int len = n_out < n_in ? n_out : n_in;
    int search = 500;
    if (search > len / 4) search = len / 4;

    // Account for Hilbert filter delay: 63 samples
    int expected_delay = 63;
    float best_corr = -1e30f;
    int best_lag = 0;

    // Compute input and output energy for normalization
    float e_in_total = 0, e_out_total = 0;
    int corr_len = len - 2 * search;
    if (corr_len < 100) corr_len = 100;
    for (int i = 0; i < corr_len && i + search < len; i++) {
        e_in_total += input[i + search] * input[i + search];
        e_out_total += output[i + search] * output[i + search];
    }

    for (int lag = expected_delay - search; lag < expected_delay + search; lag++) {
        float corr = 0;
        int count = 0;
        for (int i = 0; i < corr_len; i++) {
            int i_in = i + search;
            int i_out = i + search + lag;
            if (i_in >= 0 && i_in < n_in && i_out >= 0 && i_out < n_out) {
                corr += input[i_in] * output[i_out];
                count++;
            }
        }
        if (count > 0) corr /= count;
        if (corr > best_corr) {
            best_corr = corr;
            best_lag = lag;
        }
    }

    // Normalize correlation
    float norm = sqrtf((e_in_total / corr_len) * (e_out_total / corr_len));
    float norm_corr = norm > 0 ? best_corr / norm : 0;

    printf("\n=== SIMILARITY ===\n");
    printf("  Best alignment lag: %d samples (%.2f ms)\n",
           best_lag, 1000.0f * best_lag / sample_rate);
    printf("  Normalized cross-correlation: %.4f (1.0 = perfect)\n", norm_corr);

    // SNR estimate: align signals and compute signal vs error power
    float sig_power = 0, err_power = 0;
    int snr_count = 0;
    // Find output scaling factor (output may have different gain)
    float scale_num = 0, scale_den = 0;
    for (int i = 0; i < corr_len; i++) {
        int i_in = i + search;
        int i_out = i + search + best_lag;
        if (i_in >= 0 && i_in < n_in && i_out >= 0 && i_out < n_out) {
            scale_num += input[i_in] * output[i_out];
            scale_den += input[i_in] * input[i_in];
        }
    }
    float scale = scale_den > 0 ? scale_num / scale_den : 1.0f;

    for (int i = 0; i < corr_len; i++) {
        int i_in = i + search;
        int i_out = i + search + best_lag;
        if (i_in >= 0 && i_in < n_in && i_out >= 0 && i_out < n_out) {
            float ref = input[i_in] * scale;  // scaled reference
            float err = output[i_out] - ref;
            sig_power += ref * ref;
            err_power += err * err;
            snr_count++;
        }
    }

    float snr = (err_power > 0 && snr_count > 0) ?
        10.0f * log10f(sig_power / err_power) : 99.0f;
    printf("  Estimated SNR: %.1f dB\n", snr);
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char *argv[]) {
    std::vector<float> input_audio;
    int input_rate = SAMPLE_RATE;

    // ── Load input audio ─────────────────────────────────────────────────────
    if (argc > 1) {
        WavData wav;
        if (!read_wav(argv[1], wav)) {
            fprintf(stderr, "Failed to read %s\n", argv[1]);
            return 1;
        }
        input_audio = wav.samples;
        input_rate = wav.sample_rate;
        printf("Loaded %s: %d samples @ %d Hz (%d-bit)\n",
               argv[1], (int)wav.samples.size(), wav.sample_rate, wav.bits_per_sample);
    } else {
        // Use embedded audio data
        printf("No input file specified, using embedded audio_data.h\n");
        input_audio.resize(wav_data_len);
        for (uint32_t i = 0; i < wav_data_len; i++)
            input_audio[i] = (float)wav_data[i] / 32768.0f;
        input_rate = 32000;  // audio_data.h is stored at 32kHz
        printf("Loaded embedded audio: %u samples @ %d Hz\n", wav_data_len, input_rate);
    }

    // ── Resample to target rate ──────────────────────────────────────────────
    std::vector<float> audio = resample(input_audio, input_rate, SAMPLE_RATE);
    printf("Resampled to %d Hz: %d samples (%.2f seconds)\n",
           SAMPLE_RATE, (int)audio.size(), (float)audio.size() / SAMPLE_RATE);

    // ── Keep a copy of the input (at target rate) for quality comparison ─────
    std::vector<float> input_resampled = audio;

    // ── Reset all DSP state ──────────────────────────────────────────────────
    polar_mod_reset();
#if USE_SPEECH_DSP
    speech_dsp_reset();
#endif
    modulation_mode = MOD_USB;

    // ── Process through firmware DSP chain ───────────────────────────────────
    uint32_t n = (uint32_t)audio.size();
    std::vector<float> output(n);
    double phase_accu = 0;

    // Hardware simulation state
    float smooth_ampl = 0.0f;
    float sd_error = 0.0f;
    const float mod_depth = 0.75f;  // Matches main.cpp

    printf("\nProcessing %u samples through DSP chain...\n", n);
    printf("  Speech DSP: %s\n", USE_SPEECH_DSP ? "ON" : "OFF");
    printf("  Quantization sim: %s\n", SIMULATE_QUANTIZATION ? "ON" : "OFF");
    printf("  Modulation depth: %.2f\n", mod_depth);

    for (uint32_t i = 0; i < n; i++) {
        float sample = audio[i];

        // ── Step 1: Speech DSP (AGC + clip + filter) ─────────────────────────
#if USE_SPEECH_DSP
        sample = speech_process(sample);
#endif

        // ── Step 2: Polar modulation (Hilbert → AM/PM) ──────────────────────
        float f_ampl, f_phase_diff;
        modulation_am_pm_f(sample, &f_ampl, &f_phase_diff);

        // ── Step 3: Hardware simulation ──────────────────────────────────────
        float hw_ampl;

#if SIMULATE_QUANTIZATION
        // Scale amplitude with modulation depth (same as main.cpp)
        float target_ampl = f_ampl * 65535.0f * mod_depth;
        if (target_ampl > 65535.0f) target_ampl = 65535.0f;
        if (target_ampl < 0.0f) target_ampl = 0.0f;

        // Amplitude smoothing (same 0.15/0.85 as main.cpp)
        smooth_ampl = smooth_ampl * 0.15f + target_ampl * 0.85f;
        int next_ampl = (int)smooth_ampl;

        // Sigma-delta noise shaping + 10-bit quantization
        float sd_target = (float)next_ampl / 64.0f + sd_error;
        int sd_pwm = (int)(sd_target + 0.5f);
        if (sd_pwm < 0) sd_pwm = 0;
        if (sd_pwm > 1023) sd_pwm = 1023;
        sd_error = sd_target - (float)sd_pwm;

        // Convert back to normalized amplitude (0.0 - 1.0)
        hw_ampl = (float)sd_pwm / 1023.0f;
#else
        // Ideal (no quantization): just scale by mod_depth
        hw_ampl = f_ampl * mod_depth;
        if (hw_ampl > 1.0f) hw_ampl = 1.0f;
#endif

        // ── Step 4: Phase accumulation (double precision, same as firmware) ──
        phase_accu += (double)f_phase_diff;

        // ── Step 5: SSB demodulation (coherent product detection) ────────────
        // Equivalent to receiver multiplying by local oscillator at carrier freq
        output[i] = hw_ampl * (float)cos(phase_accu);
    }

    // ── Normalize output to use full range ───────────────────────────────────
    float out_peak = 0;
    for (uint32_t i = 0; i < n; i++) {
        float a = fabsf(output[i]);
        if (a > out_peak) out_peak = a;
    }
    if (out_peak > 0) {
        float gain = 0.95f / out_peak;  // Leave a bit of headroom
        for (uint32_t i = 0; i < n; i++)
            output[i] *= gain;
    }

    // ── Write output WAV ─────────────────────────────────────────────────────
    const char *out_path = "loopback_out.wav";
    if (write_wav(out_path, output, SAMPLE_RATE)) {
        printf("\nOutput saved to %s (%d Hz, %d samples)\n",
               out_path, SAMPLE_RATE, (int)output.size());
    } else {
        fprintf(stderr, "Failed to write %s\n", out_path);
    }

    // ── Also save speech-DSP-only output (before modulation) for comparison ──
    // Re-run just the speech DSP to get the "ideal" processed audio
    polar_mod_reset();
#if USE_SPEECH_DSP
    speech_dsp_reset();
    std::vector<float> speech_only(n);
    for (uint32_t i = 0; i < n; i++)
        speech_only[i] = speech_process(input_resampled[i]);

    if (write_wav("speech_dsp_out.wav", speech_only, SAMPLE_RATE)) {
        printf("Speech DSP only saved to speech_dsp_out.wav\n");
    }
#endif

    // ── Print quality metrics ────────────────────────────────────────────────
    print_metrics(input_resampled, output, SAMPLE_RATE);

    printf("\nDone.\n");
    return 0;
}
