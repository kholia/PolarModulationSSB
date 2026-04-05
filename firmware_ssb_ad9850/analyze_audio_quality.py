#!/usr/bin/env python3
"""
Analyze SSB transmission quality by comparing original and recorded audio.
Computes objective quality metrics: SNR, frequency response, spectral analysis, etc.
"""

import numpy as np
import soundfile as sf
from scipy import signal
from scipy.fft import fft, fftfreq
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')

def load_and_resample(filepath):
    """Load audio file and return signal data."""
    data, samplerate = sf.read(filepath)
    if len(data.shape) > 1:
        data = data.mean(axis=1)  # Convert to mono
    return data, samplerate

def compute_sn(original, recorded, samplerate):
    """Compute Signal-to-Noise Ratio."""
    # Align signals using cross-correlation
    # Use shorter signal for correlation speed
    min_len = min(len(original), len(recorded))
    orig_short = original[:min_len]
    rec_short = recorded[:min_len]

    correlation = signal.correlate(rec_short, orig_short, mode='full')
    lag = np.argmax(np.abs(correlation)) - len(orig_short) + 1

    # Truncate to same length after alignment
    if lag >= 0:
        orig_aligned = original[:min_len]
        rec_aligned = recorded[lag:lag+min_len]
    else:
        orig_aligned = original[-lag:min_len-lag]
        rec_aligned = recorded[:min_len]

    # Make sure they're the same length
    actual_len = min(len(orig_aligned), len(rec_aligned))
    orig_aligned = orig_aligned[:actual_len]
    rec_aligned = rec_aligned[:actual_len]

    # Normalize both signals
    orig_norm = orig_aligned / np.max(np.abs(orig_aligned))
    rec_norm = rec_aligned / np.max(np.abs(rec_aligned))

    # Compute noise as difference
    noise = orig_norm - rec_norm
    snr = 10 * np.log10(np.sum(orig_norm**2) / np.sum(noise**2 + 1e-10))

    return snr, orig_norm, rec_norm

def analyze_frequency_content(sig, samplerate, label):
    """Analyze frequency content and bandwidth."""
    # Compute FFT
    n = len(sig)
    yf = fft(sig[:min(n, 8192)])  # Use first 8192 samples for speed
    xf = fftfreq(8192, 1/samplerate)[:4096]

    magnitude = 2.0/n * np.abs(yf[:4096])

    # Find dominant frequencies
    peaks, properties = signal.find_peaks(magnitude, height=np.max(magnitude)*0.1)

    # Compute spectral centroid (brightness indicator)
    spectral_centroid = np.sum(xf * magnitude) / np.sum(magnitude + 1e-10)

    return xf, magnitude, peaks, spectral_centroid

def compute_thd(sig, samplerate):
    """Estimate Total Harmonic Distortion (simplified)."""
    # Find fundamental frequency
    n = min(len(sig), 8192)
    yf = np.abs(fft(sig[:n]))
    xf = fftfreq(n, 1/samplerate)

    # Find peak (fundamental)
    pos_freqs = xf[:n//2] > 50  # Ignore DC and very low frequencies
    peak_idx = np.argmax(yf[:n//2] * pos_freqs[:n//2])
    fundamental_freq = xf[peak_idx]

    if fundamental_freq < 100 or fundamental_freq > 4000:
        return None, fundamental_freq  # Not a strong tone

    # Look for harmonics
    fundamental_power = yf[peak_idx]**2
    harmonic_power = 0
    for h in range(2, 6):  # Check 2nd through 5th harmonics
        harm_freq = fundamental_freq * h
        if harm_freq > samplerate/2:
            break
        harm_idx = np.argmin(np.abs(xf - harm_freq))
        if harm_idx < n//2:
            harmonic_power += yf[harm_idx]**2

    thd = 10 * np.log10(harmonic_power / (fundamental_power + 1e-10))
    return thd, fundamental_freq

def dynamic_range(sig):
    """Compute dynamic range in dB."""
    max_val = np.max(np.abs(sig))
    # Estimate noise floor as 10th percentile
    noise_floor = np.percentile(np.abs(sig), 10)

    if noise_floor < 1e-10:
        return 120  # Very low noise floor

    dr = 20 * np.log10(max_val / (noise_floor + 1e-10))
    return min(dr, 120)  # Cap at 120 dB

def plot_comparison(original, recorded, samplerate, snr, orig_xf, orig_mag, rec_xf, rec_mag):
    """Create comparison plots."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('SSB Transmission Quality Analysis', fontsize=16, fontweight='bold')

    # Waveform comparison (first 0.1 seconds)
    n_samples = int(0.1 * samplerate)
    t = np.arange(n_samples) / samplerate

    axes[0, 0].plot(t, original[:n_samples], 'b-', alpha=0.7, linewidth=0.5)
    axes[0, 0].set_title('Original Signal (0.1s)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Amplitude')
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(t, recorded[:n_samples], 'r-', alpha=0.7, linewidth=0.5)
    axes[0, 1].set_title('Recorded SSB Signal (0.1s)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Amplitude')
    axes[0, 1].grid(True, alpha=0.3)

    # Spectrum comparison
    axes[1, 0].plot(orig_xf, orig_mag, 'b-', alpha=0.7, linewidth=0.5)
    axes[1, 0].set_title('Original Spectrum')
    axes[1, 0].set_xlabel('Frequency (Hz)')
    axes[1, 0].set_ylabel('Magnitude')
    axes[1, 0].set_xlim(0, 5000)
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(rec_xf, rec_mag, 'r-', alpha=0.7, linewidth=0.5)
    axes[1, 1].set_title('Recorded SSB Spectrum')
    axes[1, 1].set_xlabel('Frequency (Hz)')
    axes[1, 1].set_ylabel('Magnitude')
    axes[1, 1].set_xlim(0, 5000)
    axes[1, 1].grid(True, alpha=0.3)

    # Overlay comparison
    axes[2, 0].plot(orig_xf, orig_mag, 'b-', alpha=0.5, linewidth=0.5, label='Original')
    axes[2, 0].plot(rec_xf, rec_mag, 'r-', alpha=0.5, linewidth=0.5, label='Recorded')
    axes[2, 0].set_title('Spectrum Overlay')
    axes[2, 0].set_xlabel('Frequency (Hz)')
    axes[2, 0].set_ylabel('Magnitude')
    axes[2, 0].set_xlim(0, 5000)
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)

    # Metrics summary
    axes[2, 1].axis('off')
    metrics_text = f"""
    QUALITY METRICS
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    SNR: {snr:.1f} dB

    Original Dynamic Range: {dynamic_range(original):.1f} dB
    Recorded Dynamic Range: {dynamic_range(recorded):.1f} dB

    Original Samples: {len(original)}
    Recorded Samples: {len(recorded)}

    Sample Rate: {samplerate} Hz

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    ASSESSMENT:
    """

    if snr > 20:
        assessment = "EXCELLENT - High quality SSB"
    elif snr > 15:
        assessment = "GOOD - Acceptable quality"
    elif snr > 10:
        assessment = "FAIR - Some degradation"
    else:
        assessment = "POOR - Significant issues"

    metrics_text += f"    {assessment}"

    axes[2, 1].text(0.1, 0.9, metrics_text, transform=axes[2, 1].transAxes,
                    fontsize=10, verticalalignment='top',
                    fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    plt.savefig('ssb_quality_analysis.png', dpi=150, bbox_inches='tight')
    print("Saved analysis plot to ssb_quality_analysis.png")

def main():
    print("Loading audio files...")

    try:
        original, sr_orig = load_and_resample('original.wav')
        recorded, sr_rec = load_and_resample('Recording-Winston-OTA-3.wav')

        # Resample recorded to match original if needed
        if sr_orig != sr_rec:
            print(f"Resampling from {sr_rec} to {sr_orig} Hz...")
            num_samples = int(len(recorded) * sr_orig / sr_rec)
            recorded = signal.resample(recorded, num_samples)

        samplerate = sr_orig
        print(f"Original: {len(original)} samples @ {sr_orig} Hz ({len(original)/sr_orig:.2f}s)")
        print(f"Recorded: {len(recorded)} samples @ {sr_rec} Hz ({len(recorded)/sr_orig:.2f}s)")

        # Compute SNR
        print("\nComputing SNR...")
        snr, orig_aligned, rec_aligned = compute_sn(original, recorded, samplerate)
        print(f"SNR: {snr:.1f} dB")

        # Frequency analysis
        print("\nAnalyzing frequency content...")
        orig_xf, orig_mag, orig_peaks, orig_centroid = analyze_frequency_content(original, samplerate, "Original")
        rec_xf, rec_mag, rec_peaks, rec_centroid = analyze_frequency_content(recorded, samplerate, "Recorded")

        print(f"Original spectral centroid: {orig_centroid:.0f} Hz")
        print(f"Recorded spectral centroid: {rec_centroid:.0f} Hz")

        # Dynamic range
        print(f"\nOriginal dynamic range: {dynamic_range(original):.1f} dB")
        print(f"Recorded dynamic range: {dynamic_range(recorded):.1f} dB")

        # THD estimation
        thd_orig, fund_orig = compute_thd(original, samplerate)
        thd_rec, fund_rec = compute_thd(recorded, samplerate)

        if thd_orig is not None:
            print(f"\nOriginal THD: {thd_orig:.1f} dB (fundamental: {fund_orig:.0f} Hz)")
        if thd_rec is not None:
            print(f"Recorded THD: {thd_rec:.1f} dB (fundamental: {fund_rec:.0f} Hz)")

        # Generate plots
        print("\nGenerating comparison plots...")
        plot_comparison(original, recorded, samplerate, snr,
                       orig_xf, orig_mag, rec_xf, rec_mag)

        # Final assessment
        print("\n" + "="*50)
        print("QUALITY ASSESSMENT")
        print("="*50)

        if snr > 20:
            print("✅ EXCELLENT quality - SSB transmission is high fidelity")
            print("   Your system is performing very well.")
        elif snr > 15:
            print("✅ GOOD quality - Acceptable for communications")
            print("   Minor improvements possible, but functional.")
        elif snr > 10:
            print("⚠️  FAIR quality - Noticeable degradation")
            print("   Consider improvements listed in recommendations.")
        else:
            print("❌ POOR quality - Significant issues detected")
            print("   Review transmission chain for problems.")

        print("\nRecommendations file: ssb_improvement_recommendations.txt")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
