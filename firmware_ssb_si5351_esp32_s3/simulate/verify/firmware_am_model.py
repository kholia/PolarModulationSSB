"""Model of the drain-AM firmware timing (LEDC-locked ticks + hardware fades).

Reads a dump_dsp_stream two-tone stream and simulates exactly what the TX loop
does per sample:
  * sample ticks on PWM boundaries (10 x 3.125 us at 32 ksample/s);
  * the Si5351 frequency step lands when the variable-length I2C write STOPs;
  * the envelope is a staircase of 3.125 us PWM periods: one linear hardware
    fade per sample in 1/16-LSB units of a 7-bit duty, with the integer-scale
    remainder carried (drain_plan_ramp), latched on PWM boundary `boundary`;
  * delay D splits into whole-sample history on one path plus that boundary
    (drain_split_delay).
Prints two-tone IMD and out-of-band power versus D.

  firmware_am_model.py STREAM [D,D,...]     env: RATE, MOD_DEPTH, INT_DUTY=1
"""
import os, sys, numpy as np
from timing_model import ms_div_fw, pll, bdiffs, writes, C, XTAL

RATE = int(os.environ.get("RATE", "32000"))
STEPS = 320000 // RATE
STEP_US = 1e6 / 320000
T = 1e6 / RATE
SCL = 2.0                      # I2C bits per microsecond
F = 14_200_000
DUTY_MAX_Q4 = 127 << 4
MOD_DEPTH = float(os.environ.get("MOD_DEPTH", "0.75"))
INT_DUTY = os.environ.get("INT_DUTY") == "1"   # ignore LEDC fractional dither
DT = STEP_US / 8               # 0.39 us simulation grid

def split_delay(d):
    q = (d - 1) // STEPS
    return max(q, 0), max(-q, 0), d - q * STEPS

def plan_ramp(duty, target):
    target = min(max(target, 0), DUTY_MAX_Q4)
    delta = target - duty
    scale = min(abs(delta) // STEPS, 1023)
    return duty + (scale * STEPS if delta >= 0 else -scale * STEPS), (1 if delta >= 0 else -1) * scale

def simulate(amp, ph, d):
    n = len(ph)
    a_samp, p_samp, boundary = split_delay(d)
    div = ms_div_fw(F); p1, p2 = pll(F, div)
    factor = C * div * RATE / (2 * np.pi * XTAL)
    bd = bdiffs(ph, factor)
    targets = np.floor(np.clip(amp * MOD_DEPTH, 0, 1) * DUTY_MAX_Q4 + 0.5).astype(int)
    # Value used at tick i is DSP output i-1-delay (0 before history exists).
    def hist(arr, i, delay):
        j = i - 1 - delay
        return arr[j] if j >= 0 else 0
    phase_vals = np.array([hist(bd, i, p_samp) for i in range(n)])
    lengths, _ = writes(p1, p2, phase_vals)
    grid = int(round(n * T / DT))
    # Frequency (p2 units) from STOP of each write.
    freq_change = np.zeros(grid + 1)
    prev = 0
    for i in range(n):
        start = i * T + (0.6 if boundary == 1 else 0.0)   # fade goes out first
        t_stop = start + ((2 + lengths[i]) * 9 + 1) / SCL
        k = min(int(round(t_stop / DT)), grid)
        v = 128 * phase_vals[i]
        freq_change[k] += v - prev; prev = v
    freq = np.cumsum(freq_change)[:grid]
    phi = np.cumsum(freq) * DT / (128 * factor * T)
    # Envelope staircase per PWM period.
    duty_per_period = np.zeros(n * STEPS + STEPS * 2)
    duty = 0; current = np.zeros_like(duty_per_period)
    for i in range(n):
        start_period = i * STEPS + boundary
        start, scale_signed = duty, 0
        duty, scale_signed = plan_ramp(duty, hist(targets, i, a_samp))
        for m in range(STEPS):
            current[start_period + m] = start + scale_signed * m
    # Fill before first fade with zero; each period holds its duty value.
    per = current[: n * STEPS]
    if INT_DUTY:
        per = np.floor(per / 16) * 16
    env = np.repeat(per / DUTY_MAX_Q4, int(round(STEP_US / DT)))[:grid]
    x = env * np.exp(1j * phi)
    x = x[int(0.25e6 / DT):]
    dec = 16
    x = x[: (len(x) // dec) * dec].reshape(-1, dec).mean(1)
    fs = 1e6 / (DT * dec)
    X = np.abs(np.fft.fftshift(np.fft.fft(x * np.blackman(len(x))))) ** 2
    fr = np.fft.fftshift(np.fft.fftfreq(len(x), 1 / fs))
    band = lambda lo, hi: X[(fr >= lo) & (fr < hi)].sum()
    tone = max(band(690, 710), band(1890, 1910))
    imd3 = max(band(-510, -490), band(3090, 3110))
    imd5 = max(band(-1710, -1690), band(4290, 4310))
    oob = band(-20000, 200) + band(3000, 20000)
    return (10 * np.log10(imd3 / tone), 10 * np.log10(imd5 / tone),
            10 * np.log10(oob / X.sum()))

if __name__ == "__main__":
    data = np.loadtxt(sys.argv[1])
    delays = ([int(v) for v in sys.argv[2].split(",")] if len(sys.argv) > 2
              else list(range(-10, 21, 2)))
    print(f"{RATE} sample/s, {STEPS} PWM periods/sample, mod depth {MOD_DEPTH}, "
          f"{'integer' if INT_DUTY else '1/16-LSB'} duty")
    for d in delays:
        a, p, b = split_delay(d)
        i3, i5, oob = simulate(data[:, 0], data[:, 1], d)
        print(f"D={d:+3d} ({d * STEP_US:+6.2f} us; amp+{a} phase+{p} boundary {b:2d}): "
              f"IMD3 {i3:6.1f} dBc  IMD5 {i5:6.1f} dBc  out-of-band {oob:6.1f} dB")
