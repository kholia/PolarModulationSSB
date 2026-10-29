# Firmware get_two_tone_sample(): t = (float)index / rate; sinf(2*pi*f*t) in float32.
# Measures the phase error of that time base versus exact math, after N seconds of TX.
import numpy as np
rate=32000
for secs in (1,10,60,300):
    i=np.arange(secs*rate-rate//10, secs*rate, dtype=np.int64)
    t32=i.astype(np.float32)/np.float32(rate)
    arg32=np.float32(2*np.pi)*np.float32(1900)*t32          # float32 product chain as in firmware
    exact=2*np.pi*1900*i/rate
    err=np.angle(np.exp(1j*(arg32.astype(np.float64)-exact)))
    rms=np.sqrt(np.mean(err**2))
    print(f"after {secs:4d} s: 1900 Hz tone phase error rms {rms*1e3:7.3f} mrad -> noise/spur floor ~{20*np.log10(rms+1e-12):6.1f} dBc")
