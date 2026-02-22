import matplotlib.pyplot as plt
import pandas as pd

# Load the sweep data (manual load since it's a simple list)
# SweepFreq(Hz), Wanted(dB), Unwanted(dB), Suppression(dB)
data = [
    [300, 46.98, 28.35, 18.63],
    [600, 47.26, -4.32, 51.58],
    [900, 47.31, -56.90, 104.20],
    [1200, 48.01, -39.55, 87.56],
    [1500, 48.25, -36.62, 84.87],
    [1800, 48.05, -38.74, 86.79],
    [2100, 47.39, -43.55, 90.94],
    [2400, 47.41, -50.14, 97.55],
    [2700, 48.11, -62.48, 110.59],
    [3000, 48.36, -64.01, 112.37]
]
df = pd.DataFrame(data, columns=['Freq', 'Wanted', 'Unwanted', 'Suppression'])

plt.figure(figsize=(10, 6))
plt.plot(df['Freq'], df['Suppression'], marker='o', linestyle='-', color='g')
plt.title('Opposite Sideband Suppression (63-tap FIR Hilbert)')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Suppression (dB)')
plt.grid(True)
plt.ylim(0, 120)
plt.savefig('osb_sweep.png')
print("Sweep plot saved to osb_sweep.png")
