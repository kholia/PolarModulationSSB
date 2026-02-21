import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Load the data
df = pd.read_csv('output.csv')
df.columns = ['Freq', 'Mag']

# Sort by frequency for plotting
df = df.sort_values('Freq')

plt.figure(figsize=(10, 6))
plt.plot(df['Freq'], df['Mag'])
plt.title('SSB Simulation FFT (Two-Tone: 700Hz, 1900Hz)')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude (dB)')
plt.grid(True)
plt.xlim(-4000, 4000) # Focus on the signal area
plt.ylim(df['Mag'].max() - 120, df['Mag'].max() + 5) # 120dB range

# Mark the peaks
peaks = df.nlargest(2, 'Mag')
for i, row in peaks.iterrows():
    plt.annotate(f"{row['Freq']:.1f}Hz", (row['Freq'], row['Mag']),
                 textcoords="offset points", xytext=(0,10), ha='center')

plt.savefig('ssb_fft.png')
print("Plot saved to ssb_fft.png")
