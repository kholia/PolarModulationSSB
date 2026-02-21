import numpy as np
taps = 127
M = (taps - 1) / 2
h = np.zeros(taps)
for i in range(taps):
    if i != M:
        h[i] = (2 / np.pi) * (np.sin(np.pi * (i - M) / 2)**2 / (i - M))
h *= np.blackman(taps)
print('{', end='')
print(', '.join([f'{val:.8f}f' for val in h]), end='')
print('}')
