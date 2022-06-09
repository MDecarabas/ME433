from ulab import numpy as np # to get access to ulab numpy functions
import time
n = 1024
sinSum = np.zeros([n])

for x in range(n):
    sin1 = np.sin(x)
    sin2 = np.sin(5*x)
    sin3 = np.sin(x*10)
    sinSum[x] = sin1 + sin2 + sin3

freq = np.fft.fft(sinSum)[0]

for x in range(n):
    time.sleep(0.1)
    print((freq[x], ))