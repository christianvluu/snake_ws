
import numpy as np
import matplotlib.pyplot as plt

num_mods = 14
for i in range(0, 100):
    noise_db = 10
    noise_watts = 10 ** (noise_db/10)
    noise = np.random.normal(0, np.sqrt(noise_watts), 14)
    print noise
print noise_watts