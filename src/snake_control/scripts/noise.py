import numpy as np
from scipy.signal import butter, lfilter
from scipy import signal
import matplotlib.pyplot as plt

####### USAGE #######
# If a function returns "noise" only, then you add "noise" to the efforts
# If a function returns "noise_factor" then you multiple the "noise" with efforts
#####################


def add_white_gaussian_noise(raw_data):
    noise = np.random.normal(0, 2, len(raw_data)) # originally is 0.2
    return noise

# Butterworth stuff:
# https://scipy-cookbook.readthedocs.io/items/ButterworthBandpass.html
def butterworth_bandpass(low_cut, high_cut, fs, order):
    nyquist_freq = fs/2
    low_cut = low_cut/nyquist_freq
    high_cut = high_cut/nyquist_freq
    coeff_b, coeff_a = butter(order, [low_cut, high_cut], btype = 'band')
    return (coeff_b, coeff_a)

def butterworth_bandpass_filter(raw_data, low_cut, high_cut, fs, order):
    (coeff_b, coeff_a) = butterworth_bandpass(low_cut, high_cut, fs, order)
    filtered_data = lfilter(coeff_b, coeff_a, raw_data)
    return filtered_data

def add_low_freq_noise(raw_data, time): # time is in seconds
    white_noise = np.zeros(len(raw_data))
    period = 2 # seconds
    if (time % period < 0.1):
        white_noise = np.random.normal(0, 0.5, len(raw_data))
    return white_noise


def add_current_spike(prev_pos, curr_pos, loops, status, i):
    # loops, status are arrays (used as pointers) of len=1
    if (curr_pos > prev_pos):
        new_status = 1
    else:
        new_status = -1
    if new_status != status[0]:
        # add spike because the status changes here
        noise_factor = np.random.normal(1.575, 0.1, 1)
        loops[i] = 1
    elif (0 < loops[i] and loops[i] < 10): # add the spike for 0.1 seconds
        noise_factor = np.random.normal(1+(10-loops[i])*0.0575, 0.1, 1)
        loops[i] += 1
    else:
        noise_factor = np.ones(1) # don't add noise here
        loops[i] = 0
    status[i] = new_status
    return noise_factor[0]

def add_joint_friction(prev_pos, curr_pos):
    for i in range(0, len(prev_pos)):
        delta[i] = abs(curr_pos[i] - prev_pos[1])
    vel = delta/0.01 # 0.01 is dt, hz of sample rate
    coeff_friction = 0.05
    noise_factor = coeff_friction_vel
    return noise_factor

# # from here
# # https://dsp.stackexchange.com/questions/49460/apply-low-pass-butterworth-filter-in-python
# fs = 1000 # hz sample rate
# t = np.arange(0,1, 0.001)
# y = np.sin(2*np.pi*100*t) # pure signal # hz = 100
# x = 1*np.sin(2*np.pi*20*t) # hz = 20
# z = x + y


# fc = 30  # Cut-off frequency of the filter
# nyquist_freq = fs/2
# w = fc / (nyquist_freq) # Normalize the frequency
# b, a = signal.butter(5, w, 'lowpass')
# # for some reason b is all zeros, a looks right
# output = signal.filtfilt(b, a, z)
# print output

# #noisy_data = y + add_white_gaussian_noise(y)
# #filtered_data = butterworth_bandpass_filter(noisy_data, 0, 20, fs, 5)

# #plt.figure(1)
# plt.clf()

# #plt.plot(t, noisy_data, label="Noisy Data")
# plt.plot(t, y, label="Pure Signal")
# plt.plot(t, x, label="Added")
# plt.plot(t, output, label="Filetered")

# #plt.plot(t, filtered_data, label="Filtered Signal")
# plt.show()