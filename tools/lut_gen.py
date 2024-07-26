'''
    File for generating a lookup table for two sine waves.

    Each value in the wave is printed in a 32-bit hex format, ex: 0x07d007d0
    where the first 16 bits (ex: 07d0) hold a starting 0 followed by a 12-bit
    number in the first wave, and the second 16 bits follow the same pattern
    for the second sine wave.
'''

import numpy as np

# Define parameters
frequency = 1000  # Frequency of the sine wave in Hz
duration = (1/frequency) # Duration of the sine wave in seconds; one period
amplitude = 2000  # Amplitude of the sine wave
# TIM6 sampling_rate = 128205 Hz

num_vals = 128
sampling_rate = num_vals/duration

# Generate time values
# one wave is 1.2x the freq of the other
t1 = np.linspace(0, duration*5, num_vals, endpoint=False)
t2 = np.linspace(0, duration*6, num_vals, endpoint=False)

# Calculate sine values
sine_wave1 = (amplitude * np.sin(2 * np.pi * frequency * t1)) + amplitude
sine_wave2 = (amplitude * np.sin(2 * np.pi * frequency * t2)) + amplitude

sine1 = list(sine_wave1)
sine2 = list(sine_wave2)

lookup_table = [0] * num_vals

for i in range(0, num_vals):
    # First sine wave
    hex1 = format(round(sine1[i]), '04x')
    # Second sine wave
    hex2 = format(round(sine2[i]), '04x')
    hex_num = '0x' + hex1 + hex2
    lookup_table[i] = hex_num

# Print the lookup table
print(lookup_table)

for i in range(0, len(lookup_table), 10):
    row = lookup_table[i:i + 10]
    print(", ".join(f"{value}" for value in row) + ",")