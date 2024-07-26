'''
    File for decoding a .txt file output containing
    audio data from I2S line
'''

import matplotlib.pyplot as plt
import csv

def hex_to_twos_complement(hex_string, bits=16):
    """
        Convert a hex string to a two's complement signed integer.
    """
    value = int(hex_string, 16)
    if value >= 2**(bits - 1):
        value -= 2**bits
    return value

def parse_debugger_output(debugger_output):
    """
        Parse the debugger output to extract 32-bit hex segments and 
        convert them to two's complement values.
    """
    twos_complement_values = []
    
    # Split the input into lines
    lines = debugger_output.strip().split('\n')
    
    for line in lines:
        # Split each line into parts
        parts = line.split()
        
        # Skip the first part (the address) and process the remaining hex pairs
        hex_pairs = parts[1:]
        print(hex_pairs)
        # Combine hex pairs into 32-bit segments (8 hex characters)
        for i in range(0, len(hex_pairs), 2):
            # Accounting for little endian system 
            hex_segment = f"{hex_pairs[i+1]}{hex_pairs[i]}"
            # hex_segment = f"{hex_pairs[i]}{hex_pairs[i+1]}"

            twos_complement_value = hex_to_twos_complement(hex_segment)
            twos_complement_values.append(twos_complement_value)
    
    return twos_complement_values

def separate_even_odd_indices(values):
    """
        Separate values into even-indexed and odd-indexed lists.
    """
    even_index_values = [value for idx, value in enumerate(values) if idx % 2 == 0]
    odd_index_values = [value for idx, value in enumerate(values) if idx % 2 != 0]
    
    return even_index_values, odd_index_values

file_name = "full"

# Sample debugger output
with open(f"{file_name}.txt") as f:
    debugger_output = f.read()

# Parse the debugger output
twos_complement_values = parse_debugger_output(debugger_output)

# Take out 0s in the unused I2S channel
even_index_values, odd_index_values = separate_even_odd_indices(twos_complement_values)

# Print the results
print("Even-indexed values:", even_index_values)
# print("Odd-indexed values:", odd_index_values)

final_idx = even_index_values

plt.plot(list(range(len(final_idx))), final_idx, marker = ".")
plt.show()

# with open(f'{file_name}.csv', 'w') as f:
#     writer = csv.writer(f)
#     writer.writerows([even_index_values])