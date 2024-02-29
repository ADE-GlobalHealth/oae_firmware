import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)

t = np.linspace(0, 2*np.pi, 4096)

def inner(f, x):
    return np.sin(f*2*np.pi*x)

y = inner(1, t)

str = "{"
for num in y:
    str += f"{num}, "
str = str[:-2]
str += "}"
print(str)
# import matplotlib.pyplot as plt

# plt.plot(t, y)
# plt.show()
