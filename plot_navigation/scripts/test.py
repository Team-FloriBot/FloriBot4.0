#!/usr/bin/env python3

from matplotlib import markers
import numpy as np
import matplotlib.pyplot as plt

x = np.arange(0,10)
y = np.arange(10,20)
xx, yy = np.meshgrid(x, y)

fig, ax = plt.subplots(nrows=1, ncols=1)
ax.scatter(xx, yy, marker=',', c='k')
ax.plot([4,3,2], [5,9,12], 'or')
plt.show()
