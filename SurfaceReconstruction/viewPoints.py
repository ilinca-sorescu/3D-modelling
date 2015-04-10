#!/usr/bin/python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x = []
y = []
z = []
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for line in open("cloud.txt", "r"):
    _x, _y, _z = line.split(" ")
    x.append(float(_x))
    y.append(float(_y))
    z.append(float(_z))
ax.scatter(x, y, z)
plt.show()
