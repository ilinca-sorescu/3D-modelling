import math

# outputs [(x, y, z), ... ]
def genPoints(n, p, r):
    for i in range(p):
        a = math.pi / p * i
        b = n * 2 * math.pi / p * i
        x = r * math.sin(a) * math.cos(b)
        y = r * math.sin(a) * math.sin(b)
        z = r * math.cos(a)
        yield (x, y, z)

#outputs [(x1,x2, .. ), (y1,y2, ..), (z1, z2, ...)]
def points(n, p, r):
    return zip(*genPoints(n, p, r))

print(list(points(1, 4, 2)))

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(*points(10, 1000, 2))

plt.show()


