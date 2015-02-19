import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
from pylab import *

def getImagePoints(points, camera):
    # points list[matrix[4x1]]
    # camera matrix[3x4]

    x = []
    y = []
    for point in points:
        homo = camera * point
        x.append(homo[0,0] / homo[2,0])
        y.append(homo[1,0] / homo[2,0])

    return (x, y)


def generate3DSquare():
    points = []
    for i in np.arange(-1.0, 1.0, 0.1):
        for j in np.arange(-1.0, 1.0, 0.1):
            points.append(np.matrix([i, j, 1, 1]).getT())
    return points

def plotCamera(camera):
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_xlim([-2.0, 2.0])
    ax.set_ylim([-2.0, 2.0])

    x, y = getImagePoints(generate3DSquare(), camera)
    ax.scatter(x,y)

    plt.show()




camera = np.matrix([
    [0.948683, 0, -0.316228, -0.169031],
    [0.845154, -0.507093, -2.22045e-16, 0.267261],
    [0.534522, 0.801784, -3.74166, 0]])

plotCamera(camera)

