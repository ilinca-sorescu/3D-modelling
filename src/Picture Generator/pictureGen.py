#!/usr/bin/env python
import random
import sys
import os
import argparse
import math
import shutil

def randomCoordinatesOnSphere(r):
    x = random.randint(-r, r)
    aux = round(math.sqrt(math.fabs(r*r-x*x)))
    y = random.randint(-aux, aux)
    if random.randint(0, 1) is 0:
        sign = 1
    else:
        sign = -1
    return (x, y, round(math.sqrt(math.fabs(r*r-x*x-y*y))*sign))

def file_extensions(extensions,fname):
    ext = os.path.splitext(fname)[1][1:]
    if not ext or ext not in extensions:
       parser.error("file doesn't end with {}".format(extensions))
    return fname

def check_negative(value):
    ivalue = int(value)
    if ivalue < 0:
        parser.error("%s is an invalid positive int value" % value)
    return ivalue


dir='PovImages'

parser = argparse.ArgumentParser('Generates images from the specified .inc file. ' +
        'The output folder is: %s.'%dir)
parser.add_argument('filename', type=lambda s:file_extensions(("inc"), s), help='name of the .inc file')
parser.add_argument('N', type=check_negative, help='number of output images')
parser.add_argument('d', type=check_negative, help='the size (in PovRay units) of a cube centered in the origin such ' +
    'that the object in the specified .inc file is positioned inside the cube')
args = parser.parse_args()

distanceCameraOrigin = random.randint(args.d+5, args.d+50)

from mako.template import Template
povTemplate = Template(filename='povTemplate.mako')

if(os.path.exists(dir)):
    shutil.rmtree(dir)
os.mkdir(dir)

for i in range(0, args.N):
    camera = randomCoordinatesOnSphere(distanceCameraOrigin)
    fname = dir + '/' + str(i) + '.pov'
    pic = open(fname, 'w')
    pic.write(povTemplate.render(filename=args.filename, camera=camera))
    pic.close()
    os.system("povray %s"%(fname))
    os.remove(fname)
    txt = open(dir + '/' + str(i),'w')
    txt.write(" ".join(str(x) for x in camera))

