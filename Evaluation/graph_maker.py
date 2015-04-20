#!/usr/bin/python
import os
import subprocess
import re
import json
import pylab as pl
import numpy as np

eps = 0.000000001

def validConfigPaths(folder):
    allFiles = os.listdir(folder)
    validFiles = []
    for f in allFiles:
        if '_' in f:
            continue
        timeF = f + "_time"
        cloudF = f + "_cloud"
        if timeF in allFiles and cloudF in allFiles:
            validFiles.append(folder + "/" + f)
    return validFiles

def loadConfigData(filename):
    f = open(filename, "r")
    minRatio          = float(f.readline())
    maxRatio          = float(f.readline())
    reprojectionError = float(f.readline())
    tolerance         = float(f.readline())
    outputFilename    = f.readline().strip()
    inputFolder       = f.readline().strip()
    numPics           = int(f.readline())
    return {'minRatio':minRatio,
            'maxRatio':maxRatio,
            'reprojectionError':reprojectionError,
            'tolerance':tolerance,
            'outputFilename':outputFilename,
            'inputFolder':inputFolder,
            'numPics':numPics,
            'configFilename': filename}

def isEqual(v1, v2):
    if type(v1) == str:
        return v1 == v2
    return abs(v1-v2) <= eps

def filterConfigs(specifiedFields, configDictList):
    r = []
    for config in configDictList:
        isValid = True
        for (k, v) in specifiedFields.items():
            if not isEqual(config[k], v):
                isValid = False
                break
        if isValid:
            r.append(config)
    return r

def pointsToPlot(xField, fy, configDictList):
    p = []
    for configDict in configDictList:
        try:
            p.append({'x':configDict[xField],
                'y':fy(configDict['configFilename']),
                'dict':configDict})
        except ErrorGatheringPlotData:
            print(configDict['configFilename'])
            #pass
    return p

def prepareCloudFile(filename):
    f = open(filename, "r")
    lines = f.readlines()
    output = open("/tmp/cloud", "w")
    output.write("3\n")
    output.write(str(len(lines)) + "\n")
    output.writelines(lines)
    output.close()

def runQHull(configFilename):
    prepareCloudFile(configFilename + "_cloud")
    command = "cat /tmp/cloud | qhull FA "
    try:
        qhullOutput = subprocess.check_output(command, shell=True).decode("UTF-8")
    except subprocess.CalledProcessError:
        raise ErrorGatheringPlotData
    numbers = re.findall(r"\d+\.?\d*", qhullOutput)
    qhullDict = {
            'numVertices':numbers[0],
            'numHullVertices':numbers[2],
            'numFacets':numbers[3],
            'totalFacetArea':numbers[8],
            'totalQhullVolume':numbers[9]
            }
    f = open(configFilename + "_qhullDict", "w")
    json.dump(qhullDict, f)
    f.close()

class ErrorGatheringPlotData(Exception):
    pass

def getQhullParameter(configFilename, parameterName):
    if not os.path.isfile(configFilename + "_qhullDict"):
        runQHull(configFilename)
    f = open(configFilename + "_qhullDict", "r")
    qhullDict = json.load(f)
    return qhullDict[parameterName]

def getQhullVolume(configFilename):
    return getQhullParameter(configFilename, 'totalQhullVolume')

def getQhullNumVertices(configFilename):
    return getQhullParameter(configFilename, 'numVertices')

def getQhullNumHullVertices(configFilename):
    return getQhullParameter(configFilename, 'numHullVertices')

def getQhullFacets(configFilename):
    return getQhullParameter(configFilename, 'numFacets')

def getQhullTotalFacetArea(configFilename):
    return getQhullParameter(configFilename, 'totalFacetArea')

def getTimeData(configFilename):
    try:
        f = open(configFilename + "_time")
        s = f.read()
        numbers = re.findall(r"\d+\.?\d*", s)
        seconds = numbers[1]*60+numbers[2]
        if seconds < 60:
            print(seconds)
            print(configFilename)
            print(numbers[1])
        return seconds
    except:
        raise ErrorGatheringPlotData

def plot(points, label):
    points = sorted(points, key = lambda p : p['x'])
    xs = [p['x'] for p in points]
    ys = [p['y'] for p in points]
    pl.plot(xs, ys, label=label)

allConfigFiles = []
paths = validConfigPaths("/home/ailinca/Workspace/Project/3D-modelling/Evaluation/Data")
for configFile in paths:
    allConfigFiles.append(loadConfigData(configFile))
specifiedFields = {
            'minRatio':0,
            'maxRatio':0.5,
            'reprojectionError':0.0001,
            'tolerance':0.001,
            'inputFolder':"/home/ailinca/Workspace/Project/3D-modelling/Inputs/Rubic6"
            }
relevantConfigs = filterConfigs(specifiedFields, allConfigFiles)
print(len(relevantConfigs))
plot(pointsToPlot('numPics', getQhullFacets, relevantConfigs), "Number of facets")
plot(pointsToPlot('numPics', getQhullNumHullVertices, relevantConfigs), "Number of points in convex hull")
#plot(pointsToPlot('numPics', getTimeData, relevantConfigs), "Time")
pl.legend(loc='upper left')
pl.show()
