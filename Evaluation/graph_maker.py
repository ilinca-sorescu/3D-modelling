#!/usr/bin/python
import os
import subprocess
import re
import json
import pylab as pl
import numpy as np
import numpy.random as npr
import types

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
    if numPics == 1600:
        print(filename + " " + str(getTimeData(filename)) + "!!!!")
    return {'minRatio':minRatio,
            'maxRatio':maxRatio,
            'reprojectionError':reprojectionError,
            'tolerance':tolerance,
            'outputFilename':outputFilename,
            'inputFolder':inputFolder,
            'numPics':numPics,
            'configFilename': filename}

def isEqual(v1, v2):
    if type(v2) == types.FunctionType:
        return v2(v1)
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
            print("Error getting plot data for: " + configDict['configFilename'])
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

#def getNumberOfPointsInBounds(configFilename):
#    f = open(configFilename + "_cloud", "r")
#    lines = f.readlines()

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
        if int(numbers[0].strip()) < 60:
            raise ErrorGatheringPlotData
        seconds = int(numbers[1].strip())*60+float(numbers[2].strip())
        return seconds
    except:
        raise ErrorGatheringPlotData

def bootstrap(values, num_samples, alpha):
    num_vals = len(values)
    values = np.array(values) #for matrix magic
    randIdMatrix = npr.randint(0, num_vals, (num_samples, num_vals))
    #randIdMatrix is of size num_samples X num_vals. Each location
    #is a random id representing one of the values
    samples = values[randIdMatrix]
    #python magic: samples is also a matrix
    gauss = np.sort(np.mean(samples, 1))
    #1 represents only lines => gauss is a vector of num_samples items
    return (gauss[(int)((alpha/2.0) * num_samples)],
            gauss[(int)((1-alpha/2.0) * num_samples)])

def plot(points, label, fit=False):
    points = sorted(points, key = lambda p : p['x'])

    xs = [p['x'] for p in points]
    ys = [p['y'] for p in points]
    #pl.plot(xs, ys)

    print([p['x'] for p in points])
    allXs = sorted(set([p['x'] for p in points]))
    xs = []
    ys = []
    lower = []
    upper = []
    for x in allXs:
        valsX = sorted(filter(lambda p : p['x'] == x, points), key = lambda p : p['y'])
        yValsX = [float(v['y']) for v in valsX]
        if x == 2500:
            print("!!!!!!!!!!!!1")
            print(yValsX)
            yValsX.remove(yValsX[-1])
        if len(yValsX) < 9:
            xs.append(x)
            ys.append(np.mean(yValsX))
            lower.append(0)
            upper.append(0)
            continue
        print(yValsX)
        print("For %s len is %s" % (x, len(yValsX)))
        mean = np.mean(yValsX)
        [l, u] = bootstrap(yValsX, 3000, 0.05)
        xs.append(x)
        ys.append(mean)
        lower.append(mean-l)
        upper.append(u-mean)
    #xs = [p['x'] for p in points]
    #ys = [p['y'] for p in points]
    #print(xs)
    #print(ys)
    if len(xs) == 0:
        print("No errorbars to show!")
    else:
        pl.errorbar(xs, ys, label=label, yerr = [lower, upper], elinewidth=2, capsize=0.1)

    if fit:
        z = np.polyfit(xs, ys, 3)
        f = np.poly1d(z)
        print(z)
        print(f)
        fitx = np.linspace(xs[0], xs[-1], 50)
        fity = f(fitx)
        label="(" + str(round(f[0],1))+"x+"+str(round(f[1],1))+")x"
        pl.plot(fitx, fity, color='red', linestyle='dotted', label=label)

allConfigFiles = []
paths = validConfigPaths("/home/ailinca/Workspace/Project/3D-modelling/Evaluation/Data")
for configFile in paths:
    allConfigFiles.append(loadConfigData(configFile))
relevantConfigs = []
for i in range(6, 16):
    specifiedFields = {
                #'minRatio':0,#0.04,
                'numPics':3000,
                'maxRatio':0.5,
                'reprojectionError':0.0001,
                'tolerance':0.001,
                'inputFolder': lambda s: s.split("/")[-1] == "Rubic" + str(i),
             }
    relevantConfigs += filterConfigs(specifiedFields, allConfigFiles)
print(len(relevantConfigs))
#plot(pointsToPlot('numPics', getQhullVolume, relevantConfigs), "Total Volume")
#plot(pointsToPlot('numPics', getQhullTotalFacetArea, relevantConfigs), "Total Facet Area")
#plot(pointsToPlot('numPics', getQhullNumVertices, relevantConfigs), "Number of vertices")
plot(pointsToPlot('minRatio', getQhullVolume, relevantConfigs), "Total Volume")#, fit=True)
#plot(pointsToPlot('minRatio', getQhullTotalFacetArea, relevantConfigs), "Total Facet Area")
#pl.plot([70, 3025], [6*2.156**2, 6*2.156**2], color='red', linestyle='dotted')
#pl.plot([70, 3025], [2.156**3, 2.156**3], color='red', linestyle='dotted')
#pl.xlim(70, 3025)
pl.legend(loc='upper left')
pl.xlabel("Number of input images")
#pl.title("")
pl.show()
