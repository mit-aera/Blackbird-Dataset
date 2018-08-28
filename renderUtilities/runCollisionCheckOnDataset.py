#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# August 27th, 2018

import fire
import glob2, glob, os, sys, re, yaml, csv, shutil, subprocess
import numpy as np
import time
import threading
from multiprocess import Pool

import importlib
from flightgoggles_utils import ImageHandler as FlightGogglesUtils

cameras = ["Camera_L", "Camera_R", "Camera_D"]

def runCollisionCheckOnFile(args):
    trajectoryFile, experiment, offset, executablePath, environmentOBJFolder = args

    devnull = open(os.devnull, 'wb') #python >= 2.4

    # Get final output folder
    outputFolder = os.path.dirname(trajectoryFile)

    # Run collision check command
    command = executablePath + " --worldFile '" + os.path.join(environmentOBJFolder, experiment["environment"] + ".obj") + "' --x " + str(offset[0]) + " --y " + str(offset[1]) + " --z " + str(offset[2]) + " --yaw " + str(offset[3]) + " --poseFile '" + trajectoryFile + "'" 
    
    print command
    # process = subprocess.Popen(command, shell=True, stdout=devnull)
    process = subprocess.Popen(command, shell=True,stdout=devnull)
    returnCode = process.wait()
    print "Number of collisions: " + str(returnCode)

    # Output number of collisions
    with open(os.path.join(outputFolder,  "numCollisions_" + experiment["name"] + ".txt"), 'w') as f:
        f.write(str(returnCode))

    # List all trajectories that collided
    if (returnCode):
        return [trajectoryFile, experiment["name"], returnCode]

def runParallelCollisionCheckOnFiles(mapArgs):

    pool = Pool()
    results = pool.map(runCollisionCheckOnFile, mapArgs)
    pool.close()
    pool.join()
    # numCollisions = sum([k for (i,j,k) in results])

    return results

# Run all meta experiments in parallel
def runMetaExperiment(mapArgs):
    offset, mapArgs = mapArgs #unzip
    mapArgs = [(trajectoryFile, experiment, offset, executablePath, environmentOBJFolder) for (trajectoryFile, experiment, oldOffset, executablePath, environmentOBJFolder) in mapArgs]

    # mapArgs = [(trajectoryFile, experiment, offsetFromMap, executablePath, environmentOBJFolder) for trajectoryFile in trajectoryFiles]
    results = runParallelCollisionCheckOnFiles(mapArgs)
    collisions = np.sum([result[2] for result in results if result])

    return (results,collisions)

def runCollisionCheckOnDataset(datasetFolder, environmentOBJFolder, executablePath):
    
    

    config = yaml.safe_load( file(os.path.join(datasetFolder,"trajectoryOffsets.yaml"),'r') )

    # Only select folders that we have offsets for
    # print config["unitySettings"]
    trajectoryFolders = [ traj for traj in config["unitySettings"].keys()]
    
    ############## DEBUG OVERRIDE
    # trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice"]
    trajectoryFolders = [ "dice", "bentDice", "thrice", "tiltedThrice"]
    
    #trajectoryFolders = ["halfMoon", "oval", "ampersand", "dice", "thrice", "tiltedThrice", "winter"]
    # trajectoryFolders = ["winter"]

    # Keep track of trajectories that collided
    trajectoryResults = []


    for trajectoryFolder in trajectoryFolders:
        
        # iterate through trajectory environments
        for experiment in config["unitySettings"][trajectoryFolder]:

            
            print "Checking trajectory " + trajectoryFolder + " and experiment " + experiment["name"]

            # Find all '*_poses_centered.csv' files in folder
            trajectoryFiles = glob2.glob( os.path.join(datasetFolder, trajectoryFolder, '**/*_poses_centered.csv') )

            # Check that this experiment is applicable to this particular subset of logs
            subsetConstraint = experiment.get("yawDirectionConstraint","")
            trajectoryFiles = [f for f in trajectoryFiles if subsetConstraint in f]


            # Check collisions and re-run collision check with new offset if there are collisions.

            lowestNumCollisions = 1e10
            iterations = 0
            bestOffset = []
            bestResults = []

            # check number of collisions for normal offset
            mapArgs = [(trajectoryFile, experiment, experiment["offset"], executablePath, environmentOBJFolder) for trajectoryFile in trajectoryFiles]
            results = runParallelCollisionCheckOnFiles(mapArgs)
            # print results
            collisions = np.sum([result[2] for result in results if result])

            if collisions < lowestNumCollisions:
                lowestNumCollisions = collisions
                bestOffset = experiment["offset"]
                bestResults = results

            seedOffset = np.array(experiment["offset"])
            # Modify offset to find offset that gives no collisions
            offsetMap = []

            for dz in np.linspace(0,1,10):
                if (lowestNumCollisions is 0): break
                for dyaw in np.linspace(-10,10,10):
                    if (lowestNumCollisions is 0): break
                    for dx in np.linspace(-1,1,10):
                        if (lowestNumCollisions is 0): break
                        for dy in np.linspace(-1,1,10):
                            if (lowestNumCollisions is 0): break
                            
                            offset = seedOffset + np.array([dx,dy,dz,dyaw])
                            offsetMap.append(offset)

            batches = []
            for offset in offsetMap:
                parArgs = [(trajectoryFile, experiment, offset, executablePath, environmentOBJFolder) for trajectoryFile in trajectoryFiles]
                batches.append(parArgs)

            
            pool = Pool()
            batchResults = pool.map(lambda batch: [runCollisionCheckOnFile(fArg) for fArg in batch], batches)
            pool.close()
            pool.join()

            for i,batchResult in enumerate(batchResults):
                collisions = np.sum([result[2] for result in batchResult if result])

                if collisions < lowestNumCollisions:
                    lowestNumCollisions = collisions
                    bestOffset = offsetMap[i]
                    bestResults = batchResult

            
            
            trajectoryResults += (bestResults, bestOffset) 

            

    
    # Print out bad trajectories
    print "=========================="
    print "Bad trajectories: "
    numBad = 0
    for element in trajectoryResults:
        if element:
            print element
            numBad += 1
    print "Number of bad trajectories: " +  str(numBad) + "/" + str(len(trajectoryResults)) 



if __name__ == '__main__':
    fire.Fire(runCollisionCheckOnDataset)
    # fire.Fire(runRenderOnCSV)
