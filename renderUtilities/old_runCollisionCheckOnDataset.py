#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# August 27th, 2018

import fire
import glob2, glob, os, sys, re, yaml, csv, shutil, subprocess
import numpy as np
import time
import threading
from multiprocessing import Pool 

import importlib
from flightgogglesUtils import ImageHandler as FlightGogglesUtils

cameras = ["Camera_L", "Camera_R", "Camera_D"]

def runCollisionCheckOnFile(args):
    trajectoryFile, experiment, executablePath, environmentOBJFolder = args

    devnull = open(os.devnull, 'wb') #python >= 2.4

    # Get final output folder
    outputFolder = os.path.dirname(trajectoryFile)

    # Run collision check command
    command = executablePath + " --worldFile '" + os.path.join(environmentOBJFolder, experiment["environment"] + ".obj") + "' --x " + str(experiment["offset"][0]) + " --y " + str(experiment["offset"][1]) + " --z " + str(experiment["offset"][2]) + " --yaw " + str(experiment["offset"][3]) + " --poseFile '" + trajectoryFile + "'" 
    
    print command
    # process = subprocess.Popen(command, shell=True, stdout=devnull)
    process = subprocess.Popen(command, shell=True,stdout=devnull)
    returnCode = process.wait()
    print "Number of collisions: " + str(returnCode)

    # Output number of collisions
    with open(os.path.join(outputFolder,  "numCollisions_" + experiment["name"] + ".txt"), 'w') as f:
        f.write(str(returnCode))

    # List all trajectories
    # if (returnCode):
    return [trajectoryFile, experiment["name"], returnCode]

def runParallelCollisionCheckOnFiles(mapArgs):

    pool = Pool()
    results = pool.map(runCollisionCheckOnFile, mapArgs)
    pool.close()
    pool.join()
    # numCollisions = sum([k for (i,j,k) in results])

    return results


def runCollisionCheckOnDataset(datasetFolder, environmentOBJFolder, executablePath):
    
    

    config = yaml.safe_load( file(os.path.join(datasetFolder,"trajectoryOffsets.yaml"),'r') )

    # Only select folders that we have offsets for
    # print config["unitySettings"]
    trajectoryFolders = [ traj for traj in config["unitySettings"].keys()]
    
    ############## DEBUG OVERRIDE
    # trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice"]
    # trajectoryFolders = [ "dice", "bentDice", "thrice", "tiltedThrice"]
    # trajectoryFolders = [  "thrice", "tiltedThrice"]
    
    # trajectoryFolders = ["halfMoon", "oval", "ampersand", "dice", "thrice", "tiltedThrice", "winter"]
    # trajectoryFolders = ["winter", "clover", "mouse", "patrick","picasso","sid",] # All NYC
    # trajectoryFolders = ["winter", "mouse", "picasso"]
    # trajectoryFolders = ["clover"]

    # Keep track of trajectories that collided
    trajectoryResults = []


    for trajectoryFolder in trajectoryFolders:
        
        # iterate through trajectory environments
        for experiment in config["unitySettings"][trajectoryFolder]:

            # for subsetConstraint in ["yawForward"]:
            # for subsetConstraint in ["yawConstant", "yawForward"]:
            print "Checking trajectory " + trajectoryFolder + " and experiment " + experiment["name"]

            # Find all '*_poses_centered.csv' files in folder
            trajectoryFiles = glob2.glob( os.path.join(datasetFolder, trajectoryFolder, '**/*_poses_centered.csv') )

            # Check that this experiment is applicable to this particular subset of logs
            subsetConstraint = ""
            subsetConstraint = experiment.get("yawDirectionConstraint",subsetConstraint)
            trajectoryFiles = [f for f in trajectoryFiles if subsetConstraint in f]


            # Check collisions and re-run collision check with new offset if there are collisions.
            lowestNumCollisions = 1e10
            iterations = 0
            bestOffset = []
            bestResults = []

            # check number of collisions for normal offset
            mapArgs = [(trajectoryFile, experiment, executablePath, environmentOBJFolder) for trajectoryFile in trajectoryFiles]
            results = runParallelCollisionCheckOnFiles(mapArgs)
            # # print results

            # value fewer bad files vs fewer collisions
            numBadFiles = np.sum([1 for result in results if result[2]>0]) 
            collisions = np.sum([result[2] for result in results])

            collisionScore = numBadFiles * collisions

            if collisionScore < lowestNumCollisions:
                lowestNumCollisions = collisionScore
                bestOffset = experiment["offset"]
                bestResults = results

            seedOffset = np.array(experiment["offset"])

            if lowestNumCollisions is 0:
                continue

            # Modify offset to find offset that gives no collisions
            while (lowestNumCollisions != 0 and iterations < 100):
                iterations += 1

                # Gaussian for small offset tweaks
                offset = seedOffset + np.random.randn(4)*np.array((0.75,0.75,0,1.5)) \
                    + np.clip( np.random.randn(4)*np.array((0,0,-0.15,0)), -1e6, -seedOffset[2]) 
                #    + np.random.randint(4)*np.array((0,0,0,90)) 
                
                #  Uniform distribution for large changes in offset
                # offset = seedOffset + (np.random.random(4)-0.5)*np.array((5,5,0,2.5)) \
                #     + np.clip( np.random.randn(4)*np.array((0,0,-0.2,0)), -1e6, -seedOffset[2]) 
                    # + np.random.randint(4)*np.array((0,0,0,90)) 
                print offset
                experiment["offset"] = offset

                mapArgs = [(trajectoryFile, experiment, executablePath, environmentOBJFolder) for trajectoryFile in trajectoryFiles]
                results = runParallelCollisionCheckOnFiles(mapArgs)
                
                # value fewer bad files vs fewer collisions
                numBadFiles = np.sum([1 for result in results if result[2]>0]) 
                collisions = np.sum([result[2] for result in results])

                collisionScore = numBadFiles * collisions


                if collisionScore < lowestNumCollisions:
                    lowestNumCollisions = collisionScore
                    bestOffset = experiment["offset"]
                    bestResults = results

                if collisions == 0:
                    break

                # print lowestNumCollisions
                # print bestOffset
                # for result in bestResults:
                #     if result:
                #         print result
                # print bestResults

                # return

            trajectoryResults.append( (bestResults, bestOffset) ) 

            

    
    # Print out bad trajectories
    print "=========================="
    print "Bad trajectories: "
    numBad = 0
    numTraj = 0

    # print trajectoryResults
    for results,offset in trajectoryResults:
        print "Offset: " + str(offset)
        for element in results:
            numTraj += 1
            print element
            if element[2] != 0:
                numBad += 1
    print "Number of bad trajectories: " +  str(numBad) + "/" + str(numTraj) 



if __name__ == '__main__':
    fire.Fire(runCollisionCheckOnDataset)
    # fire.Fire(runRenderOnCSV)
