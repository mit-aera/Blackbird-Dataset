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
from flightgoggles_utils import ImageHandler as FlightGogglesUtils

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

    # List all trajectories that collided
    if (returnCode):
        return [trajectoryFile, experiment["name"], returnCode]

def runCollisionCheckOnDataset(datasetFolder, environmentOBJFolder, executablePath):
    
    

    config = yaml.safe_load( file(os.path.join(datasetFolder,"trajectoryOffsets.yaml"),'r') )

    # Only select folders that we have offsets for
    # print config["unitySettings"]
    trajectoryFolders = [ traj for traj in config["unitySettings"].keys()]
    
    ############## DEBUG OVERRIDE
    trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice"]
    #trajectoryFolders = ["halfMoon", "oval", "ampersand", "dice", "thrice", "tiltedThrice", "winter"]
    # trajectoryFolders = ["winter"]

    # Keep track of trajectories that collided
    badTrajectories = []


    for trajectoryFolder in trajectoryFolders:
        
        # iterate through trajectory environments
        for experiment in config["unitySettings"][trajectoryFolder]:

            
            print "Checking trajectory " + trajectoryFolder + " and experiment " + experiment["name"]

            # Find all '*_poses_centered.csv' files in folder
            trajectoryFiles = glob2.glob( os.path.join(datasetFolder, trajectoryFolder, '**/*_poses_centered.csv') )

            # Check that this experiment is applicable to this particular subset of logs
            subsetConstraint = experiment.get("yawDirectionConstraint","")
            trajectoryFiles = [f for f in trajectoryFiles if subsetConstraint in f]

            mapArgs = [(trajectoryFile, experiment, executablePath, environmentOBJFolder) for trajectoryFile in trajectoryFiles]


            pool = Pool()
            results = pool.map(runCollisionCheckOnFile, mapArgs)
            pool.close()
            pool.join()

            badTrajectories += results

            

    
    # Print out bad trajectories
    print "=========================="
    print "Bad trajectories: "
    numBad = 0
    for element in badTrajectories:
        if element:
            print element
            numBad += 1
    print "Number of bad trajectories: " +  str(numBad) + "/" + str(len(badTrajectories)) 



if __name__ == '__main__':
    fire.Fire(runCollisionCheckOnDataset)
    # fire.Fire(runRenderOnCSV)
