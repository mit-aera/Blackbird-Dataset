#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# August 3nd, 2018

import fire
import glob2, glob, os, sys, re, yaml, csv, shutil, subprocess
import numpy as np
import time
import threading

import importlib
from flightgoggles_utils import ImageHandler as FlightGogglesUtils

cameras = ["Camera_L", "Camera_R", "Camera_D"]

def runRendersOnDataset(datasetFolder, renderFolder, clientExecutablePath):
    devnull = open(os.devnull, 'wb') #python >= 2.4

    config = yaml.safe_load( file(os.path.join(datasetFolder,"trajectoryOffsets.yaml"),'r') )

    # Only select folders that we have offsets for
    # print config["unitySettings"]
    trajectoryFolders = [ traj for traj in config["unitySettings"].keys()]
    
    ############## DEBUG OVERRIDE
    #trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice", "winter"]
    # trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand"] # Batch 1
    #trajectoryFolders = ["halfMoon", "oval", "ampersand", "dice", "thrice", "tiltedThrice", "winter"]
    #trajectoryFolders = ["dice", "bentDice"] # Batch 2
    # trajectoryFolders = ["thrice", "tiltedThrice", "ampersand"] # Batch 3
    trajectoryFolders = ["ampersand"]


    for trajectoryFolder in trajectoryFolders:
        
        # iterate through trajectory environments
        for experiment in config["unitySettings"][trajectoryFolder]:

            # Get Trajectory offset
            envOffsetString = ' '.join( str(num) for num in experiment["offset"])

            # Find all '*_poses_centered.csv' files in folder
            trajectoryFiles = glob2.glob( os.path.join(datasetFolder, trajectoryFolder, '**/*_poses_centered.csv') )

            # Check that this experiment is applicable to this particular subset of logs
            subsetConstraint = experiment.get("yawDirectionConstraint","")            
            trajectoryFiles = [f for f in trajectoryFiles if subsetConstraint in f]
            
            ########### Limit file to 1 trajectory
            # debugTrajectory = "yawForward/winter_maxSpeed4p0"
            # trajectoryFiles = [traj for traj in trajectoryFiles if debugTrajectory in traj]

            # Render these trajectories
            for trajectoryFile in trajectoryFiles:

                print "========================================"
                print "Starting rendering of: " + trajectoryFile

                # Get final output folder
                outputFolder = os.path.dirname(trajectoryFile)
                
                # Clean up render folder
                shutil.rmtree(renderFolder,ignore_errors=True)
                os.makedirs(renderFolder)

                # Run render command
                command = clientExecutablePath + " '" + experiment["environment"] + "' " + envOffsetString + " '" + trajectoryFile + "'" 
                 
                process = subprocess.Popen(command, shell=True, stdout=devnull)
                process.wait()
		

                # Make videos from renders (async)
                def compressAndMoveVideo():
                    print "Creating video"
                    flightGogglesUtils = FlightGogglesUtils(renderFolder)                    
                    flightGogglesUtils.createVideo(cameras=cameras)

                    # Move videos to parent directory of CSV
                    print "Copying movie files."
                    
                    # Copy movies out of the image directory
                    movie_files = glob.glob(os.path.join(renderFolder,"*.mp4"))
                    for file_ in movie_files:
                        filename = experiment["name"] + ".mp4"
                        shutil.copy(file_, os.path.join(outputFolder, filename))
                    
                videoThread = threading.Thread(target=compressAndMoveVideo)
                videoThread.start()

                # Check that number of frames match for all cameras
                numFrames = []

                # Package camera images
                print "\nArchiving Images"
                for camName in cameras:
                    # Clear out camera folder
                    cameraFolder = os.path.join(renderFolder, camName)
                    shutil.rmtree(cameraFolder,ignore_errors=True)
                    os.makedirs(cameraFolder)
                    
                    # Get camera image files
                    cameraFiles = glob.glob(os.path.join(renderFolder,"*"+camName+".png"))
                    numFrames.append(len(cameraFiles)) # Log number of images from this camera
                    for file_ in cameraFiles:
                        filename = os.path.basename(file_)
                        shutil.copy(file_, os.path.join(cameraFolder,filename))

                    # Tar and cp the files
                    cameraDestArchive = os.path.join(outputFolder, camName + "_" + experiment["name"])
                    shutil.rmtree(cameraDestArchive+'.tar', ignore_errors=True)
                    shutil.make_archive(cameraDestArchive, "tar", cameraFolder)
                    print "Done archiving images"

                framesMissing = max(numFrames) - min(numFrames)
                print "Number of frames missing: " + str(framesMissing)

                with open(os.path.join(outputFolder,  "numDroppedFrames_" + experiment["name"] + ".txt"), 'w') as f:
                    f.write(str(framesMissing))

                print "Done Archiving Images. Waiting for video compression to finish."
                # Wait for video compression to finish
                videoThread.join()





    
    


if __name__ == '__main__':
    fire.Fire(runRendersOnDataset)
    # fire.Fire(runRenderOnCSV)
