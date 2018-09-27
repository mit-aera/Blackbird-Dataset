#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# August 3nd, 2018

import fire
import glob2, glob, os, sys, re, yaml, csv, shutil, subprocess, re
import numpy as np
import time
import threading

import importlib
from flightgogglesUtils import ImageHandler as FlightGogglesUtils
from blackbirdDatasetUtils import *


# trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice", "winter", "clover", "mouse", "patrick", "picasso", "sid", "star", "cameraCalibration"]


def runRendersOnDataset(renderFPS, datasetFolder, renderFolder, clientExecutablePath, cameras = ["Camera_L", "Camera_R", "Camera_D"], previewFPS=None, fileFilter = ".*"):
    # Set previewFPS to renderFPS if previewFPS is not defined
    if not previewFPS:
        previewFPS=renderFPS

    # Compile file filter regex
    print fileFilter
    fileFilter = re.compile(fileFilter)
    
    
    devnull = open(os.devnull, 'wb') #python >= 2.4

    config = yaml.safe_load( file(os.path.join(datasetFolder,"trajectoryOffsets.yaml"),'r') )

    # Only select folders that we have offsets for
    # print config["unitySettings"]
    trajectoryFolders = [ traj for traj in config["unitySettings"].keys()]

    print "Rendering the following trajectories: "
    print trajectoryFolders


    for trajectoryFolder in trajectoryFolders:
        
        # iterate through trajectory environments
        for experiment in config["unitySettings"][trajectoryFolder]:

            # Get Trajectory offset
            envOffsetString = ' '.join( "'" + str(num) + "'" for num in experiment["offset"])

            # Find all '*.log' files in folder
            logFiles = glob2.glob( os.path.join(datasetFolder, "data", trajectoryFolder, '**/*.log') )

            # Check that this experiment is applicable to this particular subset of logs
            subsetConstraint = experiment.get("yawDirectionConstraint","")            
            logFiles = [f for f in logFiles if subsetConstraint in f]
            # print logFiles
            
            logFiles = [traj for traj in logFiles if fileFilter.match(traj)]
            print logFiles

            # Render these trajectories
            for logFile in logFiles:

                print "========================================"
                print "Starting rendering of: " + logFile

                # Get associated filenames of data for log
                files = getFilenamesForLog(logFile)

                # Get final output folder
                outputFolder = os.path.dirname(logFile)
                
                # Clean up render folder
                shutil.rmtree(renderFolder,ignore_errors=True)
                os.makedirs(renderFolder)

                # Normalize trajectory in XY and Z
                # offsetPoseList(renderFPS, logFile)

                # Run render command
                command = clientExecutablePath + " '" + experiment["environment"] + "' '" + str(len(cameras)) + "' " + envOffsetString + " '" + files["poselistCenteredFilename"] + "'" 
                print command

                process = subprocess.Popen(command, shell=True, stdout=devnull)
                process.wait()
		

                # Make videos from renders (async)
                def compressAndMoveVideo():
                    print "Creating video"
                    flightGogglesUtils = FlightGogglesUtils(renderFolder)                    
                    flightGogglesUtils.createVideo(cameras=cameras, fps=previewFPS)

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
                numFrames = [ sum(1 for line in open(files["poselistCenteredFilename"])) ]

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
