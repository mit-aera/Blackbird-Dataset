#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# August 3nd, 2018

import fire
import glob2, glob, os, sys, re, yaml, csv, shutil, subprocess
import numpy as np

import importlib
from flightgoggles_utils import ImageHandler as FlightGogglesUtils

cameras = ["Camera_L", "Camera_R", "Camera_D"]

def runRendersOnDataset(datasetFolder, renderFolder, clientExecutablePath):
    devnull = open(os.devnull, 'wb') #python >= 2.4

    config = yaml.safe_load( file(os.path.join(datasetFolder,"trajectoryOffsets.yaml"),'r') )

    # Only select folders that we have offsets for
    # print config["unitySettings"]
    trajectoryFolders = [ traj for traj in config["unitySettings"].keys() if config["unitySettings"][traj] ]
    
    ############## DEBUG
    trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice", "winter"]

    for trajectoryFolder in trajectoryFolders:
        
        # iterate through trajectory environments
        for envString in config["unitySettings"][trajectoryFolder].keys():
            # Get Trajectory offset
            envOffsetArray = config["unitySettings"][trajectoryFolder][envString]
            envOffsetString = ' '.join( str(num) for num in envOffsetArray)

            # Find all '*_poses_centered.csv' files in folder
            trajectoryFiles = glob2.glob( os.path.join(datasetFolder, trajectoryFolder, '**/*_poses_centered.csv') )

            ########### Only run 1m/s flights for testing
            # trajectoryFiles = [traj for traj in trajectoryFiles if "1p0" in traj]

            # Render these trajectories
            for trajectoryFile in trajectoryFiles:

                print "========================================"
                print "Starting rendering of: " + trajectoryFile
                
                # Clean up render folder
                shutil.rmtree(renderFolder,ignore_errors=True)
                os.makedirs(renderFolder)

                # Run render command
                command = clientExecutablePath + " '" + envString + "' " + envOffsetString + " '" + trajectoryFile + "'" 
                print command
                process = subprocess.Popen(command, shell=True, stdout=devnull)
                process.wait()

                # Make videos from renders
                flightGogglesUtils = FlightGogglesUtils(renderFolder)
                flightGogglesUtils.createVideo(cameras=cameras)

                # Get final output folder
                outputFolder = os.path.dirname(trajectoryFile)

                # Move videos to parent directory of CSV
                print "Moving movie files out of render directory."

                # Move movies and text files out of the image directory
                movie_files = glob.glob(os.path.join(renderFolder,"*.mp4"))
                for file_ in movie_files:
                    filename = envString + ".mp4"
                    shutil.copy(file_, os.path.join(outputFolder, filename))

                # Check that number of frames match
                numFrames = []

                # Package camera images
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
                        shutil.move(file_, os.path.join(cameraFolder,filename))

                    # Tar and cp the files
                    cameraDestArchive = os.path.join(outputFolder, camName + "_" + envString)
                    shutil.rmtree(cameraDestArchive+'.tar', ignore_errors=True)
                    shutil.make_archive(cameraDestArchive, "tar", cameraFolder)

                framesMissing = max(numFrames) - min(numFrames)
                print "Number of frames missing: " + str(framesMissing)

                with open(os.path.join(outputFolder,  "numDroppedFrames_" + envString + ".txt"), 'w') as f:
                    f.write(str(framesMissing))





    
    


if __name__ == '__main__':
    fire.Fire(runRendersOnDataset)
    # fire.Fire(runRenderOnCSV)
