#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# August 3nd, 2018

import fire
import glob2, glob, os, sys, re, yaml, csv, shutil, subprocess, re
import numpy as np
import time
import threading

#import importlib
#from flightgogglesUtils import ImageHandler as FlightGogglesUtils
#from blackbirdDatasetUtils import *


# trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice", "winter", "clover", "mouse", "patrick", "picasso", "sid", "star", "cameraCalibration"]


def runRendersOnDataset(datasetFolder):

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

            # Get universal render offset and convert to values compatible with TF
            renderOffsetArray = -1 * np.array(experiment["offset"])
            renderOffsetTranslation = renderOffsetArray[:3]
            theta = renderOffsetArray[3]
            renderOffsetRotation = np.array([0,0,np.sin(theta*np.pi/360.0),np.cos(theta*np.pi/360.0)])

            # Find all '*.bag' files in folder
            bagFiles = glob2.glob( os.path.join(datasetFolder, "BlackbirdDatasetData", trajectoryFolder, '**/*.bag') )

            # Check that this experiment is applicable to this particular subset of logs
            subsetConstraint = experiment.get("yawDirectionConstraint","")            
            bagFiles = [f for f in bagFiles if subsetConstraint in f]
            # print bagFiles
            
            #bagFiles = [traj for traj in bagFiles if fileFilter.match(traj)]
            # print bagFiles

            # Render these trajectories
            for bagFile in bagFiles:

                print "========================================"
                print "Starting rendering of: " + bagFile

                # Get per-trajectory centering offset.
                offsetPath = bagFile[:-4] + "_poses_offset.csv"
                trajectoryOffsetArray = np.loadtxt(offsetPath, delimiter=',')
                trajectoryOffsetString = " ".join( map(str,trajectoryOffsetArray) ) + " 0 0 0 1"


                # Clean output directory
                outputDir = bagFile + "_images"
                try:
                    os.rmdir(outputDir)
                except:
                    pass

                os.mkdir(outputDir)


                # Run render command
                command = "roslaunch flightgoggles blackbirdDataset.launch bagfile_path:='"  + bagFile + "' scene_filename:=" + experiment["environment"] + " trajectory_offset_transform:='" + trajectoryOffsetString + "' render_offset_rotation:='" + " ".join(map(str,renderOffsetRotation)) + "' render_offset_translation:='" + " ".join(map(str,renderOffsetTranslation)) + "'"
                print command

                process = subprocess.Popen(command, shell=True, stdout=devnull)
                process.wait()
		time.sleep(2)


if __name__ == '__main__':
    fire.Fire(runRendersOnDataset)
    # fire.Fire(runRenderOnCSV)
