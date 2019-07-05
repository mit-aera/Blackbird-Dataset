#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# August 3nd, 2018

import fire
import glob2, glob, os, sys, re, yaml, csv, shutil, subprocess, re
import numpy as np
import time
import threading
from pathos.multiprocessing import Pool

import importlib
from compressLosslessVideo import *

#import importlib
#from flightgogglesUtils import ImageHandler as FlightGogglesUtils
#from blackbirdDatasetUtils import *


trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice", "winter", "clover", "mouse", "patrick", "picasso", "sid", "star", "cameraCalibration"]
#fileName = 'mouse'

trajectoryFolders = [ "winter", "clover", "mouse", "patrick", "picasso", "sid", "star"]

def runRendersOnDataset(datasetFolder, renderDir, render_prefix, fps):

    devnull = open(os.devnull, 'wb') #python >= 2.4

    config = yaml.safe_load( file(os.path.join(datasetFolder,"trajectoryOffsets.yaml"),'r') )

    # Only select folders that we have offsets for
    # print config["unitySettings"]
#    trajectoryFolders = [ traj for traj in config["unitySettings"].keys()]

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
            bagFiles = glob2.glob( os.path.join(datasetFolder, trajectoryFolder, '**/*.bag') )

            # Check that this experiment is applicable to this particular subset of logs
            subsetConstraint = experiment.get("yawDirectionConstraint","")            
            bagFiles = [f for f in bagFiles if subsetConstraint in f]
            # print bagFiles
            
            #bagFiles = [traj for traj in bagFiles if fileName in traj]
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
                outputDir = "{}_{}_{}".format(bagFile[:-4], experiment["name"], render_prefix)
                try:
                   shutil.rmtree(renderDir)
                except:
                    pass
                try:
                    shutil.rmtree(outputDir)
                except:
                    pass

                os.mkdir(outputDir)
                os.mkdir(renderDir)

 
                # Generate timestamp file based on timestamp files left from the ISER dataset (120hz).
                ISERDatasetPoseList = np.loadtxt(bagFile[:-4] + "_poses_centered.csv", delimiter=',')

                # print ISERDatasetPoseList
                
                lastTS = 0
                TSList = []
                # Prune timestamps to a fps <= 120
                frameLengthMicroseconds = (1.0e6/fps)*0.9
                #frameLengthMicroseconds = (10.0**6/fps) -1 #*0.9

                for ts in ISERDatasetPoseList[:,0]:
                    if ( (ts-lastTS) >= frameLengthMicroseconds):
                        lastTS = ts 
                        TSList.append(ts)

                # Save timestamps
                timestampsToRender = np.array(TSList)
                timestampFile = os.path.join(renderDir, "timestampsToRender.csv") 
                np.savetxt(timestampFile, timestampsToRender, delimiter=",", fmt='%d')

               # Run render command
                command = "roslaunch flightgoggles blackbirdDataset.launch bagfile_path:='{}' output_folder:='{}' timestampfile_path:='{}' scene_filename:='{}' trajectory_offset_transform:='{}' render_offset_rotation:='{}' render_offset_translation:='{}'".format(bagFile, renderDir, timestampFile, experiment["environment"], trajectoryOffsetString, " ".join(map(str,renderOffsetRotation)), " ".join(map(str,renderOffsetTranslation)))
                print command

                process = subprocess.Popen(command, shell=True) #, stdout=devnull)
                process.wait()

                # Loop through output folders and compress files (in parallel).
                def compressVideo(_folder_name):
                    subfolder_name = _folder_name.split('/')[-2]
                    #print subfolder_name

                    # Compress files and move to final destination
                    print "Compressing images to lossless HEVC"
                    compressLosslessVideo(_folder_name, ".ppm", output_folder=os.path.join(outputDir, subfolder_name))           

                #p = Pool()
                #map(compressVideo, glob.glob(os.path.join(renderDir, "*/")))

                for d in glob.glob(os.path.join(renderDir, "*/")):
                    compressVideo(d)
		time.sleep(2)


if __name__ == '__main__':
    fire.Fire(runRendersOnDataset)
    # fire.Fire(runRenderOnCSV)
