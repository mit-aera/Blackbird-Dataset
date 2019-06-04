#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# August 3nd, 2018

import fire
import glob2, glob, os, sys, re, yaml, csv, shutil, subprocess, re
import numpy as np
import time
import threading


#trajectoryFolders = [ "sphinx", "halfMoon", "oval", "ampersand", "dice", "bentDice", "thrice", "tiltedThrice", "winter", "clover", "mouse", "patrick", "picasso", "sid", "star", "cameraCalibration"]
trajectoryFolders = [ "bentDice", "sphinx", "oval", "ampersand", "thrice", "tiltedThrice", "winter", "mouse", "patrick", "picasso", "sid", "star", "cameraCalibration"]



def runRendersOnDataset(datasetFolder):
    '''
    Used to take an entire dataset of ppm files, convert them to png, tarball, then upload to a different location. 
    '''
    devnull = open(os.devnull, 'wb') #python >= 2.4

    config = yaml.safe_load( file(os.path.join(datasetFolder,"trajectoryOffsets.yaml"),'r') )

    # Only select folders that we have offsets for
    # print config["unitySettings"]
    #trajectoryFolders = [ traj for traj in config["unitySettings"].keys()]

    print trajectoryFolders

    for trajectoryFolder in trajectoryFolders:
        
	folderPath = os.path.join(datasetFolder, "BlackbirdDatasetData", trajectoryFolder)
	compressedFolderPath = os.path.join("~/BlackbirdDataset", trajectoryFolder)
	nasFolderPath = os.path.join("/media/medusa/NAS_FTP/AgileDrones/Datasets/BlackbirdDataset/BlackbirdDataset_RGBD_30Hz", trajectoryFolder)

	print "========================================"
	print "Starting compression of: " + trajectoryFolder

        
        # Make folder structure
        command = "find " + folderPath + " -type d -printf '%P\n' | parallel --bar -I {} mkdir -p "+compressedFolderPath+"/{}"
        print command 

        process = subprocess.Popen(command, shell=True, stdout=devnull)
        process.wait()

        # Make folder structure on NAS
        command = "find " + folderPath + " -type d -printf '%P\n' | parallel --bar -I {} mkdir -p "+nasFolderPath+"/{}"
        print command 

        process = subprocess.Popen(command, shell=True, stdout=devnull)
        process.wait()

	# Run compression command
	command = "find " + folderPath + " -name '*.ppm' -printf '%P\n' | sed s/.ppm//g | parallel --bar -I {} gm convert " + folderPath + "/{}.ppm "+compressedFolderPath+"/{}.png"
	print command

	process = subprocess.Popen(command, shell=True, stdout=devnull)
	process.wait()
	
	# Tarball folder and upload to NAS. (async, erases after done)
	command = "tar -c --to-stdout " + compressedFolderPath + " | pv -s $(du -sb "+compressedFolderPath+" | awk '{print $1}') > " + nasFolderPath + ".tar && rm -r " + compressedFolderPath
	print command
	process = subprocess.Popen(command, shell=True, stdout=devnull)
	#process.wait()

        # Delete compressed folder
	#command = "rm -r " + compressedFolderPath
        #print command
        #process = subprocess.Popen(command, shell=True, stdout=devnull)
        #process.wait()
	

       


if __name__ == '__main__':
    fire.Fire(runRendersOnDataset)
    # fire.Fire(runRenderOnCSV)
