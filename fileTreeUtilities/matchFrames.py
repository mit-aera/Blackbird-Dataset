#!/usr/bin/env python3

# Winter Guerra <winterg@mit.edu>
# March 2020

# This file looks at past RGB and Depth folders and prunes the frames such that all RGBD frames are in sync and there are no extra RGB or Depth frames.

# Usage: ./matchFrames.py BlackbirdDataset/egg/yawConstant/maxSpeed8p0/Ancient_Asia_Museum_Multiroom/ --dryrun

import numpy as np
import fire, os


rgbCameraList = ["Camera_Left_RGB", "Camera_Down_RGB"]
depthCameraList = ["Camera_Left_Depth", "Camera_Down_Depth"]
allCameraList = rgbCameraList + depthCameraList

timestampFileName = "video_frame_n_sec_timestamps.txt"

def getTimestampSetForCamera(cameraName, renderLocation):
    timestampFilePath = os.path.join(renderLocation, cameraName, timestampFileName)
    timestamp = np.loadtxt(timestampFilePath, dtype=np.int64)
    
    return set(timestamp)

def validateNewTimestampSet(RGBDTimestampSet):
    '''
    Should Make sure that dataset complies with:
    - At least 59 FPS nominal
    - Some sort of max gap check? (3 frames?)
    '''
    
    # Timestamps are in ns
    _max = max(RGBDTimestampSet)
    _min = min(RGBDTimestampSet)
    numFrames =len(RGBDTimestampSet) 
    nominalFPS = numFrames * 1.0e9 / (_max - _min)

    testsPassed = (nominalFPS >= 59.0)
    if (not testsPassed): 
        print( f"FAILED: FPS {nominalFPS}. Max: {_max}, min: {_min}, frames: {numFrames}" )
    return testsPassed

def removeFramesFromTarball(cameraName, timestampsToRemove, renderLocation)

def main(renderLocation, dryrun=True):
    # Get sets of the RGBD timestamps
    listOfOriginalTimestampSets = [getTimestampSetForCamera(c, renderLocation) for c in allCameraList] 

    # Get the timestamps of whole RGBD frames across both cameras
    RGBDTimestampSet = set.intersection(*listOfOriginalTimestampSets)

    # Check if this render batch still meets minimum criteria, otherwise skip.
    passesChecks = validateNewTimestampSet(RGBDTimestampSet)

    # Plan which tarballs/movs need to be unpacked
    listOfDeletionsForAllCameras = [ camSet - RGBDTimestampSet for camSet in listOfOriginalTimestampSets ]

    # DEBUG: Print plan
    if (not passesChecks):
        [print(f"{_camName}: {len(_originals)} frames, {len(_deletions)} deletions") for _camName,_originals,_deletions in zip(allCameraList, listOfOriginalTimestampSets, listOfDeletionsForAllCameras) ]
        
        # Do not create a plan for fixing this render.
        # @TODO: mark render for reason for failure.
        return

    # Execute plan on tarballs/movs


if __name__ == '__main__':
      fire.Fire(main)

