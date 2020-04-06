#!/usr/bin/env python3

# Winter Guerra <winterg@mit.edu>
# March 2020

# This file looks at past RGB and Depth folders and prunes the frames such that all RGBD frames are in sync and there are no extra RGB or Depth frames.

# Usage find -L ~/BlackbirdDataset -iname "*Left_Gray" | xargs -I {} ./matchFrames.py {}/../ | tee matchFramesOutput.txt
# Usage: ./matchFrames.py BlackbirdDataset/egg/yawConstant/maxSpeed8p0/Ancient_Asia_Museum_Multiroom/ --dryrun

import numpy as np
import fire, os


rgbCameraList = ["Camera_Left_RGB", "Camera_Down_RGB"]
depthCameraList = ["Camera_Left_Depth", "Camera_Down_Depth"]
allCameraList = rgbCameraList + depthCameraList

timestampFileName = "video_frame_n_sec_timestamps.txt"
fpsCuttoff = 59.0

def getTimestampSetForCamera(cameraName, renderLocation):
    timestampFilePath = os.path.join(renderLocation, cameraName, timestampFileName)
    timestamp = np.loadtxt(timestampFilePath, dtype=np.int64)
    
    return set(timestamp)


def getFPSFromTimestampSet(tsSet):
    # Timestamps are in ns
    _max = max(tsSet)
    _min = min(tsSet)
    numFrames =len(tsSet) 
    nominalFPS = numFrames * 1.0e9 / (_max - _min)
    return nominalFPS


def validateNewTimestampSet(RGBDTimestampSet):
    '''
    Should Make sure that dataset complies with:
    - At least 59 FPS nominal
    - Some sort of max gap check? (3 frames?)
    '''
    
    # Timestamps are in ns
    nominalFPS = getFPSFromTimestampSet(RGBDTimestampSet)

    testsPassed = (nominalFPS >= fpsCuttoff)
    if (not testsPassed): 
        print( f"FAILED: FPS {nominalFPS}" )
    return testsPassed

def removeFramesFromTarball(cameraName, timestampsToRemove, renderLocation):
    pass

def main(renderLocation, dryrun=True):
    # Clean up the input path
    renderLocation = os.path.realpath(renderLocation)

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
        print(renderLocation)

        
        [print(f"{_camName}: {len(_originals)} frames, {len(_deletions)} deletions") for _camName,_originals,_deletions in zip(allCameraList, listOfOriginalTimestampSets, listOfDeletionsForAllCameras) if getFPSFromTimestampSet(_originals) < fpsCuttoff ]
        
        print("======")
        print("")
        # Do not create a plan for fixing this render.
        
        return

    # Execute plan on tarballs/movs


if __name__ == '__main__':
      fire.Fire(main)

