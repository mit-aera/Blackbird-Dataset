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
segCameraList = ["Camera_Left_Segmented", "Camera_Down_Segmented"]
allCameraList = rgbCameraList + depthCameraList + segCameraList

timestampFileName = "nSecTimestamps.txt"
fpsCuttoff = 59.0

def getTimestampSetForCamera(cameraName, renderLocation):
    timestampFilePath = os.path.join(renderLocation, cameraName, timestampFileName)
    timestamp = np.loadtxt(timestampFilePath, dtype=np.int64)
    
    # If there is no .mov or .tar file alongside the timestamp file, zero out the timestamps.
    if ( not os.path.exists(os.path.join(renderLocation, cameraName, "lossless.tar")) and not os.path.exists(os.path.join(renderLocation, cameraName, "lossless.mov"))):
        timestamp = np.array([])
    
    return set(timestamp)

def getNewRenderLocation(renderLocation):
    experimentName = os.path.basename(renderLocation)
    relativePath = f"../rosbag_{experimentName}_ijrr_new_full_suite_2/"
    return os.path.realpath(os.path.join(renderLocation, relativePath))


def getFPSFromTimestampSet(tsSet):
    if (not len(tsSet)):
        return 0.0
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

def checkRender(renderLocation, dryrun=True):
    # Clean up the input path
    renderLocation = os.path.realpath(renderLocation)
    newRenderLocation = getNewRenderLocation(renderLocation)

    # Test flags
    oldStats = {"passesChecks":False,"parentDir":renderLocation}
    newStats = {"passesChecks":False,"parentDir":newRenderLocation}

    # Check current location and new renders
    for stats in [oldStats, newStats]:
    
        if (oldStats["passesChecks"]): continue
        if (not os.path.exists( os.path.join(stats["parentDir"], rgbCameraList[0], timestampFileName) )): continue

        # Get sets of the RGBD timestamps
        stats["listOfOriginalTimestampSets"] = [getTimestampSetForCamera(c, stats["parentDir"]) for c in allCameraList] 

        # Get the timestamps of whole RGBD frames across both cameras
        stats["RGBDTimestampSet"] = set.intersection(*stats["listOfOriginalTimestampSets"])

        # Check if this render batch still meets minimum criteria, otherwise skip.
        stats["passesChecks"] = validateNewTimestampSet(stats["RGBDTimestampSet"])

        # Plan which tarballs/movs need to be unpacked
        stats["listOfDeletionsForAllCameras"] = [ camSet - stats["RGBDTimestampSet"] for camSet in stats["listOfOriginalTimestampSets"] ]

    # DEBUG: Print plan
    if (not oldStats["passesChecks"]):
        
        # This experiment MUST be rerendered.
        for stats in [oldStats, newStats]:
            print(stats["parentDir"])
            if ("listOfOriginalTimestampSets" in stats.keys()):
                for _camName,_originals,_deletions in zip(allCameraList, stats["listOfOriginalTimestampSets"], stats["listOfDeletionsForAllCameras"]):
                    if getFPSFromTimestampSet(_originals) < fpsCuttoff:
                        # This camera must be rerendered.
                        print(f"Bad {_camName} {oldStats['parentDir']}")
                #[print(f"{_camName}: {len(_originals)} frames, {len(_deletions)} deletions") for _camName,_originals,_deletions in zip(allCameraList, stats["listOfOriginalTimestampSets"], stats["listOfDeletionsForAllCameras"]) if getFPSFromTimestampSet(_originals) < fpsCuttoff ]
            else:
                print("Renders not found.")
        
        print("======")
        print("")
        # Do not create a plan for fixing this render.
        
        return

    # Execute plan on tarballs/movs if needed
    if (oldStats["passesChecks"]):
        if (sum([len(s) for s in oldStats["listOfDeletionsForAllCameras"]])):
            print(f"FIX: {oldStats['parentDir']}")
        else:
            print(f"GOOD: {oldStats['parentDir']}")


def main(datasetLocation, dryrun=True):
    # Find folders with 120hzTimestamp files.
    datasetLocation = os.path.realpath(datasetLocation)
    renderFolders = [ f.parent for f in Path(datasetLocation).glob('**/120hzTimestamps.csv')]

    ( passed, failedRGBSeg, failedDepth, failedOther ) = [ checkRender() ]

if __name__ == '__main__':
      fire.Fire(checkRender)

