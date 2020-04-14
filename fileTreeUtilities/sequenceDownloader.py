#!/usr/bin/env python3

'''
Winter Guerra <winterg@mit.edu>
April 13th, 2020

Usage: sequenceDownloader.py -flight ampersand/yawForward/maxSpeed2p0/ -environment Small_Apartment -datasetFolder ~/BlackbirdDatasetData
'''

import fire, wget, urllib
from pathlib import Path

# S3 address as URI (yes, /// is on purpose)
serverAddress = "ijrr-blackbird-dataset.s3-accelerate.amazonaws.com/BlackbirdDatasetData"

# Flight files that can be used for playback / analysis
flightFileList = [
    "rosbag.bag",
    "flightNormalizationOffset.csv",
    "groundTruthPoses.csv"
]

# These are not required, but may be helpful
csvFiles = [
    "blackbird_slash_imu.csv",
    "blackbird_slash_pose_ref.csv",
    "blackbird_slash_pwm.csv",
    "blackbird_slash_rotor_rpm.csv",
    "blackbird_slash_state.csv",
    "camera_d_slash_camera_info.csv",
    "camera_l_slash_camera_info.csv",
    "camera_r_slash_camera_info.csv",
    "tf.csv",
]

globalFiles = [
    "trajectoryOffsets.yaml"
]

environmentFiles = [
    "120hzTimestamps.csv"
]

cameras = [
    "Camera_Left_RGB",
    "Camera_Down_RGB",
    "Camera_Left_Gray",
    "Camera_Right_Gray",
    "Camera_Down_Gray",
    "Camera_Left_Depth",
    "Camera_Down_Depth",
    "Camera_Left_Segmented",
    "Camera_Down_Segmented",
]

# Files next to movie files.
# @TODO: change this to the IJRR filename in the PDF
timestampFile = "nSecTimestamps.txt"

def getCamFile(camName):
    return "lossless.tar" if ("Depth" in camName or "Segmented" in camName) else "lossless.mov"

def append( root, files ):
    return [ root / f for f in files]

# def download(relPaths, datasetFolder):
#     for relPath in relPaths:
#         # Make sure that directory exists
#         relPath.parent.mkdir(parents=True, exist_ok=True)
#         # Download the file
#         wget(serverAddress / relPath, datasetFolder / relPath)

def downloadSequence(flight, environment, datasetFolder):


    flight = Path(flight)
    environment = Path(environment)
    datasetFolder = Path(datasetFolder).expanduser()

    print(flight, environment, datasetFolder)

    # Inline definition
    def download(relPaths):
        for relPath in relPaths:
            relPath = Path(relPath)
            dest = datasetFolder / relPath
            src = "http://" + str(serverAddress / relPath)
            # Make sure that directory exists
            dest.parent.mkdir(parents=True, exist_ok=True)
            # Check that the file does not already exist
            if (not dest.exists()):
                # Download the file
                print(f"Downloading {relPath}")
                try:
                    wget.download(src, out=str(dest))
                except urllib.error.HTTPError as err:
                    print(err)
                
            else:
                print(f"Skipping {relPath}")

    download(globalFiles)
    download(append(flight / "csv", csvFiles))
    download(append(flight, flightFileList))
    download(append(flight / environment, environmentFiles))

    # Download camera files
    for cam in cameras:
        download( [
            flight / environment / cam / getCamFile(cam),
            flight / environment / cam / timestampFile
        ])



# CLI
if __name__ == "__main__":
    fire.Fire(downloadSequence)