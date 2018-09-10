#!/usr/bin/env python2

import sys, subprocess, glob, os
import os.path as path
import datetime, time
import numpy as np
import fire
from more_itertools import peekable

# Global vars

class ImageHandler(object):
    def __init__(self, image_dir):
        # Config settings
        self.image_dir = path.normpath(path.expanduser(image_dir))

    def convertPPMsToPNGs(self):
        print "Starting PPM to PNG conversion"
        command = "find " + self.image_dir + " -iname *.ppm -type f -print0 | parallel --progress -0 -j +0 'mogrify -format png {}'"
        process = subprocess.Popen(command, shell=True)
        process.wait()
        print "Finished PPM to PNG conversion"

    def deletePPMs(self):
        print "Deleting temporary PPMs"
        ppm_files = glob.glob(os.path.join(self.image_dir,"*.ppm"))
        for ppm_file in ppm_files:
            os.remove(ppm_file)
        print "Finished deleting temporary PPMs"
    
    @staticmethod
    def getFileTimestamp(filename):
        timestamp = int(''.join(c for c in filename if c.isdigit()))
        return timestamp


    def encodeVideoTimestampsUsingPPMs(self, cameras, file_extension, fps):
        
        encoding_paths = []

        for camera_name in cameras:
            print "Creating encoding file for ffmpeg for camera: " + camera_name
            encoding_filename = camera_name + "_ffmpeg_encoding_table.txt"
            encoding_path = path.join(self.image_dir, encoding_filename)
            encoding_paths.append(encoding_path)
            # Get all ppm files
            files = sorted(glob.glob(path.join(self.image_dir, "*_" + camera_name + file_extension)))

            # Append to encoding file
            with open(encoding_path, "w") as f:
                for i, ppm_path in enumerate(files):
                    (unused, ppm_filename) = path.split(ppm_path)

                    if i < len(files)-1:
                        # Get name of next file
                        next_ppm_path = files[i+1]
                        # Calculate file display duration
                        (unused, next_ppm_filename) = path.split(next_ppm_path)
                        duration_us = self.getFileTimestamp(next_ppm_filename) - self.getFileTimestamp(ppm_filename)
                        duration_s = duration_us*1e-6

                        # Log to encoding file
                        f.write("file '" + ppm_path + "'\n")
                        #f.write("duration " + str(duration_s)+ "\n")

        print "Finished creating encoding file for ffmpeg"
        print "Starting video encoding using ffmpeg"
        inputs = ' '.join(" -f concat -safe 0 -r " + str(fps) + " -i " + encoding_path for encoding_path in encoding_paths)

        # Uses local ffmpeg compiled with NVENC GPU compression.
        # command = "/home/medusa/Downloads/ffmpeg-4.0.2/ffmpeg -y -loglevel fatal " + inputs + " -filter_complex hstack=inputs=" + str(len(cameras)) + " -vcodec hevc_nvenc -preset slow -rc vbr_minqp -qmin 20 -qmax 27 -tier high -pix_fmt yuv420p -r 60 " + path.join(self.image_dir, "video.mp4")

        # CPU-based h.264 compression (better compatibility and better quality)
        # Puts all 3 (1024x768) inputs in a tile pattern
        command = "ffmpeg -y -loglevel fatal " + inputs + """ -filter_complex 'nullsrc=size=2048x1536 [base]; \
 [0:v]setpts=PTS-STARTPTS, scale=1024x768[upperleft]; \
 [1:v]setpts=PTS-STARTPTS, scale=1024x768[upperright]; \
 [2:v]setpts=PTS-STARTPTS, scale=1024x768[lowermiddle]; \
 [base][upperleft]overlay=shortest=1[tmp1]; \
 [tmp1][upperright]overlay=shortest=1:x=1024[tmp2]; \
 [tmp2][lowermiddle]overlay=shortest=1:x=512:y=768[out]' """ + " -map '[out]' -crf 14 -vcodec libx264 -pix_fmt yuv420p -r 60 " + path.join(self.image_dir, "video.mp4")

        print command
            
        process = subprocess.Popen(command, shell=True)
        process.wait()
        print "Finished video encoding using ffmpeg"

    def createVideosAndCompress(self, cameras=['Camera_L', 'Camera_L_Depth', 'Camera_R'], file_extension=".png", fps=60):
        for camera in cameras:
            self.encodeVideoTimestampsUsingPPMs(camera, file_extension, fps)

    def createVideo(self, cameras=['Camera_L', 'Camera_R', 'Camera_D'], file_extension=".png", fps=120):
        self.encodeVideoTimestampsUsingPPMs(cameras, file_extension, fps)

####################
# HIGH LEVEL FUNCTIONS
####################

# def createVideo(image_dir, cameras=['Camera_L', 'Camera_R', 'Camera_D'], _file_extension=".png", _fps=60):
#     imageHandler = ImageHandler(image_dir)
#     imageHandler.encodeVideoTimestampsUsingPPMs(cameras, _file_extension, _fps)

# def compressImages(image_dir):
#     imageHandler = ImageHandler(image_dir)
#     imageHandler.convertPPMsToPNGs()
#     imageHandler.deletePPMs()

# def createVideosAndCompress(image_dir, cameras=['Camera_L', 'Camera_L_Depth', 'Camera_R'], _file_extension=".png", _fps=60):
#     for camera in cameras:
#         createVideo(image_dir, camera, file_extension=_file_extension, fps=_fps)
    
#     compressImages(image_dir)

# ###################
# # MAIN FUNCTION
# ###################

if __name__ == '__main__':

    fire.Fire({
        'createVideo': createVideo,
        'compressImages': compressImages,
        'createVideoAndCompress': createVideosAndCompress
    })
    
