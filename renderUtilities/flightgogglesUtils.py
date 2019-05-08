#!/usr/bin/env python2

import sys, subprocess, glob, os
import os.path as path
import datetime, time
import numpy as np
import fire
#from more_itertools import peekable

# Global vars

class ImageHandler(object):
    def __init__(self, image_dir):
        # Config settings
        self.image_dir = path.normpath(path.expanduser(image_dir))
	print "Starting operations on image directory: " + self.image_dir

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


def encodeVideoTimestampsUsingPPMs(input_folder, file_extension, fps, ffmpeg_folder="", output_folder=None):
    if (output_folder is None):
        output_folder=input_folder

    output_folder = output_folder.replace(".","")

    # Make sure that output folder exists
    process = subprocess.Popen("mkdir -p "+output_folder, shell=True)
    process.wait()

    files = sorted(glob.glob(path.join(input_folder, "*" + file_extension)))
    
    # Prefetch files
    print "Prefetching files to process"
    command = "find "+input_folder+" -name *"+file_extension+" | parallel -j100 vmtouch -tq "
    #print command
    process = subprocess.Popen(command, shell=True)
    process.wait()
    #time.sleep(10)

    decoding_path = path.join(output_folder, "ffmpeg_decoding_table.txt")
    with open(decoding_path, "w") as f:
        for fileString in files:
                f.write(fileString)
                f.write('\n')

    print "Finished creating decoding file for ffmpeg"
    print "Starting video encoding using ffmpeg"

    # UNUSED EXAMPLE: Uses local ffmpeg compiled with NVENC GPU compression.
    # Results in worse quality
    # command = "/home/medusa/Downloads/ffmpeg-4.0.2/ffmpeg -y -loglevel fatal " + inputs + " -filter_complex hstack=inputs=" + str(len(cameras)) + " -vcodec hevc_nvenc -preset slow -rc vbr_minqp -qmin 18 -qmax 19 -tier high -pix_fmt yuv420p -r 60 " + path.join(input_folder, "video.mp4")

    inputSources = " -thread_queue_size 512 -framerate "+str(fps)+" -pattern_type glob -i '" + path.join(input_folder, "*" + file_extension) + "' " 

    #encoderSettings = " -vcodec hevc_nvenc -preset lossless -tier high -pix_fmt yuv420p "
    #encoderSettings = " -vcodec libx265 -crf 0 -pix_fmt yuv420p -preset slow "

    # GPU Encoder. Requires FFMPEG to have been compiled with GPU support
    encoderSettings = " -vcodec hevc_nvenc -2pass 1 -preset lossless -tier high -pix_fmt yuv444p "

    outputRateSettings = " -r "+str(fps)+" "

    # DEFINE CAMERA TILING FILTERS

    # Puts all 3 (1024x768) inputs in a tile pattern
    #tileFilter = " -filter_complex 'color=s=2048x1536:c=black:rate="+str(fps)+ """[base]; \
    #[0:v]scale=1024x768[upperleft]; \
    #[1:v]scale=1024x768[upperright]; \
    #[2:v]scale=1024x768[lowermiddle]; \
    #[base][upperleft]overlay=shortest=1[tmp1]; \
    #[tmp1][upperright]overlay=shortest=1:x=1024[tmp2]; \
    #[tmp2][lowermiddle]overlay=shortest=1:x=512:y=768[out]' -map '[out]' """

    # Stacks cameras left to right horizontally
    #hstackFilter = " -filter_complex hstack=inputs=" + str(len(cameras)) + " "
    # Passthrough for monocam video
    passthroughFilter = " "

    # Pick the right filter to use.
    #cameraFilter = ""
    #if (len(cameras) is 3):
    #    cameraFilter = tileFilter
    #elif (len(cameras) is 1):
    #    cameraFilter = passthroughFilter
    #else:
    #    cameraFilter = hstackFilter

    cameraFilter = passthroughFilter

    # CPU-based h.264 compression (better compatibility and better quality)
    #command = "~/software/ffmpeg/ffmpeg -y " + inputSources + cameraFilter + encoderSettings + outputRateSettings + path.join(input_folder, camera_name+"_lossless.mov")
    command = ffmpeg_folder+ "ffmpeg -y " + inputSources + cameraFilter + encoderSettings + outputRateSettings + path.join(output_folder, "lossless.mov")


    print command
        
    process = subprocess.Popen(command, shell=True)
    process.wait()
    print "Finished video encoding using ffmpeg"

    print "Evicting cache"
    process = subprocess.Popen("vmtouch -e "+input_folder, shell=True)
    process.wait()
 

def createVideo(self, cameras=['Camera_L', 'Camera_R', 'Camera_D'], file_extension=".png", fps=60):
    self.encodeVideoTimestampsUsingPPMs(cameras, file_extension, fps)

####################
# HIGH LEVEL FUNCTIONS
####################

def createVideo(image_dir, cameras=['Camera_L', 'Camera_R', 'Camera_D'], file_extension=".png", fps=60):
    imageHandler = ImageHandler(image_dir)
    print cameras
    imageHandler.encodeVideoTimestampsUsingPPMs(cameras, file_extension, fps)

# def compressImages(image_dir):
#     imageHandler = ImageHandler(image_dir)
#     imageHandler.convertPPMsToPNGs()
#     imageHandler.deletePPMs()

def createVideos(image_dir, cameras=['Camera_L', 'Camera_L_Depth', 'Camera_R'], _file_extension=".png", _fps=120):
    for camera in cameras:
        createVideo(image_dir, [camera], file_extension=_file_extension, fps=_fps)
    
#     compressImages(image_dir)

# ###################
# # MAIN FUNCTION
# ###################
if __name__ == '__main__':

     fire.Fire()
    
