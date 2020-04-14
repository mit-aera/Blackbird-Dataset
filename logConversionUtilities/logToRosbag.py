#!/usr/bin/env python3

"""Process lcm log and write it to a rosbag file according to a json configuration file."""
__author__ = "Amado Antonini"

import rospy

import os
import json
import glob2
import lcm
import rosbag
import fire
from tqdm import tqdm

from msgConverters import *
from sensor_msgs.msg import CameraInfo
import numpy as np
from multiprocessing import Pool

class RosbagWriter:
    """
    Manager for rosbag writer and message converters.

    For all messages in an lcm log, if the channel is in 'lcm_channels', call the correct
    message converter from msgConverters and then write the ros message to rosbag.

    Attributes:
        verbose (bool): Whether to print info to console
        bag_path (str): Path to rosbag to write
        bag (rosbag.Bag): Rosbag writer
        switcher (dict): Mapping from lcm channel name to converter from msgConverters
        counter (dict): Counter for each channel's seen messages

    """

    def __init__(self, bag_path, settings):
        self.verbose = settings["log-wise_verbose"]
        self.bag_path = bag_path
        self.bag = rosbag.Bag(self.bag_path, 'w')
        self.switcher = {
            "poseMoCap":    PoseMoCapConverter(settings["lcm_channels"]["poseMoCap"], settings["image_timestamps"]),
            "imuRaw":       ImuConverter(settings["lcm_channels"]["imuRaw"]),
            "poseRef":      PoseRefConverter(settings["lcm_channels"]["poseRef"]),
            "rpmFeedback":  RpmConverter(settings["lcm_channels"]["rpmFeedback"]),
            "motorsPwms":   PwmConverter(settings["lcm_channels"]["motorsPwms"])
        }
        self.counter = dict()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.verbose:
            print(("Wrote rosbag to %s" % self.bag_path))
        self.bag.close()

    def write_message(self, channel, data):
        topics, ros_msgs = self.switcher.get(channel, self.unknown_channel)(channel, data)
        if self.verbose:
            print(topics)
        for topic, ros_msg in zip(topics, ros_msgs):
            # print topic, ros_msg
            if hasattr(ros_msg, 'header'):
                self.bag.write(topic, ros_msg, ros_msg.header.stamp)
            else:
                # This must be a TFMessage, which has no header.
                self.bag.write(topic, ros_msg, ros_msg.transforms[0].header.stamp)

    def write_ros_msg(self, topic, msg):
        self.bag.write(topic, msg, msg.header.stamp)

    def get_count(self, channel):
        self.counter[channel] = self.counter.get(channel, -1) + 1
        return self.counter[channel]

    def unknown_channel(self, channel, _):
        if self.verbose:
            print(("Warning: Writer for channel '%s' has not been implemented, skipping." % channel))


def process_one_log(args):
    """
    Walk through one lcm log converting desired channels to a rosbag.

    Args:
        log_path (str): Path to raw log file to convert to ROS
        rosbag_path (str): Path to rosbag to be created
        settings (dict): Container for the channels to write and their channel-specific settings

    """
    log_path, rosbag_path, csv_path, settings_static = args

    settings = settings_static.copy()

    lcm_channels = settings["lcm_channels"]

    # open the lcm log
    try:
        log = lcm.EventLog(log_path, "r")
    except IOError:
        sys.stderr.write("Unable to read " + log_path)
        sys.exit(1)

    # Set image timestamps
    poseList = np.loadtxt(open(csv_path, "rb"), delimiter=",")
    settings["image_timestamps"] = poseList[:,0]

    # open the rosbag writer object
    with RosbagWriter(rosbag_path, settings) as bag_writer:
        # walk through and process the messages in the lcm log
        #t = tqdm(total=log.size(), bar_format="{postfix[1][value]:6.0f}% {postfix[0]}",
        #         postfix=[os.path.basename(log_path), dict(value=0)])

        # walk through the log events and save them to bag.
        for event in log:
            #t.postfix[1]["value"] = float(log.tell())/log.size()*100
            #t.update()
            if event.channel in lcm_channels:
                bag_writer.write_message(event.channel, event.data)
        

        # # Add camera info message for each published image.
        # cam_info = CameraInfo()
        # cam_info.height = 768
        # cam_info.width  = 1024
        # cam_info.distortion_model = 'plumb_bob'
        # cam_info.K = [665.107510106, 0., 511.5, 0., 665.107510106, 383.5, 0., 0., 1.]
        
        # for pose in poseList:
        #     cam_info.header = Header()
        #     # Needs frame_id + seq.
        #     cam_info.header.stamp = rospy.Time.from_sec(pose[0]*1e-6)
        #     bag_writer.write_ros_msg("/cameraD/camera_info", cam_info) 
            
        #     cam_info.P = [665.107510106, 0., 511.5, 0., 0., 665.107510106, 383.5, 0., 0., 0., 1., 0]
        #     bag_writer.write_ros_msg("/cameraL/camera_info", cam_info) 
        #     cam_info.P = [665.107510106, 0., 511.5, 0.1, 0., 665.107510106, 383.5, 0.1, 0., 0., 1., 0]
        #     bag_writer.write_ros_msg("/cameraR/camera_info", cam_info) 

        # Make sure that bag is correctly timestamped
        # bag_writer.bag.reindex()

def process_logs(settings_file):
    """
    Convert all lcm logs in a directory to rosbags.

    Args:
        dir_settings (dict): Container for all settings for the operation

    """

    # Load settings
    with open(settings_file, 'r') as f:
        dir_settings = json.load(f)

    dir_lcm_logs = os.path.expanduser(dir_settings["dir_lcm_logs"])
    if not os.path.isdir(dir_lcm_logs):
        sys.stderr.write("%s is not a directory." % dir_lcm_logs)
        sys.exit(1)

    dir_rosbags = os.path.expanduser(dir_settings["dir_rosbags"])
    if os.path.exists(dir_rosbags):
        if not os.path.isdir(dir_rosbags):
            sys.stderr.write("%s exists but it's not a directory." % dir_rosbags)
            sys.exit(1)
        elif os.listdir(dir_rosbags) and not dir_settings["override"]:
            sys.stderr.write("The given directory, %s, exists but it's not empty.\n"
                             "Set 'override' to true if this is intended." % dir_rosbags)
            sys.exit(1)
    else:
        os.makedirs(dir_rosbags)

    
    log_list = [f for f in glob2.glob(os.path.join(dir_lcm_logs,'**/*.log'))]
    num_logs = len(log_list)

    print(("Converting %r logs from %s to %s" % (num_logs, dir_lcm_logs, dir_rosbags)))

    # Helper function for parallel processing
    def getArgsForParallelMap(logname):
        log_path = os.path.join(dir_lcm_logs, logname)
        rosbag_path = os.path.join(dir_rosbags, os.path.splitext(logname)[0] + '.bag')
        csv_path = os.path.join(dir_lcm_logs, os.path.splitext(logname)[0] + '_poses_centered.csv')
        
        rosbag_path = rosbag_path.replace(dir_lcm_logs, dir_rosbags)
        os.makedirs(os.path.split(rosbag_path)[0])
        
        return (log_path, rosbag_path, csv_path, dir_settings)

    tasks = [getArgsForParallelMap(log) for log in log_list]

    threads = Pool()
    # Shows progress.
    for _ in tqdm(threads.imap_unordered(process_one_log, tasks), total=len(tasks)):
        pass
    threads.close()
    threads.join()

    # for i, log_name in enumerate(log_list):
    #     # print overall progress
    #     progress = (i+1)*100./num_logs
    #     print("%d%%\t" % progress)
    #     log_path = os.path.join(dir_lcm_logs, log_name)
    #     rosbag_path = os.path.join(dir_rosbags, os.path.splitext(log_name)[0] + '.bag')
    #     print dir_rosbags

    #     csv_path = os.path.join(dir_lcm_logs, os.path.splitext(log_name)[0] + '_poses_centered.csv')
    #     rosbag_path = rosbag_path.replace(dir_lcm_logs, dir_rosbags)
    #     os.makedirs(os.path.split(rosbag_path)[0])
    #     print rosbag_path
    #     print log_path
    #     print csv_path
    #     process_one_log((log_path, rosbag_path, csv_path, dir_settings))

    print(("Done writing logs to %s ." % dir_rosbags))


if __name__ == '__main__':

    fire.Fire(process_logs)

