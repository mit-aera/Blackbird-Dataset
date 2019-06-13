#!/bin/bash

#find $1 -name '*.bag' | parallel -I {} mv {} {}.orig

find $1 -name '*.bag' | parallel "rosbag filter {} {.}_filtered.bag 'topic != \"/tf\" or (topic == \"/tf\" and m.transforms[0].header.frame_id == \"mocap_NED\" and m.transforms[0].child_frame_id == \"body_frame\")' && rosbag reindex {.}_filtered.bag"
