#!/bin/bash

# convert ffmpeg_decoding_table.txt to nanosecond timestamp file of name video_frame_n_sec_timestamps.txt
# 1534108150829255
find -name "ffmpeg_decoding_table.txt" | xargs realpath | parallel 'cat {} | sed -E "s%.*/%%" | sed -E "s%_.*%%" | sed "s/$/000/" > {//}/video_frame_n_sec_timestamps.txt'
