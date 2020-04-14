#!/bin/bash

# convert ffmpeg_decoding_table.txt to nanosecond timestamp file of name nSecTimestamps.txt
# 1534108150829255
find -name "ffmpeg_decoding_table.txt" | xargs realpath | parallel 'cat {} | sed -E "s%.*/%%" | sed -E "s%_.*%%" | sed "s/$/000/" > {//}/nSecTimestamps.txt'
