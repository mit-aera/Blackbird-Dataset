#!/bin/bash
trap "exit" INT

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

for i in `seq 0 24`;
do
        echo "Running setup: $i" 
        
       $DIR/tileVideos.rb 3072x2160 4x5 tile_${i}.mp4 `find -name "*run_${i}_*cropped.mp4" | sort`

done
