#!/bin/bash

# Winter Guerra <winterg@mit.edu>
# Requires one folder argument.

# Get segmentation folders
SEGMENTATION_FOLDERS=`find -L $1 -iname "*_Segmented"`

# rosbag_Ancient_Asia_Museum_Room_ijrr_new_raycast_depth_5

# Prepare output file with list of good depth renders
echo "" > goodDepthRenders.txt
echo "" > badDepthRenders.txt


# For each folder:
for FOLDER in $SEGMENTATION_FOLDERS; do

# Check that the depth folder exists
EXPERIMENT=`echo $FOLDER | awk -F '/' '{print $(NF-1)}'`
CAMERA=`echo $FOLDER | awk -F '/' '{print $NF}' | sed s%_Segmented%%`
DEPTH_FOLDER=${FOLDER}/../../rosbag_${EXPERIMENT}_ijrr_new_raycast_depth_5/${CAMERA}_Depth
FINAL_FOLDER_DEST=${FOLDER}/../

NUM_GRAY_FRAMES=`cat ${FINAL_FOLDER_DEST}/120HzTimestamps.csv | wc -l`
EXPECTED_FRAMES=$(( NUM_GRAY_FRAMES / 2 ))

if [ ! -d $DEPTH_FOLDER ]; then
  echo "WARNING! $DEPTH_FOLDER DOES NOT exist."
  echo "$FINAL_FOLDER_DEST" >> badDepthRenders.txt
  continue
fi
  

# Compare the number of images in both tarballs
REF_NUM_IMAGES=`tar -tf ${FOLDER}/lossless.tar | wc -l`
CUR_NUM_IMAGES=`tar -tf $DEPTH_FOLDER/lossless.tar | wc -l`

echo "Processed $FOLDER"

# Print an errors
if [ "$REF_NUM_IMAGES" -ne "$CUR_NUM_IMAGES" ]; then
  echo "WARNING! $FOLDER does not have all images. Ref: $REF_NUM_IMAGES != $CUR_NUM_IMAGES. Expected: $EXPECTED_FRAMES"
  echo "$FINAL_FOLDER_DEST" >> badDepthRenders.txt
else
  # Output file is of form "render_loc final_parent_dir"
  echo "$DEPTH_FOLDER $FINAL_FOLDER_DEST" >> goodDepthRenders.txt
fi

echo 

done
