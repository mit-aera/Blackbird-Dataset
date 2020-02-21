#!/bin/bash

# Winter Guerra <winterg@mit.edu>
# Requires one folder argument.
# Checks that 60Hz 



# Get segmentation folders
SEGMENTATION_FOLDERS=`find -L $1 -iname "*_Segmented"`

# rosbag_Ancient_Asia_Museum_Room_ijrr_new_raycast_depth_5

# Prepare output file with list of good depth renders
echo "" > badSegRenders.txt
echo "" > badRGBRenders.txt
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
EXPECTED_FRAMES=$(( EXPECTED_FRAMES - 20 )) # Add a buffer

# Check that RGB, Semantic, & Depth renders have the correct number of frames
RGB_NUM_IMAGES=`cat ${FINAL_FOLDER_DEST}/${CAMERA}_RGB/video_frame_n_sec_timestamps.txt | wc -l`
SEG_NUM_IMAGES=`cat ${FOLDER}/video_frame_n_sec_timestamps.txt | wc -l`

if [ ! -d $DEPTH_FOLDER ]; then
  #echo "WARNING! $DEPTH_FOLDER DOES NOT exist." # Silent error for now
  #echo "$FINAL_FOLDER_DEST" >> badDepthRenders.txt
  #continue
  DEPTH_NUM_IMAGES=0
else
  # Check number of depth images
  DEPTH_NUM_IMAGES=`cat $DEPTH_FOLDER/video_frame_n_sec_timestamps.txt | wc -l`
fi
  
echo "RGB: $RGB_NUM_IMAGES, Seg: $SEG_NUM_IMAGES, Depth: $DEPTH_NUM_IMAGES, GT with buffer: $EXPECTED_FRAMES"


# Check if Depth render maps correctly to ground truth
if (( "$DEPTH_NUM_IMAGES" != "$RGB_NUM_IMAGES" )); then
  echo "WARNING! RGB: $RGB_NUM_IMAGES, Seg: $SEG_NUM_IMAGES, Depth: $DEPTH_NUM_IMAGES, GT with buffer: $EXPECTED_FRAMES"
  
  # Check that depth image number >= ground truth expected
  if (( "$DEPTH_NUM_IMAGES" < "$EXPECTED_FRAMES" )); then
    echo "Depth needs to be rerendered."
    echo "$FINAL_FOLDER_DEST" >> badDepthRenders.txt
  else
    # Depth is actually good!
    echo "$DEPTH_FOLDER $FINAL_FOLDER_DEST" >> goodDepthRenders.txt
  fi 
  
  # Check that segmented image number >= ground truth expected
  if (( "$SEG_NUM_IMAGES" < "$EXPECTED_FRAMES" )); then
    echo "Segmented needs to be rerendered."
    echo "$FOLDER" >> badSegRenders.txt
  fi 
 
  # Check that rgb image number >= ground truth expected
  if (( "$RGB_NUM_IMAGES" < "$EXPECTED_FRAMES" )); then
    echo "RGB needs to be rerendered."
    echo "${FINAL_FOLDER_DEST}/${CAMERA}_RGB" >> badRGBRenders.txt
  fi 
   
else
  # Output file is of form "render_loc final_parent_dir"
  echo "$DEPTH_FOLDER $FINAL_FOLDER_DEST" >> goodDepthRenders.txt
fi

echo "Processed $FOLDER"

done
