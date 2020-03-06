#!/bin/bash

# Winter Guerra <winterg@mit.edu>
# Requires one folder argument.
# Finds all subfolders with renders and checks that each render has all frames for all RGBD feeds.


# Get folders of reference Grayscale renders
GRAY_FOLDERS=`find -L $1 -iname "*Left_Gray"`

# Prepare output file with list of good depth renders
echo "" > badGrayRenders.txt
echo "" > goodNewRenders.txt
echo "" > badNewRenders.txt

# For each folder:
for GRAY_FOLDER in $GRAY_FOLDERS; do

  EXPERIMENT=`echo $GRAY_FOLDER | awk -F '/' '{print $(NF-1)}'`
  CAMERA=`echo $GRAY_FOLDER | awk -F '/' '{print $NF}' | sed s%_Gray%%`
  NEW_FOLDER=${GRAY_FOLDER}/../../rosbag_${EXPERIMENT}_ijrr_new_full_suite_1/${CAMERA}_RGB
  FINAL_FOLDER_DEST=`realpath ${GRAY_FOLDER}/../`
  
  # Get number of Grayscale frames
  GT_NUM_FRAMES=`wc -l < ${FINAL_FOLDER_DEST}/120HzTimestamps.csv`
  EXPECTED_120_FRAMES=$(( GT_NUM_FRAMES - 20 )) # Add a buffer
  EXPECTED_60_FRAMES=$(( GT_NUM_FRAMES / 2 ))
  EXPECTED_60_FRAMES=$(( EXPECTED_60_FRAMES - 20 )) # Add a buffer
  
  # Check that Current RGB & Depth renders have the correct number of frames
  GRAY_NUM_IMAGES=`wc -l < ${FINAL_FOLDER_DEST}/${CAMERA}_Gray/video_frame_n_sec_timestamps.txt`
  

  # Check that the new renders have the correct number of frames (if folder exists)
  NEW_RGB_NUM_IMAGES=0
  if [ -d $NEW_FOLDER ]; then
    # Check number of new images
    NEW_RGB_NUM_IMAGES=`wc -l < ${NEW_FOLDER}/video_frame_n_sec_timestamps.txt`
  fi
    
  # Check that the new renders have the correct number of frames (if folder exists)
  RGB_NUM_IMAGES=0
  if [ -d ${FINAL_FOLDER_DEST}/${CAMERA}_RGB ]; then
    # Check number of new images
    RGB_NUM_IMAGES=`wc -l < ${FINAL_FOLDER_DEST}/${CAMERA}_RGB/video_frame_n_sec_timestamps.txt`
  fi

  # Check that the new renders have the correct number of frames (if folder exists)
  DEPTH_NUM_IMAGES=0
  if [ -d ${FINAL_FOLDER_DEST}/${CAMERA}_Depth ]; then
    # Check number of new images
    DEPTH_NUM_IMAGES=`wc -l < ${FINAL_FOLDER_DEST}/${CAMERA}_Depth/video_frame_n_sec_timestamps.txt`
  fi

  echo "Expected Frames from GT: $EXPECTED_60_FRAMES, Orig Gray: $GRAY_NUM_IMAGES, Orig RGB: $RGB_NUM_IMAGES, Orig Depth: $DEPTH_NUM_IMAGES, New: $NEW_RGB_NUM_IMAGES"
  
  # Cases to check:
  #########
  # Originals are good: (Gray >= 120Hz minimum) && (Orig RGB >=  60Hz minimum ) && (Orig Depth == Orig RGB )
  # New renders are good: (Gray >= 120Hz minimum) && (New RGB >=  60Hz minimum ) 
  ### if something is wrong: ###
  # Orig Gray bad: (Gray < 120Hz Minimum)
  # New renders are bad: (New RGB <  60Hz minimum )
  ####

  # Check that 
  if (( "$GRAY_NUM_IMAGES" >= "$EXPECTED_120_FRAMES" )) && (( "$RGB_NUM_IMAGES" >= "$EXPECTED_60_FRAMES" )) && (( "$DEPTH_NUM_IMAGES" == "$RGB_NUM_IMAGES" )); then
    # THE ORIGINALS ARE GOOD!
    echo Good!
  elif (( "$GRAY_NUM_IMAGES" >= "$EXPECTED_120_FRAMES" )) && (( "$NEW_RGB_NUM_IMAGES" >= "$EXPECTED_60_FRAMES" )); then
    # The new renders are good!
    echo Good!
    echo "`realpath $NEW_FOLDER` $FINAL_FOLDER_DEST" >> goodNewRenders.txt

  else 
	  
    if (( "$GRAY_NUM_IMAGES" < "$EXPECTED_120_FRAMES" )); then
      # The original gray images are bad
      echo Bad original grayscale images!
      echo "$FOLDER" >> badGrayOriginals.txt

    fi 
    if (( "$NEW_RGB_NUM_IMAGES" < "$EXPECTED_60_FRAMES" )); then
      # The new renders are bad
      echo Bad new renders!
      echo "$FINAL_FOLDER_DEST" >> badNewRenders.txt
    fi
  
  fi

   # # Check that depth image number >= ground truth expected
   # if (( "$DEPTH_NUM_IMAGES" < "$EXPECTED_60_FRAMES" )); then
   #   echo "Depth needs to be rerendered."
   #   echo "$FINAL_FOLDER_DEST" >> badDepthRenders.txt
   # else
   #   # Depth is actually good!
   #   echo "$NEW_FOLDER $FINAL_FOLDER_DEST" >> goodDepthRenders.txt
   # fi 
   # 
   # # Check that segmented image number >= ground truth expected
   # if (( "$SEG_NUM_IMAGES" < "$EXPECTED_60_FRAMES" )); then
   #   echo "Segmented needs to be rerendered."
   #   echo "$FOLDER" >> badSegRenders.txt
   # fi 
   #
   # # Check that rgb image number >= ground truth expected
   # if (( "$RGB_NUM_IMAGES" < "$EXPECTED_60_FRAMES" )); then
   #   echo "RGB needs to be rerendered."
   #   echo "${FINAL_FOLDER_DEST}/${CAMERA}_RGB" >> badRGBRenders.txt
   # fi 
     
  #else
  #  # Output file is of form "render_loc final_parent_dir"
  #  echo "$NEW_FOLDER $FINAL_FOLDER_DEST" >> goodDepthRenders.txt
  #fi
  
  echo "Processed $FOLDER"

done
