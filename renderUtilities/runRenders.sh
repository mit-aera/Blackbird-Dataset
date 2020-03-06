# Render datasets at 60hz
#./renderDatasetUsingROSToHEVC.py --datasetFolder /media/medusa/NAS/Datasets/IJRRBlackbirdDataset/v3/BlackbirdDatasetData/ --renderDir /home/medusa/render_temp/ --render-prefix "ijrr_new_sensors_4"

# For Egg
# ./renderDatasetUsingROSToHEVC.py --datasetFolder /media/medusa/NAS/Datasets/IJRRBlackbirdDataset/v3/BlackbirdDatasetData/ --renderDir /media/medusa/NVME/home/medusa/render_temp/ --trajectoryFolders ["sphinx","halfMoon","oval","ampersand","dice","bentDice","thrice","tiltedThrice","winter","clover","mouse","patrick","picasso","sid","star"] --renderPrefix "ijrr_new_raycast_depth_5"

#./renderDatasetUsingROSToHEVC.py /media/medusa/RAID/IJRRBlackbirdDataset/v2/BlackbirdDatasetData/ /home/medusa/render_temp/ "_120hz_try_4" 121


./renderDatasetUsingROSToHEVC.py --datasetFolder /media/medusa/NAS/Datasets/IJRRBlackbirdDataset/v3/BlackbirdDatasetData/ --renderDir /media/medusa/NVME/home/medusa/render_temp/ --renderPrefix "ijrr_new_full_suite_2" --bagfileWhitelistFile "badRGBRenderListNormalized.txt"
