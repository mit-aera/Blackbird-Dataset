# Render datasets at 60hz
#./renderDatasetUsingROSToHEVC.py --datasetFolder /media/medusa/NAS/Datasets/IJRRBlackbirdDataset/v3/BlackbirdDatasetData/ --renderDir /home/medusa/render_temp/ --render-prefix "ijrr_new_sensors_4"

# For Egg
./renderDatasetUsingROSToHEVC.py --datasetFolder /media/medusa/NAS/Datasets/IJRRBlackbirdDataset/v3/BlackbirdDatasetData/ --renderDir /media/medusa/NVME/home/medusa/render_temp/ --experimentList ["NYC_Subway_Station"] --render-prefix "ijrr_new_depth_sensor_2"

#./renderDatasetUsingROSToHEVC.py /media/medusa/RAID/IJRRBlackbirdDataset/v2/BlackbirdDatasetData/ /home/medusa/render_temp/ "_120hz_try_4" 121
