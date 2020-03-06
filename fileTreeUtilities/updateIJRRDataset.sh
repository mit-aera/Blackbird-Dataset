#!/bin/bash

aws s3 sync --delete /media/medusa/NAS/Datasets/IJRRBlackbirdDataset/v3/BlackbirdDatasetData s3://ijrr-blackbird-dataset/BlackbirdDatasetData/
