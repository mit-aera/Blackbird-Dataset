#!/bin/bash

aws s3 sync  --exclude "*videos*" --delete /media/medusa/NAS/Datasets/IJRRBlackbirdDataset/static_index/ s3://blackbird-dataset-static-index/
