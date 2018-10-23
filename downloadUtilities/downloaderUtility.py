#!/usr/bin/env python2

# Winter Guerra <winterg@mit.edu>
# October 21st, 2018

# Downloader script for BlackbirdDataset. Simultaneously uses HTTP and BitTorrent links to accelerate download speeds.
# Also allows for selectively downloading specific portions of the dataset based on speed, difficulty, environment, and trajectory.
# View README.md for usage instructions. 

import fire
import glob2, glob, os, sys, re
import numpy as np

# For parsing torrent files
import bencode

downloadUtilitiesLocation = os.path.dirname(os.path.realpath(__file__))

torrentFileIndexLocation = os.path.join(downloadUtilitiesLocation, "torrentFileIndex.txt")
torrentFileLocation = os.path.join(downloadUtilitiesLocation, "blackbirdDataset.torrent")
fullDatasetFileIndexLocation = os.path.join(downloadUtilitiesLocation, "fullDatasetFileIndex.csv")


def downloadSubset(flights, files, downloadLocation):

    # Evaluate filters for flights and files into python expressions.
    flightFilter =


# Dataset file index is torrentFileIndex of .tar files + a list of all other files.
def compileFileIndex(downloadLocation):
    nonTorrentFiles = glob2.glob( os.path.join(downloadLocation, '**/*.{csv,yaml,log,bag,mp4}') )

    # Load files that are in the torrent file 
    torrentFiles = []
    with open(torrentFileLocation, 'rb') as f:
        torrentFiles = bencode.bdecode(f.read())["info"]["files"]

    # Create numpy table of files
    numFiles = len(nonTorrentFiles) + len(torrentFiles)
    fileTable = np.zeros( numfiles, dtype={'names':['filePath', 'trajectory', 'topSpeed', 'difficulty', 'yawType', 'location', 'environment'], 'formats'=[]})


