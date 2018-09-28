#! /usr/bin/env node

var createTorrent = require('create-torrent')
var fs = require('fs')

// Load list of files to put in torrent index from disk
var fileList = fs.readFileSync('torrentFileIndexSmall.txt').toString().split("\n");

console.log(fileList)

createTorrent(fileList,
    {
        // name: "BlackbirdDatasetImageRenders",            // name of the torrent (default = basename of `path`, or 1st file's name)
        name: "data",            // name of the torrent (default = basename of `path`, or 1st file's name)
        comment: "The Blackbird Dataset: A large-scale dataset for UAV perception in aggressive flight",         // free-form textual comments of the author
        announceList: [
            ["udp://tracker.openbittorrent.com:80"],
            ["udp://tracker.internetwarriors.net:1337"],
            ["udp://tracker.leechers-paradise.org:6969"],
            ["udp://tracker.coppersurfer.tk:6969"],
            ["udp://exodus.desync.com:6969"],
            ["wss://tracker.btorrent.xyz"],
            ["wss://tracker.openwebtorrent.com"],
            ["wss://tracker.fastcast.nz"]
        ], // custom trackers (array of arrays of strings) (see [bep12](http://www.bittorrent.org/beps/bep_0012.html))
        urlList: ["http://blackbird-dataset.mit.edu/"]        // web seed urls (see [bep19](http://www.bittorrent.org/beps/bep_0019.html))
    }, 
    function (err, torrent) {
        if (!err) {
            // `torrent` is a Buffer with the contents of the new .torrent file
            fs.writeFile('blackbirdDataset.torrent', torrent)
        } else {
            console.error(err)
        }

    }
)