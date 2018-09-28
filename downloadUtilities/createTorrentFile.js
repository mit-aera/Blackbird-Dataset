#! /usr/bin/env node

var createTorrent = require('create-torrent')
var fs = require('fs')

// Load list of files to put in torrent index from disk
var fileList = fs.readFileSync('../downloadUtilities/torrentFileIndex.txt').toString().split("\n");

// Create streams
var streamList = fileList.map(fPath => {
    var stream = fs.createReadStream(fPath)
    stream.name = fPath
    return stream 
});

console.log(fileList)

createTorrent(streamList,
    {
        // name: "BlackbirdDatasetImageRenders",            // name of the torrent (default = basename of `path`, or 1st file's name)
        name: "data",            // name of the torrent (default = basename of `path`, or 1st file's name)
        comment: "The Blackbird Dataset: A large-scale dataset for UAV perception in aggressive flight",         // free-form textual comments of the author
        announceList: [
            ["http://academictorrents.com/announce.php"],

            ["udp://tracker.coppersurfer.tk:6969"],
            
            ["udp://tracker.opentrackr.org:1337/announce"],
            
            ["udp://tracker.leechers-paradise.org:6969"]
        ], // custom trackers (array of arrays of strings) (see [bep12](http://www.bittorrent.org/beps/bep_0012.html))
        urlList: ["http://blackbird-dataset.mit.edu/"],        // web seed urls (see [bep19](http://www.bittorrent.org/beps/bep_0019.html))
        pieceLength: 128000000 // 128MB chunks to keep torrent file under 1MB 
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