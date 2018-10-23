#! /usr/bin/env node

var createTorrent = require('create-torrent')
var fs = require('fs')

// Load list of files to put in torrent index from disk
var fileList = fs.readFileSync('../downloadUtilities/torrentFileIndex.txt').toString().split("\n");

// Create streams
var streamList = fileList.map(fPath => {
    var stream = fs.createReadStream(fPath, { highWaterMark: 1024**2 })
    stream.name = fPath
    return stream 
});

console.log(fileList)

createTorrent(streamList,
    {
        // name: "BlackbirdDatasetImageRenders",            // name of the torrent (default = basename of `path`, or 1st file's name)
        name: "BlackbirdDatasetData",            // name of the torrent (default = basename of `path`, or 1st file's name)
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
            fs.writeFile('../downloadUtilities/blackbirdDataset.torrent', torrent)
        } else {
            console.error(err)
        }

    }
)

// ```
// For Academic torrent tracker:
// @inproceedings{antonini2018blackbird,
//     title= {The Blackbird Dataset: A large-scale dataset for UAV perception in aggressive flight},
//     author= {Antonini, Amado and Guerra, Winter and Murali, Varun and Sayre-McCord, Thomas and Karaman, Sertac},
//     booktitle= {2018 International Symposium on Experimental Robotics (ISER)},
//     year= {2018},
//     abstract= {The Blackbird unmanned aerial vehicle (UAV) dataset is a arge-scale indoor dataset collected using a custom-built quadrotor platform for use in evaluation of agile perception. The dataset contains over 10  hours  of  flight  data  from  168  flights  over  17  flight  trajectories  and 5 environments at velocities up to 8.0 m/s. Each flight includes sensordata from 120 Hz stereo and downward-facing photorealistic virtual cam-eras, 100 Hz IMU, 190 Hz motor speed sensors, and 360 Hz millimeter-accurate  motion  capture  ground  truth.  Camera  images  for  each  flight were photorealistically rendered using FlightGoggles across a variety of environments to facilitate experimentation of perception algorithms.The dataset is available at http://blackbird-dataset.mit.edu.},
//     keywords= {},
//     terms= {Copyright 2018 Sertac Karaman
    
//     Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
    
//     The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
    
//     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.},
//     license= {MIT License},
//     superseded= {},
//     url= {http://blackbird-dataset.mit.edu}
//     }
// ```