# Description

## Algorithm Options
The first algorithm tried is simi-global-matching for depth estimation. It has cuda implementation and 
is fast in real-time setting. Not sure its performance under motion blur yet.

## Data Source
[6D_vision](http://www.6d-vision.com/scene-labeling)

## Build GPU version of OpenCV
* Build from source: https://gist.github.com/raulqf/a3caa97db3f8760af33266a1475d0e5e
* Fix zlib issue: https://stackoverflow.com/questions/48306849/lib-x86-64-linux-gnu-libz-so-1-version-zlib-1-2-9-not-found