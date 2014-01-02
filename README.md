JPSA* path finding
==================

There was no simple to use implementation of jump point search A*
for C++ and OpenCV available. This implementation is based on the
paper "Online Graph Pruning for Pathfinding On Grid Maps" written
by Daniel Harabor and Alban Grastien.

### Dependencies
* C++03
* OpenCV

### Usage
Integrate the following files

* JPSAStar.cpp
* JPSAStar.hpp 

into your build setup, add OpenCV dependencies and you are ready
to go.


jpsastar tool and unit tests
----------------------------

### Dependencies
* C++11
* OpenCV
* Boost
* Google Test

### Compile
Run following commands in the repository root directory:

    mkdir build
    cd build
    cmake ..
    make -j3
    make doc

### Usage
    bin/jpsastar path_to_map_image

First click on the image sets the start point. The second click
sets the target point. The path will be displayed in green.
