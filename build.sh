#!/bin/bash

if [ "$1" = "arm" ]
then
    echo "arm"
    if [ "$2" = "1" ]
    then
        rm -rf build_arm
        mkdir build_arm
        cd build_arm
        cmake -D CMAKE_BUILD_TYPE=RELEASE ..
    else
        cd build_arm
    fi

    make $3
else
    echo $(arch)
    if  [ "$2" = "1" ]
    then
        rm -rf build
        mkdir build
        cd build
        cmake -D CMAKE_BUILD_TYPE=RELEASE ..
    else
        cd build
    fi
    
    make $3
fi