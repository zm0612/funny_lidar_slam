#!/bin/bash

function abspath() {
    # generate absolute path from relative path
    # $1     : relative filename
    # return : absolute path
    if [ -d "$1" ]; then
        # dir
        (cd "$1" || exit; pwd)
    elif [ -f "$1" ]; then
        # file
        if [[ $1 = /* ]]; then
            echo "$1"
        elif [[ $1 == */* ]]; then
            echo "$(cd "${1%/*}" || exit; pwd)/${1##*/}"
        else
            echo "$(pwd)/$1"
        fi
    fi
}

FUNNY_LIDAR_SLAM_DIR=$(abspath "..")

xhost +local:

docker run\
    -it \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --net host \
    --gpus all \
    --name funny_lidar_slam \
    -v "${FUNNY_LIDAR_SLAM_DIR}":/root/funny_lidar_slam_ws/src/funny_lidar_slam \
    funny_lidar_slam:v0 \
    /bin/bash