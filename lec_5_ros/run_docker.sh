#!/usr/bin/env bash

docker run  -ti --rm \
            -e "DISPLAY" \
            -e "QT_X11_NO_MITSHM=1" \
            -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            -e XAUTHORITY \
            -v ./catkin_ws:/catkin_ws \
            --net=host \
            --privileged \
            --name simple_ros simple_ros_img
