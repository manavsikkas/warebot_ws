#!/bin/bash

# 1. Allow the container to use your display (for RViz/Gazebo)
xhost +local:docker

# 2. Run the container
docker run -it --rm \
    --runtime nvidia \
    --network host \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/warebot_ws:/warebot_ws \
    --name warebot_dev \
    warebot-jazzy
