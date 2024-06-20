#!/usr/bin/env bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Specific for NVIDIA drivers, required for OpenGL >= 3.3
docker run -it \
    --rm \
    --name orca4 \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "/home/parallels/projects/orca4/orca_bringup/cfg/sub.parm:/home/orca4/colcon_ws/src/orca4/orca_bringup/cfg/sub.parm:ro" \
    -e LIBGL_ALWAYS_SOFTWARE=1  \
    --privileged \
    --security-opt seccomp=unconfined \
    orca4:latest
