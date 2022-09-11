#!/bin/bash

xhost +local:docker || true

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]; then
  docker run -ti --rm \
            --gpus all \
            -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
            -e NVIDIA_VISIBLE_DEVICES=all \
            -e "DISPLAY" \
            -e "QT_X11_NO_MITSHM=1" \
            -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            -e XAUTHORITY \
            -v /dev:/dev \
            -v $ROOT_DIR:/workspace \
            --net=host \
            --privileged \
            --name ur5rl ur5rl-img 

  elif [[ $1 = "--mac" ]] || [[ $1 = "-m" ]]; then
      docker run -ti --rm \
            -e DISPLAY=host.docker.internal:0 \
            -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            -e XAUTHORITY \
            -v /dev:/dev \
            -v $ROOT_DIR:/workspace \
            --net=host \
            --privileged \
            --name ur5rl ur5rl-img
  else
      echo "[!] If you wanna use nvidia gpu, please use script with -n or --nvidia argument 
[!] If you wanna use mac, please use script with -m or --mac argument"
      docker run -ti --rm \
            -e "DISPLAY" \
            -e "QT_X11_NO_MITSHM=1" \
            -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            -e XAUTHORITY \
            -v /dev:/dev \
            -v $ROOT_DIR:/workspace \
            --net=host \
            --privileged \
            --name ur5rl ur5rl-img
fi