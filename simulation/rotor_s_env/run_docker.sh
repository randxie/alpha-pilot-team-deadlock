#!/bin/bash
xhost +local:root

nvidia-docker run -it \
--device /dev/snd \
--device /dev/nvidia0 \
--device /dev/nvidiactl \
--device /dev/nvidia-modeset \
--device /dev/nvidia-uvm \
--device /dev/nvidia-uvm-tools \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v $XAUTHORITY:/tmp/.host_Xauthority:ro \
-v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket:ro \
-v /dev/shm:/dev/shm \
-v /etc/machine-id:/etc/machine-id \
-v /etc/localtime:/etc/localtime \
-v $PWD:/shared \
--net=host \
-e "DISPLAY=unix:1" -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged \
 alphapilot:1.0

xhost -local:root

# docker exec -it [container-id] bash
