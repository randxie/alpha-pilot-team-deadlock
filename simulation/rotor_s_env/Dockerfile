FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04

RUN apt-get update && \
    apt-get -y install wget

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && \
    apt-get -y install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox python-pip ros-kinetic-geographic-msgs libgeographic-dev && \
    pip install future

RUN rm /bin/sh && \
    ln -s /bin/bash /bin/sh

RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    source /opt/ros/kinetic/setup.bash && \
    catkin_init_workspace && \
    wstool init && \
    wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall && \
    wstool merge rotors_hil.rosinstall && \
    wstool update

RUN cd ~/catkin_ws/src/rotors_simulator/rotors_hil_interface/ && \
    touch CATKIN_IGNORE && \
    cd ~/catkin_ws/ && \
    source /opt/ros/kinetic/setup.bash && \
    catkin build && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    /bin/sh ~/.bashrc && \
    . ~/catkin_ws/devel/setup.bash

#RUN roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
#  roslaunch gazebo_ros empty_world.launch
