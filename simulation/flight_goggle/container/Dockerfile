FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04

RUN apt-get update && \
    apt-get -y install wget

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && \
    apt-get -y install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-rosinstall ros-kinetic-control-toolbox ros-kinetic-geographic-msgs && \
    apt-get -y install python-rosinstall-generator python-wstool python-catkin-tools build-essential protobuf-compiler libgoogle-glog-dev python-pip libgeographic-dev && \
    pip install future && \
    pip install catkin_pkg

RUN rm /bin/sh && \
    ln -s /bin/bash /bin/sh

RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/ && \
    source /opt/ros/kinetic/setup.bash && \
    catkin init && \
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc && \
    rosdep init && \
    rosdep update

RUN cd ~/catkin_ws/src && \
    source /opt/ros/kinetic/setup.bash && \
    wstool init && \
    wstool merge https://raw.githubusercontent.com/mit-fast/FlightGoggles/master/flightgoggles.rosinstall && \
    wstool update

RUN cd ~/catkin_ws && \
    source /opt/ros/kinetic/setup.bash && \
    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y && \
    apt install -y libzmqpp-dev libeigen3-dev && \
    apt install -y libvulkan1 mesa-vulkan-drivers vulkan-utils && \
    cd ~/catkin_ws/src/flightgoggles && \
    git pull && \
    catkin clean flightgoggles && \
    catkin build && \
    source ~/.bashrc
