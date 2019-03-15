## How to run

* Set up environment by running ```. set_up_env.sh```
* Launch flight goggle by ```roslaunch flightgoggles core.launch ignore_collisions:=1 use_sim_time:=0```
* [For state estimation], ```roslaunch vins_estimator fg.launch```
* [Optional] Launch vins mono by ```roslaunch vins_estimator fg_rviz.launch```
* Run ```python2 run_state_machine_vins_mono.py```

We are using Python2 here because tf package is supported in Python2.

## Caution
* This issue should have been fixed because of an issue with the Queue, but if you see the action's value is hanging there, stop and rerun the python script immediately.

## Installing [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) 
## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu  16.04.
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```


1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 16.04, ROS Kinetic, OpenCV 3.3.1, Eigen 3.3.3) 

## 2. Build VINS-Mono on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
