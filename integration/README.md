## How to run

* Set up environment by running ```. set_up_env.sh```
* Launch flight goggle by ```roslaunch flightgoggles core.launch ignore_collisions:=1 use_sim_time:=0```
* [For state estimation], ```roslaunch vins_estimator fg.launch```
* [Optional] Launch vins mono by ```roslaunch vins_estimator fg_rviz.launch```
* Run ```python2 run_state_machine_vins_mono_gt.py```

We are using Python2 here because tf package is supported in Python2.

* If running scorer, make sure scorer.py is in same directory when you do rosrun

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
1. Download this tar.gz http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
2. Make sure you have the following dependencies: (copy pasted from http://ceres-solver.org/installation.html)
```
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev
# - However, if you want to build Ceres as a *shared* library, you must
#   add the following PPA:
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev
```
3. Build, test, and install Ceres. 
```
tar zxf ceres-solver-1.14.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.14.0
make -j3
make test
make install
```
4. Build VINS-Mono on ROS
``` 
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
