# Alpha Pilot: Team DeadLock

## Introduction

This repo contains all the code that Team Deadlock developed in the [alpha pilot competition](https://www.lockheedmartin.com/en-us/news/events/ai-innovation-challenge.html). Developing a vision-based autonomous flying system is challenging, and we are proud of building this prototype from scratch in 3 months. Now, all the trails and failures we had are open-sourced. This hopefully can provide some hints for future participants.

## Tasks for qualifier
* Vision
    * Object detection
    * Depth estimation
* Control
    * Quadcopter controller
        * Cascading PID
    * Trajectory generation
        * MST
* Simulation
    * Simple dynamic simulation in numpy
    * ROS full simulation
* SLAM
    * Visual inertial odometry (use VINS-MONO)
    
## Contribution guide
If you are working on a subcomponent, feel free to commit to the corresponding folder directly. If you are trying to modify the integration folder, please code in another branch and submit a pull request. Something to pay attention before submitting your code.

* Remove unncessary data (especially large files) to avoid exceeding repo limit (1 GB)
* When you are using a third party package, either specify its version (e.g. tensorflow==1.12) or bring in the source code (C++ project, unless it is under GPL license).
