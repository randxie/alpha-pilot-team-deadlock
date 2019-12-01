# Alpha Pilot: Team DeadLock

## Introduction

This repo contains all the code that Team Deadlock developed in the [alpha pilot competition](https://www.lockheedmartin.com/en-us/news/events/ai-innovation-challenge.html). Developing a vision-based autonomous flying system is challenging, and we are proud of building this prototype from scratch in 3 months. Now, all the trails and failures we had are open-sourced. This hopefully can provide some hints for future participants.

## Tasks for qualifier
* Vision
    * [Object detection using MaskRCNN](vision/gate_detection) 
    * [Depth estimation](vision/depth_estimation): not used in the final system.
* Control
    * Quadcopter controller
        * [Cascading PID](e2e_system/controller/basic_controller.py)
    * Trajectory generation
        * [Rapid trajectory generation for quadrocopters](trajectory/RapidQuadrocopterTrajectories)
* Simulation
    * [Simple dynamic simulation in numpy](control/quad_sim)
    * [ROS example simulation in rotor_s](simulation/rotor_s_env)
    * [Official competition simulator FlightGoggle config](flight_goggle_config)
* SLAM
    * [Visual inertial odometry using VINS-MONO](slam/vins_mono_custom)
* End-to-end System
    * [Fly using ground truth states](e2e_system/run_state_machine_gt.py)
    * [Fly using ground truth gate locations](e2e_system/run_state_machine_vins_mono_gt.py)
    * [Fly with estimated gate locations](e2e_system/run_state_machine_final.py)
    
## Contribution guide
If you are working on a subcomponent, feel free to commit to the corresponding folder directly. If you are trying to modify the integration folder, please code in another branch and submit a pull request. Something to pay attention before submitting your code.

* Remove unncessary data (especially large files) to avoid exceeding repo limit (1 GB)
* When you are using a third party package, either specify its version (e.g. tensorflow==1.12) or bring in the source code (C++ project, unless it is under GPL license).
