# Alpha Pilot: Team DeadLock

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
* Code Generation
    * Use TVM to generate optimized embedded code
    * Or probably TensorRT that's used in Jetson
* SLAM
    * Visual inertial odometry (use VIO-MONO)
    
## Contribution guide
If you are working on a subcomponent, feel free to commit to the corresponding folder directly. If you are trying to modify the integration folder, please code in another branch and submit a pull request. Something to pay attention before submitting your code.

* Remove unncessary data (especially large files) to avoid exceeding repo limit (1 GB)
* When you are using a third party package, either specify its version (e.g. tensorflow==1.12) or bring in the source code (C++ project, unless it is under GPL license).
