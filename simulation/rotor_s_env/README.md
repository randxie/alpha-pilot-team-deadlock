## How to use
* Build the container by ```sh build_container.sh```
* Run the container using ```sh run_contaier.sh```
* In the container, try to set up an empty environment first. Then run the rotor_s environment

Note: for those who do not have Nvidia GPUs, please use the Dockerfile-cpu version to build the container.

## Useful ROS commands
* Find all mav_msgs: ```ls /root/catkin_ws/src/mav_comm/mav_msgs/msg/```
* Find out message contents: ```rosmsg show -r mav_msgs/Actuators```
* Find out all topics: ```rostopic list```
* Example actuator control commands: ```rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'```

