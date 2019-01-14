# try to start an empty environment first before starting the rotor_s simulation platform
roslaunch gazebo_ros empty_world.launch
roslaunch rotors_gazebo mav_hovering_example.launch

# enter the running container
docker ps
docker exec -it <CONTAINER ID> /bin/bash # please replace the container id with yours
