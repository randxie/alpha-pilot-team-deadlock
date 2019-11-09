## Test 3 Submission files
## Official files do not work. Follow the steps below using modified files
* Copy scorer.launch into ../flightlaunch directory.
* Copy scorer.sh into ../flightgoggles/flightgoggles directory, and do 'chmod +x scorer.sh'.
* Copy both the /challenges and /perturbations folder into your ../config directory.
* rosrun flightgoggles scorer.sh (**make sure scorer.py is in the same directory when you rosrun**)
* Delete the /results folder generated scorer.sh, because it will try to mkdir that folder again and will cause an error.


See following for reference on the issues:
1. https://github.com/mit-fast/FlightGoggles/issues/80
2. See Adriano Rezende's post https://www.herox.com/alphapilot/forum/thread/3755?page=11


## Below is the official README.md
This readme accompanies scorer scripts and configuration files to evaluate entries for the Alphapilot Challenge. Below are instructions for setting up the scorer, and lists of FlightGoggles ROS parameters an ROS topics that participants are allowed to use in their code.

In an FPV drone racing scenario, the participants know the nominal gate locations from their practice rounds. However, at race time the locations of the actual gates may be slightly perturbed (unknown to the participants). FPV racers can account for this using the visual feedback they receive through the onboard camera. 

Similar to human FPV racers, autonomy algorithms should also be able to navigate even when the gates are slightly perturbed. For the challenge, this robustness is addressed as follows: The *nominal* gate locations are available for the participants to use in their algorithms. But for the evaluation of their algorithms, small perturbations will be added to these nominal gate locations. The resulting *perturbed* gate locations are used in the evaluation runs, but are not known *a priori* to the autonomy algorithms. The uniform distribution bounds on the gate location perturbations can be read from the ROS parameter server.

All values and parameters that may be used are available through ROS parameter server and ROS topics. **The autonomy algorithms are forbidden to use (the content of) any of the yaml files directly.**


## Setup
To setup the challenge files from the HeroX website in the FlightGoggles simulator, please run the following commands. 

```bash
# Download challenge files from HeroX website.
cd ~/Downloads
wget https://d253pvgap36xx8.cloudfront.net/challenges/resources/7c2dae4a31af11e99c430242ac110002/Challenge_Leaderboardtest.zip 
# Unzip files
unzip Challenge_Leaderboardtest.zip
# Copy files into flightgoggles launch and config folders
cp -r Challenge_Leaderboardtest/config/* $(rospack find flightgoggles)/config/
cp Challenge_Leaderboardtest/launch/* $(rospack find flightgoggles)/launch/
# Mark executable files as executable.
chmod +x  $(rospack find flightgoggles)/launch/scorer.sh
chmod +x  $(rospack find flightgoggles)/launch/scorer.py
```

**NOTE:** All contestants must edit `scorer.launch` to include all ROS nodes required for completing the challenges. This will be the launch file run by Lockheed Martin to run your code.


## Running the scorer
```bash
rosrun flightgoggles scorer.sh
```
This will run the new launch file 25 times with the perturbed gates in `flightgoggles/config/gate_locations_x.yaml` and accumulate the results from the reporter in a `results` folder. At the end of the evaluation, `scorer.py` is run which generates a `scores.yaml` which contains the individual scores for each run. 

**Note:** The perturbed gate locations are generated using the maximum allowed perturbation defined in the `FlightGoggles` repository in the `flightgoggles/config/challenges/gate_locations_nominal.yaml`. The perturbation defined in that file is the maximum absolute value perturbation in `x y yaw`. 


## Scoring metric
The formula for generating the individual score for every evaluation is `N.R-T` where `N` is the number of successful gate fly-throughs, `R` is the reward for every successful fly through and `T` is the total time required to complete the challenge.


## Allowed topics and params
Below are lists of ROS parameters and topics that may be published/adapted or subscribed/read, respectively.

### Allowed to publish/adapt
There is a single FlightGoggles ROS topic that the participants are allowed to publish to: `/uav/input/rateThrust` contains the angular rate and thrust commands for the low-level quadcopter controller. Also listed below are the FlightGoggles parameters that participants are allowed to change. Namely whether stereo camera rendering is enabled and the render image resolution. A description of the parameters and topics can be found in the wiki-page on the FlightGoggles GitHub repo.

```
Topics:
/uav/input/rateThrust

Parameters:
/uav/flightgoggles_ros_bridge/image_width    
/uav/flightgoggles_ros_bridge/image_height 
/uav/flightgoggles_ros_bridge/render_stereo
```

### Allowed to subscribe/read

The following list contains the FlightGoggles parameters that participants are allowed to get from the ROS parameter server and the FlightGoggles topics that participants are allowed to subscribe to. Participants are not allowed to change these parameters, or publish to these topics. A description of the parameters and topics can be found in the [wiki-page](https://github.com/mit-fast/FlightGoggles/wiki) on the FlightGoggles GitHub repo.

```
Topics:
/bounding_box_camera/RGB
/clock
/control_nodes/joy
/control_nodes/keyboard/keydown
/control_nodes/keyboard/keyup
/control_nodes/universal_teleop/controls
/control_nodes/universal_teleop/events
/uav/camera/debug/fps
/uav/camera/left/camera_info
/uav/camera/left/image_rect_color
/uav/camera/left/ir_beacons
/uav/camera/right/camera_info
/uav/camera/right/image_rect_color
/uav/collision
/uav/sensors/imu
/uav/sensors/downward_laser_rangefinder

Parameters:
/uav/Gate1/nominal_location
/uav/Gate1/perturbation_bound
/uav/Gate10/nominal_location
/uav/Gate10/perturbation_bound
/uav/Gate11/nominal_location
/uav/Gate11/perturbation_bound
/uav/Gate12/nominal_location
/uav/Gate12/perturbation_bound
/uav/Gate13/nominal_location
/uav/Gate13/perturbation_bound
/uav/Gate14/nominal_location
/uav/Gate14/perturbation_bound
/uav/Gate15/nominal_location
/uav/Gate15/perturbation_bound
/uav/Gate16/nominal_location
/uav/Gate16/perturbation_bound
/uav/Gate17/nominal_location
/uav/Gate17/perturbation_bound
/uav/Gate18/nominal_location
/uav/Gate18/perturbation_bound
/uav/Gate19/nominal_location
/uav/Gate19/perturbation_bound
/uav/Gate2/nominal_location
/uav/Gate2/perturbation_bound
/uav/Gate20/nominal_location
/uav/Gate20/perturbation_bound
/uav/Gate21/nominal_location
/uav/Gate21/perturbation_bound
/uav/Gate22/nominal_location
/uav/Gate22/perturbation_bound
/uav/Gate23/nominal_location
/uav/Gate23/perturbation_bound
/uav/Gate3/nominal_location
/uav/Gate3/perturbation_bound
/uav/Gate4/nominal_location
/uav/Gate4/perturbation_bound
/uav/Gate5/nominal_location
/uav/Gate5/perturbation_bound
/uav/Gate6/nominal_location
/uav/Gate6/perturbation_bound
/uav/Gate7/nominal_location
/uav/Gate7/perturbation_bound
/uav/Gate8/nominal_location
/uav/Gate8/perturbation_bound
/uav/Gate9/nominal_location
/uav/Gate9/perturbation_bound
/uav/challenge_name
/uav/flightgoggles_imu/accelerometer_variance
/uav/flightgoggles_imu/gyroscope_variance
/uav/flightgoggles_laser/rangefinder_variance
/uav/flightgoggles_laser/rangefinder_max_range
/uav/flightgoggles_lpf/gain_p
/uav/flightgoggles_lpf/gain_q
/uav/flightgoggles_pid/gain_d_pitch
/uav/flightgoggles_pid/gain_d_roll
/uav/flightgoggles_pid/gain_d_yaw
/uav/flightgoggles_pid/gain_i_pitch
/uav/flightgoggles_pid/gain_i_roll
/uav/flightgoggles_pid/gain_i_yaw
/uav/flightgoggles_pid/gain_p_pitch
/uav/flightgoggles_pid/gain_p_roll
/uav/flightgoggles_pid/gain_p_yaw
/uav/flightgoggles_pid/int_bound_pitch
/uav/flightgoggles_pid/int_bound_roll
/uav/flightgoggles_pid/int_bound_yaw
/uav/flightgoggles_ros_bridge/baseline
/uav/flightgoggles_ros_bridge/collider_radius
/uav/flightgoggles_uav_dynamics/angular_process_noise
/uav/flightgoggles_uav_dynamics/clockscale
/uav/flightgoggles_uav_dynamics/drag_coefficient
/uav/flightgoggles_uav_dynamics/ignore_collisions
/uav/flightgoggles_uav_dynamics/init_pose
/uav/flightgoggles_uav_dynamics/linear_process_noise
/uav/flightgoggles_uav_dynamics/max_prop_speed
/uav/flightgoggles_uav_dynamics/min_arming_thrust
/uav/flightgoggles_uav_dynamics/moment_arm
/uav/flightgoggles_uav_dynamics/motor_time_constant
/uav/flightgoggles_uav_dynamics/reset_timeout
/uav/flightgoggles_uav_dynamics/thrust_coefficient
/uav/flightgoggles_uav_dynamics/torque_coefficient
/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx
/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy
/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz
/uav/flightgoggles_uav_dynamics/vehicle_mass
/uav/gate_names
/uav/gate_width
/uav/timeout
/use_sim_time
```

## Forbidden Parameters
The following list contains the FlightGoggles parameters that participants are not allowed to read from the ROS parameter server.

```
Parameters:
/run_id
/uav/Gate1/location
/uav/Gate10/location
/uav/Gate11/location
/uav/Gate12/location
/uav/Gate13/location
/uav/Gate14/location
/uav/Gate15/location
/uav/Gate16/location
/uav/Gate17/location
/uav/Gate18/location
/uav/Gate19/location
/uav/Gate2/location
/uav/Gate20/location
/uav/Gate21/location
/uav/Gate22/location
/uav/Gate23/location
/uav/Gate3/location
/uav/Gate4/location
/uav/Gate5/location
/uav/Gate6/location
/uav/Gate7/location
/uav/Gate8/location
/uav/Gate9/location
/uav/results_location
```

## Regarding the `/tf` and `/tf_static` topics
The `/tf` and `/tf_static` topics contain transformations between several reference frames, as described in the wiki-page on the FlightGoggles GitHub repo. Participants are only allowed to query transformations limited to the following reference frames:  
```
/uav/imu
/uav/camera/right
/uav/camera/left
```

However, it is **forbidden** to query transformations involving any of the following reference frames. These ground-truth frames are included for debugging purposes only during the development of the contestants’ autonomy algorithms:

```
/world 
/world_ned
```


## Evaluation of Autonomy Software

During the evaluation, the autonomy software will be evaluated on 25 other sets of perturbed gate locations based on the same nominal values and allowed perturbations. The variations for these 25 tests will be accomplished by changing the following parameters only: 

```
/run_id
/uav/Gate1/location
/uav/Gate10/location
/uav/Gate11/location
/uav/Gate12/location
/uav/Gate13/location
/uav/Gate14/location
/uav/Gate15/location
/uav/Gate16/location
/uav/Gate17/location
/uav/Gate18/location
/uav/Gate19/location
/uav/Gate2/location
/uav/Gate20/location
/uav/Gate21/location
/uav/Gate22/location
/uav/Gate23/location
/uav/Gate3/location
/uav/Gate4/location
/uav/Gate5/location
/uav/Gate6/location
/uav/Gate7/location
/uav/Gate8/location
/uav/Gate9/location 
