## How to run

* Set up environment by running ```. set_up_env.sh```
* Launch flight goggle by ```roslaunch flightgoggles core.launch ignore_collisions:=1```
* [For state estimation], ```roslaunch vins_estimator fg.launch```
* [Optional] Launch vins mono by ```roslaunch vins_estimator fg_rviz.launch```
* Run ```python2 run_state_machine_gt.py```

We are using Python2 here because tf package is supported in Python2.

## Caution
* If you saw the action's value is hanging there, stop and rerun the python script immediately.

