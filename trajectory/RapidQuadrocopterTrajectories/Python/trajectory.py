from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
import numpy as np
import yaml
import rospy


def callback(data):
    rospy.loginfo(rospy.get_caller_id(), data.data);

def main():

    # Initiate subscriber
    rospy.init_node('trajectory_subscriber', anonymous = True);
    rospy.Subscriber('DESIRED_WAYPOINTS', ,callback);
    rospy.spin();

    # Predefined gates
    with open('gate_locations_0.yaml') as handle:
        gate_locations = yaml.safe_load(handle);

    Gate1 = np.mean(gate_locations['Gate1']['location'], axis = 0);
    Gate2 = np.mean(gate_locations['Gate2']['location'], axis = 0);
    Gate3 = np.mean(gate_locations['Gate3']['location'], axis = 0);
    Gate4 = np.mean(gate_locations['Gate4']['location'], axis = 0);
    Gate5 = np.mean(gate_locations['Gate5']['location'], axis = 0);
    Gate6 = np.mean(gate_locations['Gate6']['location'], axis = 0);
    Gate7 = np.mean(gate_locations['Gate7']['location'], axis = 0);
    Gate8 = np.mean(gate_locations['Gate8']['location'], axis = 0);
    Gate9 = np.mean(gate_locations['Gate9']['location'], axis = 0);
    Gate10 = np.mean(gate_locations['Gate10']['location'], axis = 0);
    Gate11 = np.mean(gate_locations['Gate11']['location'], axis = 0);
    Gate12 = np.mean(gate_locations['Gate12']['location'], axis = 0);
    Gate13 = np.mean(gate_locations['Gate13']['location'], axis = 0);
    Gate14 = np.mean(gate_locations['Gate14']['location'], axis = 0);
    Gate15 = np.mean(gate_locations['Gate15']['location'], axis = 0);
    Gate16 = np.mean(gate_locations['Gate16']['location'], axis = 0);
    Gate17 = np.mean(gate_locations['Gate17']['location'], axis = 0);
    Gate18 = np.mean(gate_locations['Gate18']['location'], axis = 0);
    Gate19 = np.mean(gate_locations['Gate19']['location'], axis = 0);
    Gate20 = np.mean(gate_locations['Gate20']['location'], axis = 0);
    Gate21 = np.mean(gate_locations['Gate21']['location'], axis = 0);
    Gate22 = np.mean(gate_locations['Gate22']['location'], axis = 0);
    Gate23 = np.mean(gate_locations['Gate23']['location'], axis = 0);

    gate = [Gate1, Gate2, Gate3, Gate4, Gate5, Gate6, Gate7, Gate8, Gate9, Gate10,
            Gate11, Gate12, Gate13, Gate14, Gate15, Gate16, Gate17, Gate18, Gate19,
            Gate20, Gate21, Gate22, Gate23];

    GATE_ORDER = [19, 10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6];

    # Define the trajectory starting state:
    pos0 = [-1, -30, 1] #position
    vel0 = [1, 1, 0] #velocity
    acc0 = [0, 0, 0] #acceleration

    # Define the goal state:
    posf = gate[GATE_ORDER[0]]  # position
    velf = [5, 5, 0]  # velocity
    accf = [0, 0, 0]  # acceleration

    # Define the duration:
    Tf = 5

    # Define the input limits:
    fmin = 5  #[m/s**2]
    fmax = 25 #[m/s**2]
    wmax = 20 #[rad/s]
    minTimeSec = 0.02 #[s]

    # Define how gravity lies:
    gravity = [0,0,-9.81]

    # Trajectory generation
    position = list();
    velocity = list();
    acceleration = list();
    thrust = list();
    ratesMagn = list();
    for i in range(len(GATE_ORDER)):
        traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity);
        traj.set_goal_position(gate[GATE_ORDER[i]-1]);
        traj.set_goal_velocity(velf);
        traj.set_goal_acceleration(accf);
        traj.generate(Tf);
        N = 100;
        time = np.linspace(0, Tf, N);
        for j in range(N):
            t = time[j];
            position.append(traj.get_position(t));
            velocity.append(traj.get_velocity(t));
            acceleration.append(traj.get_acceleration(t));
            thrust.append(traj.get_thrust(t));
            ratesMagn.append(np.linalg.norm(traj.get_body_rates(t)));

        # Test input feasibility
        inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec);

        # Test whether we fly into the floor
        floorPoint  = [0,0,0];  # a point on the floor
        floorNormal = [0,0,1];  # we want to be in this direction of the point (upwards)
        positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal);

        for i in range(3):
            print("Axis #" , i);
            print("\talpha = " ,traj.get_param_alpha(i), "\tbeta = "  ,traj.get_param_beta(i), "\tgamma = " ,traj.get_param_gamma(i));
        print("Total cost = " , traj.get_cost());
        print("Input feasibility result: ",    quadtraj.InputFeasibilityResult.to_string(inputsFeasible),   "(", inputsFeasible, ")");
        print("Position feasibility result: ", quadtraj.StateFeasibilityResult.to_string(positionFeasible), "(", positionFeasible, ")");

        # Reset using last point
        pos0 = position[-1];
        vel0 = velocity[-1];
        acc0 = acceleration[-1];
        traj.reset();

    plot(Tf, GATE_ORDER, N, position, velocity, acceleration, thrust, ratesMagn, fmin, fmax, wmax);

    # Publish waypoints
    pub = rospy.Publisher('trajectory', array, queue_size = 10)
    rospy.init_node('trajectory_publisher', anonymous = True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(position);
        pub.publish(position);
        rate.sleep();

###########################################
# Plot the trajectories, and their inputs #
###########################################

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def plot(Tf, GATE_ORDER, N, position, velocity, acceleration, thrust, ratesMagn, fmin, fmax, wmax):
    time = np.linspace(0, Tf*len(GATE_ORDER), N*len(GATE_ORDER));
    figStates, axes = plt.subplots(3,1,sharex=True)
    gs = gridspec.GridSpec(6, 2)
    axPos = plt.subplot(gs[0:2, 0])
    axVel = plt.subplot(gs[2:4, 0])
    axAcc = plt.subplot(gs[4:6, 0])

    for ax,yvals in zip([axPos, axVel, axAcc], [np.asarray(position),np.asarray(velocity),np.asarray(acceleration)]):
        cols = ['r','g','b']
        labs = ['x','y','z']
        for i in range(3):
            ax.plot(time,yvals[:,i],cols[i],label=labs[i])

    axPos.set_ylabel('Pos [m]')
    axVel.set_ylabel('Vel [m/s]')
    axAcc.set_ylabel('Acc [m/s^2]')
    axAcc.set_xlabel('Time [s]')
    axPos.legend()
    axPos.set_title('States')

    infeasibleAreaColour = [1,0.5,0.5]
    axThrust = plt.subplot(gs[0:3, 1])
    axOmega  = plt.subplot(gs[3:6, 1])
    axThrust.plot(time,thrust,'k', label='command')
    axThrust.plot([0,Tf*len(GATE_ORDER)],[fmin,fmin],'r--', label='fmin')
    axThrust.fill_between([0,Tf*len(GATE_ORDER)],[fmin,fmin],-1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axThrust.fill_between([0,Tf*len(GATE_ORDER)],[fmax,fmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axThrust.plot([0,Tf*len(GATE_ORDER)],[fmax,fmax],'r-.', label='fmax')

    axThrust.set_ylabel('Thrust [m/s^2]')
    axThrust.legend()

    axOmega.plot(time, ratesMagn,'k',label='command magnitude')
    axOmega.plot([0,Tf*len(GATE_ORDER)],[wmax,wmax],'r--', label='wmax')
    axOmega.fill_between([0,Tf*len(GATE_ORDER)],[wmax,wmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axOmega.set_xlabel('Time [s]')
    axOmega.set_ylabel('Body rates [rad/s]')
    axOmega.legend()

    axThrust.set_title('Inputs')

    #make the limits pretty:
    axThrust.set_ylim([min(fmin-1,min(thrust)), max(fmax+1,max(thrust))])
    axOmega.set_ylim([0, max(wmax+1,max(ratesMagn))])
    plt.show()

if __name__ == "__main__":
    main();
