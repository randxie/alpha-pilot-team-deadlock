from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
import numpy as np
# import rospy


def callback(data):
    rospy.loginfo(rospy.get_caller_id(), data.data);

def main():

    # # Initiate subscriber
    # rospy.init_node('trajectory_subscriber', anonymous = True);
    # rospy.Subscriber('DESIRED_WAYPOINTS', ,callback);
    # rospy.spin();

    # Define the trajectory starting state:
    pos0 = [0, 0, 0] # position
    vel0 = [0, 0, 0] # velocity
    acc0 = [0, 0, 0] # acceleration

    # Define the goal state:
    posf = [1, 2, 3]  # position
    velf = [5, 5, 0]  # velocity
    accf = [0, 0, 0]  # acceleration

    # Define the duration:
    tmin = 1; # minimum time, has to be > 0
    tmax = 10; # maximum time
    Tf = np.linspace(tmin, tmax, 1000);

    # Define the input limits:
    fmin = 5  #[m/s**2]
    fmax = 25 #[m/s**2]
    wmax = 20 #[rad/s]
    minTimeSec = 0.02 #[s]

    # Define how gravity lies:
    gravity = [0,0,-9.81]

    #########################
    # Trajectory generation #
    #########################
    position = list();
    velocity = list();
    acceleration = list();
    thrust = list();
    ratesMagn = list();
    N = 1000;

    # Grid search optimization
    for i in range(len(Tf)-1):
        traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity);
        traj.set_goal_position(posf);
        traj.set_goal_velocity(velf);
        traj.set_goal_acceleration(accf);
        traj.generate(Tf[i]);
        cost = traj.get_cost();
        if (traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec) == 0):
            print(Tf[i]);
            break;

    # Find minimum feasible time
    timeToGo = Tf[i];
    # Obtain the trajectory with the lowest time
    time = np.linspace(0, timeToGo, N);
    traj.generate(timeToGo);
    print(traj.get_cost());
    for j in range(N):
        t = time[j];
        position.append(traj.get_position(t));
        velocity.append(traj.get_velocity(t));
        acceleration.append(traj.get_acceleration(t));
        thrust.append(traj.get_thrust(t));
        ratesMagn.append(np.linalg.norm(traj.get_body_rates(t)));
    
    ###############
    # Feasibility #
    ###############
    
    # Test input feasibility
    inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec);

    # Test whether we fly into the floor or any planes
    floorPoint  = [0,0,0];  # a point on the floor
    floorNormal = [0,0,1];  # we want to be in this direction of the point (upwards)
    positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal);

    for i in range(3):
        print("Axis #" , i);
        print("\talpha = " ,traj.get_param_alpha(i), "\tbeta = "  ,traj.get_param_beta(i), "\tgamma = " ,traj.get_param_gamma(i));
    print("Total cost = " , traj.get_cost());
    print("Input feasibility result: ",    quadtraj.InputFeasibilityResult.to_string(inputsFeasible),   "(", inputsFeasible, ")");
    print("Position feasibility result: ", quadtraj.StateFeasibilityResult.to_string(positionFeasible), "(", positionFeasible, ")");

    # Reset
    traj.reset();

    plot(timeToGo, N, position, velocity, acceleration, thrust, ratesMagn, fmin, fmax, wmax);

    # # Publish waypoints
    # pub = rospy.Publisher('trajectory', array, queue_size = 10)
    # rospy.init_node('trajectory_publisher', anonymous = True)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     rospy.loginfo(position);
    #     pub.publish(position);
    #     rate.sleep();

###########################################
# Plot the trajectories, and their inputs #
###########################################

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def plot(Tf, N, position, velocity, acceleration, thrust, ratesMagn, fmin, fmax, wmax):
    time = np.linspace(0, Tf, N);
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
    axThrust.plot([0,Tf],[fmin,fmin],'r--', label='fmin')
    axThrust.fill_between([0,Tf],[fmin,fmin],-1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axThrust.fill_between([0,Tf],[fmax,fmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axThrust.plot([0,Tf],[fmax,fmax],'r-.', label='fmax')

    axThrust.set_ylabel('Thrust [m/s^2]')
    axThrust.legend()

    axOmega.plot(time, ratesMagn,'k',label='command magnitude')
    axOmega.plot([0,Tf],[wmax,wmax],'r--', label='wmax')
    axOmega.fill_between([0,Tf],[wmax,wmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
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
