# Note: This is a draft

import gym
import numpy as np
import rospy
import threading
import flightgoggles.msg as fg_msg
import mav_msgs.msg as mav_msgs
import sensor_msgs.msg as s_msgs

from transmitter_node import get_transmitter
from controller_node import PIDController
from config import GRAVITY_COEFF
from queue import Queue
from utils import get_rotation_mtx
from utils import wrap_angle

class FgEnv(gym.Env):
  # State space representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
  #   - where phi, theta and psi are Euler angles
  # Control input (Action space): 4 propellers' speed
  # Use notation in Vijar Kumar's paper "Minimum Snap Trajectory Generation and Control for Quadrotors"

  def __init__(self):
    self.states = None
    self.time = None

    # control frequency
    rospy.init_node('fg_env', anonymous=True)
    self.rate = rospy.Rate(400)  # 10hz
    self.thrust_publisher = rospy.Publisher('/uav/input/rateThrust', mav_msgs.RateThrust, queue_size=10)
    self.height = 0
    self.queue = Queue(maxsize=1)
    self.queue.put((0, 0, 0, 0, 0, 0, 0))
    self.target = (0, 0)
    self.has_target = False

    self.reset()

  def reset(self):
    self.time = 0
    self.states = np.zeros(12)

  def _collect_height(self, data):
    self.height = data.range

  def _do_action(self, action):
    tmp_msg = mav_msgs.RateThrust()
    tmp_msg.angular_rates.x = action[1]
    tmp_msg.angular_rates.y = action[2]
    tmp_msg.angular_rates.z = action[3]
    tmp_msg.thrust.z = action[0]
    self.thrust_publisher.publish(tmp_msg)
    self.rate.sleep()

  def step(self, action):
    """
    :param action: [u1, u2, u3, u4]
    :return:
    """
    # wrap angles
    self.states[3:6] = wrap_angle(self.states[3:6])
    self._do_action(action)

    return self.states, 0, False, {}

  def laser_callback(self, data):
    self.states[2] = -data.range

  def imu_callback(self, data):
    t = data.header.stamp.nsecs
    acc_x = data.linear_acceleration.x
    acc_y = data.linear_acceleration.y
    acc_z = data.linear_acceleration.z
    self.states[9] = data.angular_velocity.x
    self.states[10] = data.angular_velocity.y
    self.states[11] = data.angular_velocity.z
    
    t_p, acc_x_p, acc_y_p, acc_z_p, acc_phi_p, acc_theta_p, acc_psi_p = self.queue.get()
    self.states[6] = self.states[6] + (acc_x + acc_x_p) / 2.0 * (t - t_p) * (10 ** -9)
    self.states[7] = self.states[7] + (acc_y + acc_y_p) / 2.0 * (t - t_p) * (10 ** -9)
    self.states[8] = self.states[8] + (acc_z + acc_z_p) / 2.0 * (t - t_p) * (10 ** -9)
    self.states[3] = self.states[3] + (self.states[9] + acc_phi_p) / 2.0 * (t - t_p) * (10 ** -9)
    self.states[4] = self.states[4] + (self.states[10] + acc_theta_p) / 2.0 * (t - t_p) * (10 ** -9)
    self.states[5] = self.states[5] + (self.states[11] + acc_psi_p) / 2.0 * (t - t_p) * (10 ** -9)

    self.queue.put((t, acc_x, acc_y, acc_z, self.states[9], self.states[10], self.states[11]))
    #print(self.states[2])

  def ir_marker_callback(self, data):
    if len(data.markers):
      if self.has_target:
        pass
      else:
        self.target = (data.markers[0].x, data.markers[0].y)
        self.has_target = True
        #print(self.target)
      self.states[0] = self.target[0] - data.markers[0].x
      self.states[1] = self.target[1] - data.markers[0].y

env = FgEnv()

def listener():
  rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, env.laser_callback)
  rospy.Subscriber('/uav/sensors/imu', s_msgs.Imu, env.imu_callback)
  rospy.Subscriber('/uav/camera/left/ir_beacons', fg_msg.IRMarkerArray, env.ir_marker_callback)
  rospy.spin()


if __name__ == '__main__':

  controller = PIDController()
  transmitter = get_transmitter()

  t1 = threading.Thread(target=listener, daemon=True)
  t1.start()
  
  if transmitter is False: # autonomous flight using waypoints
    print('Started autonomous flight')
    while True:
      try:
        desired_states = [0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0] #env.target[0], env.target[1],
        action = controller.compute_action(env.states, desired_states)
        new_state, _, _, _ = env.step(action)
      except rospy.ROSInterruptException:
        break
  else:
    print('Started manual flight')
    while True: # manual flights using RC transmitter
      try:
        #transmitter.debug()
        action = transmitter.compute_action() 
        new_state, _, _, _ = env.step(action)
        print(action)
      except KeyboardInterrupt:

        break
  t1.join()
