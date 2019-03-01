# Note: This is a draft

import flightgoggles.msg as fg_msg
import gym
import mav_msgs.msg as mav_msgs
import nav_msgs.msg as nav_msgs
import numpy as np
import time
import rospy
import sensor_msgs.msg as s_msgs
import tf
import tf2_msgs.msg
import threading

from controller_node import PIDController
from queue import Queue
from utils import wrap_angle

GATE_ORDER = [19, 10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6]


class FgVinsEnv(gym.Env):
  # State space representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
  #   - where phi, theta and psi are Euler angles
  # Control input (Action space): 4 propellers' speed
  # Use notation in Vijar Kumar's paper "Minimum Snap Trajectory Generation and Control for Quadrotors"

  def __init__(self):
    self.states = None
    self.time = None

    # control frequency
    rospy.init_node('fg_env', anonymous=True)
    self.rate = rospy.Rate(100)  # 100hz
    self.thrust_publisher = rospy.Publisher('/uav/input/rateThrust', mav_msgs.RateThrust, queue_size=500)
    self.queue = Queue(maxsize=1)
    self.queue.put((0, 0, 0, 0))
    self.reset()

    self.is_vins_ready = False
    self.cur_index = 1

  def reset(self):
    """Reset environment

    :return:
    """
    self.time = 0
    self.states = np.zeros(12)

  def _do_action(self, action):
    """Publish control action.

    :param action: [u_thrust, u_phi, u_theta, u_psi]
    :return:
    """
    tmp_msg = mav_msgs.RateThrust()
    tmp_msg.thrust.z = action[0]
    if self.is_vins_ready:
      tmp_msg.angular_rates.x = action[1]
      tmp_msg.angular_rates.y = action[2]
      tmp_msg.angular_rates.z = action[3]
    self.thrust_publisher.publish(tmp_msg)
    self.rate.sleep()

  def step(self, action):
    """
    :param action: [u_thrust, u_phi, u_theta, u_psi]
    :return:
    """
    # wrap angles
    self.states[3:6] = wrap_angle(self.states[3:6])
    self._do_action(action)

    return self.states, 0, False, {}

  def range_finder_callback(self, data):
    """Use range finder to get height information. Useful for initialization.

    :param data:
    :return:
    """
    if not self.is_vins_ready:
      self.states[2] = -data.range

  def imu_callback(self, data):
    """Use IMU's angular velocity as true state.

    :param data: sensor_msgs Imu
    :return:
    """
    self.states[9] = data.angular_velocity.x
    self.states[10] = data.angular_velocity.y
    self.states[11] = data.angular_velocity.z

  def ground_truth_callback(self, data):
    """Subscribe /tf to get ground truth.

    :param data:
    :return:
    """
    # get accurate position information
    self.states[0] = data.transforms[0].transform.translation.x
    self.states[1] = data.transforms[0].transform.translation.y
    self.states[2] = data.transforms[0].transform.translation.z

    # get accurate pose information
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
      [data.transforms[0].transform.rotation.x, data.transforms[0].transform.rotation.y,
       data.transforms[0].transform.rotation.z,
       data.transforms[0].transform.rotation.w])
    self.states[3] = roll
    self.states[4] = pitch
    self.states[5] = yaw

    # do derivative to get velocity
    t = time.time()
    t_p, x_p, y_p, z_p = self.queue.get()
    self.states[6] = (self.states[0] - x_p) / (t - t_p)
    self.states[7] = (self.states[1] - y_p) / (t - t_p)
    self.states[8] = (self.states[2] - z_p) / (t - t_p)
    self.queue.put((t, self.states[0], self.states[1], self.states[2]))

  def ir_marker_callback(self, data):
    """For getting target gate

    :param data:
    :return:
    """
    for marker in data.markers:
      if int(marker.markerID.data) == self.cur_index:
        self.target = (marker.x, marker.y)

  def vins_callback(self, data):
    """Not Used. For testing VINS Mono.

    :param data:
    :return:
    """
    self.is_vins_ready = True
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
      [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
       data.pose.pose.orientation.w])
    self.states[3] = roll
    self.states[4] = pitch
    self.states[5] = yaw
    self.states[6] = data.twist.twist.linear.x
    self.states[7] = data.twist.twist.linear.y
    self.states[8] = data.twist.twist.linear.z

    self.states[0] = data.pose.pose.position.x
    self.states[1] = data.pose.pose.position.y
    self.states[2] = data.pose.pose.position.z


env = FgVinsEnv()


def listener():
  #rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, env.ground_truth_callback)
  rospy.Subscriber('/uav/sensors/imu', s_msgs.Imu, env.imu_callback)
  rospy.Subscriber('/uav/camera/left/ir_beacons', fg_msg.IRMarkerArray, env.ir_marker_callback)
  rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, env.range_finder_callback)
  rospy.Subscriber('/vins_estimator/odometry', nav_msgs.Odometry, env.vins_callback)
  rospy.spin()


if __name__ == '__main__':
  controller = PIDController()

  # run listener in daemon thread
  t1 = threading.Thread(target=listener)
  t1.daemon = True
  t1.start()

  # main loop
  t_start = time.time()
  while True:
    try:
      dt = (time.time() - t_start)
      if env.is_vins_ready:
        desired_states = env.states
        desired_states[0] += 1
      else:
        if dt < 15:
          desired_states = [0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        elif dt < 30:
          desired_states = [15, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        elif dt < 45:
          desired_states = [15, 0, 6, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, 0]
        else:
          desired_states = [15, 5, 6, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, 0]
      action = controller.compute_action(env.states, desired_states)
      new_state, _, _, _ = env.step(action)
      time.sleep(0.2)

    except rospy.ROSInterruptException:
      break

  t1.join()
