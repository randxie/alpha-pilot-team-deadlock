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

from queue import Queue
from utils.util_transform import wrap_angle

DEFAULT_CONFIG = {
  'range_finder_queue_size': 5,
  'imu_queue_size': 5,
  'gt_queue_size': 5,
  'ir_marker_queue_size': 5,
}


class AbstractEnv(gym.Env):
  """Environment Interface"""

  def __init__(self, start_time, config=DEFAULT_CONFIG):
    self.start_time = start_time

    # for control execution
    rospy.init_node('fg_env', anonymous=True)
    self.rate = rospy.Rate(100)  # 100hz
    self.thrust_publisher = rospy.Publisher('/uav/input/rateThrust', mav_msgs.RateThrust, queue_size=100)

    # measurements
    # here, internal queues are maintained for memory sharing
    self.config = config
    self.range_finder_queue = Queue(maxsize=self.config['range_finder_queue_size'])
    self.imu_queue = Queue(maxsize=self.config['imu_queue_size'])
    self.gt_queue = Queue(maxsize=self.config['gt_queue_size'])
    self.ir_marker_queue = Queue(maxsize=self.config['ir_marker_queue_size'])
    self.listener_thread = threading.Thread(target=self.attach_listeners)

    self.reset()

  def reset(self):
    self.states = np.zeros(12)

  def step(self, action):
    """
    :param action: [u_thrust, u_phi, u_theta, u_psi]
    :return:
    """
    # wrap angles
    self.states[3:6] = wrap_angle(self.states[3:6])
    self.publish_actions(action)
    self.estimate_states()

    return self.states, 0, False, {}

  def estimate_states(self):
    """Estimate state by sensor fusion."""
    raise NotImplementedError('You have to implement your state estimator.')

  def publish_actions(self, actions):
    """Publish control actions.

    :param actions: [u_thrust, u_phi, u_theta, u_psi]
    :return:
    """
    tmp_msg = mav_msgs.RateThrust()
    tmp_msg.thrust.z = actions[0]
    tmp_msg.angular_rates.x = actions[1]
    tmp_msg.angular_rates.y = actions[2]
    tmp_msg.angular_rates.z = actions[3]
    self.thrust_publisher.publish(tmp_msg)
    self.rate.sleep()

  def _range_finder_callback(self, data):
    """Use range finder to get height information. Useful for initialization.

    :param data:
    :return:
    """
    if self.range_finder_queue.full():
      self.range_finder_queue.get()
    self.range_finder_queue.put(-data.range)

  def _imu_callback(self, data):
    """Use IMU's angular velocity as true state.

    :param data: sensor_msgs Imu
    :return:
    """
    if self.imu_queue.full():
      self.imu_queue.get()
    angular_velocity = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
    self.imu_queue.put(angular_velocity)

  def _ground_truth_callback(self, data):
    """Subscribe /tf to get ground truth.

    :param data:
    :return:
    """
    cur_gt = data.transforms[0].transform
    cur_time = time.time()

    # get accurate position information
    position = np.array([cur_gt.translation.x, cur_gt.translation.y, cur_gt.translation.z])

    # get accurate pose information
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
      [cur_gt.rotation.x, cur_gt.rotation.y, cur_gt.rotation.z, cur_gt.rotation.w])
    pose = np.array([roll, pitch, yaw])

    # do derivative to get velocity
    if self.gt_queue.empty():
      t_p, position_p, pose_p = self.start_time, np.zeros(3), np.zeros(3)
    else:
      t_p, position_p, pose_p, _ = self.gt_queue.get()

    velocity = (position - position_p) / (cur_time - t_p)
    self.gt_queue.put((cur_time, position, pose, velocity))

  def _ir_marker_callback(self, data):
    """For getting target gate

    :param data:
    :return:
    """
    print('marker')
    if self.ir_marker_queue.full():
      self.ir_marker_queue.get()
    self.ir_marker_queue.put(data.markers)

  def attach_listeners(self):
    """Subscribe to different sensors.

    :return: None
    """
    raise NotImplementedError('You need to determine what listeners to attach.')

  def spin_listeners(self):
    self.listener_thread.daemon = True
    self.listener_thread.start()

  def clean_up(self):
    self.listener_thread.join()

  def __del__(self):
    self.clean_up()