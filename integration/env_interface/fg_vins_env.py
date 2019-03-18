import flightgoggles.msg as fg_msg
import nav_msgs.msg as nav_msgs
import numpy as np
import rospy
import sensor_msgs.msg as s_msgs
import tf
import tf2_msgs.msg

from .abstract_env import AbstractEnv
from queue import Queue


class FgVinsEnv(AbstractEnv):
  # State space representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
  #   - where phi, theta and psi are Euler angles
  # Control input (Action space): 4 propellers' speed
  # Use notation in Vijar Kumar's paper "Minimum Snap Trajectory Generation and Control for Quadrotors"

  def __init__(self, start_time):
    super(FgVinsEnv, self).__init__(start_time)
    self.vins_queue = Queue(maxsize=1)
    self.is_vins_inited = False

  def _vins_callback(self, data):
    """State estimated from VINS Mono

    :param data:
    :return:
    """
    if self.vins_queue.full():
      self.vins_queue.get(False)

    if not self.is_vins_inited:
      print('successfully initialize VINS!!!!')

    self.is_vins_inited = True

    # get estimated position
    position = np.array(
      [data.pose.pose.position.x + self.xyz_offset[0], data.pose.pose.position.y + self.xyz_offset[1],
       data.pose.pose.position.z + self.xyz_offset[2]])

    # get estimated pose
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
      [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
       data.pose.pose.orientation.w])
    pose = np.array(
      [roll + self.euler_angle_offset[0], pitch + self.euler_angle_offset[1], yaw + self.euler_angle_offset[2]])

    # get estimated velocity
    velocity = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
    print((position, pose, velocity))
    self.vins_queue.put((position, pose, velocity))

  def estimate_states(self):
    if not self.imu_queue.empty():
      try:
        self.states[9:12] = self.imu_queue.get(False)
      except Exception as e:
        pass

    try:
      if not self.is_vins_inited:
        if not self.range_finder_queue.empty():
          height = self.range_finder_queue.get(False)
          self.states[2] = height + self.height_offset
      else:
        if not self.vins_queue.empty():
          position, pose, velocity = self.vins_queue.get(False)
          self.states[0:3] = position
          self.states[3:6] = pose
          self.states[6:9] = velocity
    except Exception as e:
      pass

  def attach_listeners(self):
    rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self._ground_truth_callback, queue_size=1000, buff_size=2 ** 20)
    rospy.Subscriber('/uav/sensors/imu', s_msgs.Imu, self._imu_callback, queue_size=1000, buff_size=2 ** 20)
    rospy.Subscriber('/uav/camera/left/ir_beacons', fg_msg.IRMarkerArray, self._ir_marker_callback, queue_size=1,
                     buff_size=2 ** 20)
    rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, self._range_finder_callback, queue_size=1,
                     buff_size=2 ** 20)
    rospy.Subscriber('/vins_estimator/odometry', nav_msgs.Odometry, self._vins_callback, queue_size=1,
                     buff_size=2 ** 20)
    rospy.spin()
