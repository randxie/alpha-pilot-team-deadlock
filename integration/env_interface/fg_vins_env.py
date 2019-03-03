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

  def _vins_callback(self, data):
    """State estimated from VINS Mono

    :param data:
    :return:
    """
    if self.vins_queue.full():
      self.vins_queue.get()

    # get estimated position
    position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

    # get estimated pose
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
      [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
       data.pose.pose.orientation.w])
    pose = np.array([roll, pitch, yaw])

    # get estimated velocity
    velocity = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
    self.vins_queue.put((position, pose, velocity))

  def attach_listeners(self):
    rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self._ground_truth_callback)
    rospy.Subscriber('/uav/sensors/imu', s_msgs.Imu, self._imu_callback)
    rospy.Subscriber('/uav/camera/left/ir_beacons', fg_msg.IRMarkerArray, self._ir_marker_callback)
    rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, self._range_finder_callback)
    rospy.Subscriber('/vins_estimator/odometry', nav_msgs.Odometry, self._vins_callback)
    rospy.spin()