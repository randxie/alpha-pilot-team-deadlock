import cv2
import flightgoggles.msg as fg_msg
import nav_msgs.msg as nav_msgs
import matplotlib.pyplot as plt
import numpy as np
import rospy
import sensor_msgs.msg as s_msgs
import tf
import tf2_msgs.msg
import time

from .abstract_env import AbstractEnv
from queue import Queue

try:
  import pbcvt
except Exception as e:
  print("failed to import libsgm")


class GroundTruthEnv(AbstractEnv):
  # State space representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
  #   - where phi, theta and psi are Euler angles
  # Control input (Action space): 4 propellers' speed
  # Use notation in Vijar Kumar's paper "Minimum Snap Trajectory Generation and Control for Quadrotors"

  def __init__(self, start_time):
    super(GroundTruthEnv, self).__init__(start_time)
    self.vins_queue = Queue(maxsize=1)
    self.is_vins_inited = False

  def attach_listeners(self):
    rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self._ground_truth_callback, queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber('/uav/sensors/imu', s_msgs.Imu, self._imu_callback, queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber('/uav/camera/left/image_rect_color', s_msgs.Image, self._left_camera_callback, queue_size=1,
                     buff_size=2 ** 24)
    rospy.Subscriber('/uav/camera/right/image_rect_color', s_msgs.Image, self._right_camera_callback, queue_size=1,
                     buff_size=2 ** 24)
    rospy.Subscriber('/uav/camera/left/ir_beacons', fg_msg.IRMarkerArray, self._ir_marker_callback)
    # rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, self._range_finder_callback)
    # rospy.Subscriber('/vins_estimator/odometry', nav_msgs.Odometry, self._vins_callback)
    rospy.spin()

  def estimate_states(self):
    if not self.imu_queue.empty():
      try:
        self.states[9:12] = self.imu_queue.get(False)
      except Exception as e:
        pass

    if not self.gt_queue.empty():
      try:
        cur_time, position, pose, velocity = self.gt_queue.get(False)
        self.states[0:3] = position
        self.states[3:6] = pose
        self.states[6:9] = velocity
      except Exception as e:
        pass

  def plot_gate(self):
    img_left = None
    img_right = None
    if not self.left_camera_queue.empty():
      try:
        img_left = self.left_camera_queue.get(False)
      except Exception as e:
        pass
    if not self.right_camera_queue.empty():
      try:
        img_right = self.right_camera_queue.get(False)
      except Exception as e:
        pass

    loc = (self.gate_loc.get('Gate10', None))
    if img_left is not None and img_right is not None:
      #disparity = stereo.compute(img_left, img_right)
      disparity = pbcvt.depth_estimate(img_left, img_right)
      if loc:
        x, y = self.gate_loc['Gate10']
        print(disparity[x, y])
      plt.imshow(disparity, 'gray')
      plt.pause(0.0001)