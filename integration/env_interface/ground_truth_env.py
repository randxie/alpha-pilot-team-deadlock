import flightgoggles.msg as fg_msg
import rospy
import sensor_msgs.msg as s_msgs
import tf2_msgs.msg

from .abstract_env import AbstractEnv


class GroundTruthEnv(AbstractEnv):
  # State space representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
  #   - where phi, theta and psi are Euler angles
  # Control input (Action space): 4 propellers' speed
  # Use notation in Vijar Kumar's paper "Minimum Snap Trajectory Generation and Control for Quadrotors"

  def __init__(self, start_time):
    super(GroundTruthEnv, self).__init__(start_time)

  def attach_listeners(self):
    rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self._ground_truth_callback)
    rospy.Subscriber('/uav/sensors/imu', s_msgs.Imu, self._imu_callback)
    rospy.Subscriber('/uav/camera/left/ir_beacons', fg_msg.IRMarkerArray, self._ir_marker_callback)
    rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, self._range_finder_callback)
    rospy.spin()

  def estimate_states(self):
    if not self.imu_queue.empty():
      self.states[9:12] = self.imu_queue.get()

    if not self.gt_queue.empty():
      position, pose, velocity = self.gt_queue.get()
      self.states[0:3] = position
      self.states[3:6] = pose
      self.states[6:9] = velocity