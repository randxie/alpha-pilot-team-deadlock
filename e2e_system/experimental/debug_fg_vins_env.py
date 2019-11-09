import flightgoggles.msg as fg_msg
import nav_msgs.msg as nav_msgs
import numpy as np
import rospy
import sensor_msgs.msg as s_msgs
import tf
import tf2_msgs.msg
#from utils.util_transform import *
from .abstract_env import AbstractEnv
from queue import Queue


def inertial2body(euler_angles, sequence='ZYX'):
  phi = euler_angles[0]
  theta = euler_angles[1]
  psi = euler_angles[2]

  # compute constants
  cos_phi = np.cos(phi)
  cos_theta = np.cos(theta)
  cos_psi = np.cos(psi)
  sin_phi = np.sin(phi)
  sin_theta = np.sin(theta)
  sin_psi = np.sin(psi)

  # build rotational matrix around each axis
  rot_x = np.array([[1, 0, 0],
                    [0, cos_phi, sin_phi],
                    [0, -sin_phi, cos_phi]])

  rot_y = np.array([[cos_theta, 0, -sin_theta],
                    [0, 1, 0],
                    [sin_theta, 0, cos_theta]])

  rot_z = np.array([[cos_psi, sin_psi, 0],
                    [sin_psi, -cos_psi, 0],
                    [0, 0, 1]])
  if sequence == 'ZYX':
    return np.dot(rot_x, np.dot(rot_y, rot_z))
  else:  # Assume 'XYZ'
    return np.dot(rot_z, np.dot(rot_y, rot_x))


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
    positionb = np.array(
      [data.pose.pose.position.x + self.xyz_offset[0], data.pose.pose.position.y + self.xyz_offset[1],
       data.pose.pose.position.z + self.xyz_offset[2]])

    position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

    # get estimated pose
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
      [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
       data.pose.pose.orientation.w])

    body_attitude = np.array([roll, pitch, yaw])

    #init_position = np.array([self.xyz_offset[0], self.xyz_offset[1], self.xyz_offset[2]])
    init_position = self.states[0:3]
    rot_i2b = inertial2body(body_attitude)
    rot_b2i = rot_i2b.T

    vins_to_inertial = np.dot(rot_b2i, position)
    position_inertial = vins_to_inertial + init_position

    print("VINS inertial: {}".format(vins_to_inertial))
    #print("Position inertial: {}".format(position_inertial))

    #print("Inertial position = {}".format(position_inertial))
    #print("Body frame position = {}".format(positionb))
    #if self.is_vins_inited:
    #  print("VINS yaw: {}".format(yaw))
    #  print("yaw offset: {}".format(self.euler_angle_offset[2]))
    #inertial_pose =

    pose = np.array(
      [roll + self.euler_angle_offset[0], pitch + self.euler_angle_offset[1], yaw + self.euler_angle_offset[2]])

    # get estimated velocity
    velocity = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
    velocity_inertial = np.dot(rot_b2i, velocity)

    print(pose)

    #print((position, pose, velocity))
    #self.vins_queue.put((position, pose, velocity))
    self.vins_queue.put((position_inertial, pose, velocity_inertial))

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
          self.states[9] = (height - self.z_prev)*0.0075
          print("z_vel = {}".format(self.states[9]))
          self.z_prev = height
          self.states[2] = height + self.height_offset
      else:
        if not self.vins_queue.empty():
          position, pose, velocity = self.vins_queue.get(False)
          self.states[0:3] = position
          self.states[3:6] = pose
          self.states[6:9] = velocity
    except Exception as e:
      print('error')
      #pass

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
