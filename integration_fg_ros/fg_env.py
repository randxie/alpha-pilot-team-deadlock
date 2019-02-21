import gym
import numpy as np
import rospy
import threading
import mav_msgs.msg as mav_msgs
import sensor_msgs.msg as s_msgs

from controller_node import PIDController
from config import GRAVITY_COEFF
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
    self.renderer = None
    self.reset()

    # control frequency
    rospy.init_node('fg_env', anonymous=True)
    self.rate = rospy.Rate(10)  # 10hz
    self.thrust_publisher = rospy.Publisher('/uav/input/rateThrust', mav_msgs.RateThrust, queue_size=20)
    self.height = 0

  def reset(self):
    self.time = 0
    self.states = np.zeros(12)

  def _collect_height(self, data):
    self.height = data.range

  def _do_action(self, action):
    tmp_msg = mav_msgs.RateThrust()
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
    self.states[2] = self.height
    self._do_action(action)

    return self.states, 0, False, {}

  def laser_callback(self, data):
    self.height = -data.range

env = FgEnv()

def listener():
  rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, env.laser_callback)
  rospy.spin()


if __name__ == '__main__':
  desired_states = [0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  controller = PIDController()

  t1 = threading.Thread(target=listener, daemon=True)
  t1.start()

  while True:
    try:
      action = controller.compute_action(env.states, desired_states)
      new_state, _, _, _ = env.step(action)

    except rospy.ROSInterruptException:
      break

  t1.join()
