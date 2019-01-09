import functools
import gym
import numpy as np
import scipy.integrate
import datetime

from .quad_config import GRAVITY_COEFF
from .quad_config import QuadConfig
from .utils import get_rotation_mtx
from .utils import wrap_angle


class QuadcopterModel(object):
  """Represents the quad-copter frame.

  Units:
    arm_length: 'm'
    weight: 'kg' (including battery, motor, propeller, frame)
    thrust: 'N'
    omega: 'rad/s
  """

  def __init__(self):
    # propeller related
    self.k_m = QuadConfig.ROTOR_DRAG_CONSTANT
    self.k_f = QuadConfig.ROTOR_THRUST_CONSTANT

    # ------------------------
    # Frame
    # ------------------------
    self.L = QuadConfig.QUAD_ARM_LENGTH  # arm length: from center to motor
    self.mass = QuadConfig.QUAD_MASS

    # ------------------------
    # Control related
    # ------------------------
    # mapping from omega square to force/moment
    self.omega_force_mtx = np.array([[self.k_f, self.k_f, self.k_f, self.k_f],
                                     [0, self.k_f * self.L, 0, -self.k_f * self.L],
                                     [-self.k_f * self.L, 0, self.k_f * self.L, 0],
                                     [self.k_m, -self.k_m, self.k_m, -self.k_m]])

    # Based on body frame. Assume the cross terms are negligible
    rotate_mass = self.mass / 6
    self.Ixx = 1 / 12 * rotate_mass * ((2 * self.L) ** 2)
    self.Iyy = 1 / 12 * rotate_mass * ((2 * self.L) ** 2)
    self.Izz = 1 / 3 * rotate_mass * (self.L ** 2)
    self.inertial_mtx = np.array([[self.Ixx, 0, 0],
                                  [0, self.Iyy, 0],
                                  [0, 0, self.Izz]])

  def compute_control_input(self, action_vec):
    # Note: action_vec has been squared.
    return np.dot(self.omega_force_mtx, action_vec)


class QuadcopterEnv(gym.Env):
  # State space representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
  #   - where phi, theta and psi are Euler angles
  # Control input (Action space): 4 propellers' speed
  # Use notation in Vijar Kumar's paper "Minimum Snap Trajectory Generation and Control for Quadrotors"

  def __init__(self):
    self.quad_model = QuadcopterModel()
    self.step_time = 0.05 # simulation time per step() function call (unit: second)
    self.states = None
    self.time = None
    self.reset()

  def reset(self):
    self.time = 0
    self.states = np.zeros(12)

  def step(self, action):
    """
    :param action: [omega_1^2, omega_2^2, omega_3^2, omega_4^2]
    :return:
    """
    # wrap angles
    self.states[3:6] = wrap_angle(self.states[3:6])

    # compute control input
    u = self.quad_model.compute_control_input(action)

    # solve dynamic equations
    dyn_eqn = functools.partial(self.state_dot, action=u)
    sol_obj = scipy.integrate.solve_ivp(dyn_eqn, (self.time, self.time + self.step_time), self.states, 'RK23')

    # update state
    self.states = sol_obj.y[:, -1].flatten()

    return self.states, 0, False, {}

  def state_dot(self, time, states, action):
    u = action.flatten()

    # assume the action is fixed with in the time span
    euler_angles = states[3:6]

    # compute d/dt ([x, y, z, xdot, ydot, zdot])
    z_world = np.array([[0, 0, 1]]).T
    z_body_w = np.dot(get_rotation_mtx(euler_angles), np.array([[0, 0, 1]]).T)

    # get derivative of translation motion
    r_dot = states[6:9]
    r_ddot = -GRAVITY_COEFF * z_world + u[0] * z_body_w

    # compute d/dt ([phi, theta, psi, p, q, r])
    I_mtx = self.quad_model.inertial_mtx
    omega_bw = states[9:12]
    cross_prod = np.cross(omega_bw, np.dot(I_mtx, omega_bw))
    omega_dot = np.dot(np.linalg.inv(I_mtx), (-cross_prod + u[1:4]))

    return np.hstack((r_dot, omega_bw, r_ddot.flatten(), omega_dot))

  def render(self, mode='human'):
    pass


if __name__ == '__main__':
  print('Running experiments.')
  env = QuadcopterEnv()
  for i in range(10):
    action = np.array([[1, 1, 1, 1]]).T
    new_state, _, _, _ = env.step(action)
    print(new_state[0:3])


