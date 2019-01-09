import numpy as np
from .quad_config import GRAVITY_COEFF
from .quad_config import PIDControllerConfig


class AbstractController(object):
  pass

class PIDController(AbstractController):
  def __init__(self, control_obj):
    self.config = PIDControllerConfig
    self.control_obj = control_obj

  def compute_action(self, cur_states, desired_states):
    # State representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
    # Desired state is determined by desired position

    [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r] = list(cur_states)
    [x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, p_d, q_d, r_d] = list(desired_states)

    # firstly do altitude control
    uz = self.config.kp_z * (z_d - z) + self.config.kp_z_dot * (z_dot_d - z_dot)
    u1 = (GRAVITY_COEFF + uz) * self.control_obj.mass / (np.cos(phi) * np.cos(theta))

    # then we do position control
    ux = self.config.kp_x * (x_d - x) + self.config.kp_x_dot * (x_dot_d - x_dot)
    uy = self.config.kp_y * (y_d - y) + self.config.kp_y_dot * (y_dot_d - y_dot)

    theta_d = np.arctan((ux * np.cos(psi) + uy * np.sin(psi)) / (GRAVITY_COEFF + uz))
    phi_d = np.arctan((ux * np.sin(psi) - uy * np.cos(psi)) / (GRAVITY_COEFF + uz)) * np.cos(theta)

    u2 = self.config.kp_phi * (phi_d - phi) + self.config.kp_phi_dot * (p_d - p)
    u3 = self.config.kp_theta * (theta_d - theta) + self.config.kp_theta_dot * (q_d - q)
    u4 = self.config.kp_psi * (psi_d - psi) + self.config.kp_psi_dot * (r_d - r)

    omega_square = np.dot(np.linalg.inv(self.control_obj.omega_force_mtx), np.array([[u1, u2, u3, u4]]).T)

    return omega_square
