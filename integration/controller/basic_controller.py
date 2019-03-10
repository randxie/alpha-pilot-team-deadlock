import numpy as np

# quadcopter data from flight goggle's code
m = 1.00
Ix = 4.9 * (10 ** (-3))
Iy = 4.9 * (10 ** (-3))
Iz = 4.9 * (10 ** (-3))
d_torque = 2.6 * (10 ** (-7))
b = 1.91 * (10 ** (-6))
l = 0.08
GRAVITY_COEFF = 9.8


class PIDController(object):
  """
  Ref:
  [1] Visual-inertial navigation algorithm development using photorealistic camera simulation in the loop
  """

  def __init__(self):
    pass

  def compute_action(self, cur_states, desired_states):
    # State representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
    # Desired state is determined by desired position

    [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r] = list(cur_states)
    [x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, p_d, q_d, r_d] = list(desired_states)

    # firstly do altitude control
    az = 0.5
    az_dot = 4
    ez = z_d - z
    ez_dot = z_dot_d - z_dot
    uz = m / (np.cos(phi) * np.cos(theta)) * (az * ez + GRAVITY_COEFF + az_dot * ez_dot)  # - az * (ez_dot + az * ez)

    # then we do position control
    ax = 0.5
    ax_dot = 4
    ex = x_d - x
    ex_dot = x_dot_d - x_dot
    ux = (ax * ex + ax_dot * ex_dot)

    ay = 0.5
    ay_dot = 4
    ey = y_d - y
    ey_dot = y_dot_d - y_dot
    uy = (ay * ey + ay_dot * ey_dot)

    # orientation controller
    phi_d = np.arctan2((ux * np.sin(psi) - uy * np.cos(psi)), (GRAVITY_COEFF + uz)) * np.cos(theta)
    theta_d = np.arctan2((ux * np.cos(psi) + uy * np.sin(psi)), (GRAVITY_COEFF + uz))

    aphi = 3
    aphi_dot = 1
    ephi = phi_d - phi
    ephi_dot = p_d - p
    uphi = (aphi * ephi + aphi_dot * ephi_dot)

    atheta = 3
    atheta_dot = 1
    etheta = theta_d - theta
    etheta_dot = q_d - q
    utheta = (atheta * etheta + atheta_dot * etheta_dot)

    apsi = 3
    apsi_dot = 1
    epsi = psi_d - psi
    epsi_dot = r_d - r
    upsi = (apsi * epsi + apsi_dot * epsi_dot)

    return uz, uphi, utheta, upsi
