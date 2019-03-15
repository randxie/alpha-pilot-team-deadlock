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


class PDController(object):
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
    az = 2
    az_dot = 4
    ez = z_d - z
    ez_dot = z_dot_d - z_dot
    uz = m / (np.cos(phi) * np.cos(theta)) * (az * ez + GRAVITY_COEFF + az_dot * ez_dot)  # - az * (ez_dot + az * ez)

    # then we do position control
    ax = 2
    ax_dot = 3
    ex = x_d - x
    ex_dot = x_dot_d - x_dot
    ux = (ax * ex + ax_dot * ex_dot)

    ay = 2
    ay_dot = 3
    ey = y_d - y
    ey_dot = y_dot_d - y_dot
    uy = (ay * ey + ay_dot * ey_dot)

    # orientation controller
    phi_d = np.arctan2((ux * np.sin(psi) - uy * np.cos(psi)), (GRAVITY_COEFF + uz)) * np.cos(theta)
    theta_d = np.arctan2((ux * np.cos(psi) + uy * np.sin(psi)), (GRAVITY_COEFF + uz))

    if np.abs(phi_d) > np.pi/3:
      phi_d = np.pi/3 * np.sign(phi_d)

    if np.abs(theta_d) > np.pi/3:
      theta_d = np.pi/3 * np.sign(theta_d)

    aphi = 3
    aphi_dot = 0.5
    ephi = phi_d - phi
    ephi_dot = p_d - p
    uphi = (aphi * ephi + aphi_dot * ephi_dot)

    atheta = 3
    atheta_dot = 0.5
    etheta = theta_d - theta
    etheta_dot = q_d - q
    utheta = (atheta * etheta + atheta_dot * etheta_dot)

    apsi = 3
    apsi_dot = 0.5
    epsi = psi_d - psi
    epsi_dot = r_d - r
    upsi = (apsi * epsi + apsi_dot * epsi_dot)

    return uz, uphi, utheta, upsi


class PIDController(object):
  """
  Ref:
  https://github.com/lobodol/drone-flight-controller/blob/master/drone-flight-controller.ino
  """

  def __init__(self, cascaded_attitude=False):

    self.cascaded_attitude = cascaded_attitude

    # Cache the errors for derivative and integral 
    self.err_x_prev = 0.0
    self.err_y_prev = 0.0
    self.err_z_prev = 0.0
    self.err_phi_prev = 0.0
    self.err_theta_prev = 0.0
    self.err_psi_prev = 0.0
    self.err_p_prev = 0.0
    self.err_q_prev = 0.0
    self.err_r_prev = 0.0

    # integral -- sum of error
    self.err_x_sum = 0.0
    self.err_y_sum = 0.0
    self.err_z_sum = 0.0

    self.err_phi_sum = 0.0
    self.err_theta_sum = 0.0
    self.err_psi_sum = 0.0

    # derivative -- change in error
    self.err_phi_delta = 0.0
    self.err_theta_delta = 0.0
    self.err_psi_delta = 0.0

    self.err_p_delta = 0.0
    self.err_q_delta = 0.0

    # Integral saturation bounds
    self.int_sat = {'x': 1000.0,
                    'y': 1000.0,
                    'z': 1000.0,
                    'phi': 1000.0,
                    'theta': 1000.0,
                    'psi': 1000.0}

  def saturate_integral(self):
    self.err_x_sum = np.clip(self.err_x_sum, -self.int_sat['x'], self.int_sat['x'])
    self.err_y_sum = np.clip(self.err_y_sum, -self.int_sat['y'], self.int_sat['y'])
    self.err_z_sum = np.clip(self.err_z_sum, -self.int_sat['z'], self.int_sat['z'])

    self.err_phi_sum = np.clip(self.err_phi_sum, -self.int_sat['phi'], self.int_sat['phi'])
    self.err_theta_sum = np.clip(self.err_theta_sum, -self.int_sat['theta'], self.int_sat['theta'])
    self.err_psi_sum = np.clip(self.err_psi_sum, -self.int_sat['psi'], self.int_sat['psi'])

  def compute_action(self, cur_states, desired_states):
    [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r] = list(cur_states)
    [x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, p_d, q_d, r_d] = list(desired_states)
    #print(list(cur_states))

    fs = 100.0  # sample frequency in Hertz (might want to get actual value)
    dt = 1 / fs

    ax = 1
    ay = 1
    az = 1
    ax_int = 0.5
    ay_int = 0.5
    az_int = 0.5
    ax_der = 0.05
    ay_der = 0.05
    az_der = 0.05

    aphi = 0.1
    atheta = 0.1
    apsi = 0.1
    aphi_int = 0.05
    atheta_int = 0.05
    apsi_int = 0.05
    ap = 0.5
    aq = 0.5
    ar = 0.5

    # Gains for cascaded attitude controller
    ap_der = 0.2
    aq_der = 0.2
    ar_der = 0.2

    ## POSITION CONTROLLER
    err_x = x_d - x
    err_y = y_d - y
    err_z = z_d - z

    self.err_x_sum += err_x * dt
    self.err_y_sum += err_y * dt
    self.err_z_sum += err_z * dt
    self.saturate_integral()

    self.err_x_delta = (err_x - self.err_x_prev) / dt
    self.err_y_delta = (err_y - self.err_y_prev) / dt
    self.err_z_delta = (err_z - self.err_z_prev) / dt

    # Altitude control first
    uz = m / (np.cos(phi) * np.cos(theta)) * (
          GRAVITY_COEFF + az * err_z + az_int * self.err_z_sum + az_der * self.err_z_delta)
    # print('Altitude -- Integral: {}. Derivative: {}'.format(self.err_z_sum, self.err_z_delta))

    # XY controller
    ux = ax * err_x + ax_int * self.err_x_sum + ax_der * self.err_x_delta
    uy = ay * err_y + ay_int * self.err_y_sum + ay_der * self.err_y_delta

    ## ATTITUDE/ANGULAR RATE CONTROLLER    
    # Convert XY command to phi/theta 
    phi_d = np.arctan2((ux * np.sin(psi) - uy * np.cos(psi)), (GRAVITY_COEFF + uz)) * np.cos(theta)
    theta_d = np.arctan2((ux * np.cos(psi) + uy * np.sin(psi)), (GRAVITY_COEFF + uz))

    err_phi = phi_d - phi
    err_theta = theta_d - theta
    err_psi = psi_d - psi

    self.err_phi_sum += err_phi * dt
    self.err_theta_sum += err_theta * dt
    self.err_psi_sum += err_psi * dt
    self.saturate_integral()

    self.err_phi_delta = (err_phi - self.err_phi_prev) / dt
    self.err_theta_delta = (err_theta - self.err_theta_prev) / dt
    self.err_psi_delta = (err_psi - self.err_psi_prev) / dt

    if self.cascaded_attitude:  # (Control angular acceleration)
      # Attitude outer loop
      phi_dot_d = aphi * err_phi + aphi_int * self.err_phi_sum
      theta_dot_d = atheta * err_theta + atheta_int * self.err_theta_sum
      psi_dot_d = apsi * err_psi + apsi_int * self.err_psi_sum

      err_p = phi_dot_d - p
      err_q = theta_dot_d - q
      err_z = psi_dot_d - r
      self.err_p_delta = (err_p - self.err_p_prev) / dt
      self.err_q_delta = (err_q - self.err_q_prev) / dt
      self.err_r_delta = (err_r - self.err_r_prev) / dt
      self.err_p_prev = err_p
      self.err_q_prev = err_q

      # Body rates inner loop
      uphi_ddot = ap * err_p + ap_der * self.err_p_delta
      utheta_ddot = aq * err_q + aq_der * self.err_q_delta
      upsi_ddot = ar_err_r + ar_der * self.err_r_delta

      u2 = uphi_ddot
      u3 = utheta_ddot
      u4 = upsi_ddot

    else:  # Single loop attitude controller (Control angular rates)
      uphi_dot = aphi * err_phi + aphi_int * self.err_phi_sum + ap * self.err_phi_delta
      utheta_dot = atheta * err_theta + atheta_int * self.err_theta_sum + aq * self.err_theta_delta
      upsi_dot = apsi * err_psi + apsi_int * self.err_psi_sum + ar * self.err_psi_delta

      u2 = uphi_dot
      u3 = utheta_dot
      u4 = upsi_dot

      # Store for next loop
    self.err_x_prev = err_x
    self.err_y_prev = err_y
    self.err_z_prev = err_z

    self.err_phi_prev = err_phi
    self.err_theta_prev = err_theta
    self.err_psi_prev = err_psi

    #   return 10.0, 0.0, 0.0, 0.0
    return uz, u2, u3, u4
