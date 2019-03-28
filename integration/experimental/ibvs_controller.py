import numpy as np

#Ref https://www.tandfonline.com/doi/pdf/10.1080/15599610903144161

# quadcopter data from flight goggle's code
m = 1.00
Ix = 4.9 * (10 ** (-3))
Iy = 4.9 * (10 ** (-3))
Iz = 4.9 * (10 ** (-3))
d_torque = 2.6 * (10 ** (-7))
b = 1.91 * (10 ** (-6))
l = 0.08
GRAVITY_COEFF = 9.8

class IBVSController(object):
  """
  Ref:
  [1] Visual-inertial navigation algorithm development using photorealistic camera simulation in the loop
  """

  def __init__(self):
    pass

  def compute_action(self, ir_markers, gate_depth, cur_states, desired_states):
    # State representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
    # Desired state is determined by desired position

    [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r] = list(cur_states)
    [x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, p_d, q_d, r_d] = list(desired_states)
    L = []
    z = gate_depth


    lambd = 5

    feature_vec_d = np.array([  ])  # Desired IR marker locations
    feature_vec = []

    for ir in ir_markers:
      u = ir[0]
      v = ir[1]
      L_tmp = np.array([[1/z, 0, -u/z, -u*v, 1+u**2 -v],[0, 1/z, -v/z, -(1+v**2), u*v, u]])
      L.append(L_tmp)
      feature_vec.append(u)
      feature_vec.append(v)

    Ls = np.vstack((L[0], L[1], L[2], L[3]))
    feature_vec = np.array(feature_vec).reshape((8,1))
    L_pseudo_inv = np.dot(np.inv(np.dot(Ls.T, Ls)), Ls.T)

    err = feature_vec - feature_vec_d

    u_vel = -lambd*L_pseudo_inv*err   # u_vel is a 6x1 vector, [ux uy uz wx wy wz]

    ux = u_vel[0]
    uy = u_vel[1]
    uz = u_vel[2]


    # orientation controller
    phi_d = np.arctan2((ux * np.sin(psi) - uy * np.cos(psi)), (GRAVITY_COEFF + uz)) * np.cos(theta)
    theta_d = np.arctan2((ux * np.cos(psi) + uy * np.sin(psi)), (GRAVITY_COEFF + uz))

    if np.abs(phi_d) > np.pi/6:
      phi_d = np.pi/6 * np.sign(phi_d)

    if np.abs(theta_d) > np.pi/6:
      theta_d = np.pi/6 * np.sign(theta_d)

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

    if epsi < -np.pi:
      epsi += 2 * np.pi
    elif epsi > np.pi:
      epsi -= 2 * np.pi

    upsi = (apsi * epsi + apsi_dot * epsi_dot)

    return uz, uphi, utheta, upsi
