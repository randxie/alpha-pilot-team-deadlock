import numpy as np


def get_rotation_mtx(euler_angles):
  """Rotational matrix from Euler angles [phi, theta, psi]

  :param euler_angles:
  :return: rotational matrix
  """
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
                    [0, cos_phi, -sin_phi],
                    [0, sin_phi, cos_phi]])

  rot_y = np.array([[cos_theta, 0, sin_theta],
                    [0, 1, 0],
                    [-sin_theta, 0, cos_theta]])

  rot_z = np.array([[cos_psi, -sin_psi, 0],
                    [sin_psi, cos_psi, 0],
                    [0, 0, 1]])
  return np.dot(rot_z, np.dot(rot_y, rot_x))


def wrap_angle(angle):
  return (angle + np.pi) % (2 * np.pi) - np.pi
