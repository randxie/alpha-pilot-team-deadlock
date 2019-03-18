from __future__ import print_function, division
import numpy as np
import os
import rospy
import time
import yaml

FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TOTAL_NUM_GATES = 23


def estimate_perpendicular_vec(gate_loc):
  """Estimate perpendicular vector by least square.

  :param gate_loc: 4 corner points' x,y,z
  :return: Normalized perpendicular vector
  """
  A = np.array(gate_loc)
  vec = np.sum(np.matmul(np.linalg.inv(np.matmul(A.T, A)), A.T), axis=1)
  return vec / np.linalg.norm(vec)


def estimate_dimension(gate_loc):
  """Estimate gate dimension."""
  A = np.array(gate_loc)
  height = np.max(A[:, 2]) - np.min(A[:, 2])
  width = np.max(np.linalg.norm(A[1:, 0:2] - A[0, 0:2], axis=1))
  return width, height


def _get_true_gate_info():
  """Read gate's nominal location from ros parameter server.

  :return: Gate center location, Perpendicular vector, Gate dimension
  """
  gate_map = {}
  vec_map = {}
  dim_map = {}
  for i in range(1, TOTAL_NUM_GATES + 1):
    try:
      gate_loc = rospy.get_param('/uav/Gate%d/location' % i)
    except Exception as e:
      gate_loc = rospy.get_param('/uav/Gate%d/nominal_location' % i)
    gate_map[i] = np.mean(gate_loc, axis=0)
    vec_map[i] = estimate_perpendicular_vec(gate_loc)
    width_i, height_i = estimate_dimension(gate_loc)
    dim_map[i] = (width_i, height_i)
  return gate_map, vec_map, dim_map


class MissilePlanner(object):
  def __init__(self, start_time):
    """

    :param start_time: Start time to sync different components
    """
    self._start_time = start_time
    self.gate_map, self.vec_map, self.dim_map = _get_true_gate_info()

  def get_desired_state(self, cur_state, next_gate_loc=None):
    """Only used to pass the first gate.

    :param cur_state: State vector reflecting current quadcopter location
    :param next_gate_loc: Next gate location to fly to

    :return: desired state for next step
    """
    if next_gate_loc is None:
      desired_states = [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0]
      return np.array(desired_states)

    cur_x = cur_state[0]
    cur_y = cur_state[1]
    gate_x = next_gate_loc[0]
    gate_y = next_gate_loc[1]

    next_yaw = np.arctan2(gate_y - cur_y, gate_x - cur_x)

    desired_states = [next_gate_loc[0], next_gate_loc[1], next_gate_loc[2], 0, 0, next_yaw, 0, 0, 0, 0, 0, 0]

    return np.array(desired_states)

  def reset(self):
    self._start_time = time.time()
