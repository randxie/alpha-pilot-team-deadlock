from __future__ import print_function, division
import numpy as np
import os
import rospy
import tf
import time
import yaml

FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TOTAL_NUM_GATES = 23
DEBUG = True


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
  init_pose = rospy.get_param('/uav/flightgoggles_uav_dynamics/init_pose')
  xyz_offset = init_pose[0:3]
  gate_map = {}
  vec_map = {}
  perturb_map = {}
  dim_map = {}
  for i in range(1, TOTAL_NUM_GATES + 1):
    if DEBUG:
      gate_loc = rospy.get_param('/uav/Gate%d/location' % i)

    else:
      gate_loc = rospy.get_param('/uav/Gate%d/nominal_location' % i)
    gate_loc = np.array(gate_loc)

    # change of coordinates for gate location
    gate_loc[:, 0] = gate_loc[:, 0] - xyz_offset[0]
    gate_loc[:, 1] = gate_loc[:, 1] - xyz_offset[1]
    tmp = gate_loc[:, 0].copy()
    gate_loc[:, 0] = gate_loc[:, 1]
    gate_loc[:, 1] = -tmp

    # change of coordiantes for gate perturbation
    perturbation_bound = rospy.get_param('/uav/Gate%d/perturbation_bound' % i)
    perturbation_bound[0], perturbation_bound[1] = perturbation_bound[1], perturbation_bound[0]
    perturb_map[i] = perturbation_bound

    gate_map[i] = np.mean(gate_loc, axis=0)
    cur_vec = estimate_perpendicular_vec(gate_loc)
    if cur_vec[0] < 0:
      cur_vec = - cur_vec
    vec_map[i] = cur_vec
    width_i, height_i = estimate_dimension(gate_loc)
    dim_map[i] = (width_i, height_i)
  return gate_map, vec_map, dim_map, perturb_map

def _get_gate_info_switcher(cur_state, estimated_gate_loc):

  init_pose = rospy.get_param('/uav/flightgoggles_uav_dynamics/init_pose')
  xyz_offset = init_pose[0:3]
  gate_map = {}
  vec_map = {}
  perturb_map = {}
  dim_map = {}

  cur_pos = cur_state[0:3]

  SWITCH_RADIUS = 5   # If within 5 meters of nominal gate location, switch gate estimator

  for i in range(1, TOTAL_NUM_GATES + 1):

    nom_gate_loc = rospy.get_param('/uav/Gate%d/nominal_location' % i)

    if np.linalg(cur_pos - np.array(nom_gate_loc)) < SWITCH_RADIUS:
      gate_loc = estimated_gate_loc
    else:
      gate_loc = nom_gate_loc

    gate_loc = np.array(gate_loc)

    # change of coordinates for gate location
    gate_loc[:, 0] = gate_loc[:, 0] - xyz_offset[0]
    gate_loc[:, 1] = gate_loc[:, 1] - xyz_offset[1]
    tmp = gate_loc[:, 0].copy()
    gate_loc[:, 0] = gate_loc[:, 1]
    gate_loc[:, 1] = -tmp

    # change of coordiantes for gate perturbation
    perturbation_bound = rospy.get_param('/uav/Gate%d/perturbation_bound' % i)
    perturbation_bound[0], perturbation_bound[1] = perturbation_bound[1], perturbation_bound[0]
    perturb_map[i] = perturbation_bound

    gate_map[i] = np.mean(gate_loc, axis=0)
    cur_vec = estimate_perpendicular_vec(gate_loc)
    if cur_vec[0] < 0:
      cur_vec = - cur_vec
    vec_map[i] = cur_vec
    width_i, height_i = estimate_dimension(gate_loc)
    dim_map[i] = (width_i, height_i)
  return gate_map, vec_map, dim_map, perturb_map

class MissilePlanner(object):
  def __init__(self, start_time):
    """
    :param start_time: Start time to sync different components
    """
    self._start_time = start_time
    self.gate_map, self.vec_map, self.dim_map, self.perturb_map = _get_true_gate_info()

  def get_desired_state(self, cur_state, next_gate_loc=None):
    """Only used to pass the first gate.

    :param cur_state: State vector reflecting current quadcopter location
    :param next_gate_loc: Next gate location to fly to

    :return: desired state for next step
    """
    if next_gate_loc is None:
      desired_states = [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0]
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
