from __future__ import print_function, division
import trajectory_lib.quadrocopter_trajectory as quadtraj
import numpy as np
import os
import rospy
import time
import yaml

FILE_DIR = os.path.dirname(os.path.realpath(__file__))
TOTAL_NUM_GATES = 23

# trajectory planner constants
TARGET_ACCELERATION = [0, 0, 0]
GRAVITY = [0, 0, -9.81]

# input limits
fmin = 5  # [m/s**2]
fmax = 25  # [m/s**2]
wmax = 20  # [rad/s]
minTimeSec = 0.02  # [s]


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


def _load_gate_info(filename):
  """Read gate information from yaml file.

  :param filename: A string specifying gate yaml file
  :return: Gate center location, Perpendicular vector, Gate dimension
  """
  with open(os.path.join(FILE_DIR, '..', 'gate_locations', filename)) as f:
    gate_locations = yaml.safe_load(f)

  gate_map = {}
  vec_map = {}
  dim_map = {}
  for i in range(1, TOTAL_NUM_GATES + 1):
    gate_loc = gate_locations['Gate%d' % i]['nominal_location']
    gate_map[i] = np.mean(gate_loc, axis=0)
    vec_map[i] = estimate_perpendicular_vec(gate_loc)
    width_i, height_i = estimate_dimension(gate_loc)
    dim_map[i] = (width_i, height_i)
  return gate_map, vec_map, dim_map


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


class FastTrajectoryPlanner(object):
  def __init__(self, start_time, use_true_gate=True):
    """

    :param start_time: Start time to sync different components
    """
    self._start_time = start_time

    if use_true_gate:
      self.gate_map, self.vec_map, self.dim_map = _get_true_gate_info()
    else:
      self.gate_map, self.vec_map, self.dim_map = _load_gate_info('gt_gate_location.yaml')
    self._is_computed = False
    self._time_between_gates = 5.0
    self.traj = None

  def get_desired_state(self, cur_state, next_gate_loc=None, next_gate_yaw=0, cur_gate_yaw=0):
    """Only used to pass the first gate.

    :param cur_state: State vector reflecting current quadcopter location
    :param next_gate_loc: Next gate location to fly to

    :return: desired state for next step
    """
    if next_gate_loc is None:
      desired_states = [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0]
      return np.array(desired_states)

    position = cur_state[0:3]
    velocity = cur_state[6:9]
    if not self._is_computed:
      self.traj = quadtraj.RapidTrajectory(position, velocity, TARGET_ACCELERATION, GRAVITY)
      self.traj.set_goal_position(next_gate_loc)
      self.traj.set_goal_velocity([0, 0, 0])
      self.traj.set_goal_acceleration(TARGET_ACCELERATION)
      # Goal acceleration and velocity can be unconstrained using null argument e.g. ([None, None, None])

      self.traj.generate(self._time_between_gates)
      self._is_computed = True

    dt = time.time() - self._start_time
    if dt > self._time_between_gates:
      desired_states = [next_gate_loc[0], next_gate_loc[1], next_gate_loc[2], 0, 0, next_gate_yaw, 0, 0, 0, 0, 0, 0]
    else:
      p = self.traj.get_position(dt)
      v = self.traj.get_velocity(dt)
      r = self.traj.get_body_rates(dt)
      a = self.traj.get_pose(dt);
      if dt < (self._time_between_gates - 0.2):
        cur_yaw = cur_gate_yaw
      else:
        cur_yaw = next_gate_yaw * dt / self._time_between_gates
      desired_states = [p[0], p[1], p[2], a[0], a[1], cur_yaw, v[0], v[1], v[2], r[0], r[1], r[2]]
      # Small angle assumption used when directly using r for body rates. Rotate these by calling
      # self.traj.get_rotation_matrix(psi, theta, phi) and carry out the matrix multiplication
    return np.array(desired_states)

  def reset(self):
    self.traj.reset()
    self._is_computed = False
    self._start_time = time.time()


if __name__ == "__main__":
  planner = FastTrajectoryPlanner(time.time())
  target_gate_loc = [1, 0, 5]
  cur_state = [0] * 12
  for i in range(100):
    next_state = planner.get_desired_state(cur_state, target_gate_loc)
    print(next_state[2])
