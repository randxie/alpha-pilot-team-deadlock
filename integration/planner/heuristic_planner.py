import numpy as np
import rospy
import time


class HeuristicPlanner():
  def __init__(self, start_time):
    """

    :param start_time: Start time to sync different components
    """
    self._start_time = start_time

  def get_desired_state(self, cur_state, next_gate_loc):
    """Only used to pass the first gate.

    :param cur_state: State vector reflecting current quadcopter location
    :param next_gate_loc: Next gate location to fly to

    :return: desired state for next step
    """
    dt = time.time() - self._start_time
    if dt < 30:
      desired_states = [0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    elif dt < 60:
      desired_states = [15, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    elif dt < 90:
      desired_states = [15, 0, 6, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, 0]
    else:
      desired_states = [15, 5, 6, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, 0]

    return np.array(desired_states)
