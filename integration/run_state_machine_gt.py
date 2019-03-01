"""Execute the state machine with ground truth information."""
import numpy as np
import rospy
from controller.basic_controller import PIDController
from env_interface.ground_truth_env import GroundTruthEnv
from enum import Enum
from planner.heuristic_planner import HeuristicPlanner

GATE_ORDER = [19, 10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6]


class SystemState(Enum):
  START = 0
  HOVERING = 1
  EXPLORING = 2
  GATE_TRACKING = 3
  STOP = 4


class StateMachine(object):
  def __init__(self, env_interface, controller_node, planner_node, localizer_node, explorer_node):
    self._env = env_interface
    self._controller = controller_node
    self._planner = planner_node
    self._localizer = localizer_node
    self._explorer = explorer_node
    self._sys_state = SystemState.START

    self.hovering_height = 6

  def gate_searching(self):
    pass

  def gate_tracking(self):
    pass

  def spin(self):
    if self._sys_state == SystemState.START:
      self.try_state_transition(SystemState.HOVERING)
    elif self._sys_state == SystemState.HOVERING:
      self.try_state_transition(SystemState.GATE_TRACKING)

  def try_state_transition(self, target_sys_state):
    if target_sys_state == SystemState.HOVERING:
      if np.abs(self._env.state[3] - self.hovering_height) < 0.2:
        self._sys_state = SystemState.HOVERING
        return
      else:
        desired_states = np.array([0, 0, self.hovering_height, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    elif target_sys_state == SystemState.GATE_TRACKING:
      desired_states = self._planner.get_desired_state(self._env.states, next_gate_loc=None)

    actions = self._controller.compute_action(self._env.states, desired_states)
    self._env.publish_actions(actions)


if __name__ == '__main__':
  start_time = rospy.get_rostime()
  controller = PIDController()  # controller should be independent of time
  env = GroundTruthEnv(start_time)
  planner = HeuristicPlanner(start_time)
  localizer = None
  explorer = None
  state_machine = StateMachine(env, controller, planner, localizer, explorer)

  while True:
    try:
      state_machine.spin()
    except rospy.ROSInterruptException:
      break
