"""Execute the state machine with ground truth information."""
import numpy as np
import rospy
from controller.basic_controller import PIDController
from env_interface.ground_truth_env import GroundTruthEnv
from planner.heuristic_planner import HeuristicPlanner
from planner.fast_trajectory_planner import FastTrajectoryPlanner
import time

GATE_ORDER = [10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6] # 19,
TARGET_PSi = [np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, 0, np.pi/2, np.pi/2, np.pi/2, np.pi/2]


class SystemState(object):
  START = 0
  HOVERING = 1
  EXPLORING = 2
  GATE_TRACKING = 3
  GATE_PASSING = 4
  STOP = 5


class StateMachine(object):
  def __init__(self, env_interface, controller_node, planner_node, localizer_node, explorer_node):
    self._env = env_interface
    self._controller = controller_node
    self._planner = planner_node
    self._localizer = localizer_node
    self._explorer = explorer_node
    self._sys_state = SystemState.START
    self._cur_gate_id = 0
    self.hovering_height = 5

  def gate_searching(self):
    pass

  def gate_tracking(self):
    pass

  def spin(self):
    if self._sys_state == SystemState.START:
      self.try_state_transition(SystemState.HOVERING)
    elif self._sys_state == SystemState.HOVERING:
      self.try_state_transition(SystemState.EXPLORING)
    elif self._sys_state == SystemState.EXPLORING:
      self.try_state_transition(SystemState.GATE_TRACKING)
    elif self._sys_state == SystemState.GATE_TRACKING:
      self.try_state_transition(SystemState.GATE_PASSING)

  def try_state_transition(self, target_sys_state):
    if target_sys_state == SystemState.HOVERING:
      if np.abs(self._env.states[2] - self.hovering_height) < 0.5:
        self._sys_state = SystemState.HOVERING
      desired_states = np.array([0, 0, self.hovering_height, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    elif target_sys_state == SystemState.EXPLORING:
      # explorer only activates after passing the gate or at hovering state.
      gate_found = True  # should interact with gate detector / env
      if self._sys_state == SystemState.HOVERING:
        if gate_found:
          self._sys_state = SystemState.GATE_TRACKING
          # reset planner for gate tracking
          self._planner._start_time = time.time()
        else:
          # should get desired_state from explorer
          pass
      elif self._sys_state == SystemState.GATE_PASSING:
        pass
      desired_states = np.array([0, 0, self.hovering_height, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    elif target_sys_state == SystemState.GATE_PASSING:
      # if it is close enough to gate, transit to gate passing, otherwise stay
      next_gate_loc = self._planner.gate_map.get(GATE_ORDER[self._cur_gate_id], None)
      desired_states = self._planner.get_desired_state(self._env.states, next_gate_loc=next_gate_loc)
      desired_states[5] = TARGET_PSi[self._cur_gate_id]
      print(self._cur_gate_id, next_gate_loc)
      if np.linalg.norm(np.array(self._env.states[0:3]) - np.array(next_gate_loc)) < 5:
        self._planner.reset()
        self._cur_gate_id += 1
        if self._cur_gate_id >= len(GATE_ORDER):
          self._sys_state = SystemState.STOP
          print('Passing all the gates')
          raise rospy.ROSInterruptException('Stop program')

    actions = self._controller.compute_action(self._env.states, desired_states)
    print('action', actions)
    self._env.step(actions)


if __name__ == '__main__':
  start_time = time.time()
  controller = PIDController()  # controller should be independent of time
  env = GroundTruthEnv(start_time)
  env.spin_listeners()
  planner = FastTrajectoryPlanner(start_time)
  localizer = None
  explorer = None
  state_machine = StateMachine(env, controller, planner, localizer, explorer)

  while True:
    rospy.sleep(0.05)
    try:
      state_machine.spin()
    except rospy.ROSInterruptException:
      break
