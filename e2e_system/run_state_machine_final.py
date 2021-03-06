"""Execute the state machine with ground truth information."""
import argparse
import numpy as np
import rospy
from controller.basic_controller import PDController
from env_interface.fg_vins_env import FgVinsEnv
from gate_estimator.simple_estimator import SimpleEstimator
from gate_estimator.bayesian_estimator import BayesianEstimator
from planner.missile_planner import MissilePlanner
import time

GATE_ORDER = [10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6]
TARGET_HEADING = [1, 1, -1, -1, -1, -1, 1, 1, 1, 1, 1]


class SystemState(object):
  START = 0
  HOVERING = 1
  EXPLORING = 2
  GATE_FOUND = 3
  GATE_TRACKING = 4
  GATE_PASSING = 5
  GATE_ADJUST_POSE = 6
  GATE_PASSED = 7
  STOP = 8


class StateMachine(object):
  def __init__(self, env_interface, controller_node, planner_node, localizer_node, explorer_node, gate_estimator):
    self._env = env_interface
    self._controller = controller_node
    self._planner = planner_node
    self._localizer = localizer_node
    self._explorer = explorer_node
    self._gate_estimator = gate_estimator
    self._sys_state = SystemState.START
    self._cur_gate_id = 0
    self.hovering_height = self._env.height_offset + 2

  def gate_searching(self):
    pass

  def gate_tracking(self):
    pass

  def spin(self):
    if self._sys_state == SystemState.START:
      self.try_state_transition(SystemState.HOVERING)
    elif self._sys_state == SystemState.HOVERING:
      self.try_state_transition(SystemState.GATE_FOUND)
    elif self._sys_state == SystemState.GATE_FOUND:
      self.try_state_transition(SystemState.GATE_TRACKING)
    elif self._sys_state == SystemState.GATE_TRACKING:
      self.try_state_transition(SystemState.GATE_PASSING)
    elif self._sys_state == SystemState.GATE_PASSING:
      self.try_state_transition(SystemState.GATE_ADJUST_POSE)
    elif self._sys_state == SystemState.GATE_ADJUST_POSE:
      self.try_state_transition(SystemState.GATE_PASSED)

  def try_state_transition(self, target_sys_state):
    if target_sys_state == SystemState.HOVERING:
      """
      From start to hovering
      """
      if np.abs(self._env.states[2] - self.hovering_height) < 0.3 and self._env.is_vins_inited:
        self._sys_state = SystemState.HOVERING
      if self._env.is_vins_inited:
        desired_states = np.array(
          [0, 0, self.hovering_height, 0, 0, 0,
           0, 0, 0, 0, 0, 0])
      else:
        desired_states = np.array(
          [0, 0, self.hovering_height, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    elif target_sys_state == SystemState.GATE_FOUND:
      """
      Explorer only activates after passing the gate or at hovering state.
      """
      if True:
        self._sys_state = SystemState.GATE_FOUND
        return
    elif target_sys_state == SystemState.GATE_TRACKING:
      if self._cur_gate_id < len(GATE_ORDER):
        self._sys_state = SystemState.GATE_TRACKING
        self._env.target_gate = GATE_ORDER[self._cur_gate_id]
        # clear previous marker as we are entering a new gate.
        self._env.ir_marker_queue.queue.clear()
        self._planner._start_time = time.time()
        return
      else:
        self._sys_state = SystemState.STOP
        print('Passing all the gates')
        raise rospy.ROSInterruptException('Stop program')
    elif target_sys_state == SystemState.GATE_PASSING:
      """
      if it is close enough to gate, transit to gate passing, otherwise stay
      """
      gate_center = self._planner.gate_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_vec_loc = self._planner.vec_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_loc = gate_center - gate_vec_loc * 2 * TARGET_HEADING[self._cur_gate_id]
      if np.linalg.norm(np.array(self._env.states[0:3]) - np.array(gate_loc)) < 3:
        self._sys_state = SystemState.GATE_PASSING
      desired_states = self._planner.get_desired_state(self._env.states, next_gate_loc=gate_loc)
    elif target_sys_state == SystemState.GATE_ADJUST_POSE:
      gate_center = self._planner.gate_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_vec_loc = self._planner.vec_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_loc = gate_center - gate_vec_loc * 2 * TARGET_HEADING[self._cur_gate_id] * np.sign(gate_center[0])
      gate_width, gate_height = self._planner.dim_map[GATE_ORDER[self._cur_gate_id]]
      if np.linalg.norm(np.array(self._env.states[0:3]) - np.array(gate_loc)) < (gate_width / 2.5):
        self._sys_state = SystemState.GATE_ADJUST_POSE

      if np.linalg.norm(np.array(self._env.states[0:3]) - np.array(gate_loc)) < 6:
        gate_loc_ref = gate_center + gate_vec_loc * 3.5 * TARGET_HEADING[self._cur_gate_id]
        desired_state_ref = self._planner.get_desired_state(self._env.states, next_gate_loc=gate_loc_ref)
      else:
        desired_state_ref = None

      desired_states = self._planner.get_desired_state(self._env.states, next_gate_loc=gate_loc)
      if desired_state_ref is not None:
        desired_states[5] = desired_state_ref[5]
        #print('adjust pose: ', self._cur_gate_id, gate_loc, self._env.states[5], desired_state_ref[5])
    elif target_sys_state == SystemState.GATE_PASSED:
      """
      if it is close enough to gate, transit to gate passing, otherwise stay
      """
      gate_center = self._planner.gate_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_vec_loc = self._planner.vec_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_loc = gate_center + 4 * gate_vec_loc * TARGET_HEADING[self._cur_gate_id]
      desired_states = self._planner.get_desired_state(self._env.states, next_gate_loc=gate_loc)
      if np.linalg.norm(np.array(self._env.states[0:3]) - np.array(gate_loc)) < 1.5:
        self._planner.reset()
        self._cur_gate_id += 1
        self._sys_state = SystemState.HOVERING
      #print('flying after gate: ', self._cur_gate_id, gate_loc)

    actions = self._controller.compute_action(self._env.states, desired_states)
    """
    estimate_xyz = self._gate_estimator.estimate(self._env.ir_marker_queue, GATE_ORDER[self._cur_gate_id])
    if estimate_xyz is not None:
      print('est: ', estimate_xyz, 'true: ', self._planner.gate_map[GATE_ORDER[self._cur_gate_id]])
    """
    self._env.step(actions)


if __name__ == '__main__':
  start_time = time.time()
  controller = PDController()  # controller should be independent of time
  env = FgVinsEnv(start_time)
  env.spin_listeners()
  planner = MissilePlanner(start_time)
  localizer = None
  explorer = None
  gate_estimator = BayesianEstimator(planner.gate_map, planner.vec_map, planner.dim_map, planner.perturb_map)
  state_machine = StateMachine(env, controller, planner, localizer, explorer, gate_estimator)

  while True:
    rospy.sleep(0.008)
    try:
      state_machine.spin()
    except rospy.ROSInterruptException:
      break
