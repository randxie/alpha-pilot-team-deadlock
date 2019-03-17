"""Execute the state machine with ground truth information."""
# Work In Progress

import argparse
import numpy as np
import rospy
from controller.basic_controller import PDController
from controller.basic_controller import PIDController
from env_interface.fg_vins_env import FgVinsEnv
from gate_estimator.simple_estimator import SimpleEstimator
from planner.fast_trajectory_planner import FastTrajectoryPlanner
import time

GATE_ORDER = [10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6]
TARGET_PSi = [np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, 0, np.pi / 2, np.pi / 2, np.pi / 2,
              np.pi / 2, np.pi / 2]

parser = argparse.ArgumentParser()
parser.add_argument("--gate_yaml", default=None, type=str, help="Gate location yaml file")


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
    self.hovering_height = 5
    self._cur_psi = 0

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
      if np.abs(self._env.states[2] - self.hovering_height) < 0.5:
        self._sys_state = SystemState.HOVERING
      desired_states = np.array([0, 0, self.hovering_height, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    elif target_sys_state == SystemState.GATE_FOUND:
      """
      Explorer only activates after passing the gate or at hovering state.
      """
      gate_found = True  # should interact with gate detector / env
      if gate_found:
        self._sys_state = SystemState.GATE_FOUND
        return
      else:
        # exploring
        pass
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
      gate_loc = gate_center - gate_vec_loc * 2 * np.sign(TARGET_PSi[self._cur_gate_id]) * np.sign(gate_center[1])
      if np.linalg.norm(np.array(self._env.states[0:3]) - np.array(gate_loc)) < 3:
        self._sys_state = SystemState.GATE_PASSING
      desired_states = self._planner.get_desired_state(self._env.states, next_gate_loc=gate_loc, next_gate_yaw=TARGET_PSi[self._cur_gate_id], cur_gate_yaw=self._cur_psi)
      #desired_states = np.array([gate_loc[0], gate_loc[1], gate_loc[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
      #desired_states[5] = self._cur_psi
      #print('passing', self._cur_gate_id, gate_loc, desired_states[0:3])
    elif target_sys_state == SystemState.GATE_ADJUST_POSE:
      gate_center = self._planner.gate_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_vec_loc = self._planner.vec_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_loc = gate_center - gate_vec_loc * 1.25 * np.sign(TARGET_PSi[self._cur_gate_id]) * np.sign(gate_center[1])
      if np.linalg.norm(np.array(self._env.states[0:3]) - np.array(gate_loc)) < 0.5:
        self._cur_psi = TARGET_PSi[self._cur_gate_id + 1]
        self._sys_state = SystemState.GATE_ADJUST_POSE
      #print('adjust pose', self._cur_gate_id, gate_loc, self._env.states[5], TARGET_PSi[self._cur_gate_id])
      desired_states = np.array([gate_loc[0], gate_loc[1], gate_loc[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
      desired_states[5] = TARGET_PSi[self._cur_gate_id]
    elif target_sys_state == SystemState.GATE_PASSED:
      """
      if it is close enough to gate, transit to gate passing, otherwise stay
      """
      gate_center = self._planner.gate_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_vec_loc = self._planner.vec_map.get(GATE_ORDER[self._cur_gate_id], None)
      gate_loc = gate_center + 4 * gate_vec_loc * np.sign(TARGET_PSi[self._cur_gate_id]) * np.sign(gate_center[1])
      #desired_states = self._planner.get_desired_state(self._env.states, next_gate_loc=gate_loc)
      desired_states = np.array([gate_loc[0], gate_loc[1], gate_loc[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
      desired_states[5] = self._cur_psi
      #print('passed', self._cur_gate_id, gate_loc)
      if np.linalg.norm(np.array(self._env.states[0:3]) - np.array(gate_loc)) < 1:
        self._planner.reset()
        self._cur_gate_id += 1
        self._sys_state = SystemState.GATE_FOUND

    actions = self._controller.compute_action(self._env.states, desired_states)
    #print('states', self._env.states[0:3], desired_states[0:3], self._env.states[9:12])
    self._env.step(actions)

    # distance estimation
    cur_x_y = self._env.states[0:2]
    gate_x_y = self._planner.gate_map[GATE_ORDER[self._cur_gate_id]][0:2]
    estimated_depth = self._gate_estimator.estimate(self._env.ir_marker_queue, GATE_ORDER[self._cur_gate_id])
    if estimated_depth:
      print('gate: %d, true depth: %f, estimated depth: %f' % (self._env.target_gate, np.linalg.norm(gate_x_y - cur_x_y), estimated_depth))


if __name__ == '__main__':
  args = parser.parse_args()
  start_time = time.time()
  controller = PDController()  # controller should be independent of time
  env = FgVinsEnv(start_time)
  env.spin_listeners()
  planner = FastTrajectoryPlanner(start_time)
  localizer = None
  explorer = None
  gate_estimator = SimpleEstimator(planner.dim_map)
  state_machine = StateMachine(env, controller, planner, localizer, explorer, gate_estimator)

  while True:
    rospy.sleep(0.01)
    try:
      state_machine.spin()
    except rospy.ROSInterruptException:
      break
