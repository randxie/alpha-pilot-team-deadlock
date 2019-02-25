"""Stores quadcopter related constants. Use Python file as config."""

class QuadConfig(object):
  ROTOR_THRUST_CONSTANT = 100
  ROTOR_DRAG_CONSTANT = 100
  QUAD_ARM_LENGTH = 1
  QUAD_MASS = 1

class PIDControllerConfig(object):
  # position control
  kp_x = 0.01
  kp_x_dot = 2
  kp_y = 0.01
  kp_y_dot = 2
  kp_z = 5
  kp_z_dot = 2

  # angular control
  kp_phi = 10
  kp_phi_dot = 0.9
  kp_theta = 10
  kp_theta_dot = 0.9
  kp_psi = 10
  kp_psi_dot = 0.9


GRAVITY_COEFF = 9.8