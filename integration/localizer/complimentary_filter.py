import numpy as np


# Ref: https://www.youtube.com/watch?v=whSw42XddsU

class ComplimentaryFilter(object):
  def __init__(self, phi_init=0.0, gyro_init=0.0):
    self.phi_prev = phi_init
    self.theta_prev = gyro_init

  def fuse(self, raw_accel, raw_gyro, dt):
    # Takes in noisy accelerometer and gyroscope data. Caches roll and pitch angle. Outputs non-drifting roll and pitch angle.

    acc_x = raw_accel[0]
    acc_y = raw_accel[1]
    acc_z = raw_accel[2]

    p = raw_gyro[0]
    q = raw_gyro[1]
    r = raw_gyro[2]

    acc_phi = np.arctan2(acc_x, acc_z)
    acc_theta = np.arctan2(acc_y, acc_z)

    alpha = 0.98  # low pass filter gain, typically between 0.90 - 0.99
    phi_fused = alpha * (self.phi_prev + p * dt) + (1 - alpha) * acc_phi
    theta_fused = alpha * (self.theta_prev + q * dt) + (1 - alpha) * acc_theta

    # cache
    self.phi_prev = phi_fused
    self.theta_prev = theta_fused

    return phi_fused, theta_fused
