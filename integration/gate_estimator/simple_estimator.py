import cv2
import numpy as np
from planner.fast_trajectory_planner import estimate_dimension

FX = 548  # focal length in pixel space
FY = 548  # focal length in pixel space
CX = 512  # principal point x
CY = 384  # principal point y

def estimate_dimension_pixel(gate_pixel):
  """Estimate gate dimension."""
  A = np.array(gate_pixel)
  if A.shape[0] <= 1:
    return None, None
  height = np.max(A[:, 2]) - np.min(A[:, 2])
  width = np.max(np.linalg.norm(A[1:, 0:2] - A[0, 0:2], axis=1))
  return width, height

class SimpleEstimator(object):

  def __init__(self, dim_map):
    self._dim_map = dim_map

  def estimate(self, ir_queue, target_gate=10):
    gate_width, gate_height = self._dim_map[target_gate]
    if not ir_queue.empty():
      for data in list(ir_queue.queue):
        gate_pixel, states = data
        width_pixel, height_pixel = estimate_dimension_pixel(gate_pixel)
        if width_pixel:
          depth = gate_height * FY / height_pixel

          return depth

    return None



