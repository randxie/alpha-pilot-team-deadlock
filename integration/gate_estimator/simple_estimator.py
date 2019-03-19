import cv2
import numpy as np
from planner.fast_trajectory_planner import estimate_dimension
import tf

FX = 548  # focal length in pixel space
FY = 548  # focal length in pixel space
CX = 512  # principal point x
CY = 384  # principal point y


def estimate_dimension_pixel(gate_pixel):
  """Estimate gate dimension."""
  num_pts = gate_pixel.shape[0]
  if num_pts <= 1:
    return None, None
  elif num_pts == 2:
    height_pixel = np.max(gate_pixel[:, 2]) - np.min(gate_pixel[:, 2])
    width_pixel = np.max(gate_pixel[:, 1]) - np.min(gate_pixel[:, 1])
    if height_pixel > width_pixel:
      return None, height_pixel
    else:
      return width_pixel, None
  elif num_pts == 3:
    height_pixel = np.max(gate_pixel[:, 2]) - np.min(gate_pixel[:, 2])
    width_pixel = np.max(gate_pixel[:, 1]) - np.min(gate_pixel[:, 1])
    return width_pixel, height_pixel
  else:
    top_center = np.mean(gate_pixel[0:2, :], axis=0)
    bottom_center = np.mean(gate_pixel[2:, :], axis=0)
    left_center = np.mean(gate_pixel[[0, 3], :], axis=0)
    right_center = np.mean(gate_pixel[[1, 2], :], axis=0)
    height_pixel = np.linalg.norm(top_center - bottom_center)
    width_pixel = np.linalg.norm(left_center - right_center)
    return width_pixel, height_pixel


class SimpleEstimator(object):

  def __init__(self, dim_map):
    self._dim_map = dim_map

  def estimate(self, ir_queue, target_gate=10):
    gate_width, gate_height = self._dim_map[target_gate]
    if not ir_queue.empty():
      data_list = list(ir_queue.queue)
      gate_loc = np.zeros((len(data_list), 3))
      weight_vec = np.zeros(len(data_list))
      for i, data in enumerate(data_list):
        gate_pixel, states = data

        # find the vertical edge of gate and use that as reference
        width_pixel, height_pixel = estimate_dimension_pixel(gate_pixel)

        # more points have higher confidence
        weight_vec[i] = gate_pixel.shape[0]

        ratio = []
        if width_pixel:
          ratio.append(gate_width / width_pixel)

        if height_pixel:
          ratio.append(gate_height / height_pixel)

        if len(ratio):
          ratio = np.mean(ratio)
          # ref: https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
          # estimate distance to the gate using focal length
          dx = ratio * FY

          # ref: https://www.pyimagesearch.com/2016/04/04/measuring-distance-between-objects-in-an-image-with-opencv/
          # estimate distance in y-direction and z-direction by mapping pixel distance to meter

          # find gate center
          gate_cx = np.average(gate_pixel[:, 1])
          gate_cy = np.average(gate_pixel[:, 2])

          # compute distance to image center
          dy_pixel = CX - gate_cx
          dz_pixel = CY - gate_cy

          # convert pixel to meter
          dy = dy_pixel * ratio
          dz = dz_pixel * ratio

          psi = states[5]

          gate_loc[i, 0] = states[0] + dx * np.cos(psi) - dy * np.sin(psi)
          gate_loc[i, 1] = states[1] - dx * np.sin(psi) + dy * np.cos(psi)
          gate_loc[i, 2] = states[2] + dz

      # take the average
      return np.average(gate_loc, weights=weight_vec, axis=0)

    return None
