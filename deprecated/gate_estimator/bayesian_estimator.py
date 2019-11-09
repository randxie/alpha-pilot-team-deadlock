import cv2
import numpy as np
from planner.fast_trajectory_planner import estimate_dimension
from utils.util_vision import order_points
import rospy
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


class BayesianEstimator(object):

  def __init__(self, gate_map, vec_map, dim_map, perturb_map):
    self.gate_map = gate_map
    self.vec_map = vec_map
    self.dim_map = dim_map
    self.perturb_map = perturb_map
    self.data = []
    self.weight = []
    self.camera_matrix = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]])

  def reset(self):
    self.data = []
    self.weight = []

  def estimate(self, ir_queue, target_gate=10):
    gate_width, gate_height = self.dim_map[target_gate]
    if not ir_queue.empty():
      data_list = list(ir_queue.queue)
      for i, data in enumerate(data_list):
        #gate_loc = (self.gate_map[target_gate])
        init_pose = rospy.get_param('/uav/flightgoggles_uav_dynamics/init_pose')
        xyz_offset = init_pose[0:3]

        gate_loc = np.array(rospy.get_param('/uav/Gate%d/nominal_location' % target_gate))

        gate_loc[:, 0] = gate_loc[:, 0] - xyz_offset[0]
        gate_loc[:, 1] = gate_loc[:, 1] - xyz_offset[1]
        tmp = gate_loc[:, 0].copy()
        gate_loc[:, 0] = gate_loc[:, 1]
        gate_loc[:, 1] = -tmp

        # convert to camera coordinate
        gate_loc_2 = gate_loc.copy()
        gate_loc[:, 2] = gate_loc_2[:, 0]
        gate_loc[:, 0] = -gate_loc_2[:, 1]
        gate_loc[:, 1] = -gate_loc_2[:, 2]

        tmp = gate_loc[0, :].copy()
        gate_loc[0, :] = gate_loc[1, :]
        gate_loc[1, :] = tmp

        gate_pixel, states = data
        gate_pixel = gate_pixel.astype(np.float32)

        # get center points
        gate_loc_center = np.zeros((5, 3))
        gate_pixel_center = np.zeros((5, 2))
        for i in range(4):
          if i < 3:
            gate_loc_center[i, :] = (gate_loc[i, :] + gate_loc[i+1, :]) / 2
            gate_pixel_center[i, :] = (gate_loc[i, :] + gate_loc[i + 1, :]) / 2
          else:
            gate_loc_center[i, :] = (gate_loc[i, :] + gate_loc[0, :]) / 2
            gate_pixel_center[i, :] = (gate_loc[i, :] + gate_loc[0, :]) / 2

        gate_loc_center[5, :] = np.mean(gate_loc, axis=0)
        gate_pixel_center[5, :] = np.mean(gate_pixel, axis=0)

        gate_loc = np.concatenate((gate_loc, gate_loc_center), axis=0)
        gate_pixel = np.concatenate((gate_pixel, gate_pixel_center), axis=0)

        _, _, tvec = cv2.solvePnP(gate_loc, gate_pixel, self.camera_matrix, None)

        #loc = states[0:2] - tvec.ravel()[0:2]
        #print(gate_loc)
        #print(gate_pixel)
        print(tvec)
        return None


        # take the average
        #return np.average(gate_loc, weights=weight_vec, axis=0)

    return None
