import cv2
import numpy as np
from planner.fast_trajectory_planner import estimate_dimension
import tf
from scipy.spatial.transform import Rotation as Rot

FX = 548  # focal length in pixel space
FY = 548  # focal length in pixel space
CX = 512  # principal point x
CY = 384  # principal point y

camWidth = 1024
camHeight = 768
camFOV = 70.0
camDepthScale = 0.20
f = (camHeight / 2.0) / np.tan((np.pi * (camFOV / 180.0)) / 2.0)
cx = camWidth / 2.0
cy = camHeight / 2.0
tx, ty = 0.0, 0.0
camera_D = np.array([0.0, 0.0, 0.0, 0.0, 0.0],dtype=np.float64) # distortion
camera_K = np.array([[f, 0.0, cx],[0.0, f, cy],[0.0, 0.0, 1.0]],dtype=np.float64)
camera_R = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]],dtype=np.float64)
camera_P = np.array([[f, 0.0, cx],[tx, 0.0, f],[cy, ty, 0.0],[0.0, 1.0, 0.0]],dtype=np.float64)

def euler_to_rot_mat(roll, pitch, yaw):
  r = Rot.from_euler('xyz', [roll, pitch, yaw]).as_dcm()
  return r

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

class PnpEstimator(object):
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
        #if width_pixel:
        #ratio.append(gate_width / width_pixel)

        if height_pixel:
          ratio.append(gate_height / height_pixel)

        if len(ratio):
          image_points = gate_pixel
        
          # https://github.com/Tetragramm/opencv_contrib/blob/master/modules/mapping3d/src/positionCalc.cpp#L141
          # http://answers.opencv.org/question/205999/convert-image-points-to-azimuth-and-elevation/
          # http://answers.opencv.org/question/150451/how-to-tranform-2d-image-coordinates-to-3d-world-coordinated-with-z-0/

          # lets see if we can use state estimated by vins-mono
          position = states[0:3]
          pose = states[3:6]

          los1 = np.array([ gate_pixel[0,0], gate_pixel[0,1], 1.0 ], dtype=np.float64) 
          los2 = np.array([ gate_pixel[1,0], gate_pixel[1,1], 1.0 ], dtype=np.float64) 
          los3 = np.array([ gate_pixel[2,0], gate_pixel[2,1], 1.0 ], dtype=np.float64) 
          los4 = np.array([ gate_pixel[3,0], gate_pixel[3,1], 1.0 ], dtype=np.float64) 

          # convert pose to Rotation matrix
          R = euler_to_rot_mat(*pose).T     # 3 x 3

          # rotation_matrix = np.zeros(shape=(3,3))
          # cv2.Rodrigues(rvecs, rotation_matrix)


          tvec = np.array(position, dtype=np.float64).T

          inv_cam = np.linalg.inv(camera_K)

          los1 = np.dot(inv_cam , los1)
          los2 = np.dot(inv_cam , los2)
          los3 = np.dot(inv_cam , los3)
          los4 = np.dot(inv_cam , los4)

          # create pose matrix (3 x 4)
          camera_translation = - np.dot(R, tvec)
          los1 = np.dot(R, los1)  
          los2 = np.dot(R, los2)
          los3 = np.dot(R, los3)
          los4 = np.dot(R, los4)

          gate_loc[:,0] = - camera_translation[0] + states[0]
          gate_loc[:,1] = - camera_translation[1] + states[1]
          gate_loc[:,2] = - camera_translation[2] + states[2]

      # take the average
      return np.average(gate_loc, weights=weight_vec, axis=0)

    return None
