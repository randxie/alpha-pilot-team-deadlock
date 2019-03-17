import cv2
import numpy as np
from scipy.spatial import distance as dist


def order_points(pts):
  # sort the points based on their x-coordinates
  xSorted = pts[np.argsort(pts[:, 0]), :]

  # grab the left-most and right-most points from the sorted
  # x-roodinate points
  leftMost = xSorted[:2, :]
  rightMost = xSorted[2:, :]

  # now, sort the left-most coordinates according to their
  # y-coordinates so we can grab the top-left and bottom-left
  # points, respectively
  leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
  (tl, bl) = leftMost

  # now that we have the top-left coordinate, use it as an
  # anchor to calculate the Euclidean distance between the
  # top-left and right-most points; by the Pythagorean
  # theorem, the point with the largest distance will be
  # our bottom-right point
  num_right_pts = rightMost.shape[0]
  if num_right_pts > 0:
    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    if num_right_pts > 1:
      (br, tr) = rightMost[np.argsort(D)[::-1], :]

      # return the coordinates in top-left, top-right,
      # bottom-right, and bottom-left order
      return np.array([tl, tr, br, bl], dtype="float32")
    else:
      tr = rightMost[0, :]
      # return the coordinates in top-left, top-right,
      # bottom-right, and bottom-left order
      return np.array([tl, tr, bl], dtype="float32")
  else:
    # only two points, up then down
    return np.array([tl, bl], dtype="float32")
