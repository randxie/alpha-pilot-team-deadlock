import numpy as np
import pbcvt
import matplotlib.pyplot as plt
from pgm_util import read_pgm
import matplotlib.image as mpimg


def rgb2gray(rgb):
  r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
  gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
  return gray


"""
left_img = read_pgm('sample_data/imgleft000000009.pgm').astype(np.uint16)
right_img = read_pgm('sample_data/imgright000000009.pgm').astype(np.uint16)
"""

left_img = (255.0 * rgb2gray(mpimg.imread("sample_data/left.png"))).astype(np.uint16)
right_img = (255.0 * rgb2gray(mpimg.imread("sample_data/right.png"))).astype(np.uint16)

depth_img = pbcvt.depth_estimate(left_img, right_img)
plt.imshow(depth_img)
plt.show()
