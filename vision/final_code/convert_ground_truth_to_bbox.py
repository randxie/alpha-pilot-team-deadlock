import cv2
import json
import numpy as np
import os
import matplotlib
import matplotlib.pyplot as plt
from utils import util_plotting

ORIGINAL_IMAGE_WIDTH = 1296
ORIGINAL_IMAGE_HEIGHT = 864
SCALE_FACTOR = 1.6

ground_truth_filename = 'training/training_GT_labels.json'
image_dir = 'training/images'

with open(ground_truth_filename, 'r') as f:
  ground_truth_dict = json.load(f)

output_dict = {}
for img_key, bbox in ground_truth_dict.items():
  bbox_i = bbox[0]
  if bbox_i:
    x1 = bbox_i[0]
    y1 = bbox_i[1]
    x2 = bbox_i[2]
    y2 = bbox_i[3]
    x3 = bbox_i[4]
    y3 = bbox_i[5]
    x4 = bbox_i[6]
    y4 = bbox_i[7]

    xmin = min(x1, x2, x3, x4)
    xmax = max(x1, x2, x3, x4)
    ymin = min(y1, y2, y3, y4)
    ymax = max(y1, y2, y3, y4)

    xcenter = (xmin + xmax) / 2.0
    ycenter = (ymin + ymax) / 2.0
    w = xmax - xmin
    h = ymax - ymin

    # expand bounding box by 1.4 times
    x_min = max(xcenter - SCALE_FACTOR * w / 2.0, 0)
    x_max = min(xcenter + SCALE_FACTOR * w / 2.0, ORIGINAL_IMAGE_WIDTH)
    y_min = max(ycenter - SCALE_FACTOR * h / 2.0, 0)
    y_max = min(ycenter + SCALE_FACTOR * h / 2.0, ORIGINAL_IMAGE_HEIGHT)

    coordinates = np.array([xmin, ymin, xmax, ymin, xmax, ymax, xmin, ymax]).astype(np.int)
    output_dict[img_key] = {'x_min': int(x_min), 'y_min': int(y_min), 'x_max': int(x_max), 'y_max': int(y_max)}
  else:
    print(img_key)

with open('training/training_gt_bbox.json', 'w') as outfile:
  json.dump(output_dict, outfile)
