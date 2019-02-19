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
visualize = False

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

    x_min = min(x1, x2, x3, x4)
    x_max = max(x1, x2, x3, x4)
    y_min = min(y1, y2, y3, y4)
    y_max = max(y1, y2, y3, y4)

    x_center = (x_min + x_max) / 2.0
    y_center = (y_min + y_max) / 2.0
    w = x_max - x_min
    h = y_max - y_min

    dw = SCALE_FACTOR * w / 2.0
    dh = SCALE_FACTOR * h / 2.0

    # usually height is sufficient, while width is too small
    aspect_ratio = dh / dw
    if aspect_ratio > 2.5:
      dw = dh / 2.5

    # expand bounding box by 1.4 times
    x_min = max(x_center - dw, 0)
    x_max = min(x_center + dw, ORIGINAL_IMAGE_WIDTH)
    y_min = max(y_center - dh, 0)
    y_max = min(y_center + dh, ORIGINAL_IMAGE_HEIGHT)

    coordinates = np.array([x_min, y_min, x_max, y_min, x_max, y_max, x_min, y_max]).astype(np.int)
    if visualize:
      original_image = cv2.imread(os.path.join(image_dir, img_key))
      util_plotting.plot_bbox(original_image, [coordinates, bbox_i])
      plt.show()
      plt.close()

    output_dict[img_key] = {'x_min': int(x_min), 'y_min': int(y_min), 'x_max': int(x_max), 'y_max': int(y_max),
                            'segmentation': bbox_i}
  else:
    print(img_key)

with open('training/training_gt_bbox.json', 'w') as outfile:
  json.dump(output_dict, outfile)
