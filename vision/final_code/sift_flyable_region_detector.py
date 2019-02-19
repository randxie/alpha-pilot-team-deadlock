import cv2
import json
import glob
import numpy as np
import os

from collections import defaultdict
from matplotlib import pyplot as plt

MIN_MATCH_COUNT = 25
MAX_MATCH_POINTS = 5
FLANN_INDEX_KDTREE = 0
INDEX_PARAMS = dict(algorithm=FLANN_INDEX_KDTREE, trees=10)
SEARCH_PARAMS = dict(checks=50)

# use file directory as root path
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_DIR = os.path.join(PROJECT_DIR, 'template')


class SiftFlyableRegionDetector(object):

  def __init__(self, template_path=os.path.join(TEMPLATE_DIR, 'template.jpg')):
    self.template_img = cv2.imread(template_path, 0)
    self.feature_extractor = cv2.xfeatures2d.SIFT_create(nfeatures=1000)

    # find the keypoints and descriptors with SIFT
    self.kp_template, self.des_template = self.feature_extractor.detectAndCompute(self.template_img, None)
    self.matcher = cv2.FlannBasedMatcher(INDEX_PARAMS, SEARCH_PARAMS)

  def detect(self, image, bbox, visualize=False):
    ymin, xmin, ymax, xmax = bbox
    img_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    img_to_match = img_gray[ymin:ymax, xmin:xmax]
    try:
      kp_to_match, des_to_match = self.feature_extractor.detectAndCompute(img_to_match, None)
    except:
      print(bbox)

    transform, matches_mask, good_matches, succeeded = self.match_features(kp_to_match, des_to_match)

    if succeeded:
      h, w = self.template_img.shape
      y_c = h / 2.0
      x_c = w / 2.0
      dh = h / 2.0 / 1.4
      dw = w / 2.0 / 1.4
      pts = np.float32(
        [[x_c - dw, y_c - dh], [x_c - dw, y_c + dh], [x_c + dw, y_c + dh], [x_c + dw, y_c - dh]]).reshape(-1, 1, 2)
      dst = cv2.perspectiveTransform(pts, transform)
      area = cv2.contourArea(dst)

      # filter out too small region, very likely to get wrong matching
      if area < 1000:
        dst = []
        succeeded = False

      if succeeded:
        rect = np.array(dst)[:, 0]
        x_min = max(int(np.min(rect[:, 0])), 0)
        x_max = int(np.max(rect[:, 0]))
        y_min = max(int(np.min(rect[:, 1])), 0)
        y_max = int(np.max(rect[:, 1]))
        img_w = x_max - x_min
        img_h = y_max - y_min
      else:
        print('Can not match template')

      if succeeded and visualize:
        img_to_draw = cv2.polylines(img_to_match, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
        draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                           singlePointColor=None,
                           matchesMask=matches_mask,  # draw only inliers
                           flags=2)
        cv2.rectangle(img_to_draw, (x_min, y_min), (x_min + img_w, y_min + img_h), (0, 255, 0), 5)
        plt.imshow(img_to_draw, 'gray'), plt.show()
    else:
      dst = []

    # return
    ymin, xmin, ymax, xmax = bbox

    if len(dst) > 0:
      coordinates = []
      for pairs in dst:
        cur_pt = pairs[0]
        cur_x = xmin + cur_pt[0]
        cur_y = ymin + cur_pt[1]
        coordinates.append(cur_x)
        coordinates.append(cur_y)
    else:
      coordinates = [xmin, ymin, xmin, ymax, xmax, ymax, xmax, ymin]

    return coordinates

  def match_features(self, kp_to_match, des_to_match):
    matches = self.matcher.knnMatch(self.des_template, des_to_match, k=2)
    point_map = defaultdict(int)

    # store all the good matches as per Lowe's ratio test.
    good_matches = []

    for m, n in matches:
      point_map[m] += 1
      point_map[n] += 1
      if m.distance < 0.7 * n.distance:
        good_matches.append(m)

    max_num_match = max(point_map.values())

    if len(good_matches) > MIN_MATCH_COUNT and max_num_match < MAX_MATCH_POINTS:
      src_pts = np.float32([self.kp_template[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
      dst_pts = np.float32([kp_to_match[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

      transform, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
      matches_mask = mask.ravel().tolist()

      return transform, matches_mask, good_matches, True

    else:
      return None, None, None, False


if __name__ == '__main__':
  from predictor import Predictor

  model_dir = 'weights/ssd-mobilenet'
  image_dir = 'training/small'
  flyable_region_detector = SiftFlyableRegionDetector()
  img_predictor = Predictor(model_dir, image_dir, batch_size=1, flyable_region_detector=flyable_region_detector)
  img_predictor.run_inference(visualize=True)
