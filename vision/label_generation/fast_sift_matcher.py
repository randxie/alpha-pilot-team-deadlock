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
LABEL_DIR = os.path.join(PROJECT_DIR, 'label')
TEMPLATE_DIR = os.path.join(PROJECT_DIR, 'template')
DATA_DIR = os.path.join(PROJECT_DIR, 'data')

class FastSiftLabeller(object):

  def __init__(self, template_path=os.path.join(TEMPLATE_DIR, 'template.jpg'), label_dir=LABEL_DIR):
    self.label_dir = label_dir
    self.template_img = cv2.imread(template_path, 0)
    self.feature_extractor = cv2.xfeatures2d.SIFT_create(nfeatures=1000)

    # find the keypoints and descriptors with SIFT
    self.kp_template, self.des_template = self.feature_extractor.detectAndCompute(self.template_img, None)
    self.matcher = cv2.FlannBasedMatcher(INDEX_PARAMS, SEARCH_PARAMS)

  def generate_labels(self, mode='train', visualize=False):
    auto_generated_labels = {}
    unmatched_files = []
    num_labeled = 0
    for img_path in glob.glob(os.path.join(DATA_DIR, mode, '*')):
      print('working on image: %s' % img_path)
      img_to_match = cv2.imread(os.path.join(PROJECT_DIR, img_path), 0)
      kp_to_match, des_to_match = self.feature_extractor.detectAndCompute(img_to_match, None)

      transform, matches_mask, good_matches, succeeded = self.match_features(kp_to_match, des_to_match)

      if succeeded:
        try:
          h, w = self.template_img.shape
          pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
          dst = cv2.perspectiveTransform(pts, transform)
          area = cv2.contourArea(dst)

          # validate matched area by heuristics. too large and too small areas are filtered out
          if area < 1000 or area > 1800000:
            succeeded = False

          if succeeded:
            num_labeled += 1
            rect = np.array(dst)[:, 0]
            x_min = max(int(np.min(rect[:, 0])), 0)
            x_max = int(np.max(rect[:, 0]))
            y_min = max(int(np.min(rect[:, 1])), 0)
            y_max = int(np.max(rect[:, 1]))
            img_w = x_max - x_min
            img_h = y_max - y_min
            print('number of labeled images: %d' % num_labeled)

            bbox = {'x_min': x_min, 'y_min': y_min, 'x_max': x_max, 'y_max': y_max}
            auto_generated_labels[os.path.basename(img_path)] = bbox
          else:
            unmatched_files.append(os.path.basename(img_path))

          if succeeded and visualize:
            img_to_draw = cv2.polylines(img_to_match, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
            draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                               singlePointColor=None,
                               matchesMask=matches_mask,  # draw only inliers
                               flags=2)

            # img_to_draw = cv2.drawMatches(self.template_img, self.kp_template, img_to_draw, kp_to_match, good_matches, None, **draw_params)

            cv2.rectangle(img_to_draw, (x_min, y_min), (x_min + img_w, y_min + img_h), (0, 255, 0), 5)
            plt.imshow(img_to_draw, 'gray'), plt.show()
        except Exception as e:
          unmatched_files.append(os.path.basename(img_path))
          print('hitting error on image %s due to %s' % (img_path, e))

    with open(os.path.join(LABEL_DIR, 'sift_auto_generated_label.json'), 'w') as outfile:
      json.dump(auto_generated_labels, outfile)

    with open(os.path.join(LABEL_DIR, 'sift_patches.json'), 'w') as outfile:
      json.dump({}, outfile)

    with open(os.path.join(LABEL_DIR, 'sift_unmatched.txt'), 'w') as f:
      for unmatched_file in unmatched_files:
        f.write("%s\n" % unmatched_file)

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
  matcher_obj = FastSiftLabeller()
  matcher_obj.generate_labels(visualize=False)
