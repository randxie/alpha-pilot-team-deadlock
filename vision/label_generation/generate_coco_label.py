import os
import cv2
import json
import matplotlib.pyplot as plt
import numpy as np

from PIL import Image

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
LABEL_DIR = os.path.join(PROJECT_DIR, 'label')
TEMPLATE_DIR = os.path.join(PROJECT_DIR, 'template')
DATA_DIR = os.path.join(PROJECT_DIR, 'data')
LABEL_FILENAME = 'training_gt_bbox.json' # was 'sift_auto_generated_label.json'

class AlphaPilotAutoGenLabel(object):
  """Convert annotations to COCO Json format."""

  def __init__(self):
    self.info = {"year": 2019,
                 "version": "1.0",
                 "description": "Auto generated label by SIFT feature matcher",
                 "contributor": "Rand Xie",
                 "url": "https://www.herox.com/alphapilot/77-test-2",
                 "date_created": "2019"
                 }
    self.licenses = []
    self.type = "instances"

    self.categories = [{"id": 0, "name": "gate", "supercategory": "racing"}]
    self.cat2id = {cat["name"]: cat_id + 1 for cat_id, cat in enumerate(self.categories)}
    self.labels = json.load(open(os.path.join(LABEL_DIR, LABEL_FILENAME), 'r'))

    all_img_list = list(self.labels.keys())
    splited_img_list = {'train': all_img_list[0:8500], 'val': all_img_list[8500:]}

    for mode in ["train", "val"]:
      images, annotations = self.get_image_annotation_pairs(splited_img_list[mode])
      json_data = {"info": self.info,
                   "images": images,
                   "licenses": self.licenses,
                   "type": self.type,
                   "annotations": annotations,
                   "categories": self.categories}

      with open(os.path.join(PROJECT_DIR, "annotations", "gate_" + mode + ".json"), "w") as jsonfile:
        json.dump(json_data, jsonfile, sort_keys=True, indent=4)

  def get_image_annotation_pairs(self, img_list, mode='train'):
    images = []
    annotations = []
    for img_id, img_path in enumerate(img_list):
      print('working on: %s' % img_path)
      img = np.array(Image.open(os.path.join(DATA_DIR, mode, img_path)).convert('RGB'))
      label_dict = self.labels[img_path]

      # get bounding box information
      img_height = img.shape[0]
      img_width = img.shape[1]
      bbox_h = min(label_dict['y_max'], img_height) - label_dict['y_min']
      bbox_w = min(label_dict['x_max'], img_width) - label_dict['x_min']
      bbox = [label_dict['x_min'], label_dict['y_min'], bbox_w, bbox_h]

      images.append({"date_captured": "2019",
                     "file_name": img_path,
                     "id": img_id + 1,
                     "license": 1,
                     "url": "",
                     "height": img_height,
                     "width": img_width})

      annotations.append({"area": np.float(bbox_w * bbox_h),
                          "iscrowd": 0,
                          "image_id": img_id + 1,
                          "bbox": bbox,
                          "category_id": self.cat2id['gate'],
                          "id": img_id + 1})
    return images, annotations


if __name__ == "__main__":
  AlphaPilotAutoGenLabel()

  # test
  from PIL import Image
  img_idx = 0
  labels = json.load(open(os.path.join('annotations', 'gate_train.json'), 'r'))

  img_path = labels['images'][img_idx]['file_name']
  img_to_draw = cv2.imread(os.path.join('data', 'train', img_path))
  [x_min, y_min, img_w, img_h] = labels['annotations'][img_idx]['bbox']
  cv2.rectangle(img_to_draw, (x_min, y_min), (x_min + img_w, y_min + img_h), (0, 255, 0), 5)
  plt.imshow(img_to_draw)
  plt.show()

