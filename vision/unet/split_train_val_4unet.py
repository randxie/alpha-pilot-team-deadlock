"""
BEFORE RUNNING:
All training/validation images must be in alpha-pilot\vision\final_code\training\images

PURPOSE:
Takes the gate_<train/val>.json's in alpha-pilot\vision\label_generation\annotations
Splits from alpha-pilot\vision\final_code\training\images to train and validation sets
Creates symbolic links in ...\images\train and ...\images\val folders
Creates mask .jpgs, binary segmentation

.JPG's kept because original training images are in jpg
This might be the best workaround to use with the U-net model training code found in https://github.com/jakeret/tf_unet
tf_unet modified for Team Deadlock Alpha Pilot use.

"""

import cv2
import json
import os
import numpy as np
from shutil import copy2

cv2.setUseOptimized(True)

IMG_RESIZE = 0.5

FUNC_DIR = os.path.dirname(os.path.realpath(__file__))
IMG_DIR = os.path.join(FUNC_DIR, "..\\final_code\\training\\images")
TRAIN_LABEL = os.path.join(FUNC_DIR, "..\\label_generation\\annotations\\gate_train.json")
VAL_LABEL = os.path.join(FUNC_DIR, "..\\label_generation\\annotations\\\\gate_val.json")

TRAIN_DIR = os.path.join(FUNC_DIR, "train")
VAL_DIR = os.path.join(FUNC_DIR, "val")

try:
  os.mkdir(TRAIN_DIR)
  os.mkdir(VAL_DIR)
except:
  None

with open(TRAIN_LABEL) as trainFile:
  train_dict = json.load(trainFile)

with open(VAL_LABEL) as valFile:
  val_dict = json.load(valFile)

train_ID_list = [a["id"] for a in train_dict["annotations"]]
val_ID_list = [a["id"] for a in val_dict["annotations"]]

for img_info in train_dict["images"]:
  img_ID = img_info["id"]
  img_name = img_info["file_name"]
  coords = np.array(train_dict["annotations"][train_ID_list.index(img_ID)]["segmentation"])
  coords = coords.reshape((4, 2))

  img_path = os.path.join(IMG_DIR, img_name)

  og_img = cv2.imread(img_path)
  img_shape = np.shape(og_img)
  img_h = img_shape[0]
  img_w = img_shape[1]

  mask_img = np.zeros((img_h, img_w, 3), np.uint8)
  mask_img = cv2.fillConvexPoly(mask_img, coords, color=(255, 255, 255), lineType=cv2.LINE_AA)

  cv2.imwrite(os.path.join(TRAIN_DIR, img_name.replace('.JPG', '') + '_mask.JPG'), mask_img)
  copy2(img_path, os.path.join(TRAIN_DIR, img_name))

for img_info in val_dict["images"]:
  img_ID = img_info["id"]
  img_name = img_info["file_name"]
  coords = np.array(val_dict["annotations"][val_ID_list.index(img_ID)]["segmentation"])
  coords = coords.reshape((4, 2))

  img_path = os.path.join(IMG_DIR, img_name)

  og_img = cv2.imread(img_path)
  img_shape = np.shape(og_img)
  img_h = img_shape[0]
  img_w = img_shape[1]

  mask_img = np.zeros((img_h, img_w, 3), np.uint8)
  mask_img = cv2.fillConvexPoly(mask_img, coords, color=(255, 255, 255), lineType=cv2.LINE_AA)

  cv2.imwrite(os.path.join(VAL_DIR, img_name.replace('.JPG', '') + '_mask.JPG'), mask_img)
  copy2(img_path, os.path.join(VAL_DIR, img_name))