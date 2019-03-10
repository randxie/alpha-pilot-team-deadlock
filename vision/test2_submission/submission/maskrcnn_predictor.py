import cv2
import numpy as np
import os
import scipy
import tensorflow as tf
from tensorflow.contrib import predictor


CATEGORY_INDEX = {1: {'name': 'gate'}}

# validate this assumption with competition host
ORIGINAL_IMAGE_WIDTH = 1296
ORIGINAL_IMAGE_HEIGHT = 864

cv2.setUseOptimized(True)

class MaskRCNNPredictor(object):

  def __init__(self, model_dir):
    self.model_dir = model_dir
    self.predict_fn = predictor.from_saved_model(model_dir, config=tf.ConfigProto(log_device_placement=True))

  def predict(self, img):
    original_image = img
    image_array = [original_image]
    output_dict = self.predict_fn({'inputs': image_array})
    output_dict = self._reformat_output_dict(output_dict)

    # create output array
    output_array = np.zeros(9)
    output_array[-1] = output_dict['detection_scores'][0]

    if output_array[-1] > 0.2:
      bbox = output_dict['detection_boxes'][0]
      mask = output_dict['detection_masks'][0, 0, :, :]

      coordinates = self._create_coordinates_from_mask(bbox, mask, img_width=ORIGINAL_IMAGE_WIDTH,
                                                       img_height=ORIGINAL_IMAGE_HEIGHT, image=image_array[0])
      
      output_array[0:8] = coordinates
      output_array = [output_array.tolist()]
      return output_array
    else:
      output_array = []
      coordinates = []
      return [output_array]

  def _create_coordinates_from_mask(self, bbox, mask, img_width=ORIGINAL_IMAGE_WIDTH, img_height=ORIGINAL_IMAGE_HEIGHT,
                                    image=None):
    ymin = int(bbox[0] * img_height)
    xmin = int(bbox[1] * img_width)
    ymax = int(bbox[2] * img_height)
    xmax = int(bbox[3] * img_width)
    w = int(xmax - xmin)
    h = int(ymax - ymin)

    # resize mask back to the bbox size and use bbox coordinate as reference
    mask = scipy.misc.imresize((mask > 0.2).astype(np.uint8), (h, w), interp='nearest')

    # convert mask to threshold and find contour
    ret, thresh = cv2.threshold(mask, 0.5, 1, 0)

    contours, _ = cv2.findContours(thresh.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # generate approximate polygon, this should be rectangle most of the time
    epsilon = 0.025 * cv2.arcLength(contours[0], True)
    hull = cv2.convexHull(contours[0])
    approx = cv2.approxPolyDP(hull, epsilon, True)

    if approx.size != 8:
      # if approxPolyDP does not generate a rectangle, fit a simple rectangle
      rect = cv2.minAreaRect(contours[0])
      approx = cv2.boxPoints(rect)
      approx[:, 0] = approx[:, 0] + xmin
      approx[:, 1] = approx[:, 1] + ymin

    else:
      approx[:, 0, 0] = approx[:, 0, 0] + xmin
      approx[:, 0, 1] = approx[:, 0, 1] + ymin

      new_mask = np.zeros((img_height, img_width))
      new_mask[ymin:(ymin + h), xmin:(xmin + w)] = mask

    return approx.flatten()

  def _create_coordinates(self, bbox, img_width=ORIGINAL_IMAGE_WIDTH, img_height=ORIGINAL_IMAGE_HEIGHT):
    """Create required output array.

    Assume only one gate in the image, so the highest probability bounding box is taken.

    :param bbox: Normalized bounding box
    :return:
    """
    ymin = bbox[0] * img_height
    xmin = bbox[1] * img_width
    ymax = bbox[2] * img_height
    xmax = bbox[3] * img_width
    return np.array([xmin, ymin, xmin, ymax, xmax, ymax, xmax, ymin]).astype(np.int)

  def _reformat_output_dict(self, output_dict):
    """Extract first element out and convert to desired data type.

    :param output_dict: A dict with detection outputs
    :return: Reformatted dict
    """
    output_dict['num_detections'] = int(output_dict['num_detections'][0])
    output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
    output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
    output_dict['detection_scores'] = output_dict['detection_scores'][0]

    return output_dict
