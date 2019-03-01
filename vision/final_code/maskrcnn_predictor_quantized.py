import cv2
import glob
import json
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import scipy
import tensorflow as tf
import time

try:
  from object_detection.utils import ops as utils_ops
  from object_detection.utils import visualization_utils as vis_util
  objd_imp = True
except:
  objd_imp = False
from multiprocessing  import Queue
from threading import Thread
from tensorflow.contrib import predictor

from shapely.geometry import Polygon
from tqdm import tqdm
import util_plotting

matplotlib.use('TKAgg')

CATEGORY_INDEX = {1: {'name': 'gate'}}

# validate this assumption with competition host
ORIGINAL_IMAGE_WIDTH = 1296
ORIGINAL_IMAGE_HEIGHT = 864


class MaskRCNNPredictor(object):

  def __init__(self, model_dir, image_dir, batch_size=1, prefetch_buffer_size=4, ground_truth_dict=None,
               flyable_region_detector=None):
    self.model_dir = model_dir
    self.image_dir = image_dir
    self.predict_fn = self.quantize_model(model_dir)
    if ground_truth_dict is None:
      self.all_filenames = glob.glob(os.path.join(image_dir, '*.JPG'))
    else:
      self.all_filenames = list(ground_truth_dict.keys())
    self.batch_size = batch_size
    self.prefetch_buffer_size = prefetch_buffer_size
    self.ground_truth_dict = ground_truth_dict
    self.avg_time = 2  # on average, each image can not exceed 2 sec inference time
    
  def quantize_model(self, model_dir):
    converter = tf.lite.TFLiteConverter.from_saved_model(model_dir, config=tf.ConfigProto(log_device_placement=True))
    converter.post_training_quantize = True
    tflite_model=converter.convert()
    return tflite_model

  def predict(self, image_array):
    tic = time.monotonic()
    output_dict = self.predict_fn({'inputs': image_array})
    output_dict = self._reformat_output_dict(output_dict)

    # create output array
    output_array = np.zeros(9)
    output_array[-1] = output_dict['detection_scores'][0]

    if output_array[-1] > 0.3:
      bbox = output_dict['detection_boxes'][0]
      mask = output_dict['detection_masks'][0, 0, :, :]
      coordinates = self._create_coordinates_from_mask(bbox, mask, img_width=ORIGINAL_IMAGE_WIDTH,
                                                       img_height=ORIGINAL_IMAGE_HEIGHT, image=image_array[0])
      output_array[0:8] = coordinates
      output_array = [output_array.tolist()]
    else:
      output_array = []
      coordinates = []

    # store necessary information
    toc = time.monotonic()
    self.time_all.append(toc - tic)

    return output_array, coordinates

  def run_inference(self, visualize=False):
    # for getting time and predictions
    self.time_all = []
    self.pred_dict = {}

    # Get an image tensor and print its value.
    for i in tqdm(range(len(self.all_filenames))):
      cur_filename = self.all_filenames[i]
      cur_filename = os.path.basename(cur_filename)

      # generate prediction
      original_image = cv2.imread(os.path.join(self.image_dir, cur_filename))
      original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
      image_array = [original_image]
      output_array, coordinates = self.predict(image_array)
      self.pred_dict[cur_filename] = output_array

      # visualize original images as well as bounding box
      if visualize:
        if self.ground_truth_dict:
          if cur_filename in self.ground_truth_dict:
            util_plotting.plot_GT_pred(image_array[0], [coordinates], self.ground_truth_dict[cur_filename])
        else:
          util_plotting.plot_bbox(original_image, [coordinates])
        plt.show()
        plt.close()

  def _create_coordinates_from_mask(self, bbox, mask, img_width=ORIGINAL_IMAGE_WIDTH, img_height=ORIGINAL_IMAGE_HEIGHT,
                                    image=None):
    ymin = int(bbox[0] * img_height)
    xmin = int(bbox[1] * img_width)
    ymax = int(bbox[2] * img_height)
    xmax = int(bbox[3] * img_width)
    w = int(xmax - xmin)
    h = int(ymax - ymin)

    # resize mask back to the bbox size and use bbox coordinate as reference
    mask = scipy.misc.imresize(mask, (h, w), interp='bilinear')

    # convert mask to threshold and find contour
    ret, thresh = cv2.threshold(mask, 0.5, 1, 0)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # generate approximate polygon, this should be rectangle most of the time
    epsilon = 0.05 * cv2.arcLength(contours[0], True)
    approx = cv2.approxPolyDP(contours[0], epsilon, True)
    if approx.size != 8:
      # if approxPolyDP does not generate a rectangle, fit a simple rectangle
      rect = cv2.minAreaRect(contours[0])
      approx = cv2.boxPoints(rect)
      approx[:, 0] = approx[:, 0] + xmin
      approx[:, 1] = approx[:, 1] + ymin
    else:
      approx[:, 0, 0] = approx[:, 0, 0] + xmin
      approx[:, 0, 1] = approx[:, 0, 1] + ymin

    # DO NOT REMOVE, FOR DEBUGGING
    """
    new_mask = np.zeros((img_height, img_width))
    new_mask[ymin:(ymin + h), xmin:(xmin + w)] = mask

    plt.imshow(image)
    plt.imshow(new_mask, alpha=0.8)
    plt.show()
    plt.close()
    """
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

  def _visualize_image_with_bbox(self, image_array, output_dict, title=None):
    """Plot image with bounding box

    :param image_array: A numpy array
    :param output_dict: A dict with reformatted detection outputs
    :param title: Filename
    :return: None
    """
    if self.batch_size == 1 and objd_imp:
      # currently only handle cases when batch size is equal to 1
      vis_util.visualize_boxes_and_labels_on_image_array(
        image_array,
        output_dict['detection_boxes'],
        output_dict['detection_classes'],
        output_dict['detection_scores'],
        CATEGORY_INDEX,
        use_normalized_coordinates=True,
        line_thickness=8)
      plt.figure()
      plt.imshow(image_array)
      if title:
        plt.title(title)
      plt.show()

  def output_submission_file(self, output_filename='final_submission.json'):
    avg_time = np.mean(self.time_all)
    ci_time = 1.96 * np.std(self.time_all)
    print('Time stats -- Mean: %f, Std: %f' % (avg_time, ci_time))
    with open(output_filename, 'w') as f:
      json.dump(self.pred_dict, f)
    self.avg_time = avg_time


if __name__ == '__main__':
  model_dir = 'weights/maskrcnn-inception-v2'
  image_dir = 'training/small'
  img_predictor = MaskRCNNPredictor(model_dir, image_dir, batch_size=1)
  img_predictor.run_inference(visualize=True)
  img_predictor.output_submission_file(output_filename='submission_maskrcnn_all.json')
