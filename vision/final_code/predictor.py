import cv2
import json
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import tensorflow as tf
import time

from object_detection.utils import ops as utils_ops
from object_detection.utils import visualization_utils as vis_util
from tensorflow.contrib import predictor

from shapely.geometry import Polygon
from utils import util_plotting

matplotlib.use('TKAgg')

CATEGORY_INDEX = {1: {'name': 'gate'}}

# validate this assumption with competition host
ORIGINAL_IMAGE_WIDTH = 1296
ORIGINAL_IMAGE_HEIGHT = 864


class Predictor(object):

  def __init__(self, model_dir, image_dir, batch_size=1, prefetch_buffer_size=4, ground_truth_dict=None,
               flyable_region_detector=None):
    self.model_dir = model_dir
    self.image_dir = image_dir
    self.predict_fn = predictor.from_saved_model(model_dir)
    self.filename_dataset = tf.data.Dataset.list_files(os.path.join(image_dir, '*.JPG'))
    self.batch_size = batch_size
    self.flyable_region_detector = flyable_region_detector
    self.prefetch_buffer_size = prefetch_buffer_size
    self.ground_truth_dict = ground_truth_dict
    self.avg_time = 2  # on average, each image can not exceed 2 sec inference time

  def run_inference(self, visualize=False):
    # iterator to get image array
    image_data = self.filename_dataset.map(
      lambda x: tf.image.decode_jpeg(tf.read_file(x)))
    image_data.batch(self.batch_size)
    image_data.prefetch(self.prefetch_buffer_size)
    image_iterator = image_data.make_one_shot_iterator()

    # iterator to get filenames
    filename_data = self.filename_dataset
    filename_data.batch(self.batch_size)
    filename_data.prefetch(self.prefetch_buffer_size)
    filename_iterator = filename_data.make_one_shot_iterator()

    next_images = image_iterator.get_next()
    next_filenames = filename_iterator.get_next()

    # for getting time and predictions
    self.time_all = []
    self.pred_dict = {}
    with tf.Session() as sess:
      try:
        while True:
          # Get an image tensor and print its value.
          cur_filename = sess.run(next_filenames)
          cur_filename = cur_filename.decode("utf-8").split('/')[-1]

          # generate prediction
          original_image = cv2.imread(os.path.join(self.image_dir, cur_filename))
          original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
          image_array = [original_image]

          tic = time.monotonic()
          output_dict = self.predict_fn({'inputs': image_array})
          output_dict = self._reformat_output_dict(output_dict)

          # create output array
          output_array = np.zeros(9)
          output_array[-1] = output_dict['detection_scores'][0]

          bbox_coordinate = self._create_coordinates(output_dict['detection_boxes'][0], img_width=ORIGINAL_IMAGE_WIDTH,
                                                   img_height=ORIGINAL_IMAGE_HEIGHT)
          if self.flyable_region_detector:
            # TODO: Craig to improve flyable region detector
            # TODO: Akita to test convex hull algorithms
            bbox = [bbox_coordinate[1], bbox_coordinate[0], bbox_coordinate[3], bbox_coordinate[4]]
            coordinates = self.flyable_region_detector.detect(image_array[0], bbox)
          else:
            coordinates = bbox_coordinate

          output_array[0:8] = coordinates

          # store necessary information
          toc = time.monotonic()
          self.pred_dict[cur_filename] = [output_array.tolist()]
          self.time_all.append(toc - tic)

          # visualize original images as well as bounding box
          if visualize:
            # use real coordinates
            original_image = cv2.imread(os.path.join(self.image_dir, cur_filename))
            original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

            if self.ground_truth_dict:
              if cur_filename in self.ground_truth_dict:
                util_plotting.plot_GT_pred(image_array[0], [coordinates], self.ground_truth_dict[cur_filename])
            else:
              util_plotting.plot_bbox(original_image, [coordinates])
            plt.show()
            plt.close()

      except tf.errors.OutOfRangeError:
        pass

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
    if self.batch_size == 1:
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
  model_dir = 'weights/ssd-mobilenet'
  image_dir = 'training/images'
  img_predictor = Predictor(model_dir, image_dir, batch_size=1)
  img_predictor.run_inference(visualize=False)
  img_predictor.output_submission_file(output_filename='submission_all.json')
