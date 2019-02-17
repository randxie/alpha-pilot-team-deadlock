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

matplotlib.use('TKAgg')

CATEGORY_INDEX = {1: {'name': 'gate'}}


class Predictor(object):

  def __init__(self, model_dir, image_dir, batch_size=1):
    self.predict_fn = predictor.from_saved_model(model_dir)
    self.filename_dataset = tf.data.Dataset.list_files(os.path.join(image_dir, '*.JPG'))
    self.batch_size = batch_size
    self.flyable_region_detector = None

  def run_inference(self, visualize=False):
    # iterator to get image array
    image_data = self.filename_dataset.map(
      lambda x: tf.image.decode_jpeg(tf.read_file(x)))
    image_data.batch(self.batch_size)
    image_iterator = image_data.make_one_shot_iterator()

    # iterator to get filenames
    filename_data = self.filename_dataset
    filename_data.batch(self.batch_size)
    filename_iterator = filename_data.make_one_shot_iterator()

    next_images = image_iterator.get_next()
    next_filenames = filename_iterator.get_next()

    # for getting time and predictions
    self.time_all = []
    self.pred_dict = {}
    with tf.Session() as sess:
      try:
        while True:
          tic = time.monotonic()
          # Get an image tensor and print its value.
          image_array = sess.run([next_images])
          cur_filename = sess.run(next_filenames)
          cur_filename = cur_filename.decode("utf-8").split('/')[-1]

          # generate prediction
          output_dict = self.predict_fn({'inputs': image_array})
          output_dict = self._reformat_output_dict(output_dict)

          # create output array
          output_array = np.zeros(9)
          output_array[-1] = output_dict['detection_scores'][0]

          img_height, img_width, _ = image_array[0].shape
          if self.flyable_region_detector:
            coordinates = self.flyable_region_detector.detect(image_array[0], output_dict)
          else:
            coordinates = self._create_coordinates(output_dict['detection_boxes'][0], img_width, img_height)

          output_array[0:8] = coordinates

          # store necessary information
          toc = time.monotonic()
          self.pred_dict[cur_filename] = output_array.tolist()
          self.time_all.append(toc - tic)

          # visualize if needed
          if visualize:
            self._visualize_image_with_bbox(image_array[0], output_dict, title=cur_filename)
            plt.close()

      except tf.errors.OutOfRangeError:
        pass

  def _create_coordinates(self, bbox, img_width, img_height):
    """Create required output array.

    Assume only one gate in the image, so the highest probability bounding box is taken.

    :param bbox: Normalized bounding box
    :param prob: Bounding box probably
    :return:
    """
    x1, y1, w, h = bbox
    x1 = x1 * img_width
    x2 = (x1 + w) * img_width
    y1 = y1 * img_height
    y2 = (y1 + h) * img_height

    return np.array([x1, y1, x1, y2, x2, y2, x1, y2])

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

  def output_submission_file(self):
    mean_time = np.mean(self.time_all)
    ci_time = 1.96 * np.std(self.time_all)
    print('Time stats -- Mean: %f, Std: %f' % (mean_time, ci_time))
    with open('first_submission.json', 'w') as f:
      json.dump(self.pred_dict, f)

  """
  def create_poly(self, box):
    x1, y1, x2, y2, x3, y3, x4, y4 = box
    return [(x1, y1), (x2, y2), (x3, y3), (x4, y4), (x1, y1)]

  def calculate_iou(self, polygon1, polygon2):
    polygon1 = Polygon(polygon1)
    polygon2 = Polygon(polygon2)
    polygon1 = polygon1.buffer(0)
    polygon2 = polygon2.buffer(0)
    intersection = polygon1.intersection(polygon2)
    union = polygon1.area + polygon2.area - intersection.area
    return (intersection.area / union)

  def iou_calc(self, bbox, gtruth):
    #     Returns iou value
    [ymin, xmin, ymax, xmax] = bbox
    ymin = image.shape[0] * ymin
    ymax = image.shape[0] * ymax
    xmin = image.shape[1] * xmin
    xmax = image.shape[1] * xmax

    [ymin1, xmin1, ymax1, xmax1] = gtruth
    ymin1 = image.shape[0] * ymin1
    ymax1 = image.shape[0] * ymax1
    xmin1 = image.shape[1] * xmin1
    xmax1 = image.shape[1] * xmax1

    det_box = [xmin, ymin, xmax, ymin, xmax, ymax, xmin, ymax]
    truth_box = [xmin1, ymin1, xmax1, ymin1, xmax1, ymax1, xmin1, ymax1]
    iou = self.calculate_iou(self.create_poly(det_box), self.create_poly(truth_box))
    return (iou)
  """


if __name__ == '__main__':
  model_dir = 'weights/ssd-mobilenet'
  image_dir = 'training/images'
  img_predictor = Predictor(model_dir, image_dir, batch_size=1)
  img_predictor.run_inference(visualize=False)
  img_predictor.output_submission_file()
