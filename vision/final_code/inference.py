import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from distutils.version import StrictVersion
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt

from PIL import Image
import os
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from object_detection.data_decoders.tf_example_decoder import TfExampleDecoder
from object_detection.core.preprocessor import random_rotate_along_y_axis

sys.path.append("..")
from object_detection.utils import ops as utils_ops
from shapely.geometry import Polygon
import numpy as np

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


class Inference():

  def __init__(self):
    pass

  def load_image(self, path_to_img):
    #     Loads image given tfrecord
    record_iterator = tf.python_io.tf_record_iterator(path=path_to_image)
    decoder = TfExampleDecoder()
    data_string = next(record_iterator)
    tensor_dict = decoder.decode(data_string)

    def decode_string(data_string):
      example = tf.train.Example()
      example.ParseFromString(data_string)
      return example

    example = decode_string(data_string)
    with tf.Session() as sess:
      image = tensor_dict['image'].eval()
      gtruth = tensor_dict['groundtruth_boxes'].eval()

    return (image, gtruth)

  def load_model(self, path_to_model):
    # Loads frozen graph model
    detection_graph = tf.Graph()
    with detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(path_to_model, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    return detection_graph

  def single_img_inference(self, img, graph):
    # returns
    #     1. Output dict containing object detection information
    #     2. First Bounding box predicted
    image = np.expand_dims(img, axis=0)
    with graph.as_default():
      with tf.Session() as sess:
        # Get handles to input and output tensors
        ops = tf.get_default_graph().get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        tensor_dict = {}
        for key in [
          'num_detections', 'detection_boxes', 'detection_scores',
          'detection_classes', 'detection_masks'
        ]:
          tensor_name = key + ':0'
          if tensor_name in all_tensor_names:
            tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
              tensor_name)
        if 'detection_masks' in tensor_dict:
          # The following processing is only for single image
          detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
          detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
          # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
          real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
          detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
          detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
          detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
            detection_masks, detection_boxes, image.shape[0], image.shape[1])
          detection_masks_reframed = tf.cast(
            tf.greater(detection_masks_reframed, 0.5), tf.uint8)
          # Follow the convention by adding back the batch dimension
          tensor_dict['detection_masks'] = tf.expand_dims(
            detection_masks_reframed, 0)
        image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

        # Run inference
        output_dict = sess.run(tensor_dict,
                               feed_dict={image_tensor: np.expand_dims(image, 0)})

        # all outputs are float32 numpy arrays, so convert types as appropriate
        output_dict['num_detections'] = int(output_dict['num_detections'][0])
        output_dict['detection_classes'] = output_dict[
          'detection_classes'][0].astype(np.uint8)
        output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
        output_dict['detection_scores'] = output_dict['detection_scores'][0]
        if 'detection_masks' in output_dict:
          output_dict['detection_masks'] = output_dict['detection_masks'][0]
    return (output_dict, output_dict['detection_boxes'][:1][0])

  def visualize(self, image, output_dict):
    #     Plots image with detected boxes
    vis_util.visualize_boxes_and_labels_on_image_array(
      image_np,
      output_dict['detection_boxes'],
      output_dict['detection_classes'],
      output_dict['detection_scores'],
      category_index,
      instance_masks=output_dict.get('detection_masks'),
      use_normalized_coordinates=True,
      line_thickness=8)
    plt.figure(figsize=IMAGE_SIZE)
    plt.imshow(image_np)

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
