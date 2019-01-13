import nnvm
import nnvm.testing.tf
import nnvm.compiler
import numpy as np
import os
import tensorflow as tf
import tvm

from config import IMAGE_HEIGHT
from config import IMAGE_WIDTH
from config import CODEGEN_TARGET
from config import CODEGEN_HOST
from config import CODEGEN_LAYOUT
from config import CODEGEN_CONTEXT
from PIL import Image
from tvm.contrib import graph_runtime


def _load_model(model_path):
  graph = tf.Graph()
  graph_def = tf.GraphDef()

  with tf.gfile.FastGFile(model_path, 'rb') as f:
    graph_def.ParseFromString(f.read())

  with graph.as_default():
    tf.import_graph_def(graph_def, name='')

  # Call the utility to import the graph definition into default graph.
  graph_def = nnvm.testing.tf.ProcessGraphDefParam(graph_def)

  return graph_def

def _load_image(image_path):
  cur_image = Image.open(image_path).resize((IMAGE_HEIGHT, IMAGE_WIDTH))
  return np.array(cur_image)


def _load_graph_to_nnvm(graph_def, layout):
  sym, params = nnvm.frontend.from_tensorflow(graph_def)
  return sym, params


def compile_tf_graph(sym, params):
  shape_dict = {'DecodeJpeg/contents': (IMAGE_HEIGHT, IMAGE_WIDTH)}
  dtype_dict = {'DecodeJpeg/contents': 'uint8'}
  graph, lib, compile_params = nnvm.compiler.build(sym, shape=shape_dict, target=CODEGEN_TARGET,
                                                   target_host=CODEGEN_HOST,
                                                   dtype=dtype_dict, params=params)
  return graph, lib, compile_params


def execute_graph(graph, lib, params):
  dtype = 'uint8'
  m = graph_runtime.create(graph, lib, CODEGEN_CONTEXT)
  # set inputs
  m.set_input('DecodeJpeg/contents', tvm.nd.array(x.astype(dtype)))
  m.set_input(**params)
  # execute
  m.run()
  # get outputs
  tvm_output = m.get_output(0, tvm.nd.empty(((1, 2)), 'float32'))

if __name__ == '__main__':
  from config import CODEGEN_SRC_PATH

  graph_def = _load_model(
    os.path.join(CODEGEN_SRC_PATH,
                 '../gate_detection/pretrained/ssd_mobilenet_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03/frozen_inference_graph.pb'))
  _load_graph_to_nnvm(graph_def, CODEGEN_LAYOUT)
