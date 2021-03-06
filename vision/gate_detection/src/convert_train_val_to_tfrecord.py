import os
from common import DATA_DIR
from common import IMG_DIR

# NOTE: be careful of the hard-coded string here
from object_detection.dataset_tools.create_coco_tf_record import _create_tf_record_from_coco_annotations


def convert_to_tfrecord():
  for data_type in ['train', 'val']:
    ann_file = os.path.join(DATA_DIR, 'annotations/instances_%s2017.json' % data_type)
    image_dir = os.path.join(IMG_DIR, data_type)
    output_dir = os.path.join(IMG_DIR, data_type)
    num_shards = 10 if data_type == 'train' else 1
    _create_tf_record_from_coco_annotations(ann_file, image_dir, output_dir, False, num_shards)

def convert_gate_data_to_tfrecord(include_mask=False):
  for data_type in ['train', 'val']:
    print('converting %s data' % data_type)
    ann_file = os.path.join(DATA_DIR, 'gate_%s.json' % data_type)
    image_dir = os.path.join(DATA_DIR, 'train')
    output_dir = os.path.join(IMG_DIR, data_type)
    num_shards = 20 if data_type == 'train' else 8
    _create_tf_record_from_coco_annotations(ann_file, image_dir, output_dir, include_mask, num_shards)

if __name__ == '__main__':
  convert_gate_data_to_tfrecord(include_mask=True)
