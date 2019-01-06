import os
from common import DATA_DIR
from common import IMG_DIR
from object_detection.dataset_tools.create_coco_tf_record import _create_tf_record_from_coco_annotations


def convert_to_tfrecord():
  for data_type in ['train', 'val']:
    ann_file = os.path.join(DATA_DIR, 'annotations/instances_%s2017.json' % data_type)
    image_dir = os.path.join(IMG_DIR, data_type)
    output_dir = os.path.join(IMG_DIR, data_type)
    num_shards = 10 if data_type == 'train' else 1
    _create_tf_record_from_coco_annotations(ann_file, image_dir, output_dir, False, num_shards)


if __name__ == '__main__':
  convert_to_tfrecord()
