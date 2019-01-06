"""Only keep images related to traffic light."""
import os
import shutil
from pycocotools.coco import COCO

DATA_DIR = '../data/'
SAVE_DIR = '../images'
INTERESTED_CLASS = 'traffic light'

def get_class_image_ids(data_type, class_name):
  ann_file = os.path.join(DATA_DIR, 'annotations/instances_%s.json' % data_type)
  coco_api = COCO(ann_file)
  category_id = coco_api.getCatIds(catNms=[class_name])
  image_ids = coco_api.getImgIds(catIds=category_id)

  return image_ids

def copy_images(img_ids, src_folder, des_folder):
  for img_id in img_ids:
    img_path = '%012d.jpg' % img_id
    if os.path.exists(os.path.join(src_folder, img_path)):
      shutil.copy(os.path.join(src_folder, img_path), os.path.join(des_folder, img_path))


def build_train_val_set(classname):
  os.makedirs(SAVE_DIR, exist_ok=True)

  for data_type in ['train', 'val']:
    src_folder = os.path.join(DATA_DIR, '%s2017' % data_type)
    des_folder = os.path.join(SAVE_DIR, data_type)
    img_ids = get_class_image_ids('%s2017' % data_type, classname)
    os.makedirs(des_folder, exist_ok=True)
    copy_images(img_ids, src_folder, des_folder)


if __name__ == '__main__':
  build_train_val_set(INTERESTED_CLASS)


