"""Only keep images related to traffic light."""
import os
import shutil
from common import DATA_DIR
from common import IMG_DIR
from pycocotools.coco import COCO

INTERESTED_CLASS = 'traffic light'


def get_class_image_filenames(data_type, class_name):
  ann_file = os.path.join(DATA_DIR, 'annotations/instances_%s.json' % data_type)
  coco_api = COCO(ann_file)
  category_id = coco_api.getCatIds(catNms=[class_name])
  image_ids = coco_api.getImgIds(catIds=category_id)
  all_images = coco_api.loadImgs(image_ids)
  image_filenames = [image['file_name'] for image in all_images]

  return image_filenames


def copy_images(img_filenames, src_folder, des_folder):
  for img_filename in img_filenames:
    img_path = img_filename
    if os.path.exists(os.path.join(src_folder, img_path)):
      shutil.copy(os.path.join(src_folder, img_path), os.path.join(des_folder, img_path))


def build_train_val_set(class_name):
  os.makedirs(IMG_DIR, exist_ok=True)

  for data_type in ['train', 'val']:
    src_folder = os.path.join(DATA_DIR, '%s2017' % data_type)
    des_folder = os.path.join(IMG_DIR, data_type)
    img_filenames = get_class_image_filenames('%s2017' % data_type, class_name)
    os.makedirs(des_folder, exist_ok=True)
    copy_images(img_filenames, src_folder, des_folder)


if __name__ == '__main__':
  build_train_val_set(INTERESTED_CLASS)
