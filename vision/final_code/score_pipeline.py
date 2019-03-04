import argparse
import json
from numpy import loadtxt
from predictor import Predictor
from scorer.scorer import mAPScorer
from sift_flyable_region_detector import SiftFlyableRegionDetector

from pathlib import Path
FUNC_DIR = str(Path(__file__))
DEFAULT_SAVEFAILDIR = str(Path(FUNC_DIR).resolve().parents[0]) + '\\failed_images'


parser = argparse.ArgumentParser()
parser.add_argument("--data", default='small', type=str, help="full or small")
parser.add_argument("--visualize", action='store_true', default=False, help="whether to visualize output")
parser.add_argument("--quantized", action='store_true', default=False, help="quantized version or not")
parser.add_argument("--flyable_region_detector", default='mask', type=str, help="get flyable region by mask or not")
parser.add_argument("--iou_threshold", default=.80, type=float, help="threshold by which image passes or fails")
parser.add_argument("--save_fail_dir", default=DEFAULT_SAVEFAILDIR, type=str, help="directory to save failed images")

def score_pipeline(args):
  if args.quantized:
    from maskrcnn_predictor_quantized import MaskRCNNPredictor
  else:
    from maskrcnn_predictor import MaskRCNNPredictor

  if args.data == 'full' or args.data == 'val':
    ground_truth_filename = 'training/training_GT_labels.json'
    submission_filename = 'submission_all.json'
    image_dir = 'training/images'
  elif args.data == 'test':
    ground_truth_filename = None
    submission_filename = None
    image_dir = 'testing/images'
  else:
    ground_truth_filename = 'training/training_GT_labels_small.json'
    submission_filename = 'submission_small.json'
    image_dir = 'training/small'

  if ground_truth_filename:
    with open(ground_truth_filename, 'r') as f:
      ground_truth_dict = json.load(f)
  else:
    ground_truth_dict = None

  if args.data == 'val':
    with open("training/val_images.txt", 'r') as f:
      val_img_filenames = [line.strip('\n') for line in f.readlines()]

    new_ground_truth_dict = {}
    for img in val_img_filenames:
      new_ground_truth_dict[img] = ground_truth_dict[img]

    ground_truth_dict = new_ground_truth_dict

  # First version solution
  """
  model_dir = 'weights/ssd-mobilenet'
  flyable_region_detector = SiftFlyableRegionDetector()
  predictor = Predictor(model_dir, image_dir, batch_size=1, ground_truth_dict=ground_truth_dict,
                        flyable_region_detector=flyable_region_detector)
  """

  # mask rcnn solution (or hough from predicted bbox)
  model_dir = 'weights/maskrcnn-inception-v2'
  predictor = MaskRCNNPredictor(model_dir, image_dir, batch_size=1,
                                ground_truth_dict=ground_truth_dict,
                                flyable_region_detector=args.flyable_region_detector)
  predictor.run_inference(visualize=args.visualize)

  if submission_filename:
    predictor.output_submission_file(output_filename=submission_filename)

    with open(submission_filename, 'r') as f:
      submission_dict = json.load(f)

    scorer = mAPScorer()
    coco_score = scorer.COCO_mAP(ground_truth_dict, submission_dict)

    print('COCO mAP for detector is {}'.format(coco_score))
    print('Average time is {}'.format(predictor.avg_time))

    algorithm_score = 35 * (2 * coco_score - predictor.avg_time)
    print('Estimated algorithm score is {}'.format(algorithm_score))

    scorer.check_iou(image_dir, ground_truth_dict, submission_dict, predictor.output_dict,
                     args.iou_threshold, args.save_fail_dir)


if __name__ == '__main__':
  args = parser.parse_args()
  score_pipeline(args)
