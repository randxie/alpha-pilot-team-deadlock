import argparse
import json
from predictor import Predictor
from maskrcnn_predictor import MaskRCNNPredictor
from scorer.scorer import mAPScorer
from sift_flyable_region_detector import SiftFlyableRegionDetector

parser = argparse.ArgumentParser()
parser.add_argument("--data", default='small', type=str, help="full or small")
parser.add_argument("--visualize", action='store_true', default=False, help="whether to visualize output")


def score_pipeline(args):


  if args.data == 'full':
    ground_truth_filename = 'training/training_GT_labels.json'
    submission_filename = 'submission_all.json'
    image_dir = 'training/images'
  else:
    ground_truth_filename = 'training/training_GT_labels_small.json'
    submission_filename = 'submission_small.json'
    image_dir = 'training/small'

  with open(ground_truth_filename, 'r') as f:
    ground_truth_dict = json.load(f)

  # First version solution
  """
  model_dir = 'weights/ssd-mobilenet'
  flyable_region_detector = SiftFlyableRegionDetector()
  predictor = Predictor(model_dir, image_dir, batch_size=1, ground_truth_dict=ground_truth_dict,
                        flyable_region_detector=flyable_region_detector)
  """

  # mask rcnn solution
  model_dir = 'weights/maskrcnn-inception-v2'
  predictor = MaskRCNNPredictor(model_dir, image_dir, batch_size=1, ground_truth_dict=ground_truth_dict)
  predictor.run_inference(visualize=args.visualize)
  predictor.output_submission_file(output_filename=submission_filename)

  with open(submission_filename, 'r') as f:
    submission_dict = json.load(f)

  scorer = mAPScorer()
  coco_score = scorer.COCO_mAP(ground_truth_dict, submission_dict)

  print('COCO mAP for detector is {}'.format(coco_score))
  print('Average time is {}'.format(predictor.avg_time))

  algorithm_score = 35 * (2 * coco_score - predictor.avg_time)
  print('Estimated algorithm score is {}'.format(algorithm_score))


if __name__ == '__main__':
  args = parser.parse_args()
  score_pipeline(args)
