import argparse
import json
from predictor import Predictor
from sift_flyable_region_detector import SiftFlyableRegionDetector

parser = argparse.ArgumentParser()
parser.add_argument("--visualize", action='store_true', default=False, help="whether to visualize output")


def score_pipeline(args):
  model_dir = 'weights/ssd-mobilenet'
  submission_filename = 'submission_final.json'
  image_dir = 'testing/images'

  flyable_region_detector = SiftFlyableRegionDetector()
  predictor = Predictor(model_dir, image_dir, batch_size=1, flyable_region_detector=flyable_region_detector)
  predictor.run_inference(visualize=args.visualize)
  predictor.output_submission_file(output_filename=submission_filename)


if __name__ == '__main__':
  args = parser.parse_args()
  score_pipeline(args)
