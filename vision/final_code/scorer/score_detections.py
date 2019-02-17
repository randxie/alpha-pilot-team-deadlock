import json
from .scorer import *
import argparse

# usage: python score_detections.py -g {ground truth json} -p {prediction json}

parser = argparse.ArgumentParser()
parser.add_argument("-g","--groundtruthfile", type=str,
                    help="name of groundtruth file")
parser.add_argument("-p","--predictionfile", type=str,
                    help="name of prediction file")

args = parser.parse_args()
mAP_scorer = mAPScorer()

with open(args.groundtruthfile,'r') as f:
    GT_data = json.load(f)
with open(args.predictionfile,'r') as f:
    pred_data = json.load(f)
    
n_GT = mAP_scorer.countBoxes(GT_data)
n_Pred = mAP_scorer.countBoxes(pred_data)

coco_score = mAP_scorer.COCO_mAP(GT_data,pred_data)

print("COCO mAP for detector is {}".format(coco_score))
