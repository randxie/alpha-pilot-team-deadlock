import json
from scorer.scorer import mAPScorer

GROUND_TRUTH_FILE = 'training/training_GT_labels.json'
SUBMISSION_FILE = 'first_submission.json'

with open(GROUND_TRUTH_FILE, 'r') as f:
  GT_data = json.load(f)

with open(SUBMISSION_FILE, 'r') as f:
  pred_data = json.load(f)

scorer = mAPScorer()
n_GT = scorer.countBoxes(GT_data)
n_Pred = scorer.countBoxes(pred_data)

coco_score = scorer.COCO_mAP(GT_data, pred_data)

print("COCO mAP for detector is {}".format(coco_score))
