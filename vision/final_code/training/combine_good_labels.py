import glob
import json

final_dict = {}

for file in glob.glob('good_labels/*.json'):
  tmp_dict = json.load(open(file, 'r'))
  final_dict.update(tmp_dict)

with open('training_GT_labels_updated', 'w') as f:
  json.dump(final_dict, f)