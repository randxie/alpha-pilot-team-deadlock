import os
from .common import PROJECT_PATH
from .common import SRC_PATH

MODEL_NAME = 'ssd-resnet50-v1-fpn'

with open(os.path.join(SRC_PATH, "configs/%s/template.config" % MODEL_NAME), 'r') as f:
  config = f.readlines()

with open(os.path.join(SRC_PATH, "configs/%s/train.config" % MODEL_NAME), 'w') as f:
  for line in config:
    if '$PROJECT_PATH' in line:
      line = line.replace('$PROJECT_PATH', PROJECT_PATH)
    f.write(line)