import os
from .common import PROJECT_PATH
from .common import SRC_PATH

with open(os.path.join(SRC_PATH, "configs/ssd-mobilenet-v1-fpn/template.config"), 'r') as f:
  config = f.readlines()

with open(os.path.join(SRC_PATH, "configs/ssd-mobilenet-v1-fpn/train.config"), 'w') as f:
  for line in config:
    if '$PROJECT_PATH' in line:
      line = line.replace('$PROJECT_PATH', PROJECT_PATH)
    f.write(line)