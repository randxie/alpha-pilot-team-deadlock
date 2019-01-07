import os
from pathlib import Path
SRC_PATH = os.path.dirname(os.path.abspath(__file__))
PROJECT_PATH = str(Path(SRC_PATH).parent)

DATA_DIR = os.path.join(SRC_PATH, '../data/')
IMG_DIR = os.path.join(SRC_PATH, '../images')
RESULT_DIR = os.path.join(SRC_PATH, '../results')