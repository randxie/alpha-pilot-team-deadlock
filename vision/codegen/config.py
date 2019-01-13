import os
import tvm

CODEGEN_SRC_PATH = os.path.dirname(os.path.abspath(__file__))

CODEGEN_TARGET = 'llvm' # use cuda for GPU
CODEGEN_HOST = 'llvm'
CODEGEN_LAYOUT = None
CODEGEN_CONTEXT = tvm.cpu(0)

IMAGE_HEIGHT = 640
IMAGE_WIDTH = 640
