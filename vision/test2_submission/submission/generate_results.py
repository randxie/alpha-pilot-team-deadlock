# This script is to be filled by the team members. 
# Import necessary libraries
# Load libraries
import json
import cv2
import numpy as np
from maskrcnn_predictor import MaskRCNNPredictor

# Implement a function that takes an image as an input, performs any preprocessing steps and outputs a list of bounding box detections and assosciated confidence score. 

class GenerateFinalDetections():
    def __init__(self):
        self.seed = 2018
        
    def predict(self,img):
        model_dir = 'weights/maskrcnn-inception-v2-105x105-conv4' # 'weights/maskrcnn-inception-v2'
        predictor = MaskRCNNPredictor(model_dir, img)
        output_array = predictor.run_inference()
        return output_array