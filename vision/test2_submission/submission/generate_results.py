# This script is to be filled by the team members. 
# Import necessary libraries
# Load libraries
import json
import cv2
import numpy as np
from maskrcnn_predictor import MaskRCNNPredictor

# Implement a function that takes an image as an input, performs any preprocessing steps and outputs a list of bounding box detections and assosciated confidence score. 

class GenerateFinalDetections():
    def __init__(self, model_dir = 'weights/maskrcnn-inception-v2-105x105-conv4'):
        self.predictor = MaskRCNNPredictor(model_dir)
        
    def predict(self,img):
         # 'weights/maskrcnn-inception-v2'
        output_array = self.predictor.predict(img)
        return output_array