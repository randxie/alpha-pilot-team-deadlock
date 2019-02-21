# This script is to be filled by the team members. 
# Import necessary libraries
# Load libraries
import json
import cv2
import numpy as np

# Implement a function that takes an image as an input, performs any preprocessing steps and outputs a list of bounding box detections and assosciated confidence score. 


class GenerateFinalDetections():
    def __init__(self):
        self.seed = 2018
        
    def predict(self,img):
        np.random.seed(self.seed)
        n_boxes = np.random.randint(4)
        if n_boxes>0:
            bb_all = 400*np.random.uniform(size = (n_boxes,9))
            bb_all[:,-1] = 0.5
        else:
            bb_all = []
        return bb_all.tolist()
        
