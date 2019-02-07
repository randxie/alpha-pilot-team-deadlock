import numpy as np
import cv2
import os
import json
from pathlib import Path
from matplotlib import pyplot as plt


FUNC_DIR = str(Path(__file__))
DATA_DIR = str(Path(FUNC_DIR).resolve().parents[1]) + '\\data'
LABELFILE = str(Path(FUNC_DIR).resolve().parents[2]) + "\\label_generation\\label\\sift_auto_generated_label.json"

with open(LABELFILE) as labelFile:
    bbox_dict = json.load(labelFile)

class FlyRegionDetector(object):
    
    def __init__(self, bbox = bbox_dict['IMG_0005.JPG'], img_path=(DATA_DIR + '\\IMG_0005.JPG')):
        self.bbox = bbox
        self.img_path = img_path
        self.img = cv2.imread(img_path, 0)

    def generate_region(self, visualize=False):
        # bbox = {'x_min': x_min, 'y_min': y_min, 'x_max': x_max, 'y_max': y_max}
        x_min = self.bbox['x_min']
        y_min = self.bbox['y_min']
        x_max = self.bbox['x_max']
        y_max = self.bbox['y_max']
        
        h = y_max - y_min
        w = x_max - x_min
        
        x_cen = (x_max + x_min)/2
        y_cen = (y_max + y_min)/2
             
        bbox_img = self.img[y_min:y_max, x_min:x_max]
        bbox_img_gray = cv2.cvtColor(bbox_img, cv2.COLOR_GRAY2BGR)
        plt.imshow(bbox_img_gray), plt.show()
        
        v = np.median(bbox_img)
        upper = int(max(0, (1.0 - 0.33) * v))
        lower = int(max(0, (1.0 + 0.33) * v))
        
        minLength = min(h/2, w/2)
        maxGap = 5
        
        minCanny = min(h, w)
        maxCanny = minCanny * 1.5
        
        # Canny edge
        img_edge = cv2.Canny(bbox_img_gray, threshold1 = lower, threshold2 = upper)
        plt.imshow(img_edge), plt.show()
        
        #Gaussian Blur
        img_gaus = cv2.GaussianBlur(img_edge, (3,3), 0)
        plt.imshow(img_gaus), plt.show()
        # Hough transform
        lines = cv2.HoughLinesP(img_gaus, 1, np.pi/180, 180, minLineLength = minLength, maxLineGap = maxGap) 
        
        # Reverse affine transformation?
        
        # Transform back to original image
        
        # Visualize
        if visualize == True:
            for line in lines:
                coords = line[0]
                cv2.line(bbox_img_gray, (coords[0],coords[1]), (coords[2],coords[3]), [255,255,0], 3)
        
        plt.imshow(bbox_img_gray), plt.show()        
        return lines
        
if __name__ == '__main__':
    region_obj = FlyRegionDetector()
    region_obj.generate_region(visualize=True)