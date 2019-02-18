import numpy as np
import cv2
import os
import json
#import scipy.cluster.hierarchy as hcluster
from pathlib import Path
from matplotlib import pyplot as plt


FUNC_DIR = str(Path(__file__))
DATA_DIR = str(Path(FUNC_DIR).resolve().parents[1]) + '\\gate_detection\\data'
LABELFILE = str(Path(FUNC_DIR).resolve().parents[1]) + "\\label_generation\\label\\sift_auto_generated_label.json"

with open(LABELFILE) as labelFile:
    bbox_dict = json.load(labelFile)

class flyable_region_detector(object):
    
    def __init__(self, bbox = bbox_dict['IMG_0045.JPG'], img_path=(DATA_DIR + '\\IMG_0045.JPG')):
        self.bbox = bbox
        self.img_path = img_path
        self.img = cv2.imread(img_path, 0)
        
    def closest_pt(self, pt, pt_list):
        pt = np.array(pt)
        pt_list = np.asarray(pt_list)
        dist_2 = np.sum((pt_list - pt)**2, axis=1)
        return pt_list[np.argmin(dist_2), :]

    def detect(self, cannySigma = 0.33, gausKernelSize = (3,3), gausSigmaX = 0, gausSigmaY = 0, gausBorderType = 0, visualize=False):
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
        bbox_img_gray_inv = ~bbox_img_gray #invert image
        
        # Should homomorphic filtering be applied?
        if visualize == True:
            plt.imshow(bbox_img_gray), plt.show()
        
        v = np.median(bbox_img)
        upper = int(max(0, (1.0 - cannySigma) * v))
        lower = int(min(255, (1.0 + cannySigma) * v))
        
        minLength = min(h*0.5, w*0.5)
        maxGap = 5 # Make this an adaptable parameter?
        
        minCanny = min(h, w)
        maxCanny = minCanny * 1.5
        
        # Canny edge
        img_edge = cv2.Canny(bbox_img_gray_inv, threshold1 = lower, threshold2 = upper)
        if visualize == True:
            plt.imshow(img_edge), plt.show()
        
        # Gaussian Blur
        img_gaus = cv2.GaussianBlur(img_edge, ksize = gausKernelSize, sigmaX = gausSigmaX, sigmaY = gausSigmaY, borderType = gausBorderType)
        if visualize == True:
            plt.imshow(img_gaus), plt.show()
        
        # Dilate
        img_dil = cv2.dilate(img_gaus, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))
        if visualize == True:
            plt.imshow(img_dil), plt.show()
        
        # Hough transform
        lines = cv2.HoughLinesP(img_dil, 1, np.pi/180, 180, minLineLength = minLength, maxLineGap = maxGap)
        if visualize == True:
            bbox_img_gray_tmp = bbox_img_gray.copy()
            for line in lines:
                coords = line[0]
                cv2.circle(bbox_img_gray_tmp, (coords[0],coords[1]), 5, (255,0,0),-1)
                cv2.circle(bbox_img_gray_tmp, (coords[2],coords[3]), 5, (255,0,0),-1)
                cv2.line(bbox_img_gray_tmp, (coords[0],coords[1]), (coords[2],coords[3]), [255,255,0], 2)
            plt.imshow(bbox_img_gray_tmp), plt.show()
        
        # Segment the lines for intersection
        h_lines = []
        v_lines = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
             # Call all angles < 45 horizontal, else vertical
            lin_ang = np.abs(np.arctan2((y2-y1), (x2-x1)))
                
            if lin_ang <= np.pi/4:
                h_lines.append(line)
            else:
                v_lines.append(line)
        
        # Find intersections
        pt_list = []
        for h_line in h_lines:
            for v_line in v_lines:
                x1, y1, x2, y2 = h_line[0]
                x3, y3, x4, y4 = v_line[0]
                
                # compute determinant
                Px = np.float32(((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)))
                Py = np.float32(((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)))
                
                pt_list.append((Px, Py))
                
        pt_list = np.array(pt_list)
        
        if visualize == True:
            bbox_img_gray_tmp = bbox_img_gray.copy()
            for pt in pt_list:
                cv2.circle(bbox_img_gray_tmp, (pt[0], pt[1]), 1, (0,0,255),-1)
            plt.imshow(bbox_img_gray_tmp), plt.show()
        
        # kmeans clustering?
        
        
#        clusters = hcluster.fclusterdata(pt_list_array,20, criterion="distance")
        
        # Find best corners based on quadrants and center
        
        quad1 = pt_list[(pt_list[:,0] >= w/2) * (pt_list[:,1] < h/2),:]
        quad2 = pt_list[(pt_list[:,0] >= w/2) * (pt_list[:,1] >= h/2),:]
        quad3 = pt_list[(pt_list[:,0] < w/2) * (pt_list[:,1] < h/2),:]
        quad4 = pt_list[(pt_list[:,0] < w/2) * (pt_list[:,1] >= h/2),:]
        
        pt_1 = self.closest_pt((h/2,w/2), quad1)
        pt_2 = self.closest_pt((h/2,w/2), quad2)
        pt_3 = self.closest_pt((h/2,w/2), quad3)
        pt_4 = self.closest_pt((h/2,w/2), quad4)
        
        coords_local = np.array([pt_1, pt_2, pt_3, pt_4])
        
        coords = coords_local + np.array([x_min, y_min])
        
        # ConvexHull
        #hull = cv2.convexHull(lines,returnPoints = True)
        
        # Reverse affine transformation?
        
        # Transform back to original image
        
        # Visualize
        if visualize == True:
#            for line in lines:
#                coords = line[0]
#                cv2.line(bbox_img_gray, (coords[0],coords[1]), (coords[2],coords[3]), [255,255,0], 3)
            for pt in coords_local:
                cv2.circle(bbox_img_gray, (pt[0], pt[1]), 5, (0,255,0),-1)
        
        plt.imshow(bbox_img_gray), plt.show()        
        return coords
        
if __name__ == '__main__':
    region_obj = flyable_region_detector()
    region_obj.detect(visualize=True)
