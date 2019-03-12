import os, cv2
import numpy as np
import sys, json
from pprint import pprint

os.chdir(os.getcwd())
grey3 = lambda grey : cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)

def get_files(folder='screenshots/'):
    files = os.listdir(folder)
    return [f for f in files if f[:6] == "screen"]



def get_contour(img1, img2, viz=True):
    img = cv2.cvtColor(img1-img2 ,cv2.COLOR_BGR2GRAY)
    blur = cv2.blur(img, (10, 10)) # blur the image
    ret,thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY)
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=2)
    im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:

        hull = []
        # calculate points for each contour
            # Find the index of the largest contour
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        cnt = contours[max_index]

        perimeter = cv2.arcLength(cnt,True)
        epsilon = 0.01*cv2.arcLength(cnt,True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)

        hull.append(cv2.convexHull(cnt, False))
        
        print(len(approx))
        
        if viz == True:
            final = np.hstack((img1, img2, grey3(blur)))
            final2 = np.hstack((cv2.drawContours(img1, contours, -1, (0,255,0), 3), grey3(im2), cv2.drawContours(img2, [approx] , -1 , (0,255,0), 3)))

            final_final = np.vstack((final, final2))

            cv2.imshow('final', final_final)
            cv2.waitKey(1000)
        return [a.tolist() for a in approx]
    else:
        return [[],[],[],[]]

def process_files(folder='screeshots/'):
    dct = {}
    files = get_files(folder)
    for f in files:
        img1 = cv2.imread(folder+f)
        img2 = cv2.imread(folder+'mask_'+f)
        cnt = get_contour(img1, img2)
        dct[f] = cnt
    return dct

def write_json(dct, out='simulated_results.json'):
    with open(out, 'w') as f:
        f.write(json.dumps(dct))

if __name__ == "__main__":
    dct = process_files(sys.argv[1])
    pprint(dct)
    write_json(dct)