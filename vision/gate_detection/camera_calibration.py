import numpy as np
import cv2
import glob
import argparse 
import pylab as plt

parser = argparse.ArgumentParser()
parser.add_argument("-f","--filename", help="display a square of a given number",
                    required=True)
args = parser.parse_args()

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:9].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.JPG')
cnt=0
for fname in images:
    
    img = cv2.imread(fname)
    
    #resizing only for visualization purposes
    img=cv2.resize(img,(640,480))
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,9),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)
   
        # Draw and display the corners
        #img = cv2.drawChessboardCorners(img, (9,7), corners2,ret)
        #cv2.imshow('img',img)
        #cv2.waitKey(0)
#calculating the camera calibration matrix
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print("calibrateCamera done")

img1 = cv2.imread(args.filename+'.JPG')
img1=cv2.resize(img,(640,480))
H,  W = img1.shape[:2]
print(H,W)
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(W,H),1,(W,H))
#print(mtx,newcameramtx)
# undistort
dst = cv2.undistort(img1, mtx, dist, None,newcameramtx)

#print(roi)
# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite(args.filename+'_ud.jpg',dst)
dst=cv2.resize(dst,(W,H))
#print(dst.shape[:2])




#visualisation
fig = plt.figure()
fig.add_subplot(1,2,1)
plt.title('original image')
plt.imshow(img1)
fig.add_subplot(1,2,2)
plt.title('undistorted image')
plt.imshow(dst)
plt.show()
#cv2.waitKey(0)

