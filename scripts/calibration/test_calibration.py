import numpy as np
import cv2
from matplotlib import pyplot as plt


CAMERA_MATRIX = [	[668.26209391,  	 		0.,         299.10258721],
 					[  0.,         	 646.96066863, 			231.98017084],
 					[  0.,           			0.,           		  1.]]
DISTORTION = [[-0.48698219, -0.51573954,  0.01664705, -0.00720086,  2.29695665]]


# Read an example image and acquire its size
img = cv2.imread("snapshot_640_480_20.jpg")
h, w = img.shape[:2]
print('yo1')
# Generate new camera matrix from parameters
NEW_CAMERA_MATRIX, roi = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DISTORTION, 
	(w,h), 1, (w,h))
print('yo2')

# undistort
newimg = cv2.undistort(img, NEW_CAMERA_MATRIX, DISTORTION, None, NEW_CAMERA_MATRIX)
print('yo3')

# crop the image
x,y,w,h = roi
newimg = newimg[y:y+h, x:x+w]
print('yo4')

# cv2.imwrite('calibresult.png',dst)

# # Generate look-up tables for remapping the camera image
# mapx, mapy = cv2.initUndistortRectifyMap(K, d, None, newcameramatrix, (w, h), 5)

# # Remap the original image to a new image
# newimg = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

# Display old and new image
fig, (oldimg_ax, newimg_ax) = plt.subplots(1, 2)
oldimg_ax.imshow(img)
oldimg_ax.set_title('Original image')
newimg_ax.imshow(newimg)
newimg_ax.set_title('Unwarped image')
plt.show()