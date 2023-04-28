import cv2
import numpy as np
import pdb
import os
#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

# read in a test image
img_num = 6
img_orig = cv2.imread(f"phone_johnson_photos/{img_num}.jpg")
kernel_width = 3

kernel = np.ones((kernel_width,kernel_width),np.float32)/(kernel_width*kernel_width)
img = cv2.filter2D(img_orig,-1,kernel)

# edge detection
min_thresh = 150
max_thresh = 200
img_edges = cv2.Canny(img,min_thresh,max_thresh)
# image_print(img_edges)

# Copy edges to the images that will display the results in BGR
cdst = cv2.cvtColor(img_edges, cv2.COLOR_GRAY2BGR)
cdstP = np.copy(cdst)

# hough transform
rho = 1
theta = np.pi/180
intersect_thresh = 100
lines = cv2.HoughLines(img_edges, rho, theta, intersect_thresh, None, 0, 0)
# Draw the lines
if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

cv2.imshow("Source", img)
cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
cv2.waitKey()

# slicing image
#lower = .5
#upper = .75

#img[:int(lower*img.shape[0])] = (0,0,0)
#img[int(upper*img.shape[0]):] = (0,0,0)

#cv2.imshow("slice", img)


#cv2.waitKey(0)
#cv2.destroyAllWindows()

# cd_color_segmentation(img)