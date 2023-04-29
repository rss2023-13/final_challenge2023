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

def get_lines(img):
    """
    Returns the list of lines in the images computed by the Hough transform
    """
    # First blur the image a bit for edge detection purposes
    num_blur_times = 1
    kernel_width = 11
    kernel = np.ones((kernel_width,kernel_width),np.float32)/(kernel_width*kernel_width)
    # kernel = np.ones((kernel_width,kernel_width),np.float32)
    for i in range(num_blur_times):
        img = cv2.filter2D(img,-1,kernel)

    # Create a mask. Threshold the RGB image to get only the white pixels
    lower_white = np.array([170, 170, 170])
    upper_white = np.array([255, 255, 255])

    img_edges = cv2.inRange(img, lower_white, upper_white)

    # image_print(img_edges)

    # edge detection
    min_thresh = 150
    max_thresh = 200
    img_edges = cv2.Canny(img_edges,min_thresh,max_thresh)

    # # Cut out the top portion of the image
    mask_portion = 0.7
    mask = np.zeros_like(img_edges)
    mask[int(mask_portion * mask.shape[0]):] = 1
    img_edges *= mask

    # hough transform
    rho = 1
    theta = np.pi/180
    intersect_thresh = 100
    lines = cv2.HoughLines(img_edges, rho, theta, intersect_thresh, None, 0, 0)

    return lines

def draw_lines(img, lines, img_width):
    """
    Draw the lines given by the hough transform
    """
    cdst = np.copy(img)
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + img_width*(-b)), int(y0 + img_width*(a)))
            pt2 = (int(x0 - img_width*(-b)), int(y0 - img_width*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    
    cv2.imshow("Source", img)
    cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    cv2.waitKey()

# read in test images for our hough transform
for i in range(1,8):
    img_num = i
    img_orig = cv2.imread(f"phone_johnson_photos/{img_num}.jpg")

    lines = get_lines(img_orig)
    img_width = 5000 # Change this for the car
    draw_lines(img_orig, lines, img_width)