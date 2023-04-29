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
    kernel_width = 11
    kernel = np.ones((kernel_width,kernel_width),np.float32)/(kernel_width*kernel_width)
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

def draw_lines(img, filtered_lines, img_width):
    """
    Draw the lines given by the hough transform
    """
    cdst = np.copy(img)
    if filtered_lines is not None:
        for i in range(0, len(filtered_lines)):
            rho = filtered_lines[i][0][0]
            theta = filtered_lines[i][0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + img_width*(-b)), int(y0 + img_width*(a)))
            pt2 = (int(x0 - img_width*(-b)), int(y0 - img_width*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    
    # cv2.imshow("Source", img)
    # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    # cv2.waitKey()

def filter_lines(lines):
    """
    Given all the lines from the Hough transform, pick only the lines that
    could plausibly be a lane
    """
    good_lines = []
    for line in lines:
        if line is not None:
            theta = line[0][1] * 180/np.pi
            if (theta > 105 or theta < 75):
                good_lines.append(line)
    return good_lines

def determine_lookahead_point(image, lines):
    """
    Given a set of good lines, determine a candidate for the lookahead point.
    """
    DESIRED_Y = 0

# read in test images for our hough transform
for i in range(7):
    img_num = i+1
    img_orig = cv2.imread("phone_johnson_photos/" + str(img_num) + ".jpg")

    lines = get_lines(img_orig)
    good_lines = filter_lines(lines)
    img_width = 5000 # Change this for the car
    print("IMAGE NUMBER " + str(i+1))
    print(np.array(good_lines))
    # draw_lines(img_orig, good_lines, img_width)