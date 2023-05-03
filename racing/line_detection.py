import cv2
import numpy as np
import pdb
import os

# TODO (srinathm): Update the good line filtering parameters to remove more horizontal looking stuff
 
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
    # print("ORIGINAL IMAGE")
    # image_print(img)
    
    # First blur the image a bit for edge detection purposes
    kernel_width = 5
    kernel = np.ones((kernel_width,kernel_width),np.float32)/(kernel_width*kernel_width)
    img = cv2.filter2D(img,-1,kernel)
    
    # print("BLURRED IMAGE")
    # image_print(img)

    # Create a mask. Threshold the RGB image to get only the white pixels
    lower_white = np.array([170, 170, 170])
    upper_white = np.array([255, 255, 255])

    img_edges = cv2.inRange(img, lower_white, upper_white)

    # print("COLOR FILTER MASKED IMAGE")
    # image_print(img_edges)

    # image_print(img_edges)

    # edge detection
    min_thresh = 150
    max_thresh = 200
    img_edges = cv2.Canny(img_edges,min_thresh,max_thresh)

    # print("EDGE DETECTION")
    # image_print(img_edges)

    # # Cut out the top portion of the image
    mask_portion = 0.25
    mask = np.zeros_like(img_edges)
    mask[int(mask_portion * mask.shape[0]):] = 1
    img_edges *= mask

    # print("CROPPED IMAGE")
    # image_print(img_edges)

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
    k = img_width
    if filtered_lines is not None:
        for i in range(0, len(filtered_lines)):
            rho = filtered_lines[i][0][0]
            theta = filtered_lines[i][0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + k*(-b)), int(y0 + k*(a)))
            pt2 = (int(x0 - k*(-b)), int(y0 - k*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    
    return cdst

def filter_lines(lines):
    """
    Given all the lines from the Hough transform, pick only the lines that
    could plausibly be a lane
    """
    good_lines = []
    for line in lines:
        theta = line[0][1] * 180/np.pi
        if (theta > 105 or theta < 75):
            good_lines.append(line)
    return good_lines

def determine_lookahead_point(img_height, lines):
    """
    Given a set of good lines, determine a candidate for the lookahead point.
    """
    y_desired = int(2/3 * img_height)
    line_x_coordinates = []
    for line in lines:
        rho = line[0][0]
        theta = line[0][1]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x_shift_amt = (y_desired - y0)/a
        x_coord = x0 + x_shift_amt * (-b)
        line_x_coordinates.append(x_coord)
    # Take "unique" x coordinates to avoid double counting lines
    good_x_coordinates = []
    dist_tol = 20
    for x_coord in line_x_coordinates:
        add_this_coord = True
        for good_x_coord in good_x_coordinates:
            if np.abs(good_x_coord - x_coord) < dist_tol:
                add_this_coord = False
        if add_this_coord:
            good_x_coordinates.append(x_coord)
    print(good_x_coordinates)
    return int(np.mean(good_x_coordinates)), y_desired

def load_image(dir_name, img_num):
    img_orig = cv2.imread(dir_name + '/' + str(img_num) + ".jpg")

    # Get filtered lines
    lines = get_lines(img_orig)
    good_lines = filter_lines(lines)

    print("IMAGE NUMBER " + str(img_num))
    # Visualize the lines
    img_width = 5000 # Change this for the car
    cdst = draw_lines(img_orig, good_lines, img_width)

    # Visualize the lookahead point
    img_height = img_orig.shape[0]
    x_lookahead, y_lookahead = determine_lookahead_point(img_height, good_lines)
    lookahead_img = cv2.circle(cdst, (x_lookahead, y_lookahead), radius=10, color=(0,0,0), thickness=-1)
    print("LOOKAHEAD POINT:", x_lookahead, y_lookahead)
    cv2.imshow("Lookahead", lookahead_img)
    cv2.waitKey()
    # print(x_lookahead, y_lookahead)

# read in test images for our hough transform
DIR_NAME = "img/johnson_lane4_run2"
num_imgs = 12
for img_num in range(1, num_imgs+1):
    # img_num = 12
    load_image(DIR_NAME, img_num)