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
    kernel_width = 5
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
    mask_portion = 0.25
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

def get_lookahead_pixel(img_height, img_width, lines):
    """
    Given a set of good lines, determine a candidate for the lookahead point.
    """
   # y_desired = int(2. / 3. * img_height)
    y_desired = 220
    # width_tol = 65
    
    # Calculate the x coordinate for each line, and separate the lines into left and right sides
    left_lines = []
    right_lines = []
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
        
        # Calculate coordinate representing car's position
        x_car, y_car = img_width/2., img_height

        # A line is on the left side if at y_coord = y_car, the x_coord is on x_car
        x_line_shift = (y_car - y0)/a
        x_line_coord = x0 + x_line_shift * (-b)

        if x_line_coord < x_car:
            left_lines.append(x_coord)
        elif x_line_coord > x_car:
            right_lines.append(x_coord)

    # Take a weighted average of the right and left side points to weight both sides equally.
    # If only one side has lines then make the lookahead point the center
    left_average = np.mean(left_lines)
    right_average = np.mean(right_lines)
    recover_dist = 100 # 100 for 4 m/s, 50 for 5 m/s, 25 for 6 m/s
    if (len(left_lines) >= 1 and len(right_lines) >= 1):
        center_point = int(np.mean([left_average, right_average]))
    elif (len(left_lines) == 0):
        print("RECOVER TO THE LEFT")
        # Nudge it slightly to the left if there are no visible lines to the left
        center_point = int(img_width/2.) - recover_dist
    elif (len(left_lines) >= 1 and len(right_lines) == 0):
        print("RECOVER TO THE RIGHT")
        center_point = int(img_width/2.) + recover_dist
    else:
        print("NO LINES SEEN")
        center_point = int(img_width/2.)

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
    # print(good_x_coordinates)
    # Return the number of points as well
    # center_point = int(np.mean(line_x_coordinates))
    # center_point = int(np.mean(good_x_coordinates))
    # cam_x_coord = img_width / 2.

    # if cam_x_coord <= center_point + width_tol and cam_x_coord >= center_point - width_tol:
    #     return int(cam_x_coord), y_desired
    # elif cam_x_coord >= center_point + width_tol:
    #     return int(center_point + width_tol), y_desired
    # else:
    #     return int(center_point - 0.5*width_tol), y_desired

    return center_point, y_desired

def lookahead_px_from_img(cv2_img):
    """Given a cv2 image, return the lookahead pixel coordinates

    Args:
        cv2_img (_type_): BGR CV2 image array
    """
    # Get filtered lines
    lines = get_lines(cv2_img)
    good_lines = filter_lines(lines) # exclude horizontal lines


    img_height, img_width = cv2_img.shape[0], cv2_img.shape[1]
    cdst = draw_lines(cv2_img, good_lines, img_width)


    # Visualize the lookahead point
    x_lookahead, y_lookahead = get_lookahead_pixel(img_height, img_width, good_lines)
    lookahead_img = cv2.circle(cdst, (x_lookahead, y_lookahead), radius=10, color=(0,0,0), thickness=-1)

    return x_lookahead, y_lookahead, lookahead_img

