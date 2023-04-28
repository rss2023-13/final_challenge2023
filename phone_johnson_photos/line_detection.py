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

# test imgs
img_num = 2
img = cv2.imread(f"phone_johnson_photos/{img_num}.jpg")
# image_print(img)
img_edges = cv2.Canny(img,180,200)
image_print(img_edges)

# slicing image
#lower = .5
#upper = .75

#img[:int(lower*img.shape[0])] = (0,0,0)
#img[int(upper*img.shape[0]):] = (0,0,0)

#cv2.imshow("slice", img)


#cv2.waitKey(0)
#cv2.destroyAllWindows()

# cd_color_segmentation(img)