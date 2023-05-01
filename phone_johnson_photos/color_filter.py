import cv2
import numpy as np

# read input image
img = cv2.imread('3.jpg')

# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# define range of blue color in HSV
# lower_white = np.array([int(70 / 360 * 179),int(0 / 100 * 255), int(80 / 100 * 255)])
# upper_white = np.array([int(130 / 360 * 179),int(20 / 100 * 255),int(100 / 100 * 255)])

lower_white = np.array([190, 190, 190])
upper_white = np.array([255, 255, 255])

# Create a mask. Threshold the HSV image to get only yellow colors
mask = cv2.inRange(img, lower_white, upper_white)

# Bitwise-AND mask and original image
result = cv2.bitwise_and(img,img, mask= mask)

# display the mask and masked image
cv2.imshow('Mask',mask)
cv2.waitKey(0)
# cv2.imshow('Masked Image',result)
# cv2.waitKey(0)
# cv2.destroyAllWindows()