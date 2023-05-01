import os
import rosbag
import matplotlib.pyplot as plt
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

FILE_HEADING = 'johnson_track_lane_4'
img_path = 'img/' + FILE_HEADING
if not os.path.exists(img_path):
   os.makedirs(img_path)

BAG_FILE = FILE_HEADING + '.bag'
print("Bag file is: " + BAG_FILE)
bag = rosbag.Bag(BAG_FILE)
bridge = CvBridge() # Converts between ROS images and OpenCV Images

# Save the images
counter = 0
for topic, image_msg, time in bag.read_messages(topics=[]):
    counter += 1
    if counter % 50 == 0: # Save every 50th image
        img_orig = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        filename = img_path + '/' + time + '.jpg'
        cv2.imwrite(filename, img_orig)
    # print(img_orig)

bag.close()
