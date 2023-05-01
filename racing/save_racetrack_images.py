import os
import rosbag
import matplotlib.pyplot as plt
import math
from cv_bridge import CvBridge, CvBridgeError

FILE_HEADING = 'johnson_track_lane_4'
if not os.path.exists('img/' + FILE_HEADING):
   os.makedirs('img/' + FILE_HEADING)

BAG_FILE = FILE_HEADING + '.bag'
print("Bag file is: " + BAG_FILE)
bag = rosbag.Bag(BAG_FILE)
bridge = CvBridge() # Converts between ROS images and OpenCV Images

# Save the images
for topic, image_msg, time in bag.read_messages(topics=[]):
    print(time)
    # img_orig = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    # print(img_orig)

bag.close()
