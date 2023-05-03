import cv2
#import rospy

import numpy as np
#from sensor_msgs.msg import Image
#from geometry_msgs.msg import Point
from detector import StopSignDetector
import matplotlib.pyplot as plt
from IPython.display import display 

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector(threshold = 0.7)
        #self.publisher = rospy.Publisher("/stop_sign", Point, queue_size=1)
        #self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)

    def callback(self, img_msg):
        # TESTING ON CAR:
        # np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        # bgr_img = np_img[:,:,:-1]
        # rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        # is_stopsign, bounding_box = self.detector.predict(rgb_img)
        # print(is_stopsign, bounding_box)

        # cv2.imshow(self.detector.draw_box(rgb_img, bounding_box))

        #TESTING ON COMPUTER:
        np_img = img_msg.copy()
        rgb_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)

        is_stopsign, bounding_box = self.detector.predict(rgb_img)
        x = cv2.rectangle(np_img, (int(bounding_box[0]), int(bounding_box[1])), (int(bounding_box[2]), int(bounding_box[3])), (255,0,0), 2)
        
        image_print(x)

def image_print(img):
	"""
	Helper function to print out images, for debugging.
	Press any key to continue.
	"""
	winname = "Image"
	cv2.namedWindow(winname)        # Create a named window
	cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
	cv2.imshow(winname, img)
	cv2.waitKey()
	cv2.destroyAllWindows()


if __name__=="__main__":
    #rospy.init_node("stop_sign_detector")
    img_path = "./test_images/10.png"
    detect = SignDetector()
    detect.callback(img_msg = cv2.imread(img_path))
    #rospy.spin()
