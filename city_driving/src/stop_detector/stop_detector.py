import cv2
#import rospy

import numpy as np
#from sensor_msgs.msg import Image
#from geometry_msgs.msg import Point
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector(threshold = 0.7)
        #self.publisher = rospy.Publisher("/stop_sign", Point, queue_size=1)
        #self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        is_stopsign, bounding_box = self.detector.predict(rgb_img)
        print(is_stopsign, bounding_box)


if __name__=="__main__":
    #rospy.init_node("stop_sign_detector")
    img_path = "./test_images/stop1.jpeg"
    detect = SignDetector()
    detect.callback(img_msg = cv2.imread(img_path))
    #rospy.spin()
