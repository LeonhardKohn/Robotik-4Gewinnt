import numpy as np
import cv2
#cap = cv2.VideoCapture(4)
num = 0
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image as msg_Image

def img_callback(data):
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    #, img = cap.read()
    cv2.imshow("Image", img)
    key = cv2.waitKey(10)
    if key == ord('c'):
        cv2.imwrite("img_" + str(num) + ".jpg", img)
        print("Image saved")
        num +=1

rospy.init_node('image_test')
rospy.Subscriber("/camera/color/image_raw", msg_Image, img_callback)
rospy.spin()

#rosservice call /frankX_service_node/frankX_move -- 0.45 -0.338 0.6 5 0.1 1.57 0 0 0
