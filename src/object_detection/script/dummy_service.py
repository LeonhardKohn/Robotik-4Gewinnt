import rospy
import numpy as np
from object_detection.srv import DetectGrid, DetectGridResponse

class YOLOServiceNode:
    def __init__(self):
        rospy.Service('detect_grid', DetectGrid, self.handle_detect_grid)
        rospy.loginfo("Dummy Service Ready")     
    
    
    def handle_detect_grid(self, req):
        detected_grid = np.zeros((6, 7), dtype=int)
        detected_grid[5, 3] = 1
        detected_grid[5, 2] = -1
        detected_grid[4, 3] = 1
        detected_grid[3, 3] = 1
        return DetectGridResponse(detected_grid.flatten().tolist())

if __name__ == "__main__":
    
    rospy.init_node('yolo_service_node')
    YOLOServiceNode()
    rospy.spin()
