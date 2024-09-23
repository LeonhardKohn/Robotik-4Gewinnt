import math
import sys
import time
import cv2
import numpy as np
from ultralytics import YOLO
from object_detection.srv import DetectGrid, DetectGridResponse
import rospy
import rospkg
from scipy import ndimage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image

# Commands to run:
# -----------------
# rosrun object_detection yolo_service_node.py
# rosservice call /detect_grid 1
# roslaunch realsence2_camera rs_camera.launch
# -----------------

class YOLOServiceNode:
    def __init__(self):
        self.cheating = 0
        if len(sys.argv) > 1:
            if int(sys.argv[1]) == 0:
                print("Cheating is off")
            else:
                self.cheating = 1
                print("Cheating is on")
        else:
            print("No Number given => Cheating is off")
        
        
        self.cv_bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw",msg_Image,self.img_callback)
        #self.cap = cv2.VideoCapture(6)
        self.path_org = rospkg.RosPack().get_path('object_detection')
        self.model_path = self.path_org + "/script/model/runs/detect/train/weights/best.pt"
        self.model = YOLO(self.model_path)
        self.grid_rows = 6
        self.grid_cols = 7
        self.rerolles = 0
        self.classNames = ['blue', 'empty', 'red']
        self.classValues = [-1, 0, 1]
        self.old_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        self.lastImageGottenAt = time.time()
        self.img = None
        rospy.Service('detect_grid', DetectGrid, self.handle_detect_grid)
        rospy.loginfo("YOLO Grid Detection Service Ready")
    
    def img_callback(self,data):
        try:
            self.img = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            self.lastImageGottenAt = time.time()
        except CvBridgeError as err:
            rospy.loginfo(rospy.get_caller_id() + "%s", err)

    def remove_floating_stones(self,grid):
        for j in range(self.grid_cols):
            col = grid[:, j] 
            first_empty_space = False

            for i in reversed(range(self.grid_rows)):
                if col[i] == 1 or col[i] == -1:
                    if first_empty_space:
                        rospy.loginfo("Warning: Removed Floating Stone")
                        grid[i,j] = 0    
                        
                if col[i] == 0:
                    first_empty_space = True
        return grid
        
    def check_for_floting_stones(self,grid):
        for j in range(self.grid_cols):

            col = grid[:, j]
            first_empty_space = False

            for i in reversed(range(self.grid_rows)):
                if col[i] != 0:
                    if first_empty_space:
                        return True    
                if col[i] == 0:
                    first_empty_space = True
        return False

    
    def errorDetection(self,detected_grid,old_grid):
        diff = detected_grid-old_grid
        deff = len(np.where(diff != 0)[0])
        new_blue_stones = len(np.where(diff == -1)[0])
        new_red_stones = len(np.where(diff == 1)[0])

        detected_grid[detected_grid == 0] = old_grid[detected_grid == 0]
        
        differences = detected_grid * -1 == old_grid
        detected_grid[differences] = old_grid[differences] 
        detected_grid = self.remove_floating_stones(detected_grid)
        # Cheating Detection
        if(self.cheating == 1):
            if(self.current_turn == 0):
                if(self.isStartPlayer == 1): #you
                    if(len(np.where(detected_grid == 0)[0])) == 42:
                        return detected_grid
                    else:
                        rospy.loginfo("Error: kein leeres Brett am Anfang erkannt. Der Gegner cheatet!")
                        return None
                elif(self.isStartPlayer == -1):
                    if(deff!=1):
                        rospy.loginfo("Error: Nach erstem Zug mehr als einen Stein erkannt. Der Gegner cheatet!")
                        return None

            if(deff!=2 or new_blue_stones!=1 or new_red_stones != 1 or self.check_for_floting_stones(detected_grid)==True):
                if(self.cheating == 1):
                    if(len(np.where(detected_grid == 0)[0])) == 42:
                        rospy.loginfo("Error: leeres Brett erkannt")
                        return None

            # Steine, die bereits erkannt wurden und nicht wiedererkannt wurden rüberziehen            

        diff = detected_grid-old_grid
        deff = len(np.where(diff != 0)[0])
        new_blue_stones = len(np.where(diff == -1)[0])
        new_red_stones = len(np.where(diff == 1)[0])

        if(self.cheating == 1):
            if deff>2 or new_blue_stones!= 1 or new_red_stones != 1:
                rospy.loginfo("Das andere Team cheatet")
                return None
            if deff<2:
                rospy.loginfo("Bild nicht richtig erkannt.")
                if(self.rerolles<20):
                    rospy.loginfo("Bilderkennung wird neu gestartet.")
                    self.rerolles += 1
                    self.handle_detect_grid(self.req)
                else:
                    rospy.loginfo("Bilderkennung konnte nicht richtig funktionieren. Erkennung gegebenenfalls fehlerhaft")
                    return detected_grid
                return detected_grid
        self.rerolles = 0
        return detected_grid
    
    def check_for_overlapping(self,grid):
        toRemove = set()
        grid_copy = grid[:]
        for current_chip in grid_copy:
            # (conf, x1, y1, x2, y2, cls, cx, cy)
            current_chip_width = current_chip[3]-current_chip[1]
            current_chip_hight= current_chip[4]-current_chip[2]
            current_center_x = current_chip[1]+current_chip_width/2
            current_center_y = current_chip[2]+current_chip_hight/2

                # check for overlapping Stone
            for stone in grid_copy:
                if(stone == current_chip):
                    continue
                stone_width = stone[3]-stone[1]
                stone_hight = stone[4]-stone[2]
                if (stone[1] <= current_center_x <= stone[1] + stone_width) and (stone[2] <= current_center_y <= stone[2] + stone_hight):
                    if(current_chip[0]>stone[0]):
                        #remove overlapping stone 
                        toRemove.add(stone)
                    else:
                        toRemove.add(current_chip)
                    
                    print("Revoved Stone from Board, because it was overlapping")
                    print("----")
                    print(current_chip)
                    print(stone)
                    print("----")

        return [x for x in grid if (x not in toRemove)]         
           
    
    def handle_detect_grid(self, req):
        self.req = req
        self.current_turn = req.current_turn # only my turns
        self.isStartPlayer = req.isStartPlayer  # you is 1, other is -1
        self.color = req.color # red is 1, blue is 0
        self.blue_stone = 0
        self.red_stone = 0
        detected_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)

        # Fail if image is older than 1 second
        if time.time() - self.lastImageGottenAt > 1:
            rospy.logerr("Image too old! Check Camera!")
            return DetectGridResponse([])

        img = self.img

        # Fail if no image is saved yet
        if img is None:
            rospy.logerr("No Image recieved yet! Check Camera!")
            return DetectGridResponse([])

        img = ndimage.rotate(img, 180)

         # For testing: 
        # ----------
        # img_path = self.path_org + "/script/model/runs/detect/test/img_75.jpg"    
        # img = cv2.imread(img_path)
        # ----------
        
        results = self.model(img, stream=True)
        all_boxes = []
        # create a list with all boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2                
                conf = math.ceil((box.conf[0])) 
                cls = int(box.cls[0])
                all_boxes.append((conf, x1, y1, x2, y2, cls, cx, cy))
        
        if(self.cheating == 1):
        # if no box was found, there is some cheating 
            if(len(all_boxes)==0):
                print("Cheating: Kein Board vorhanden!!!")
                return None
        # cheack for all overlapping boxes and remove these

        all_boxes = self.check_for_overlapping(all_boxes)
        
        # take the boxes with the highest confidence
        all_boxes.sort(key=lambda x: x[0], reverse=True) 
        top_boxes = all_boxes[:42]

        top_boxes.sort(key=lambda x: x[7])
        grouped_boxes_line = [top_boxes[i:i + 7] for i in range(0, len(top_boxes), 7)]
        for group in grouped_boxes_line:
            group.sort(key=lambda x: x[6])
        sorted_boxes = [box for group in grouped_boxes_line for box in group]
        
        for idx, box in enumerate(sorted_boxes):
            _, _, _, _, _, cls, _, _ = box
            grid_x = idx % self.grid_cols
            grid_y = idx // self.grid_cols
            
            class_value = self.classValues[cls] if cls < len(self.classNames) else 0

            detected_grid[grid_y, grid_x] = class_value
            if class_value == 1:
                self.blue_stone += 1
            elif class_value == -1:
                self.red_stone += 1

        # testing 
        # self.red_stone = 0
        # self.blue_stone = 0
        # detected_grid = np.array([
        #                 [0, 0, 0, 0, 0, 0, 0],
        #                 [0, 0, 0, 0, 0, 0, 0],
        #                 [0, 0, 0, 0, 0, 0, 0],
        #                 [0, 0, 0, 0, 0, 0, 0],
        #                 [0, 0, 0, 0, 0, 0, 0],
        #                 [0, 0, 0, 0, 0, 0, 0]
        #             ])
        #print("test")
        #print("Old grid: "+self.old_grid)
        #print("Starting player: "+ self.isStartPlayer)
        

        detected_grid = self.errorDetection(detected_grid,self.old_grid)
        if(detected_grid is None):
            rospy.logwarn("Abbruch!!!")
            return None  
        self.old_grid = detected_grid

        print(detected_grid)
        
        return DetectGridResponse(detected_grid.flatten().tolist())

if __name__ == "__main__":
    
    rospy.init_node('yolo_service_node')
    YOLOServiceNode()
    rospy.spin()
