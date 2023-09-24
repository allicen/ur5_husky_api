#! /usr/bin/env python3

import sys
sys.path.insert(0,'/home/administrator/Repos/manipulator/geom_ws/devel/lib/python3/dist-packages')

import numpy as np
# np.set_printoptions(threshold=sys.maxsize)

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String

class Camera():

    def __init__(self):
        rospy.init_node('camera_depth', anonymous=True)

        self.rosbag = rospy.get_param("~rosbag")
        self.delay = rospy.get_param("~delay")

        self.cv_bridge = CvBridge()

        self.ImageGripperDepth = None
        self.can_get_image_gripper_depth = False

        self.width = 1280
        self.height = 720

        self.gripper_busy = False

        self.figure_len_detect = 500

        self.line_height = 5
        gripper_close_height = 110
        gripper_close_correct = 35

        self.image_border_start = (0, self.height - int(self.line_height/2))
        self.image_border_end = (self.width - int(self.line_height/2), self.height - int(self.line_height/2))

        self.gripper_border_start = (0, self.height - int(self.line_height/2) - gripper_close_height)
        self.gripper_border_end = (self.width - int(self.line_height/2), self.height - int(self.line_height/2) - gripper_close_height)

        self.gripper_close_start = (int(self.width/2 - self.line_height/2) + gripper_close_correct, 0)
        self.gripper_close_end = (int(self.width/2 - self.line_height/2) + gripper_close_correct, self.height)
        

        # Images from RealSense 
        rospy.Subscriber("/realsense_gripper/aligned_depth_to_color/image_raw", Image, self.camera_gripper_depth)

        self.pub_gripper_state = rospy.Publisher('gripper_state_camera_depth', String, queue_size=10)

        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)


    def camera_gripper_depth(self, msg):
        # delay from get image camera
        if self.can_get_image_gripper_depth:
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
                self.ImageGripperDepth = cv_image
                self.can_get_image_gripper_depth = False

            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            self.ImageGripperDepth = None


    def process_depth_image(self, cv_image):
        depth_array = np.array(cv_image, dtype=np.float32)
        ones = np.ones(depth_array.shape, dtype=np.float32)

        np.copyto(ones, depth_array, where=depth_array == 0)
        
        bl = cv2.medianBlur(ones, 5)

        hsv_min = np.array((0, 0, 0), np.uint8)
        hsv_max = np.array((0, 0, 0), np.uint8)
        
        img = cv2.cvtColor(bl, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        dilated = cv2.dilate(thresh, kernel)

        contours, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cntrs_find = []
        cntrs_white_masked = []

        for cnt in contours:
            if len(cnt) > 1:
                add_mask = True
                for ci in cnt:
                    if ci[0][1] > self.gripper_border_start[1]:
                        for cj in cnt:
                            if cj[0][1] >= self.image_border_start[1]:
                                cntrs_find.append(cnt)
                                add_mask = False
                                break
                if add_mask:
                    cntrs_white_masked.append(cnt)
                    
        cv2.fillPoly(img, contours, [0,0,0])            
        cv2.fillPoly(img, cntrs_white_masked, [255,255,255])

        cntrs_find = sorted(cntrs_find, key=cv2.contourArea, reverse=True)

        for i, cnt in enumerate(cntrs_find):
            if i == 0:
                cv2.drawContours(img, cnt, -1, (0,0,255), 3)
                L1 = self.line(self.image_border_start, self.image_border_end)
                L2 = self.line(self.gripper_border_start, self.gripper_border_end)
                L3 = self.line(self.gripper_close_start, self.gripper_close_end)

                R1 = self.intersection(L1, L3)
                R2 = self.intersection(L2, L3)

                msg = String()
                msg.data = 'ready'

                if R1 and R2:
                    dist1 = cv2.pointPolygonTest(cnt, R1, True)
                    dist2 = cv2.pointPolygonTest(cnt, R2, True)

                    if dist1 > 0 and dist2 > 0:
                        self.gripper_busy = True
                        msg.data = 'busy'
                    else:
                        msg.data = 'ready'
                else:
                    msg.data = 'ready'

                self.pub_gripper_state.publish(msg)

            else:
                pass
        return img


    def line(self, p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0]*p2[1] - p2[0]*p1[1])
        return A, B, -C

    def intersection(self, L1, L2):
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x,y
        else:
            return False


    def spin(self):
        time_now = rospy.Time.now()
        time_prev = time_now

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            delta_time = now - time_prev
            
            if delta_time.secs >= self.delay:
                self.can_get_image_gripper_depth = True
                time_prev = now

            if self.ImageGripperDepth is not None:
                img = self.process_depth_image(self.ImageGripperDepth)
                cv2.line(img, self.image_border_start, self.image_border_end, (0,255,255), self.line_height) 
                cv2.line(img, self.gripper_border_start, self.gripper_border_end, (255,0,0), self.line_height)
                cv2.line(img, self.gripper_close_start, self.gripper_close_end, (255,0,0), self.line_height)

                font = cv2.FONT_HERSHEY_SIMPLEX
                bottomLeftCornerOfText = (10, self.gripper_border_start[1]-10)
                fontScale = 1
                fontColor = (255,0,0)
                thickness = 1
                lineType = 5
                cv2.putText(img,'Gripper Line', bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

                self.rate.sleep()
                
                # cv2.imshow("depth", img)
                # cv2.waitKey(3)
            self.rate.sleep()

    def shutdown(self):
        rospy.sleep(1)


camera = Camera()
camera.spin()
