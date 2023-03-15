#!/usr/bin/env python3

import rospy
from april_tag import april_tags
import cv2
from cv_bridge import CvBridge
import apriltag
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32
from turbojpeg import TurboJPEG
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from duckietown_msgs.msg import BoolStamped, VehicleCorners


ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = False
ENGLISH = False

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = rospy.get_param("~veh")
        # Publishers & Subscribers
        self.pub = rospy.Publisher("/" + self.veh + "/output/image/mask/compressed",
                                   CompressedImage,
                                   queue_size=1)
        self.sub = rospy.Subscriber("/" + self.veh + "/camera_node/image/compressed",
                                    CompressedImage,
                                    self.callback,
                                    queue_size=1,
                                    buff_size="20MB")
        self.vel_pub = rospy.Publisher("/" + self.veh + "/car_cmd_switch_node/cmd",
                                       Twist2DStamped,
                                       queue_size=1)
        self.distance_sub = rospy.Subscriber("/csc22919/duckiebot_distance_node/distance", 
                                        Float32, 
                                        self.distance_callback, 
                                        queue_size=1)
    
        self.detection_sub = rospy.Subscriber("/csc22919/duckiebot_detection_node/detection", 
                                         BoolStamped, 
                                         self.detection_callback, 
                                         queue_size=1)
        
        self.image_sub = rospy.Subscriber("/csc22919/camera_node/image/compressed", 
                                          CompressedImage, 
                                          self.image_callback)
        
        self.center_sub = rospy.Subscriber("/csc22919/duckiebot_detection_node/centers",
                                           VehicleCorners,
                                           self.centers_callback,
                                           queue_size=1)

        self.jpeg = TurboJPEG()

        self.loginfo("Initialized")

        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -220
        else:
            self.offset = 220
        self.velocity = 0.4
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        self.P = 0.049
        self.D = -0.004
        self.last_error = 0
        self.last_time = rospy.get_time()

        # Detection variables
        self.vehicle_distance = 1
        self.detecting = True
        self.gray = None
        self.tagID = 999
        self.x = 360
        # Shutdown hook
        rospy.on_shutdown(self.hook)

    def callback(self, msg):
        img = self.jpeg.decode(msg.data)
        crop = img[300:-1, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        crop = cv2.bitwise_and(crop, crop, mask=mask)
        contours, hierarchy = cv2.findContours(mask,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        # Search for lane in front
        max_area = 20
        max_idx = -1
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > max_area:
                max_idx = i
                max_area = area

        if max_idx != -1:
            M = cv2.moments(contours[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.proportional = cx - int(crop_width / 2) + self.offset
                if DEBUG:
                    cv2.drawContours(crop, contours, max_idx, (0, 255, 0), 3)
                    cv2.circle(crop, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass
        else:
            self.proportional = None

        if DEBUG:
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)

    def image_callback(self, msg):
        self.br = CvBridge()
        self.image = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.detect = self.april_tag_detector()
        
    def distance_callback(self, msg):
        self.vehicle_distance = msg.data

    def detection_callback(self, msg):
        self.detecting = msg.data

    def centers_callback(self, msg):
        if(len(msg.corners) > 10):
            self.x = msg.corners[10].x
        # print(self.x)

    def april_tag_detector(self):
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        # print("[INFO] detecting AprilTags...")
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(self.gray)
        # print("[INFO] {} total AprilTags detected".format(len(results)))

        for r in results:
            self.tagID = r.tag_id
            (ptA, ptB, ptC, ptD) = r.corners
            self.l = ptB - ptA
            print(self.l)



    def drive(self):
        # if self.x == 360:
        #     self.twist.omega = 0
        # else:
        #     # P Term
        #     P = -self.x * self.P

        #     # D Term
        #     d_error = (self.x - self.last_error) / (rospy.get_time() - self.last_time)
        #     self.last_error = self.x
        #     self.last_time = rospy.get_time()
        #     D = d_error * self.D

        #     self.twist.v = self.velocity
        #     self.twist.omega = P + D
        #     if DEBUG:
        #         self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)

        # self.vel_pub.publish(self.twist)
        
        # print("Restarting again")

        # Going straight
        # print(self.tagID)
        print(self.detecting)

        if (self.tagID == 153) or (self.tagID == 133) or (self.tagID == 58) or (self.tagID == 62) or (self.tagID == 133) or (self.tagID == 162):
            if self.l[0] > 45:
                self.twist.v = 0
                self.twist.omega = 0
                self.vel_pub.publish(self.twist)
                rospy.sleep(0.7)
                print("stopping because of april tag")
                self.tagID = 999
                print(self.vehicle_distance)

        if self.detecting == True:  
                print("I am detecting")
                if self.vehicle_distance < 0.45:
                    # print("Stopping because less distance")
                    self.twist.v = 0
                    self.twist.omega = 0
                    self.vel_pub.publish(self.twist)

                elif self.vehicle_distance >= 0.45:
                    print(self.x)
                    if( 260 < self.x < 380):
                        print("going straight")

                        if self.proportional is None:
                            self.twist.omega = 0

                        else:
                            # P Term
                            # print("Moving when True")
                            P = -self.proportional * self.P

                            # D Term
                            d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
                            self.last_error = self.proportional
                            self.last_time = rospy.get_time()
                            D = d_error * self.D

                            self.twist.v = self.velocity
                            self.twist.omega = P + D
                            if DEBUG:
                                self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)
                        


                    # Going left
                    elif (self.x < 260):
                        print("turning left")
                        self.twist.v = 0.3
                        self.twist.omega = 3
                        self.vel_pub.publish(self.twist)
                        rospy.sleep(0.5)

                    # Going right
                    elif( self.x > 380):
                        print("turning right")
                        self.twist.v = 0.3
                        self.twist.omega = -3
                        self.vel_pub.publish(self.twist)
                        rospy.sleep(0.5)


                self.vel_pub.publish(self.twist)

        else:
            print("Using lane following")
            if self.proportional is None:
                self.twist.omega = 0

            else:
                # P Term
                # print("Moving when True")
                P = -self.proportional * self.P

                # D Term
                d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
                self.last_error = self.proportional
                self.last_time = rospy.get_time()
                D = d_error * self.D

                self.twist.v = self.velocity
                self.twist.omega = P + D
                if DEBUG:
                    self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)

                    rospy.sleep(0.5)

            self.vel_pub.publish(self.twist)

        # if ( 250 < self.x < 400):
        #     print("going straight")
        #     if self.detecting == True:  
        #         print("I am detecting")
        #         if self.vehicle_distance < 0.60:
        #             print("Stopping because less distance")
        #             self.twist.v = 0
        #             self.twist.omega = 0

        #         elif self.proportional is None:
        #             self.twist.omega = 0

        #         else:
        #             # P Term
        #             # print("Moving when True")
        #             P = -self.proportional * self.P

        #             # D Term
        #             d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
        #             self.last_error = self.proportional
        #             self.last_time = rospy.get_time()
        #             D = d_error * self.D

        #             self.twist.v = self.velocity
        #             self.twist.omega = P + D
        #             if DEBUG:
        #                 self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)
            
        #     else:
        #         # print("False")
        #         if self.proportional is None:
        #             self.twist.omega = 0

        #         else:
        #             # P Term
        #             print("Moving when False")
        #             P = -self.proportional * self.P

        #             # D Term
        #             d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
        #             self.last_error = self.proportional
        #             self.last_time = rospy.get_time()
        #             D = d_error * self.D

        #             self.twist.v = self.velocity
        #             self.twist.omega = P + D
        #             if DEBUG:
        #                 self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)

        #     self.vel_pub.publish(self.twist)

        # # Going left
        # elif( self.x < 250):
        #     print("turning left")
        #     self.twist.v = 0.3
        #     self.twist.omega = 3
        #     self.vel_pub.publish(self.twist)
        #     rospy.sleep(1)

        # # Going right
        # elif( self.x > 400):
        #     print("turning right")
        #     self.twist.v = 0.3
        #     self.twist.omega = -2.5
        #     self.vel_pub.publish(self.twist)
        #     rospy.sleep(0.5)



        # self.prev_distance = self.vehicle_distance


    def hook(self):
        print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)
        for i in range(8):
            self.vel_pub.publish(self.twist)

if __name__ == "__main__":
    node = LaneFollowNode("lanefollow_node")
    rate = rospy.Rate(8)  # 8hz
    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()