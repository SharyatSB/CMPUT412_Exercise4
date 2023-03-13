#! /usr/bin/python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import apriltag
import led_emitter
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.msg import LEDPattern

import os

class april_tags:
    def __init__(self):
        # self.host = str(os.environ['VEHICLE_NAME'])
        self.host = "csc22919"
        rospy.wait_for_service('/' + self.host + '/led_emitter_node/set_custom_pattern')
        self.service = rospy.ServiceProxy('/' + self.host + '/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
        self.led_msg = LEDPattern()

        self.image_sub = rospy.Subscriber("/csc22919/camera_node/image/compressed", CompressedImage, self.image_callback)
        self.image_pub = rospy.Publisher("/output/detected_image/compressed", CompressedImage, queue_size=1)
        self.gray = None


    # Image call back function. It takes the CompressedImage message and returns a CV2 image
    def image_callback(self, msg):
        self.br = CvBridge()
        self.image = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.gray = self.color_to_gray(self.image)

    def color_to_gray(self, color_image):
        return cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    def detector(self):
        print("[INFO] detecting AprilTags...")
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(self.gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))

        for r in results:
            print("tag ID: ", r.tag_id)
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(self.image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(self.image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(self.image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(self.image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(self.image, (cX, cY), 5, (0, 0, 255), -1)

            tagID = r.tag_id
            cv2.putText(self.image, str(tagID), (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.image_pub.publish(self.br.cv2_to_compressed_imgmsg(self.image))
            # return image

        # cv2.imshow("Image", image)

        leds = self.system(results)

    def system(self, results):
        for r in results:
            # Ualberta tag
            if (r.tag_id == 201) or (r.tag_id == 38) or (r.tag_id == 78):
                self.led_msg.color_list = ['red', 'purple', 'yellow', 'purple', 'red']
                self.led_msg.color_mask = [1, 1, 1, 1, 1]
                self.led_msg.frequency = 0
                self.led_msg.frequency_mask = [0, 0, 0, 0, 0]
                response = self.service(self.led_msg)
                rospy.loginfo(response)

            elif (r.tag_id == 91) or (r.tag_id == 23):
                self.led_msg.color_list = ['red', 'green', 'blue', 'purple', 'yellow']
                self.led_msg.color_mask = [1, 1, 1, 1, 1]
                self.led_msg.frequency = 0
                self.led_msg.frequency_mask = [0, 0, 0, 0, 0]
                response = self.service(self.led_msg)
                rospy.loginfo(response)


def main():
    
    rospy.init_node('april_tag_detector')

    ap = april_tags()    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # Call some function
        ap.detector()
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
