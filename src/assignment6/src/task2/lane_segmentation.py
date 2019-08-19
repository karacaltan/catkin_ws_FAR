#!/usr/bin/env python

import roslib
import rospy
import cv2
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


class LaneSegmentation:

    def __init__(self):
        rospy.init_node('laneSegmentation', anonymous=True)
        self.image_pub = rospy.Publisher("laneSegmentation", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)

        cv2.imshow("Image window", cv_image)

        image_to_binary(cv_image)
        image_contours(image_to_binary(cv_image))

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def image_to_binary(image):
    th, dst = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
    # cv2.imshow("Binary Image", dst)
    cv2.waitKey(3)
    return dst


def image_contours(image):
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(im_gray, 127, 255, 0)
    im2, contours, hierarchy = cv2.findContours(image=thresh,
                                                mode=cv2.RETR_TREE,
                                                method=cv2.CHAIN_APPROX_SIMPLE)

    filtered_contours = []
    for i in range(len(contours)):
        if i < 40 or (i > 50 and i < 60):
            filtered_contours.append(contours[i])
    cropped_img = image[110:110+370, 106:106+470]

    cv2.imshow("cropped", cropped_img)
    cv2.drawContours(image, filtered_contours, -1, (0, 255, 0), 3)
    #print("Number of Contours found = " + str(len(contours)))
    cv2.imshow('Contour Image', image)


def main(args):
    ic = LaneSegmentation()
    # rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
