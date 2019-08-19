#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):

        rospy.init_node('image_converter', anonymous=True)
        self.image_pub = rospy.Publisher("/sensors/camera/infra1/image_rect_raw", Image, queue_size=10)

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
    # create binary image
    t = 200
    gray = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(src=gray,
                            ksize=(5, 5),
                            sigmaX=0)

    (t, binary) = cv2.threshold(src=blur,
                                thresh=t,
                                maxval=255,
                                type=cv2.THRESH_BINARY)

    # find contours
    (_, contours, _) = cv2.findContours(image=binary,
                                        mode=cv2.RETR_EXTERNAL,
                                        method=cv2.CHAIN_APPROX_SIMPLE)

    # create all-black mask image
    mask = np.zeros(shape=image.shape, dtype="uint8")

    for c in contours:
        (x, y, w, h) = cv2.boundingRect(c)

        coordinates = [
            (266, 122, 5, 1),
            (415, 113, 5, 1),
            (242, 161, 10, 3),
            (441, 149, 10, 3),
            (199, 234, 16, 8),
            (496, 221, 18, 8),
        ]

        if cv2.boundingRect(c) in coordinates:
            r = 255
            g = 255
            b = 255
        else:
            r = 0
            g = 0
            b = 0

        print('%s %s ' % ('contour:', cv2.boundingRect(c)))
        cv2.rectangle(img=mask,
                      pt1=(x, y),
                      pt2=(x + w, y + h),
                      color=(r, g, b),
                      thickness=-1)

    # cv2.drawContours(mask, contours, -1, (255, 255, 0), 1)

    cv2.imshow('Contour Image', mask)

    # im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(im_gray, 127, 255, 0)
    # im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print("Number of Contours found = " + str(len(contours)))
    # cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
    # print (contours)
    # cv2.imshow('Contour Image', image)


def main(args):
    ic = image_converter()
    # rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
