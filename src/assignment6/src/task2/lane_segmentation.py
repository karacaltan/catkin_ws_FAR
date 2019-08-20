#!/usr/bin/env python

import roslib
import rospy
import cv2
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import os,sys


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

        rows, cols, channels = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)

        cv2.imshow("Image window", cv_image)

        img = image_to_binary(cv_image)
        crop_image(img)

        ransac(img)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def image_to_binary(image):
    th, dst = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
    cv2.waitKey(3)
    return dst


def crop_image(image):
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(im_gray, 127, 255, 0)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
    im2, contours, hierarchy = cv2.findContours(image=thresh,
                                                mode=cv2.RETR_TREE,
                                                method=cv2.CHAIN_APPROX_SIMPLE)

    filtered_contours = []
    for i in range(len(contours)):
        if i < 40 or (i > 50 and i < 60):
            filtered_contours.append(contours[i])
    cropped_img = image[110:110+158, 200:200+430] #y:y+h, x:x+w

    cv2.imshow("cropped", cropped_img)
    cv2.drawContours(image, filtered_contours, -1, (0, 255, 0), 3)
    #print("Number of Contours found = " + str(len(contours)))
    cv2.imshow('Contour Image', image)




def ransac(image):
    MIN_MATCH_COUNT = 10

    img1 = image  # queryImage
    img2 = crop_image(image)  # trainImage

    # Initiate SIFT detector
    sift = cv2.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1, des2, k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w = img1.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, M)

        img2 = cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

    else:
        print "Not enough matches are found - %d/%d" % (len(good), MIN_MATCH_COUNT)
        matchesMask = None

    draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                       singlePointColor=None,
                       matchesMask=matchesMask,  # draw only inliers
                       flags=2)

    img3 = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)

    plt.imshow(img3, 'gray'), plt.show()

def main(args):
    ic = LaneSegmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
