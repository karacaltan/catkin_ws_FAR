#!/usr/bin/env python

import roslib
import rospy
import cv2
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import imutils
import numpy.linalg as alg


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
        cropped = crop_image(img)
        white = get_all_white_coordinates(cropped)
        ransac(cropped, white)


        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def image_to_binary(image):
    th, dst = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
    cv2.waitKey(3)
    return dst


def crop_image(image):
    height, width = image.shape[:2]
    start_now, start_col = int(height*.15), int(width*.35)
    end_now, end_col = int(height*.5), int(width * .75)
    cropped_img = image[start_now:end_now, start_col:end_col]
    cv2.imshow("cropped", cropped_img)
    im_gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(im_gray, 127, 255, 0)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cropped_img, contours, -1, (255, 255, 255), 3)
    cv2.imshow("cropped", cropped_img)
    return cropped_img


def get_all_white_coordinates(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    indices = np.where(gray == [255])
    coordinates = zip(indices[0], indices[1])
    return coordinates


def ransac(image, coordinates):
    coordinates_number = len(coordinates)
    t = 40
    random_sample_index = np.random.randint(0, len(coordinates) - 1)
    random_sample_index2 = np.random.randint(0, len(coordinates) - 1)
    fst_point = coordinates[random_sample_index]
    snd_point = coordinates[random_sample_index2]
    coordinates_without_samples = coordinates
    del coordinates_without_samples[random_sample_index]
    del coordinates_without_samples[random_sample_index2 - 1]
    inliers = []
    for coordinate in coordinates_without_samples:
        distance = distance_point_model(np.asarray(fst_point), np.asarray(snd_point), np.asarray(coordinate))
        if distance < t:
            inliers.append(coordinate)
    slope, intercept = line_model(fst_point, snd_point)
    if len(inliers) >= coordinates_number/2:
        cv2.line(image, pt1=fst_point, pt2=snd_point, color=(0,255,0), thickness=3)
        cv2.imshow("ransac", image)

    return slope, intercept


def line_model(fst_point, snd_point):
    dy = snd_point[0] - fst_point[0]*1.0
    dx = snd_point[1] - fst_point[1]
    if dx == 0:
        slope = 0
    else:
        slope = dy/dx
    intercept = fst_point[0] - slope * fst_point[1]
    return slope, intercept


def distance_point_model(fst_point, snd_point, trd_point):
    return alg.norm(np.cross(snd_point-fst_point, fst_point-trd_point))/alg.norm(snd_point-fst_point)


def main(args):
    lane_segmentation = LaneSegmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

