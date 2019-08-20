#!/usr/bin/env python

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy.linalg as alg
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

        rows, cols, channels = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)

        cv2.imshow("Image window", cv_image)

        img_bin = image_to_binary(cv_image)
        # img_cropped = crop_image(cv_image)
        img_cropped = crop_image(img_bin)

        ransac(img_cropped)
        # ransac(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def image_to_binary(image):
    th, dst = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
    cv2.waitKey(3)
    return dst


def crop_image(image):
    cropped_img = image[110:110 + 158, 200:200 + 430]  # y:y+h, x:x+w
    cv2.imshow("cropped", cropped_img)
    cv2.waitKey(3)
    return cropped_img


def threshold_image(image):
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(im_gray, 127, 255, 0)
    im2, contours, hierarchy = cv2.findContours(image=thresh,
                                                mode=cv2.RETR_TREE,
                                                method=cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
    #
    # filtered_contours = []
    # for i in range(len(contours)):
    #     if i < 40 or (i > 50 and i < 60):
    #         filtered_contours.append(contours[i])
    # cropped_img = image[110:110 + 158, 200:200 + 430]  # y:y+h, x:x+w

    cv2.imshow("threshold", contours)
    cv2.waitKey(3)
    return contours


def ransac(image):
    # ----------------------------------------------------------------------------#
    # Draw Contours
    # ----------------------------------------------------------------------------#
    # im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(im_gray, 127, 255, 0)
    # im2, contours, hierarchy = cv2.findContours(image=thresh,
    #                                             mode=cv2.RETR_TREE,
    #                                             method=cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(image, contours, -1, (0, 255, 0), 1)
    cv2.imshow("ransac", image)
    # ----------------------------------------------------------------------------#
    # End
    # ----------------------------------------------------------------------------#

    coordinates = get_all_white_coordinates(image)
    print(coordinates[0])
    # ----------------------------------------------------------------------------#
    # Declaration
    # ----------------------------------------------------------------------------#
    number_of_samples = len(coordinates)
    sample_count = 0
    # ----------------------------------------------------------------------------#
    # End
    # ----------------------------------------------------------------------------#

    while number_of_samples > sample_count:
        random_sample_index = np.random.randint(0, len(coordinates) - 1)
        random_sample_index2 = np.random.randint(0, len(coordinates) - 1)
        random_sample = coordinates[random_sample_index]
        random_sample2 = coordinates[random_sample_index2]
        coordinates_without_samples = coordinates
        del coordinates_without_samples[random_sample_index]
        del coordinates_without_samples[random_sample_index2 - 1]
        for coordinate in coordinates_without_samples:
            distance = distance_point_model(
                np.asarray(random_sample),
                np.asarray(random_sample2),
                np.asarray(coordinate)
            )
            print distance
        m, b = get_linear_function(random_sample, random_sample2)
        print m, b
        sample_count += 1


def distance_point_model(fst_point, snd_point, trd_point):
    return alg.norm(np.cross(snd_point - fst_point, fst_point - trd_point)) / alg.norm(snd_point - fst_point)


def get_linear_function(point1, point2):
    m = (point1[0] - point2[0]) * 1.0 / (point1[1] - point2[1])
    b = (point1[1] * point2[0] - point2[1] * point1[0]) * 1.0 / (point1[1] - point2[1])
    return m, b


def get_all_white_coordinates(image):
    indices = np.where(image == [255])
    coordinates = zip(indices[0], indices[1])
    coordinates = list(dict.fromkeys(coordinates))
    return coordinates


# Function to find distance
def shortest_distance(x1, y1, a, b, c):
    d = abs((a * x1 + b * y1 + c)) / (math.sqrt(a * a + b * b))
    print("Perpendicular distance is"), d


def main(args):
    ic = LaneSegmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
