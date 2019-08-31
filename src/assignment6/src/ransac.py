import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np


class RANSAC:

    def __init__(self, data, N, t):
        self.data = data
        self.N = N
        self.t = t
        pass

    def fit(self):
        best_line = (0, 0)
        best_score = 0
        best_inlier = []
        best_outlier = []
        for i in range(self.N):
            # select two random points
            index = np.random.choice(self.data.shape[0], 2)
            points = self.data[index]
            line = self.fit_linear_model(points)
            score, inlier, outlier = self.score(line)

            if score > best_score:
                best_score = score
                best_line = line
                best_inlier = inlier
                best_outlier = outlier

        return best_line, best_inlier, best_outlier

    def score(self, line):
        m, b = line
        div = np.sqrt(1 + m * m)
        # distance to line
        inlier = self.data[np.where((np.abs(b + m * self.data[:, 1] - self.data[:, 0]) / div) < self.t)]
        outlier = self.data[np.where((np.abs(b + m * self.data[:, 1] - self.data[:, 0]) / div) >= self.t)]
        return inlier.size, inlier, outlier

    def fit_linear_model(self, points):
        # avoid division by zero for vertical line
        if points[0, 1] - points[1, 1] == 0:
            points[1, 1] += 1
        m = (points[0, 0] - points[1, 0]) / (points[0, 1] - points[1, 1])
        b = points[1, 0] - m * points[1, 1]

        return m, b


class LaneDetection:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("lane_detection", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, self.on_image, queue_size=1)
        rospy.init_node("lane_detection")

        self.rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)
            return

        cv_image = cv2.rectangle(cv_image, (0, 0), (640, 110), 0, cv2.FILLED)
        cv_image = cv2.rectangle(cv_image, (0, 240), (640, 480), 0, cv2.FILLED)
        ret, cv_image = cv2.threshold(cv_image, 230, 255, cv2.THRESH_BINARY)

        lanes = np.argwhere(cv_image == 255)

        for i in range(3):
            ransac = RANSAC(lanes, 100, 10)
            line, inlier, outlier = ransac.fit()
            m, b = line
            cv2.line(cv_image, (0, b), (640, 640 * m + b), 150, 10)
            lanes = outlier
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))


if __name__ == "__main__":
    LaneDetection()
