import numpy as np
from scipy import interpolate
from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


class DriveSplines:
    def __init__(self):
        rospy.init_node('drivesplines', anonymous=True)
        self.lane_1 = np.load('../lane/lane1.npy')[[0, 100, 150, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028,
                                                    1148, 1276], :]
        self.lane_2 = np.load('../lane/lane2.npy')[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300,
                                                    1476], :]
        self.rate = rospy.Rate(10)
        self.path = Marker()
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)

        while not rospy.is_shutdown():
            f1, f2 = self.cubic_spline(self.lane_1)
            #self.publish_marker(f1, f1)

    @staticmethod
    def cubic_spline(lane):
        x = np.array([])
        y = np.array([])
        y_ = np.array([])
        for i in range(0, len(lane)):
            x = np.append(x, lane[i][0])
            y = np.append(y, lane[i][1])
            y_ = np.append(y_, lane[i][2])
        f1 = interpolate.interp1d(x, y, kind='cubic')
        f2 = interpolate.interp1d(x, y_, kind='cubic')
        return f1, f2

    """def publish_marker(self, f1, f2):
        self.path.header.frame_id = "/map"  # publish path in map frame
        self.path.type = self.path.LINE_STRIP
        self.path.action = self.path.ADD
        self.path.lifetime = rospy.Duration(0)
        self.path.id = 0
        self.path.scale.x = 0.05
        self.path.color.a = 1.0
        self.path.color.r = 0.0
        self.path.color.g = 1.0
        self.path.color.b = 0.0
        self.path.pose.orientation.w = 1.0

        point = Point()
        point.x = 1.0
        point.y = 2.0
        self.path.points.append(point)
        self.marker_pub.publish(self.path)"""


if __name__ == "__main__":
    drive_splines = DriveSplines()

