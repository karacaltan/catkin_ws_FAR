import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import time
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand
from nav_msgs.msg import Odometry
import numpy as np
import map
import scipy.interpolate
from tf.transformations import euler_from_quaternion


class Driver:

    def __init__(self):
        rospy.init_node("initDriver", anonymous=True)
        self.speed = SpeedCommand()
        self.steer = NormalizedSteeringCommand()
        self.position = Odometry()
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.steer_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        self.position_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.on_position,
                                             queue_size=10)

        self.rate = rospy.Rate(10)

        self.speed_value = 0.0
        self.steering_value = 0.0

        self.coordinateX = 0.0
        self.coordinateY = 0.0

        self.lane_1 = np.load("lane1.npy")
        self.lane_2 = np.load("lane2.npy")

        self.lanes = [
            map.Lane(self.lane_1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]),
            # map.Lane(self.lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :])
        ]

        while not rospy.is_shutdown():
            self.drive()
            pass

        rospy.spin()

    def drive(self):
        self.speed.value = self.speed_value
        self.speed_pub.publish(self.speed)
        self.rate.sleep()

    def on_position(self, msg):
        self.steer_pub.publish(self.steer)
        coordinate_x = msg.pose.pose.position.x
        coordinate_y = msg.pose.pose.position.y
        current_point = np.array([coordinate_x, coordinate_y])
        for lane in self.lanes:
            lookahead_p, param = lane.lookahead_point(current_point, 0.5)
            print lookahead_p
            print current_point
            print self.calc_angle(current_point, lookahead_p)

    @staticmethod
    def calc_angle(point1, point2):
        point1_y, point1_x = point1[0], point1[1]
        point2_y, point2_x = point2[0], point2[1]
        scalar_product = point1_x*point2_x + point1_y*point2_y
        length_point1 = np.sqrt(point1_x*point1_x + point1_y*point1_y)
        length_point2 = np.sqrt(point2_x*point2_x + point2_y*point2_y)
        yaw = np.arccos(scalar_product/length_point1*length_point2)
        return yaw


if __name__ == "__main__":
    Driver()
