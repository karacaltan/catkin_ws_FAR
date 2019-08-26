import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import scipy.interpolate
import numpy as np


class Lane:

    def __init__(self, support_points):
        self.support_points = support_points
        self.spline_x = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [1]],
                                                      bc_type='periodic')
        self.spline_y = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [2]],
                                                      bc_type='periodic')

    def length(self):
        return self.support_points[:, 0][-1]

    def interpolate(self, param):
        return np.array([self.spline_x(param)[0], self.spline_y(param)[0]])

    def closest_point(self, point):
        step_size = 1.0
        min_param = 0.0
        max_param = self.length()
        closest_param = -1.0
        closest_distance = float('+inf')
        current_distance = 0.0

        while step_size > 0.001:
            closest_distance = float('+inf')
            closest_param = -1.0
            i = min_param
            while i <= max_param:
                current_distance = np.sum(np.power(self.interpolate(i) - point, 2.0))

                if closest_distance > current_distance:
                    closest_distance = current_distance
                    closest_param = i

                i += step_size

            min_param = max(min_param, closest_param - step_size)
            max_param = min(max_param, closest_param + step_size)
            step_size *= 0.5

        return self.interpolate(closest_param), closest_param

    def lookahead_point(self, point, lookahead_distance):
        closest_point, closest_param = self.closest_point(point)
        return self.interpolate(closest_param + lookahead_distance), closest_param + lookahead_distance


class Map:

    def __init__(self):
        self.lane_1 = np.load("lane1.npy")
        self.lane_2 = np.load("lane2.npy")
        self.lanes = [
            Lane(self.lane_1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]),
            Lane(self.lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :])]


class MapVisualization:

    def __init__(self):
        self.map = Map()
        rospy.init_node("map_visualization")
        self.lane_pub = rospy.Publisher("lane", Marker, queue_size=10)
        self.clicked_point_subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.on_click, queue_size=1)

        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            i = 0
            for lane in self.map.lanes:
                msg = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
                msg.header.frame_id = "map"
                msg.scale.x = 0.01
                msg.scale.y = 0.01
                msg.color.r = 1.0
                msg.color.a = 1.0
                msg.id = i

                for i in range(int(lane.length() * 100.0)):
                    inter = lane.interpolate(i / 100.0)
                    msg.points.append(Point(inter[0], inter[1], 0.0))
                i += 1

                self.lane_pub.publish(msg)

            self.rate.sleep()

    def on_click(self, point_msg):
        i = 2
        point = np.array([point_msg.point.x, point_msg.point.y])
        for lane in self.map.lanes:
            msg = Marker(type=Marker.SPHERE, action=Marker.ADD)
            msg.header.frame_id = "map"
            msg.scale.x = 0.1
            msg.scale.y = 0.1
            msg.scale.z = 0.1
            msg.color.b = 1.0
            msg.color.a = 1.0
            msg.id = i

            p, param = lane.closest_point(point)
            msg.pose.position.x = p[0]
            msg.pose.position.y = p[1]

            i += 1

            self.lane_pub.publish(msg)

            msg.color.b = 0.0
            msg.color.g = 1.0
            p, param = lane.lookahead_point(point, 0.5)
            msg.pose.position.x = p[0]
            msg.pose.position.y = p[1]
            msg.id = i

            i += 1

            self.lane_pub.publish(msg)


if __name__ == "__main__":
    MapVisualization()
