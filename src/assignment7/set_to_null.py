import rospy
import numpy as np
import time
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand
from nav_msgs.msg import Odometry


class PIDController:

    def __init__(self):
        rospy.init_node("initNull")
        self.speed = SpeedCommand()
        self.steer = NormalizedSteeringCommand()
        self.position = Odometry()
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.steer_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        self.position_sub = rospy.Subscriber("/communication/gps/9", Odometry, self.on_position, queue_size=10)

        self.rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.speed.value = 0.0
            self.speed_pub.publish(self.speed)

            self.steer.value = 0.0
            self.steer_pub.publish(self.steer)
            self.rate.sleep()


    def on_position(self, msg):
        print ("Ende")


if __name__ == "__main__":
    PIDController()
