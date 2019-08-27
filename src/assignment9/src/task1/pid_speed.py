import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringCommand
import tf.transformations
import numpy as np
import math


class PID:

    def __init__(self):
        rospy.init_node("pid_speed")
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.speed_sub = rospy.Subscriber("/sensors/speed", SpeedCommand, self.on_speed, queue_size=1)
        self.speed_control_sub = rospy.Subscriber("/control/speed", SpeedCommand, self.on_speed_control, queue_size=1)

        self.rate = rospy.Rate(100)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control)

        self.speed = 0.0
        self.desired_speed = 0.3
        self.max_speed = 0.5

        self.kp = 1.25
        self.ki = 0.0
        self.kd = 0.2
        self.min_i = 0.0
        self.max_i = 0.0

        self.integral_error = 0.0
        self.last_error = 0.0

        # this should be changed from a topic for future tasks
        self.desired_angle = 0

        rospy.on_shutdown(self.on_shutdown)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_speed_control(self, msg):
        print "Speed Control: " + str(msg.value)

    def on_speed(self, msg):
        self.speed = msg.value

    def on_control(self, tmr):

        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()

        # print dt
        error = self.desired_speed - self.speed

        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        pid_output = self.speed + self.kp * error + self.kd * derivative_error + self.ki * self.integral_error

        if self.max_speed < pid_output:
            pid_output = self.desired_speed
        elif pid_output < self.desired_speed:
            pid_output = self.desired_speed

        speed_msg = SpeedCommand()
        speed_msg.value = pid_output

        self.speed_pub.publish(speed_msg)

    def on_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.speed_pub.publish(speed_msg)


if __name__ == "__main__":
    PID()
