import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringCommand
import tf.transformations
import numpy


class PID:

    def __init__(self):
        rospy.init_node("pid")
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        self.localization_sub = rospy.Subscriber("/communication/gps/23", Odometry, self.on_localization, queue_size=1)
        self.localization_sub = rospy.Subscriber("/control/steering", SteeringCommand, self.on_steering, queue_size=1)

        self.pose = Odometry()
        self.rate = rospy.Rate(100)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control)

        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.2
        self.min_i = -1.0
        self.max_i = 1.0

        self.integral_error = 0.0
        self.last_error = 0.0

        # this should be changed from a topic for future tasks
        self.desired_angle = 0

        rospy.on_shutdown(self.on_shutdown)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_localization(self, msg):
        self.pose = msg

    def on_localization(self, msg):
        self.desired_angle = msg.value

    def on_control(self, tmr):

        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()

        print dt
        quat = [self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z,
                self.pose.pose.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        error = self.desired_angle - yaw

        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        pid_output = self.kp * error + self.kd * derivative_error + self.ki * self.integral_error

        speed_msg = SpeedCommand()
        speed_msg.value = 0.3
        self.speed_pub.publish(speed_msg)

        steering_msg = NormalizedSteeringCommand()
        steering_msg.value = pid_output
        self.steering_pub.publish(steering_msg)

    def on_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.speed_pub.publish(speed_msg)


if __name__ == "__main__":
    PID()
