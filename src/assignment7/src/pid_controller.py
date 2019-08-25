import rospy
import time
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PIDController:

    def __init__(self):
        rospy.init_node("initSpeed", anonymous=True)
        self.speed = SpeedCommand()
        self.steer = NormalizedSteeringCommand()
        self.position = Odometry()
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.steer_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        self.position_sub = rospy.Subscriber("/communication/gps/9", Odometry, self.on_position, queue_size=10)
        self.error_counter = 0
        self.rate = rospy.Rate(10)

        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.current_time = time.time()
        self.last_time = self.current_time
        self.error_time = 0.0
        self.error = 0.0
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.PID = 0.0

        self.max_integrator = 5.0
        self.min_integrator = -5.0

        self.speed_value = 0.5
        self.steering_value = 0.0

        while not rospy.is_shutdown():
            self.drive()
            pass

        rospy.spin()

    def drive(self):
        self.speed.value = self.speed_value
        self.speed_pub.publish(self.speed)
        self.rate.sleep()

    def on_position(self, msg):
        yaw = self.get_euler(msg)
        pid = self.pid_controller(yaw)

        self.steer.value = pid
        self.steer_pub.publish(self.steer)

    def pid_controller(self, yaw):
        self.D = 0.0
        self.error = 0 - yaw
        self.P = self.error

        print "P:" + str(self.P)

        self.current_time = time.time()
        diff_time = self.current_time - self.last_time
        diff_error = self.error - self.error_time
        self.I = self.I / 2 + self.error * diff_time

        if self.I > self.max_integrator:
            self.I = self.max_integrator
        elif self.I < self.min_integrator:
            self.I = self.min_integrator

        print "I:" + str(self.I)

        if diff_time > 0:
            self.D = diff_error / diff_time

        print "D:" + str(self.D)

        self.PID = self.P * self.error + self.I * self.Ki + self.D * self.Kd

        print "PID:" + str(self.PID)

        self.error_time = self.error
        self.last_time = time.time()

        return self.PID

    @staticmethod
    def get_euler(msg):
        orientation = msg.pose.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        euler = euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler[0], euler[1], euler[2]
        return yaw


if __name__ == "__main__":
    PIDController()
