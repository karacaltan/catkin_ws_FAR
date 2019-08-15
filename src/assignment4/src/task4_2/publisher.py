import rospy
import std_msgs.msg
from sensor_msgs.msg import CameraInfo

class BasicPublisher:

    def __init__(self):
        rospy.init_node("basic_publisher")
        self.cameraInfo_publisher = rospy.Publisher("/sensors/camera/infra1/camera_info", CameraInfo, queue_size=1)

        self.r = rospy.Rate(10)

        while not rospy.is_shutdown():
            cameraInfo_msg = CameraInfo()
            self.cameraInfo_publisher.publish(cameraInfo_msg)

            self.r.sleep()

if __name__ == "__main__":
    BasicPublisher()
